// Motion-only localization: 4x4 center sampling, adjacent 8-bit gray frames,
// Directional 3x3 erosion -> 3x3 dilation -> global bbox. No appearance hold.
// RGB/HDMI remains full resolution. Default mask is 320x180, bbox units 4px.
// A complete frame warms RAM after disable, timeout or malformed input.
// Results commit only after input validation and morphology bottom padding.
`ifndef MOTION_OPENING_DETECTOR_V
`define MOTION_OPENING_DETECTOR_V
`include "motion_binary_3x3.v"
module motion_opening_detector #(
    parameter integer IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter integer DIFFERENCE_THRESHOLD=24, MIN_CHANGED_CELLS=12,
    parameter integer SCENE_CHANGE_PERCENT=35, RUNTIME_CONTROL=1, EROSION_MODE=2,
    parameter integer SOURCE_TAG_CONTROL=0
)(
    input wire I_clk,I_rst_n,I_de,I_user,I_last,
    input wire [23:0] I_rgb,
    input wire I_enable,I_source_fresh,
    input wire [7:0] I_difference_threshold,
    // Metadata is published with nonblocking assignments at the same SOF.
    // Accept its update on the following clock, before the first (2,2) sample.
    input wire I_source_update,I_source_valid,
    input wire [15:0] I_source_frame_id,
    output reg O_update,O_valid,O_scene_change,
    output reg [11:0] O_left,O_right,O_top,O_bottom,
    output reg [15:0] O_changed_cells,O_event_count
);
    localparam integer GW=IMG_WIDTH/4, GH=IMG_HEIGHT/4, CELLS=GW*GH;
    localparam integer AB=$clog2(CELLS+1),PB=$clog2(IMG_WIDTH*IMG_HEIGHT+1);
    localparam integer SCENE_CELLS=(CELLS*SCENE_CHANGE_PERCENT+99)/100;
    (* ram_style="bram" *) reg [7:0] previous_gray[0:CELLS-1];
    reg [7:0] previous_read,sampled_gray,threshold;
    reg active,processing,complete,reference_valid,frame_reference;
    reg [11:0] px,py,sampled_x,sampled_y;
    reg [AB-1:0] sample_index;
    reg [PB-1:0] pixel_count;
    reg sample_pending;
    reg tag_pending,tag_ready,duplicate_frame;
    reg [15:0] reference_id,frame_id;
    reg [15:0] raw_count,opened_count;
    reg [11:0] box_l,box_r,box_t,box_b;
    wire sof=I_de && I_user;
    wire line_ok=I_last==(px==IMG_WIDTH-1);
    wire sample_slot=I_de && !I_user && active && line_ok &&
                     px[1:0]==2 && py[1:0]==2;
    wire raster_fault=I_de && !I_user && (!active || !line_ok);
    wire tag_fault=SOURCE_TAG_CONTROL && !sof &&
        ((tag_pending && I_source_update && !I_source_valid) ||
         (tag_ready && !I_source_valid) ||
         (active && !tag_pending && I_source_update) ||
         (sample_slot && !tag_ready));
    wire fault=raster_fault || tag_fault;
    wire [15:0] source_delta=I_source_frame_id-reference_id;
    wire abort_filter=!I_enable || !I_source_fresh || sof || fault;
    wire sample_now=sample_slot && I_enable && I_source_fresh &&
                    processing && (!SOURCE_TAG_CONTROL || tag_ready) &&
                    !duplicate_frame && !fault && sample_index<CELLS;
    // 77R+150G+29B, expressed as shifts/adds: no DSP multipliers.
    wire [15:0] r={8'b0,I_rgb[23:16]},g={8'b0,I_rgb[15:8]},b={8'b0,I_rgb[7:0]};
    wire [15:0] yr=(r<<6)+(r<<3)+(r<<2)+r;
    wire [15:0] yg=(g<<7)+(g<<4)+(g<<2)+(g<<1);
    wire [15:0] yb=(b<<4)+(b<<3)+(b<<2)+b;
    wire [15:0] gray_sum=yr+yg+yb;
    wire [7:0] difference=sampled_gray>=previous_read ?
        sampled_gray-previous_read : previous_read-sampled_gray;
    wire mask_valid=sample_pending && processing;
    wire mask_bit=frame_reference && difference!=0 && difference>=threshold;
    wire erode_valid,erode_bit,opened_valid,opened_bit;
    wire [11:0] erode_x,erode_y,opened_x,opened_y;
    motion_binary_3x3 #(.WIDTH(GW),.HEIGHT(GH),.ERODE(EROSION_MODE)) u_erode(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_abort(abort_filter),
        .I_valid(mask_valid),.I_bit(mask_bit),.I_x(sampled_x),.I_y(sampled_y),
        .O_valid(erode_valid),.O_bit(erode_bit),.O_x(erode_x),.O_y(erode_y));
    motion_binary_3x3 #(.WIDTH(GW),.HEIGHT(GH),.ERODE(0)) u_dilate(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_abort(abort_filter),
        .I_valid(erode_valid),.I_bit(erode_bit),.I_x(erode_x),.I_y(erode_y),
        .O_valid(opened_valid),.O_bit(opened_bit),.O_x(opened_x),.O_y(opened_y));
    wire hit=opened_valid && opened_bit;
    wire [15:0] final_count=opened_count+hit;
    wire [11:0] final_l=hit && (opened_count==0 || opened_x<box_l) ? opened_x : box_l;
    wire [11:0] final_r=hit && (opened_count==0 || opened_x>box_r) ? opened_x : box_r;
    wire [11:0] final_t=hit && (opened_count==0 || opened_y<box_t) ? opened_y : box_t;
    wire [11:0] final_b=hit && (opened_count==0 || opened_y>box_b) ? opened_y : box_b;
    always @(posedge I_clk) if(sample_now) begin
        previous_read<=previous_gray[sample_index];
        previous_gray[sample_index]<=gray_sum[15:8];
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            active<=0;processing<=0;complete<=0;reference_valid<=0;frame_reference<=0;
            px<=0;py<=0;sample_index<=0;pixel_count<=0;sample_pending<=0;
            tag_pending<=0;tag_ready<=0;duplicate_frame<=0;reference_id<=0;frame_id<=0;
            sampled_gray<=0;sampled_x<=0;sampled_y<=0;threshold<=DIFFERENCE_THRESHOLD;
            raw_count<=0;opened_count<=0;box_l<=0;box_r<=0;box_t<=0;box_b<=0;
            O_update<=0;O_valid<=0;O_scene_change<=0;O_left<=0;O_right<=0;
            O_top<=0;O_bottom<=0;O_changed_cells<=0;O_event_count<=0;
        end else begin
            O_update<=0;sample_pending<=sample_now;
            if(sample_now) begin
                sampled_gray<=gray_sum[15:8];sampled_x<=px>>2;sampled_y<=py>>2;
                sample_index<=sample_index+1'b1;
            end
            if(mask_valid && mask_bit) raw_count<=raw_count+1'b1;
            if(processing && hit) begin
                opened_count<=final_count;
                box_l<=final_l;box_r<=final_r;box_t<=final_t;box_b<=final_b;
            end
            if(processing && !abort_filter && opened_valid && opened_x==GW-1 && opened_y==GH-1) begin
                processing<=0;O_update<=1;
                reference_valid<=complete;
                if(complete) reference_id<=frame_id;
                O_changed_cells<=complete && frame_reference ? final_count : 16'b0;
                O_scene_change<=complete && frame_reference && raw_count>=SCENE_CELLS;
                if(complete && frame_reference && final_count>=MIN_CHANGED_CELLS && raw_count<SCENE_CELLS) begin
                    O_valid<=1;O_left<=final_l<<2;O_right<=(final_r<<2)|12'd3;
                    O_top<=final_t<<2;O_bottom<=(final_b<<2)|12'd3;
                    if(!O_valid && O_event_count!=16'hffff) O_event_count<=O_event_count+1'b1;
                end else begin
                    O_valid<=0;O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                end
            end
            if(!I_enable || !I_source_fresh || fault) begin
                active<=0;processing<=0;complete<=0;reference_valid<=0;frame_reference<=0;
                sample_pending<=0;O_valid<=0;O_update<=0;O_scene_change<=0;
                tag_pending<=0;tag_ready<=0;duplicate_frame<=0;
                O_changed_cells<=0;O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
            end else if(sof) begin
                active<=!I_last;processing<=!I_last;complete<=0;
                tag_pending<=SOURCE_TAG_CONTROL && !I_last;
                tag_ready<=!SOURCE_TAG_CONTROL;duplicate_frame<=0;
                frame_reference<=reference_valid && !active && !processing && !I_last;
                if(active || processing || I_last) begin
                    reference_valid<=0;O_valid<=0;O_scene_change<=0;
                    O_changed_cells<=0;O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                end
                px<=1;py<=0;pixel_count<=1;sample_index<=0;sample_pending<=0;
                threshold<=RUNTIME_CONTROL ? I_difference_threshold : DIFFERENCE_THRESHOLD;
                raw_count<=0;opened_count<=0;box_l<=0;box_r<=0;box_t<=0;box_b<=0;
            end else if(I_de && active) begin
                if(I_last && py==IMG_HEIGHT-1) begin
                    active<=0;
                    complete<=pixel_count==IMG_WIDTH*IMG_HEIGHT-1 &&
                        (duplicate_frame || sample_index==CELLS);
                end else begin
                    pixel_count<=pixel_count+1'b1;
                    if(I_last) begin px<=0;py<=py+1'b1;end else px<=px+1'b1;
                end
            end
            if(SOURCE_TAG_CONTROL && !sof && !fault && I_enable && I_source_fresh &&
               tag_pending && I_source_update && I_source_valid) begin
                tag_pending<=0;tag_ready<=1;frame_id<=I_source_frame_id;
                duplicate_frame<=reference_valid && source_delta==0;
                processing<=!(reference_valid && source_delta==0);
                // An out-of-order page starts a new reference session. Normal
                // forward IDs, dropped frames and 65535->0 wrap remain valid.
                if(reference_valid && source_delta[15]) begin
                    reference_valid<=0;frame_reference<=0;
                    O_valid<=0;O_scene_change<=0;O_changed_cells<=0;
                    O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                end
            end
        end
    end
endmodule
`endif
