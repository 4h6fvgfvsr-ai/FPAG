`ifndef MOTION_TARGET_TRACKER_V
`define MOTION_TARGET_TRACKER_V
// Motion-triggered, single-target appearance tracking. No servo connection.
// Capture RGB332 at the centres of 16x16 cells into ping-pong synchronous RAM.
// Freeze an 8x8 appearance template at acquisition; search nearby positions and
// three sizes every COMPLETE frame. A stationary target keeps matching forever.
// Distinctive template samples must still match, so an empty old location is
// not accepted merely because most of the rectangle contains background.
// Outputs commit only at SOF; malformed/stale streams invalidate immediately.
module motion_target_tracker #(
    parameter integer IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter integer SEARCH_RADIUS=4,
    parameter integer LOST_FRAMES=8,
    parameter integer REACQUIRE_DELAY=8,
    parameter integer MATCH_ERROR=6,
    parameter integer FEATURE_CONTRAST=6
)(
    input wire I_clk,I_rst_n,I_de,I_user,I_last,I_enable,I_source_fresh,
    input wire [23:0] I_rgb,
    input wire I_motion_update,I_motion_valid,I_scene_change,
    input wire [11:0] I_motion_left,I_motion_right,I_motion_top,I_motion_bottom,
    output reg O_update,O_valid,
    output reg [11:0] O_left,O_right,O_top,O_bottom
);
    localparam integer GW=IMG_WIDTH/16, GH=IMG_HEIGHT/16, CELLS=GW*GH;
    localparam integer AB=$clog2(2*CELLS+1), CB=$clog2(CELLS+1);
    localparam integer PB=$clog2(IMG_WIDTH*IMG_HEIGHT+1);
    localparam integer XY=$clog2(2*((GW>GH)?GW:GH)+1)+1;
    localparam [3:0] IDLE=0, INIT_READ=1, INIT_EVAL=2, CANDIDATE=3,
        SAMPLE_READ=4,SAMPLE_EVAL=5,SCORE=6,NEXT_CANDIDATE=7,FINISH=8;
    (* ram_style="bram" *) reg [7:0] grid[0:2*CELLS-1];
    reg [8:0] appearance[0:63];
    reg [7:0] grid_data;
    reg [8:0] appearance_data;
    reg write_bank,ready_bank,ready_good,active;
    reg [11:0] x,y;
    reg [PB-1:0] pixel_count;
    reg [CB-1:0] capture_index;
    reg [3:0] state;
    reg read_bank,locked,result_pending;
    reg [7:0] misses,cooldown;
    reg signed [XY-1:0] target_x,target_y,candidate_x,candidate_y,best_x,best_y;
    reg [XY-1:0] target_w,target_h,candidate_w,candidate_h,best_w,best_h;
    reg signed [5:0] dx,dy;
    reg [1:0] scale;
    reg [5:0] point_index;
    reg [7:0] background_sample;
    reg [6:0] feature_count,features_visible,features_good,samples_visible;
    reg [12:0] sum_error,best_error;
    reg [7:0] best_distance;
    reg best_found;
    wire [7:0] rgb332={I_rgb[23:21],I_rgb[15:13],I_rgb[7:6]};
    wire line_good=(I_last==(x==IMG_WIDTH-1));
    wire capture=I_de && !I_user && I_enable && I_source_fresh && active &&
        line_good && x[3:0]==8 && y[3:0]==8 && capture_index<CELLS;
    wire stream_fault=I_de && (I_user?(active || I_last):
        (!active || !line_good || (I_last && y==IMG_HEIGHT-1 &&
        (state!=IDLE || pixel_count!=IMG_WIDTH*IMG_HEIGHT-1 || capture_index!=CELLS))));
    wire [AB-1:0] capture_address=capture_index+(write_bank?CELLS:0);
    wire [XY-1:0] acquire_l=(I_motion_left>>4)>0?(I_motion_left>>4)-1:0;
    wire [XY-1:0] acquire_t=(I_motion_top>>4)>0?(I_motion_top>>4)-1:0;
    wire [XY-1:0] acquire_r=(I_motion_right>>4)+1<GW?(I_motion_right>>4)+1:GW-1;
    wire [XY-1:0] acquire_b=(I_motion_bottom>>4)+1<GH?(I_motion_bottom>>4)+1:GH-1;
    wire [XY-1:0] acquire_w=acquire_r-acquire_l+1;
    wire [XY-1:0] acquire_h=acquire_b-acquire_t+1;

    function [5:0] color_error;
        input [7:0] a,b;
        reg [2:0] dr,dg;
        reg [1:0] db;
        begin
            dr=a[7:5]>=b[7:5]?a[7:5]-b[7:5]:b[7:5]-a[7:5];
            dg=a[4:2]>=b[4:2]?a[4:2]-b[4:2]:b[4:2]-a[4:2];
            db=a[1:0]>=b[1:0]?a[1:0]-b[1:0]:b[1:0]-a[1:0];
            color_error={dr,1'b0}+{dg,1'b0}+{db,2'b00};
        end
    endfunction
    function [AB-1:0] row_address;
        input [XY:0] row;
        integer k;
        begin
            row_address=0;
            for(k=0;k<16;k=k+1)
                if((GW>>k)&1) row_address=row_address+(row<<k);
        end
    endfunction
    function [12:0] error_limit;
        input [6:0] n;
        integer k;
        begin
            error_limit=0;
            for(k=0;k<8;k=k+1)
                if((MATCH_ERROR>>k)&1) error_limit=error_limit+({6'd0,n}<<k);
        end
    endfunction
    // Sample at the centres of eight evenly spaced bins. Explicit shifts/adds
    // keep the variable coordinate arithmetic out of the scarce DSP blocks.
    function [XY-1:0] sample_offset;
        input [XY-1:0] extent;
        input [2:0] bin;
        reg [XY+3:0] product;
        begin
            product={{4{1'b0}},extent};
            if(bin[0]) product=product+({{4{1'b0}},extent}<<1);
            if(bin[1]) product=product+({{4{1'b0}},extent}<<2);
            if(bin[2]) product=product+({{4{1'b0}},extent}<<3);
            sample_offset=product>>4;
        end
    endfunction
    wire initializing=(state==INIT_READ || state==INIT_EVAL);
    wire signed [XY:0] sample_x=$signed(initializing?target_x:candidate_x)+
        $signed({1'b0,sample_offset(initializing?target_w:candidate_w,point_index[2:0])});
    wire signed [XY:0] sample_y=$signed(initializing?target_y:candidate_y)+
        $signed({1'b0,sample_offset(initializing?target_h:candidate_h,point_index[5:3])});
    wire sample_inside=sample_x>=0 && sample_x<GW && sample_y>=0 && sample_y<GH;
    wire [AB-1:0] read_address=(read_bank?CELLS:0)+row_address(sample_y)+sample_x;
    wire reading=(state==INIT_READ || state==SAMPLE_READ) && sample_inside;
    wire [5:0] residual=color_error(grid_data,appearance_data[7:0]);
    wire feature_good=residual<=MATCH_ERROR;
    wire [7:0] candidate_distance=(dx<0?-dx:dx)+(dy<0?-dy:dy)+scale;
    wire [12:0] missing_penalty=error_limit(7'd64-samples_visible);
    wire [12:0] score_error=sum_error+missing_penalty;
    wire candidate_matches=samples_visible>=16 &&
        features_visible>=2 && features_visible>=(feature_count>>2) &&
        features_good>=features_visible-(features_visible>>2) &&
        sum_error<=error_limit(samples_visible);
    // Both ports are synchronous. The capture port never writes read_bank
    // while its frame is being matched; a processing overrun is rejected.
    always @(posedge I_clk) begin
        if(capture) grid[capture_address]<=rgb332;
        if(reading) grid_data<=grid[read_address];
        if(state==SAMPLE_READ) appearance_data<=appearance[point_index];
        if(state==INIT_EVAL) appearance[point_index]<=
            {(point_index!=0 && color_error(grid_data,background_sample)>=FEATURE_CONTRAST),grid_data};
    end

    task invalidate;
        begin
            O_valid<=0;O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
            locked<=0;result_pending<=0;state<=IDLE;misses<=0;
        end
    endtask

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            write_bank<=0;ready_bank<=0;ready_good<=0;active<=0;
            x<=0;y<=0;pixel_count<=0;capture_index<=0;
            state<=IDLE;read_bank<=0;locked<=0;result_pending<=0;
            misses<=0;cooldown<=0;target_x<=0;target_y<=0;target_w<=0;target_h<=0;
            candidate_x<=0;candidate_y<=0;candidate_w<=0;candidate_h<=0;
            best_x<=0;best_y<=0;best_w<=0;best_h<=0;dx<=0;dy<=0;scale<=0;
            point_index<=0;background_sample<=0;feature_count<=0;
            features_visible<=0;features_good<=0;samples_visible<=0;
            sum_error<=0;best_error<=13'h1fff;best_distance<=255;best_found<=0;
            O_update<=0;O_valid<=0;O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
        end else begin
            O_update<=0;
            if(!I_enable || !I_source_fresh) begin
                active<=0;ready_good<=0;capture_index<=0;cooldown<=0;
                invalidate();
            end else begin
                if(capture) capture_index<=capture_index+1'b1;
                if(I_de) begin
                    if(I_user) begin
                        active<=1;x<=1;y<=0;pixel_count<=1;capture_index<=0;
                        if(active || I_last) begin
                            ready_good<=0;invalidate();
                            if(I_last) active<=0;
                        end else if(result_pending) begin
                            result_pending<=0;O_update<=1;O_valid<=locked;
                            if(locked) begin
                                O_left<=target_x<0?0:target_x<<4;
                                O_top<=target_y<0?0:target_y<<4;
                                O_right<=(target_x+$signed({1'b0,target_w})>=GW)?
                                    IMG_WIDTH-1:((target_x+$signed({1'b0,target_w}))<<4)-1;
                                O_bottom<=(target_y+$signed({1'b0,target_h})>=GH)?
                                    IMG_HEIGHT-1:((target_y+$signed({1'b0,target_h}))<<4)-1;
                            end else begin O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;end
                        end
                    end else if(active) begin
                        if(!line_good) begin
                            active<=0;ready_good<=0;invalidate();
                        end else if(I_last && y==IMG_HEIGHT-1) begin
                            active<=0;
                            ready_good<=(pixel_count==IMG_WIDTH*IMG_HEIGHT-1 && capture_index==CELLS);
                            ready_bank<=write_bank;write_bank<=!write_bank;
                            if(state!=IDLE || pixel_count!=IMG_WIDTH*IMG_HEIGHT-1 || capture_index!=CELLS)
                                invalidate();
                        end else begin
                            pixel_count<=pixel_count+1'b1;
                            if(I_last) begin x<=0;y<=y+1'b1;end else x<=x+1'b1;
                        end
                    end else begin ready_good<=0;invalidate();end
                end

                if(!stream_fault && I_motion_update && ready_good && state==IDLE) begin
                    read_bank<=ready_bank;
                    if(locked) begin
                        dx<=-SEARCH_RADIUS;dy<=-SEARCH_RADIUS;scale<=0;
                        best_error<=13'h1fff;best_distance<=255;best_found<=0;
                        state<=CANDIDATE;
                    end else if(cooldown!=0) cooldown<=cooldown-1'b1;
                    else if(I_motion_valid && !I_scene_change &&
                        I_motion_left<=I_motion_right && I_motion_top<=I_motion_bottom &&
                        I_motion_right<IMG_WIDTH && I_motion_bottom<IMG_HEIGHT) begin
                        // One-cell background border makes template foreground evidence
                        // available even for a plain, uniformly coloured moving object.
                        target_x<=acquire_w<4 && acquire_l>GW-4?GW-4:acquire_l;
                        target_y<=acquire_h<4 && acquire_t>GH-4?GH-4:acquire_t;
                        target_w<=acquire_w<4?4:acquire_w;
                        target_h<=acquire_h<4?4:acquire_h;
                        point_index<=0;feature_count<=0;state<=INIT_READ;
                    end
                end

                if(!stream_fault) case(state)
                    INIT_READ: begin
                        if(!sample_inside) begin state<=IDLE;cooldown<=2;end
                        else state<=INIT_EVAL;
                    end
                    INIT_EVAL: begin
                        if(point_index==0) background_sample<=grid_data;
                        else if(color_error(grid_data,background_sample)>=FEATURE_CONTRAST)
                            feature_count<=feature_count+1'b1;
                        if(point_index==63) begin
                            state<=IDLE;result_pending<=1;
                            locked<=feature_count+(color_error(grid_data,background_sample)>=FEATURE_CONTRAST)>=4;
                            misses<=0;
                        end else begin point_index<=point_index+1'b1;state<=INIT_READ;end
                    end
                    CANDIDATE: begin
                        candidate_x<=target_x+dx;candidate_y<=target_y+dy;
                        candidate_w<=scale==1 && target_w>4?target_w-1:
                            scale==2 && target_w<GW?target_w+1:target_w;
                        candidate_h<=scale==1 && target_h>4?target_h-1:
                            scale==2 && target_h<GH?target_h+1:target_h;
                        point_index<=0;sum_error<=0;samples_visible<=0;
                        features_visible<=0;features_good<=0;
                        state<=SAMPLE_READ;
                    end
                    SAMPLE_READ: state<=SAMPLE_EVAL;
                    SAMPLE_EVAL: begin
                        if(sample_inside) begin
                            sum_error<=sum_error+residual;samples_visible<=samples_visible+1'b1;
                            if(appearance_data[8]) begin
                                features_visible<=features_visible+1'b1;
                                if(feature_good) features_good<=features_good+1'b1;
                            end
                        end
                        if(point_index==63) state<=SCORE;
                        else begin point_index<=point_index+1'b1;state<=SAMPLE_READ;end
                    end
                    SCORE: begin
                        if(candidate_matches && (!best_found || score_error<best_error ||
                            (score_error==best_error && candidate_distance<best_distance))) begin
                            best_found<=1;best_error<=score_error;best_distance<=candidate_distance;
                            best_x<=candidate_x;best_y<=candidate_y;
                            best_w<=candidate_w;best_h<=candidate_h;
                        end
                        state<=NEXT_CANDIDATE;
                    end
                    NEXT_CANDIDATE: begin
                        if(dx<SEARCH_RADIUS) begin dx<=dx+1'b1;state<=CANDIDATE;end
                        else if(dy<SEARCH_RADIUS) begin dx<=-SEARCH_RADIUS;dy<=dy+1'b1;state<=CANDIDATE;end
                        else if(scale<2) begin dx<=-SEARCH_RADIUS;dy<=-SEARCH_RADIUS;scale<=scale+1'b1;state<=CANDIDATE;end
                        else state<=FINISH;
                    end
                    FINISH: begin
                        state<=IDLE;result_pending<=1;
                        if(best_found) begin
                            target_x<=best_x;target_y<=best_y;target_w<=best_w;target_h<=best_h;misses<=0;
                        end else if(misses+1>=LOST_FRAMES) begin
                            locked<=0;misses<=0;cooldown<=REACQUIRE_DELAY;
                        end else misses<=misses+1'b1;
                    end
                    default: ;
                endcase
            end
        end
    end
endmodule
`endif
