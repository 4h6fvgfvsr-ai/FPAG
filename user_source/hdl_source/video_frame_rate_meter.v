`ifndef VIDEO_FRAME_RATE_METER_V
`define VIDEO_FRAME_RATE_METER_V
// Count complete ISP RGB payloads. A new SOF validates the preceding payload;
// truncated/oversized frames are rejected. The 50 MHz reference measures one
// second; Gray counters and a stable-data toggle bridge the clock domains.
module video_frame_rate_meter #(
    parameter integer FRAME_BEATS=1280*720*3/16,
    parameter integer REF_CLK_HZ=50000000,
    parameter integer CAPTURE_TIMEOUT_MS=200
)(
    input wire I_frame_clk,I_ref_clk,I_display_clk,I_rst_n,
    input wire I_frame_start,I_frame_valid,
    output reg [11:0] O_fps_bcd,
    output reg O_capture_fresh
);
    localparam integer BEAT_BITS=$clog2(FRAME_BEATS+2);
    localparam integer REF_BITS=REF_CLK_HZ>1 ? $clog2(REF_CLK_HZ) : 1;
    reg [1:0] frame_reset,ref_reset,display_reset;
    always @(posedge I_frame_clk or negedge I_rst_n)
        if(!I_rst_n) frame_reset<=0; else frame_reset<={frame_reset[0],1'b1};
    always @(posedge I_ref_clk or negedge I_rst_n)
        if(!I_rst_n) ref_reset<=0; else ref_reset<={ref_reset[0],1'b1};
    always @(posedge I_display_clk or negedge I_rst_n)
        if(!I_rst_n) display_reset<=0; else display_reset<={display_reset[0],1'b1};

    reg frame_active;
    reg [BEAT_BITS-1:0] frame_beats;
    reg [15:0] frame_total,frame_gray;
    wire [15:0] next_total=frame_total+16'd1;
    always @(posedge I_frame_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin frame_active<=0;frame_beats<=0;frame_total<=0;frame_gray<=0;end
        else if(!frame_reset[1]) begin frame_active<=0;frame_beats<=0;frame_total<=0;frame_gray<=0;end
        else if(I_frame_start) begin
            if(frame_active && frame_beats==FRAME_BEATS) begin
                frame_total<=next_total;frame_gray<=next_total^(next_total>>1);
            end
            frame_active<=1;frame_beats<=I_frame_valid ? 1 : 0;
        end else if(frame_active && I_frame_valid && frame_beats<FRAME_BEATS+1)
            frame_beats<=frame_beats+1'b1;
    end

    reg [15:0] gray_sync_1,gray_sync_2; // synthesis keep
    reg [15:0] previous_total;
    reg [REF_BITS-1:0] reference_count;
    function [15:0] gray_to_binary;
        input [15:0] gray;
        integer i;
        begin
            gray_to_binary[15]=gray[15];
            for(i=14;i>=0;i=i-1) gray_to_binary[i]=gray_to_binary[i+1]^gray[i];
        end
    endfunction
    wire [15:0] sampled_total=gray_to_binary(gray_sync_2);
    wire [15:0] sampled_difference=sampled_total-previous_total;
    // Actuator freshness follows validated camera frames, independently of
    // the 1-second displayed FPS window or repeated DDR display frames.
    localparam integer CAPTURE_AGE_LIMIT=(REF_CLK_HZ/1000)*CAPTURE_TIMEOUT_MS>0 ?
        (REF_CLK_HZ/1000)*CAPTURE_TIMEOUT_MS : 1;
    localparam integer CAPTURE_AGE_BITS=$clog2(CAPTURE_AGE_LIMIT+1);
    reg [15:0] capture_gray_seen;
    reg [CAPTURE_AGE_BITS-1:0] capture_age;
    always @(posedge I_ref_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin capture_gray_seen<=0;capture_age<=CAPTURE_AGE_LIMIT;O_capture_fresh<=0;end
        else if(!ref_reset[1]) begin capture_gray_seen<=0;capture_age<=CAPTURE_AGE_LIMIT;O_capture_fresh<=0;end
        else if(gray_sync_2!=capture_gray_seen) begin
            capture_gray_seen<=gray_sync_2;capture_age<=0;O_capture_fresh<=1;
        end else if(capture_age<CAPTURE_AGE_LIMIT) begin
            capture_age<=capture_age+1'b1;
            if(capture_age==CAPTURE_AGE_LIMIT-1) O_capture_fresh<=0;
        end
    end
    reg [9:0] convert_binary;
    reg [11:0] convert_bcd,stable_bcd;
    reg [3:0] convert_left;
    reg update_toggle;
    reg [11:0] adjusted_bcd;
    integer digit;
    always @* begin
        adjusted_bcd=convert_bcd;
        for(digit=0;digit<3;digit=digit+1)
            if(convert_bcd[digit*4+:4]>=5)
                adjusted_bcd[digit*4+:4]=convert_bcd[digit*4+:4]+4'd3;
    end
    always @(posedge I_ref_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            gray_sync_1<=0;gray_sync_2<=0;previous_total<=0;reference_count<=0;
            convert_binary<=0;convert_bcd<=0;stable_bcd<=0;convert_left<=0;update_toggle<=0;
        end else if(!ref_reset[1]) begin
            gray_sync_1<=0;gray_sync_2<=0;previous_total<=0;reference_count<=0;
            convert_binary<=0;convert_bcd<=0;stable_bcd<=0;convert_left<=0;update_toggle<=0;
        end else begin
            gray_sync_1<=frame_gray;gray_sync_2<=gray_sync_1;
            if(reference_count==REF_CLK_HZ-1) begin
                reference_count<=0;previous_total<=sampled_total;
                convert_binary<=sampled_difference>999 ? 10'd999 : sampled_difference[9:0];
                convert_bcd<=0;convert_left<=10;
            end else begin
                reference_count<=reference_count+1'b1;
                if(convert_left!=0) begin
                    convert_bcd<={adjusted_bcd[10:0],convert_binary[9]};
                    convert_binary<={convert_binary[8:0],1'b0};
                    convert_left<=convert_left-1'b1;
                    if(convert_left==1) begin
                        stable_bcd<={adjusted_bcd[10:0],convert_binary[9]};
                        update_toggle<=~update_toggle;
                    end
                end
            end
        end
    end
    reg [1:0] update_sync; // synthesis keep
    reg update_seen;
    always @(posedge I_display_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin update_sync<=0;update_seen<=0;O_fps_bcd<=0;end
        else if(!display_reset[1]) begin update_sync<=0;update_seen<=0;O_fps_bcd<=0;end
        else begin
            update_sync<={update_sync[0],update_toggle};
            if(update_sync[1]!=update_seen) begin O_fps_bcd<=stable_bcd;update_seen<=update_sync[1];end
        end
    end
    // synthesis translate_off
    initial if(FRAME_BEATS<1 || REF_CLK_HZ<16)
        $fatal(1,"Invalid frame-rate measurement parameters");
    // synthesis translate_on
endmodule
`endif
