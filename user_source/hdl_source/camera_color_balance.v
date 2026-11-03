`ifndef CAMERA_COLOR_BALANCE_V
`define CAMERA_COLOR_BALANCE_V
// Color-preserving AWB: estimate G/R and G/B ONLY from near-neutral pixels.
// A green/red/blue object must not be its own white reference. One common
// gain per channel for all four lanes; gains change only at frame boundaries.
// Input/output RGB and SOF/EOL/VALID take the same two pipeline clocks.
module camera_color_balance #(
    parameter integer IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter integer MIN_NEUTRAL=64,
    // Original video_in resets its FIFO for ~10 camera clocks after SOF.
    // Retain the original 50-cycle BRAM delay and EARLY SOF in the 103 ISP.
    parameter integer FRAME_RESET_LEAD=0,
    // Preserve strongly red objects when a warm neutral reference lowers R.
    parameter integer RED_CHROMA_PRESERVE=0,
    parameter integer MIN_GAIN=3072, MAX_GAIN=6144 // Q12, 0.75x..1.50x
)(
    input wire I_clk, I_rst_n, I_tlast, I_tuser, I_tvalid,
    input wire [95:0] I_tdata,
    output wire I_tready, O_tlast, O_tuser, O_tvalid,
    output wire [95:0] O_tdata,
    input wire O_tready
);
    localparam integer FRAME_BEATS=IMG_WIDTH*IMG_HEIGHT/4;
    localparam integer CB=$clog2(FRAME_BEATS+2);
    localparam integer SB=$clog2(IMG_WIDTH*IMG_HEIGHT*255+1);
    localparam integer NB=SB+12;
    wire [7:0] r[0:3],g[0:3],b[0:3],mx[0:3],mn[0:3];
    wire [3:0] neutral;
    wire [9:0] add_r,add_g,add_b;
    genvar i;
    generate for(i=0;i<4;i=i+1) begin : lanes
        assign r[i]=I_tdata[i*24+16 +: 8];
        assign g[i]=I_tdata[i*24+8 +: 8];
        assign b[i]=I_tdata[i*24 +: 8];
        assign mx[i]=r[i]>g[i] ? (r[i]>b[i] ? r[i] : b[i]) : (g[i]>b[i] ? g[i] : b[i]);
        assign mn[i]=r[i]<g[i] ? (r[i]<b[i] ? r[i] : b[i]) : (g[i]<b[i] ? g[i] : b[i]);
        // Exclude dark noise, clipped highlights and saturated colors.
        assign neutral[i]=mx[i]>=32 && mx[i]<=224 &&
            3*({1'b0,mx[i]}-{1'b0,mn[i]}) <= mx[i];
    end endgenerate
    assign add_r=(neutral[0]?{2'b0,r[0]}:10'd0)+(neutral[1]?{2'b0,r[1]}:10'd0)+
                 (neutral[2]?{2'b0,r[2]}:10'd0)+(neutral[3]?{2'b0,r[3]}:10'd0);
    assign add_g=(neutral[0]?{2'b0,g[0]}:10'd0)+(neutral[1]?{2'b0,g[1]}:10'd0)+
                 (neutral[2]?{2'b0,g[2]}:10'd0)+(neutral[3]?{2'b0,g[3]}:10'd0);
    assign add_b=(neutral[0]?{2'b0,b[0]}:10'd0)+(neutral[1]?{2'b0,b[1]}:10'd0)+
                 (neutral[2]?{2'b0,b[2]}:10'd0)+(neutral[3]?{2'b0,b[3]}:10'd0);
    wire [2:0] add_n={2'b0,neutral[0]}+{2'b0,neutral[1]}+{2'b0,neutral[2]}+{2'b0,neutral[3]};
    reg valid_q,start_q;
    reg [9:0] r_q,g_q,b_q;
    reg [2:0] n_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin valid_q<=0;start_q<=0;r_q<=0;g_q<=0;b_q<=0;n_q<=0;end
        else begin valid_q<=I_tvalid;start_q<=I_tuser;r_q<=add_r;g_q<=add_g;b_q<=add_b;n_q<=add_n;end
    end
    reg have_frame;
    reg [CB-1:0] beats;
    reg [SB-1:0] sum_r,sum_g,sum_b;
    reg [CB+1:0] neutral_count;
    // Two small restoring dividers run once per frame, not every pixel.
    reg busy,pending;
    reg [NB-1:0] dividend_r,dividend_b,quotient_r,quotient_b;
    reg [SB-1:0] divisor_r,divisor_b,remainder_r,remainder_b;
    reg [7:0] remaining;
    reg [13:0] next_r,next_b,gain_r,gain_b;
    wire [SB:0] shift_r={remainder_r,dividend_r[NB-1]};
    wire [SB:0] shift_b={remainder_b,dividend_b[NB-1]};
    wire bit_r=shift_r>={1'b0,divisor_r},bit_b=shift_b>={1'b0,divisor_b};
    wire [SB:0] rem_r=bit_r ? shift_r-{1'b0,divisor_r} : shift_r;
    wire [SB:0] rem_b=bit_b ? shift_b-{1'b0,divisor_b} : shift_b;
    wire [NB-1:0] quo_r={quotient_r[NB-2:0],bit_r},quo_b={quotient_b[NB-2:0],bit_b};
    function [13:0] clamp_gain;
        input [NB-1:0] v;
        begin clamp_gain=v<MIN_GAIN ? MIN_GAIN : v>MAX_GAIN ? MAX_GAIN : v[13:0];end
    endfunction
    // Apply completed estimates at INPUT SOF, before multiplying its pixels.
    wire [13:0] pixel_gain_r=I_tuser && pending ? next_r : gain_r;
    wire [13:0] pixel_gain_b=I_tuser && pending ? next_b : gain_b;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            have_frame<=0;beats<=0;sum_r<=0;sum_g<=0;sum_b<=0;neutral_count<=0;
            busy<=0;pending<=0;remaining<=0;
            divisor_r<=0;divisor_b<=0;dividend_r<=0;dividend_b<=0;
            quotient_r<=0;quotient_b<=0;remainder_r<=0;remainder_b<=0;
            gain_r<=4096;gain_b<=4096;next_r<=4096;next_b<=4096;
        end else begin
            if(I_tuser && pending) begin gain_r<=next_r;gain_b<=next_b;pending<=0;end
            if(busy) begin
                dividend_r<=dividend_r<<1;dividend_b<=dividend_b<<1;
                quotient_r<=quo_r;quotient_b<=quo_b;
                remainder_r<=rem_r[SB-1:0];remainder_b<=rem_b[SB-1:0];
                remaining<=remaining-1'b1;
                if(remaining==1) begin
                    busy<=0;pending<=1;next_r<=clamp_gain(quo_r);next_b<=clamp_gain(quo_b);
                end
            end
            if(start_q) begin
                if(have_frame && beats==FRAME_BEATS && neutral_count>=MIN_NEUTRAL &&
                    sum_r!=0 && sum_b!=0 && !busy) begin
                    divisor_r<=sum_r;divisor_b<=sum_b;
                    dividend_r<={sum_g,12'b0};dividend_b<={sum_g,12'b0};
                    quotient_r<=0;quotient_b<=0;remainder_r<=0;remainder_b<=0;
                    remaining<=NB;busy<=1;
                end
                have_frame<=1;beats<=valid_q ? 1 : 0;
                sum_r<=valid_q ? r_q : 0;sum_g<=valid_q ? g_q : 0;sum_b<=valid_q ? b_q : 0;
                neutral_count<=valid_q ? n_q : 0;
            end else if(valid_q && have_frame) begin
                if(beats<FRAME_BEATS+1) beats<=beats+1'b1;
                if(beats<FRAME_BEATS) begin
                    sum_r<=sum_r+r_q;sum_g<=sum_g+g_q;sum_b<=sum_b+b_q;
                    neutral_count<=neutral_count+n_q;
                end
            end
        end
    end
    reg [95:0] pixels_1,pixels_2;
    reg [7:0] original_r_1[0:3], other_max_1[0:3];
    reg [1:0] valid_pipe,user_pipe,last_pipe;
    function [7:0] balance;
        input [7:0] channel;
        input [13:0] gain;
        reg [21:0] product;
        begin product=channel*gain;balance=|product[21:20] ? 8'd255 : product[19:12];end
    endfunction
    function [7:0] preserve_red;
        input [7:0] corrected,original,other_max;
        reg [7:0] loss,budget;
        reg [9:0] twice_red,three_other;
        begin
            preserve_red=corrected;
            twice_red={1'b0,original,1'b0};
            three_other={2'b0,other_max}+{1'b0,other_max,1'b0};
            // Continuous chroma-limited recovery: neutral pixels are untouched,
            // strong red recovers up to its original value, never beyond it.
            // A 16-level dead zone excludes weak tints and dark color noise.
            if(original>corrected && {1'b0,original}>{1'b0,other_max}+9'd16 &&
                twice_red>three_other) begin
                loss=original-corrected;
                budget=(twice_red-three_other)>>1;
                preserve_red=corrected+(loss<budget ? loss : budget);
            end
        end
    endfunction
    generate for(i=0;i<4;i=i+1) begin : output_lanes
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                pixels_1[i*24 +: 24]<=0;original_r_1[i]<=0;other_max_1[i]<=0;
                pixels_2[i*24 +:24]<=0;
            end else begin
                pixels_1[i*24 +: 24]<={balance(r[i],pixel_gain_r),g[i],balance(b[i],pixel_gain_b)};
                original_r_1[i]<=r[i];other_max_1[i]<=g[i]>b[i] ? g[i] : b[i];
                pixels_2[i*24 +:24]<={RED_CHROMA_PRESERVE ?
                    preserve_red(pixels_1[i*24+16 +:8],original_r_1[i],other_max_1[i]) :
                    pixels_1[i*24+16 +:8],pixels_1[i*24 +:16]};
            end
        end
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin valid_pipe<=0;user_pipe<=0;last_pipe<=0;end
        else begin
            valid_pipe<={valid_pipe[0],I_tvalid};user_pipe<={user_pipe[0],I_tuser};
            last_pipe<={last_pipe[0],I_tlast};
        end
    end
    generate if(FRAME_RESET_LEAD) begin : original_ddr_timing
        wire delayed_valid,unused_user;
        wire [95:0] delayed_pixels;
        signal_delay #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.DATA_WIDTH(96),.DELAY_CYCLE(50)) delay (
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_tuser(I_tuser),
            .I_valid(valid_pipe[1]),.I_data(pixels_2),.O_valid(delayed_valid),
            .O_data(delayed_pixels),.O_tuser(unused_user));
        reg v1,v2;
        reg [95:0] p1,p2;
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin v1<=0;v2<=0;p1<=0;p2<=0;end
            else begin v1<=delayed_valid;v2<=v1;p1<=delayed_pixels;p2<=p1;end
        end
        assign O_tvalid=v2;assign O_tuser=I_tuser;
        assign O_tlast=!v1 && v2;assign O_tdata=v2 ? p2 : 96'd0;
    end else begin : aligned_stream
        assign O_tvalid=valid_pipe[1];assign O_tuser=user_pipe[1];assign O_tlast=last_pipe[1];
        assign O_tdata=O_tvalid ? pixels_2 : 96'd0;
    end endgenerate
    assign I_tready=O_tready; // Original camera pipeline has no backpressure.
endmodule
`endif
