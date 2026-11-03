`ifndef VIDEO_CARTOON_REFERENCE_STREAM_V
`define VIDEO_CARTOON_REFERENCE_STREAM_V
`include "cartoon_centered_window.v"
`include "cartoon_reference_color.v"
`include "cartoon_original_tone.v"
`include "camera_display_color.v"
// Fixed-point streaming port of MATLAB reference style. The joint bilateral
// exponential is approximated by a joint RGB gate and Gaussian spatial weights.
// All six modes use the same centered raster, including replicated borders.
module video_cartoon_reference_stream #(
    parameter integer IMG_WIDTH=1280,IMG_HEIGHT=720,H_TOTAL=1650,
    parameter integer COLOR_LATENCY=9,
    parameter integer SYNC_DELAY=3*(H_TOTAL+1)+18,
    parameter integer RANGE_THRESHOLD=24,
    parameter integer EDGE_THRESHOLD=26,
    parameter integer SOBEL_WEAK_THRESHOLD=6,
    parameter integer SOBEL_STRONG_THRESHOLD=16,
    parameter integer LAST_MODE=5,
    parameter integer RUNTIME_CONTROL=0,
    // Opt-in to preserve existing users/tests. Mode 0 keeps input RGB levels
    // instead of the artistic tone curve; all other modes retain their path.
    parameter integer ORIGINAL_NATURAL=0,
    parameter integer ORIGINAL_IMPULSE_REPAIR=1,
    // Opt-in ORIGINAL/COLORTRACK quality path. Uses the existing window and
    // registers: flat-only cross denoise, edge passthrough and bounded gamma.
    parameter integer IMAGE_QUALITY_ENABLE=0,
    parameter integer CARTOON_FLAT_STYLE=0,
    parameter integer VIVID_COLOR_ENABLE=0, PHOTO_VIEW_ENABLE=0
)(
    input wire I_clk,I_rst_n,I_vsync,I_hsync,I_de,I_user,I_last,
    input wire [23:0] I_rgb,
    input wire I_photo_view,
    input wire I_mode_next,I_mode_original,I_mode_set,
    input wire [2:0] I_mode_value,
    input wire I_config_set,I_binary_auto,
    input wire [7:0] I_binary_threshold,I_sobel_weak,I_sobel_strong,
    input wire [7:0] I_cartoon_threshold,I_range_threshold,I_motion_threshold,
    output wire O_vsync,O_hsync,
    output reg O_de,O_user,O_last,
    output reg [23:0] O_rgb,
    output reg [2:0] O_mode,
    output reg O_binary_auto,
    output reg [7:0] O_binary_threshold,O_binary_effective,O_sobel_weak,O_sobel_strong,
    output reg [7:0] O_cartoon_threshold,O_range_threshold,O_motion_threshold
);
    // One complete command is staged, then committed at incoming SOF. Each
    // centered window carries a matching frame configuration to its output.
    // The final copy is published on O_user, so OSD/ACK describe displayed RGB.
    localparam [48:0] DEFAULT_CONFIG={1'b1,8'd96,SOBEL_WEAK_THRESHOLD[7:0],
        SOBEL_STRONG_THRESHOLD[7:0],EDGE_THRESHOLD[7:0],RANGE_THRESHOLD[7:0],8'd24};
    wire [48:0] command_config={I_binary_auto,I_binary_threshold,I_sobel_weak,
        I_sobel_strong,I_cartoon_threshold,I_range_threshold,I_motion_threshold};
    reg [48:0] requested_config,input_config;
    reg config_pending;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            requested_config<=DEFAULT_CONFIG;input_config<=DEFAULT_CONFIG;config_pending<=0;
        end else if(RUNTIME_CONTROL) begin
            if(I_config_set) begin requested_config<=command_config;config_pending<=1;end
            if(I_user && I_de) begin
                if(I_config_set) input_config<=command_config;
                else if(config_pending) input_config<=requested_config;
                config_pending<=0;
            end
        end
    end
    reg [48:0] config_w1,config_w2,config_w3,config_final;
    reg [2:0] requested,active;
    reg pending;
    wire [2:0] next_mode=requested==LAST_MODE ? 3'd0 : requested+1'b1;
    // Commit local and network commands only at a frame boundary.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin requested<=0;active<=0;pending<=0;end
        else begin
            if(I_mode_original) begin requested<=0;pending<=1;end
            else if(I_mode_next) begin requested<=next_mode;pending<=1;end
            else if(I_mode_set && I_mode_value<=LAST_MODE) begin requested<=I_mode_value;pending<=1;end
            if(I_user && I_de) begin
                if(I_mode_original) active<=0;
                else if(I_mode_next) active<=next_mode;
                else if(I_mode_set && I_mode_value<=LAST_MODE) active<=I_mode_value;
                else if(pending) active<=requested;
                pending<=0;
            end
        end
    end
    // Frame latency is carried in the RGB stream; hold its mode until output SOF.
    reg [2:0] frame_mode;
    reg input_photo_view;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) input_photo_view<=0;
        else if(I_user && I_de) input_photo_view<=PHOTO_VIEW_ENABLE && I_photo_view;
    end
    wire [2:0] display_active=input_photo_view ? 3'd0:active;
    wire w1_de,w1_user,w1_last;
    wire [48:0] denoise_config=w1_user ? input_config : config_w1;
    wire [31:0] range_threshold=RUNTIME_CONTROL ? {24'd0,denoise_config[15:8]} : RANGE_THRESHOLD;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) frame_mode<=0;
        else if(w1_user) frame_mode<=display_active;
    end
    wire [23:0] a1,b1,c1,d1,e1,f1,g1,h1,i1;
    cartoon_centered_window #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
        .H_TOTAL(H_TOTAL),.DATA_W(24),.TOP_W(24)) denoise_window(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(I_de),.I_user(I_user),.I_last(I_last),.I_data(I_rgb),
        .O_de(w1_de),.O_user(w1_user),.O_last(w1_last),
        .O_a(a1),.O_b(b1),.O_c(c1),.O_d(d1),.O_e(e1),.O_f(f1),.O_g(g1),.O_h(h1),.O_i(i1));
    function similar;
        input [23:0] p,q;
        input [31:0] threshold;
        reg [7:0] dr,dg,db;
        begin
            dr=p[23:16]>=q[23:16]?p[23:16]-q[23:16]:q[23:16]-p[23:16];
            dg=p[15:8]>=q[15:8]?p[15:8]-q[15:8]:q[15:8]-p[15:8];
            db=p[7:0]>=q[7:0]?p[7:0]-q[7:0]:q[7:0]-p[7:0];
            similar=dr<=threshold && dg<=threshold && db<=threshold;
        end
    endfunction
    reg [23:0] pa,pb,pc,pd,pe,pf,pg,ph,pi;
    reg [2:0] ctrl1,ctrl2;
    reg [23:0] smooth;
    // Sobel needs lower contrast sensitivity. Use Gaussian smoothing for this
    // mode; RGB range gating can retain independent channel noise as detail.
    // Observe active at the first denoised pixel, before frame_mode's NBA.
    wire sobel_filter=(w1_user ? display_active : frame_mode)==3;
    wire natural_filter=ORIGINAL_NATURAL && (w1_user ? display_active : frame_mode)==0;
    wire quality_filter=IMAGE_QUALITY_ENABLE &&
        ((w1_user ? display_active : frame_mode)==0 || (w1_user ? display_active : frame_mode)==5 ||
         (VIVID_COLOR_ENABLE && (w1_user ? display_active : frame_mode)==6));
    // Limit smoothing to a small RGB neighbourhood even when the existing
    // range control is raised for the artistic modes. Zero disables smoothing.
    wire [31:0] quality_limit=VIVID_COLOR_ENABLE ? 32'd24:32'd16;
    wire [31:0] quality_range=range_threshold<quality_limit ? range_threshold:quality_limit;
    // Opposite neighbours that agree with the centre support a connected
    // straight line even when its contrast is below the smoothing threshold.
    // Both cross-line neighbours must disagree, rejecting isolated noise.
    wire quality_line=(similar(b1,e1,32'd2) && similar(h1,e1,32'd2) &&
        !similar(d1,e1,32'd4) && !similar(f1,e1,32'd4)) ||
        (similar(d1,e1,32'd2) && similar(f1,e1,32'd2) &&
        !similar(b1,e1,32'd4) && !similar(h1,e1,32'd4)) ||
        (similar(a1,e1,32'd2) && similar(i1,e1,32'd2) &&
        !similar(c1,e1,32'd4) && !similar(g1,e1,32'd4)) ||
        (similar(c1,e1,32'd2) && similar(g1,e1,32'd2) &&
        !similar(a1,e1,32'd4) && !similar(i1,e1,32'd4));
    wire quality_flat=similar(a1,e1,quality_range) && similar(b1,e1,quality_range) &&
        similar(c1,e1,quality_range) && similar(d1,e1,quality_range) &&
        similar(f1,e1,quality_range) && similar(g1,e1,quality_range) &&
        similar(h1,e1,quality_range) && similar(i1,e1,quality_range) && !quality_line;
    // Repair only a high-contrast isolated centre surrounded by a consistent
    // ring. All eight neighbours veto repair when they support the centre,
    // protecting horizontal, vertical and diagonal one-pixel connected lines.
    // This deliberately leaves clustered impulses alone. A real isolated dot
    // is indistinguishable from an impulse; disable the parameter if needed.
    wire isolated_center= !similar(a1,e1,32'd64) && !similar(b1,e1,32'd64) &&
        !similar(c1,e1,32'd64) && !similar(d1,e1,32'd64) &&
        !similar(f1,e1,32'd64) && !similar(g1,e1,32'd64) &&
        !similar(h1,e1,32'd64) && !similar(i1,e1,32'd64);
    wire consistent_ring=similar(a1,b1,32'd32) && similar(c1,b1,32'd32) &&
        similar(d1,b1,32'd32) && similar(f1,b1,32'd32) &&
        similar(g1,b1,32'd32) && similar(h1,b1,32'd32) && similar(i1,b1,32'd32);
    wire cartoon_filter=CARTOON_FLAT_STYLE && (w1_user ? display_active : frame_mode)==4;
    wire repair_center=(natural_filter || quality_filter || cartoon_filter) && ORIGINAL_IMPULSE_REPAIR && isolated_center && consistent_ring;
    function [7:0] average_cross;
        input [7:0] b,d,f,h;
        reg [9:0] sum;
        begin
            sum={2'd0,b}+{2'd0,d}+{2'd0,f}+{2'd0,h}+10'd2;
            average_cross=sum[9:2];
        end
    endfunction
    wire [23:0] repaired_rgb={average_cross(b1[23:16],d1[23:16],f1[23:16],h1[23:16]),
        average_cross(b1[15:8],d1[15:8],f1[15:8],h1[15:8]),
        average_cross(b1[7:0],d1[7:0],f1[7:0],h1[7:0])};
    function [7:0] gaussian;
        input [7:0] a,b,c,d,e,f,g,h,i;
        reg [11:0] sum;
        begin
            sum={4'd0,a}+{3'd0,b,1'b0}+{4'd0,c}+{3'd0,d,1'b0}
                +{2'd0,e,2'b0}+{3'd0,f,1'b0}+{4'd0,g}+{3'd0,h,1'b0}+{4'd0,i}+12'd8;
            gaussian=sum[11:4];
        end
    endfunction
    function [7:0] luma;
        input [23:0] p;
        reg [15:0] sum;
        begin
            // Rec.601 coefficients in Q8. Maximum65535 fits16bits.
            sum=p[23:16]*8'd77+p[15:8]*8'd150+p[7:0]*8'd29+16'd128;
            luma=sum[15:8];
        end
    endfunction
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin pa<=0;pb<=0;pc<=0;pd<=0;pe<=0;pf<=0;pg<=0;ph<=0;pi<=0;
            ctrl1<=0;ctrl2<=0;smooth<=0;end
        else begin
            if(w1_de) begin
            if(repair_center) begin
                // Load all taps with the estimate; the existing Gaussian
                // register then passes it exactly, without extra latency/RAM.
                pa<=repaired_rgb;pb<=repaired_rgb;pc<=repaired_rgb;
                pd<=repaired_rgb;pe<=repaired_rgb;pf<=repaired_rgb;
                pg<=repaired_rgb;ph<=repaired_rgb;pi<=repaired_rgb;
            end else if(quality_filter) begin
                // Reuse the Gaussian adder register, but put the centre in
                // all four corners: weights become centre=1/2, B/D/F/H=1/8.
                // Non-flat neighbourhoods and supported straight one-pixel
                // lines bypass smoothing and retain their centre RGB.
                pa<=VIVID_COLOR_ENABLE && quality_flat ? a1:e1;
                pc<=VIVID_COLOR_ENABLE && quality_flat ? c1:e1;pe<=e1;
                pg<=VIVID_COLOR_ENABLE && quality_flat ? g1:e1;
                pi<=VIVID_COLOR_ENABLE && quality_flat ? i1:e1;
                pb<=quality_flat?b1:e1;pd<=quality_flat?d1:e1;
                pf<=quality_flat?f1:e1;ph<=quality_flat?h1:e1;
            end else begin
            pa<=sobel_filter||similar(a1,e1,range_threshold)?a1:e1;pb<=sobel_filter||similar(b1,e1,range_threshold)?b1:e1;pc<=sobel_filter||similar(c1,e1,range_threshold)?c1:e1;
            pd<=sobel_filter||similar(d1,e1,range_threshold)?d1:e1;pe<=e1;pf<=sobel_filter||similar(f1,e1,range_threshold)?f1:e1;
            pg<=sobel_filter||similar(g1,e1,range_threshold)?g1:e1;ph<=sobel_filter||similar(h1,e1,range_threshold)?h1:e1;pi<=sobel_filter||similar(i1,e1,range_threshold)?i1:e1;
            end
            end
            ctrl1<={w1_de,w1_user,w1_last};ctrl2<=ctrl1;
            if(ctrl1[2])
            smooth<={gaussian(pa[23:16],pb[23:16],pc[23:16],pd[23:16],pe[23:16],pf[23:16],pg[23:16],ph[23:16],pi[23:16]),
                     gaussian(pa[15:8],pb[15:8],pc[15:8],pd[15:8],pe[15:8],pf[15:8],pg[15:8],ph[15:8],pi[15:8]),
                     gaussian(pa[7:0],pb[7:0],pc[7:0],pd[7:0],pe[7:0],pf[7:0],pg[7:0],ph[7:0],pi[7:0])};
        end
    end
    // Nine-bit luma occupies a single2048x9 RAM; RGB only exists in row one.
    wire w2_de,w2_user,w2_last;
    wire [8:0] a2,b2,c2;
    wire [32:0] d2,e2,f2,g2,h2,i2;
    cartoon_centered_window #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
        .H_TOTAL(H_TOTAL),.DATA_W(33),.TOP_W(9)) sobel_window(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(ctrl2[2]),.I_user(ctrl2[1]),.I_last(ctrl2[0]),
        .I_data({smooth,1'b0,luma(smooth)}),.O_de(w2_de),.O_user(w2_user),.O_last(w2_last),
        .O_a(a2),.O_b(b2),.O_c(c2),.O_d(d2),.O_e(e2),.O_f(f2),.O_g(g2),.O_h(h2),.O_i(i2));
    reg signed [10:0] gx,gy;
    reg [7:0] center_y,blur_y;
    reg [23:0] center_rgb;
    reg [2:0] ctrl3,ctrl4;
    wire [10:0] ax=gx<0 ? -gx : gx;
    wire [10:0] ay=gy<0 ? -gy : gy;
    wire [10:0] major=ax>=ay?ax:ay,minor=ax>=ay?ay:ax;
    wire [11:0] mag_full={1'b0,major}+({1'b0,minor}>>2)+({1'b0,minor}>>3)+12'd2;
    wire [1:0] direction=({4'd0,ay}<<4)<=(({4'd0,ax}<<2)+({4'd0,ax}<<1)+ax) ? 2'd0 :
        (({4'd0,ax}<<4)<(({4'd0,ay}<<2)+({4'd0,ay}<<1)+ay) ? 2'd1 :
        (gx[10]==gy[10]?2'd2:2'd3));
    wire signed [8:0] delta=$signed({1'b0,center_y})-$signed({1'b0,blur_y});
    reg signed [8:0] detail;
    reg [8:0] magnitude;
    reg [1:0] dir;
    reg polarity;
    reg [23:0] rgb4;
    function [7:0] gaussian_y;
        input [8:0] a,b,c,d,e,f,g,h,i;
        reg [12:0] sum;
        begin
            sum=a+(b<<1)+c+(d<<1)+(e<<2)+(f<<1)+g+(h<<1)+i+13'd8;
            gaussian_y=sum[11:4];
        end
    endfunction
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin gx<=0;gy<=0;center_y<=0;blur_y<=0;center_rgb<=0;
            ctrl3<=0;ctrl4<=0;detail<=0;magnitude<=0;dir<=0;polarity<=0;rgb4<=0;end
        else begin
            if(w2_de) begin
            gx<=$signed({2'd0,c2})+($signed({2'd0,f2[8:0]})<<<1)+$signed({2'd0,i2[8:0]})
                -$signed({2'd0,a2})-($signed({2'd0,d2[8:0]})<<<1)-$signed({2'd0,g2[8:0]});
            gy<=$signed({2'd0,g2[8:0]})+($signed({2'd0,h2[8:0]})<<<1)+$signed({2'd0,i2[8:0]})
                -$signed({2'd0,a2})-($signed({2'd0,b2})<<<1)-$signed({2'd0,c2});
            center_y<=e2[7:0];blur_y<=gaussian_y(a2,b2,c2,d2[8:0],e2[8:0],f2[8:0],g2[8:0],h2[8:0],i2[8:0]);
            center_rgb<=e2[32:9];
            end
            ctrl3<={w2_de,w2_user,w2_last};ctrl4<=ctrl3;
            if(ctrl3[2]) begin
            magnitude<=mag_full[10:2];dir<=direction;
            polarity<=direction==1 ? gy[10] : gx[10];
            detail<=delta>15 ? 9'sd15 : (delta< -15 ? -9'sd15 : delta);
            rgb4<=center_rgb;
            end
        end
    end
    // Mean threshold, one completed denoised frame behind, serial division.
    localparam integer PIXELS=IMG_WIDTH*IMG_HEIGHT;
    // Sum and remainder follow their mathematical maxima, not32bit defaults.
    // Quotient is a byte because a frame mean is always0..255.
    localparam integer SUM_W=$clog2(PIXELS*255+1);
    localparam integer REM_W=$clog2(PIXELS+1);
    reg [SUM_W-1:0] gray_sum,dividend;
    reg [REM_W-1:0] remainder;
    reg [7:0] quotient;
    reg [5:0] divide_count;
    reg divide_busy;
    reg [7:0] mean_threshold,frame_threshold;
    reg [11:0] row_count;
    wire [REM_W:0] shifted={remainder,dividend[SUM_W-1]};
    wire bit_q=shifted>=PIXELS;
    wire [7:0] next_q={quotient[6:0],bit_q};
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin gray_sum<=0;dividend<=0;remainder<=0;quotient<=0;divide_count<=0;
            divide_busy<=0;mean_threshold<=96;frame_threshold<=96;row_count<=0;end
        else begin
            // Freeze the previous complete frame's mean for the whole current
            // output frame. The divider may finish while old rows are in flight.
            if(w2_de && w2_user)
                frame_threshold<=RUNTIME_CONTROL && !config_w1[48] ? config_w1[47:40] : mean_threshold;
            if(ctrl2[2]) begin
                if(ctrl2[1]) begin gray_sum<=luma(smooth);row_count<=0;end
                else gray_sum<=gray_sum+luma(smooth);
                if(ctrl2[0]) begin
                    row_count<=row_count+1'b1;
                    if(row_count==IMG_HEIGHT-1) begin
                        dividend<=gray_sum+luma(smooth);remainder<=0;quotient<=0;
                        divide_count<=0;divide_busy<=1;
                    end
                end
            end
            if(divide_busy) begin
                dividend<={dividend[SUM_W-2:0],1'b0};remainder<=bit_q?shifted-PIXELS:shifted;
                quotient<=next_q;divide_count<=divide_count+1'b1;
                if(divide_count==SUM_W-1) begin mean_threshold<=next_q;divide_busy<=0;end
            end
        end
    end
    wire [23:0] colored,original_tone,quality_tone;
    cartoon_reference_color #(.FLAT_STYLE(CARTOON_FLAT_STYLE)) color_core(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_rgb(rgb4),
        .I_detail(detail),.I_edge_strength(8'd0),.O_rgb(colored));
    cartoon_original_tone original_tone_core(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_rgb(rgb4),.O_rgb(original_tone));
    generate if(IMAGE_QUALITY_ENABLE) begin: quality_gamma
        if(VIVID_COLOR_ENABLE) begin: vivid
        camera_display_color quality_tone_core(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_rgb(rgb4),.O_rgb(quality_tone));
        end else begin: legacy
        cartoon_original_tone #(.QUALITY_GAMMA(1)) quality_tone_core(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_rgb(rgb4),.O_rgb(quality_tone));
        end
    end else begin: no_quality_gamma
        assign quality_tone=24'd0;
    end endgenerate
    reg [23:0] base_delay[0:COLOR_LATENCY-1];
    localparam integer QUALITY_LATENCY=VIVID_COLOR_ENABLE ? 4:2;
    reg [23:0] quality_delay[0:COLOR_LATENCY-QUALITY_LATENCY-1];
    reg [23:0] tone_delay[0:COLOR_LATENCY-3];
    reg [14:0] aux_delay[0:COLOR_LATENCY-1];
    integer k;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for(k=0;k<COLOR_LATENCY;k=k+1) begin base_delay[k]<=0;aux_delay[k]<=0;end
            for(k=0;k<COLOR_LATENCY-2;k=k+1) tone_delay[k]<=0;
            for(k=0;k<COLOR_LATENCY-QUALITY_LATENCY;k=k+1) quality_delay[k]<=0;
        end else begin
            base_delay[0]<=frame_mode==1 ? {luma(rgb4),luma(rgb4),luma(rgb4)} :
                (frame_mode==2 ? (luma(rgb4)>frame_threshold ? 24'hffffff:24'h000000) : rgb4);
            aux_delay[0]<={ctrl4,polarity,dir,magnitude};
            tone_delay[0]<=original_tone;
            quality_delay[0]<=quality_tone;
            for(k=1;k<COLOR_LATENCY-QUALITY_LATENCY;k=k+1) quality_delay[k]<=quality_delay[k-1];
            for(k=1;k<COLOR_LATENCY;k=k+1) begin base_delay[k]<=base_delay[k-1];aux_delay[k]<=aux_delay[k-1];end
            for(k=1;k<COLOR_LATENCY-2;k=k+1) tone_delay[k]<=tone_delay[k-1];
        end
    end
    wire [23:0] selected=frame_mode==4 ? colored :
        ((frame_mode==0 && (!ORIGINAL_NATURAL || IMAGE_QUALITY_ENABLE)) || frame_mode>=5 ?
            (IMAGE_QUALITY_ENABLE && (frame_mode==0 || frame_mode==5 ||
             (VIVID_COLOR_ENABLE && frame_mode==6)) ?
             quality_delay[COLOR_LATENCY-QUALITY_LATENCY-1]:tone_delay[COLOR_LATENCY-3])
             : base_delay[COLOR_LATENCY-1]);
    wire w3_de,w3_user,w3_last;
    wire [48:0] edge_config=w3_user ? config_w2 : config_w3;
    wire [31:0] weak_threshold=RUNTIME_CONTROL ? {24'd0,edge_config[39:32]} : SOBEL_WEAK_THRESHOLD;
    wire [31:0] strong_threshold=RUNTIME_CONTROL ? {24'd0,edge_config[31:24]} : SOBEL_STRONG_THRESHOLD;
    wire [31:0] cartoon_threshold=RUNTIME_CONTROL ? {24'd0,edge_config[23:16]} : EDGE_THRESHOLD;
    reg [7:0] effective_final;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            config_w1<=DEFAULT_CONFIG;config_w2<=DEFAULT_CONFIG;
            config_w3<=DEFAULT_CONFIG;config_final<=DEFAULT_CONFIG;effective_final<=96;
        end else begin
            if(w1_user && w1_de) config_w1<=input_config;
            if(w2_user && w2_de) config_w2<=config_w1;
            if(w3_user && w3_de) begin
                config_w3<=config_w2;config_final<=config_w2;effective_final<=frame_threshold;
            end
        end
    end
    wire [11:0] a3,b3,c3;
    wire [35:0] d3,e3,f3,g3,h3,i3;
    cartoon_centered_window #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
        .H_TOTAL(H_TOTAL),.DATA_W(36),.TOP_W(12)) nms_window(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(aux_delay[COLOR_LATENCY-1][14]),
        .I_user(aux_delay[COLOR_LATENCY-1][13]),.I_last(aux_delay[COLOR_LATENCY-1][12]),
        .I_data({selected,aux_delay[COLOR_LATENCY-1][11:0]}),
        .O_de(w3_de),.O_user(w3_user),.O_last(w3_last),.O_a(a3),.O_b(b3),.O_c(c3),
        .O_d(d3),.O_e(e3),.O_f(f3),.O_g(g3),.O_h(h3),.O_i(i3));
    reg keep;
    always @(*) begin
        case(e3[10:9])
            0:keep=e3[8:0]>=d3[8:0] && e3[8:0]>f3[8:0];
            1:keep=e3[8:0]>=b3[8:0] && e3[8:0]>h3[8:0];
            2:keep=e3[8:0]>=a3[8:0] && e3[8:0]>i3[8:0];
            default:keep=e3[8:0]>=c3[8:0] && e3[8:0]>g3[8:0];
        endcase
    end
    // Recover weak contours only when both tangent neighbours have adequate
    // strength and the same quantized normal direction and contrast polarity.
    // This is local support, not whole-frame Canny hysteresis or recognition.
    function sobel_support;
        input [11:0] neighbour,center;
        input [31:0] threshold;
        begin
            sobel_support=neighbour[8:0]>=threshold &&
                          neighbour[11:9]==center[11:9];
        end
    endfunction
    reg sobel_continuous;
    always @(*) begin
        case(e3[10:9])
            0:sobel_continuous=sobel_support(b3,e3[11:0],weak_threshold) && sobel_support(h3[11:0],e3[11:0],weak_threshold);
            1:sobel_continuous=sobel_support(d3[11:0],e3[11:0],weak_threshold) && sobel_support(f3[11:0],e3[11:0],weak_threshold);
            2:sobel_continuous=sobel_support(c3,e3[11:0],weak_threshold) && sobel_support(g3[11:0],e3[11:0],weak_threshold);
            default:sobel_continuous=sobel_support(a3,e3[11:0],weak_threshold) && sobel_support(i3[11:0],e3[11:0],weak_threshold);
        endcase
    end
    wire sobel_edge=e3[8:0]>=strong_threshold ||
        (e3[8:0]>=weak_threshold && keep && sobel_continuous);
    // Keep NMS-thin main contours. A half-threshold weak contour is admitted
    // only with matching direction/polarity on BOTH tangent neighbours.
    // This reduces disconnected speckles without a frame buffer or flood fill.
    wire [31:0] cartoon_weak=(cartoon_threshold+1)>>1;
    reg cartoon_continuous,cartoon_supported;
    always @(*) begin
        case(e3[10:9])
            0:begin
                cartoon_continuous=sobel_support(b3,e3[11:0],cartoon_weak) && sobel_support(h3[11:0],e3[11:0],cartoon_weak);
                cartoon_supported=sobel_support(b3,e3[11:0],cartoon_weak) || sobel_support(h3[11:0],e3[11:0],cartoon_weak);
            end
            1:begin
                cartoon_continuous=sobel_support(d3[11:0],e3[11:0],cartoon_weak) && sobel_support(f3[11:0],e3[11:0],cartoon_weak);
                cartoon_supported=sobel_support(d3[11:0],e3[11:0],cartoon_weak) || sobel_support(f3[11:0],e3[11:0],cartoon_weak);
            end
            2:begin
                cartoon_continuous=sobel_support(c3,e3[11:0],cartoon_weak) && sobel_support(g3[11:0],e3[11:0],cartoon_weak);
                cartoon_supported=sobel_support(c3,e3[11:0],cartoon_weak) || sobel_support(g3[11:0],e3[11:0],cartoon_weak);
            end
            default:begin
                cartoon_continuous=sobel_support(a3,e3[11:0],cartoon_weak) && sobel_support(i3[11:0],e3[11:0],cartoon_weak);
                cartoon_supported=sobel_support(a3,e3[11:0],cartoon_weak) || sobel_support(i3[11:0],e3[11:0],cartoon_weak);
            end
        endcase
    end
    // A strong contour needs at least one aligned tangent neighbour; a weak
    // contour needs both. Half/quarter ink avoids crushed black silhouettes.
    wire [7:0] flat_opacity=!keep ? 8'd0 :
        (e3[8:0]>=cartoon_threshold && cartoon_supported ? 8'd128 :
        (e3[8:0]>=cartoon_weak && cartoon_continuous ? 8'd64 : 8'd0));
    wire [15:0] opacity_product=e3[8:0]*8'd77;
    wire [15:0] opacity_linear=(opacity_product+16'd26)/16'd52;
    wire [7:0] opacity=keep && e3[8:0]>=cartoon_threshold ?
        (e3[8:0]>=2*cartoon_threshold || opacity_linear>=77 ? 8'd77 : opacity_linear[7:0]) : 8'd0;
    // Runtime cutoff and twice-cutoff saturation retain the original fixed
    // /52 slope. Clamp to77 even for raised cutoffs; no variable divider.
    // Divide by52 is constant. Replaced by reciprocalmultiply/shift in synthesis.
    reg [23:0] final_color;
    reg [7:0] factor;
    reg [2:0] final_ctrl;
    reg [2:0] final_mode;
    function [7:0] soften;
        input [7:0] channel,amount;
        reg [16:0] product;
        reg [9:0] scaled;
        begin
            if(CARTOON_FLAT_STYLE) begin
                // Flat ink has only0,1/4,1/2 opacity: replace three variable
                // colour multipliers with bounded add/shift arithmetic.
                case(amount)
                    8'd128:begin scaled={2'b00,channel}+10'd1;soften=scaled[8:1];end
                    8'd64:begin scaled={2'b00,channel}+{1'b0,channel,1'b0}+10'd2;soften=scaled[9:2];end
                    default:soften=channel;
                endcase
            end else begin
                product=channel*(9'd256-{1'b0,amount})+17'd128;soften=product[15:8];
            end
        end
    endfunction
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin final_color<=0;factor<=0;final_ctrl<=0;final_mode<=0;
            O_rgb<=0;O_de<=0;O_user<=0;O_last<=0;O_mode<=0;
            {O_binary_auto,O_binary_threshold,O_sobel_weak,O_sobel_strong,
             O_cartoon_threshold,O_range_threshold,O_motion_threshold}<=DEFAULT_CONFIG;
            O_binary_effective<=96;end
        else begin
            final_color<=frame_mode==3 ? (sobel_edge ? 24'hffffff:24'h000000) : e3[35:12];
            factor<=frame_mode==4 ? (CARTOON_FLAT_STYLE ? flat_opacity:opacity) : 8'd0;
            final_ctrl<={w3_de,w3_user,w3_last};final_mode<=frame_mode;
            O_rgb<={soften(final_color[23:16],factor),soften(final_color[15:8],factor),soften(final_color[7:0],factor)};
            {O_de,O_user,O_last}<=final_ctrl;
            if(final_ctrl[1] && final_ctrl[2]) begin
                O_mode<=final_mode;
                {O_binary_auto,O_binary_threshold,O_sobel_weak,O_sobel_strong,
                 O_cartoon_threshold,O_range_threshold,O_motion_threshold}<=config_final;
                O_binary_effective<=effective_final;
            end
        end
    end
    // Two-bit full-raster delay fits a single ERAM, avoiding long LUT shifts.
    reg [1:0] sync_ram[0:SYNC_DELAY-2];
    reg [12:0] sync_ptr;
    reg [12:0] sync_filled;
    reg [1:0] sync_q;
    always @(posedge I_clk) begin
        sync_ram[sync_ptr]<={I_vsync,I_hsync};sync_q<=sync_ram[sync_ptr];
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin sync_ptr<=0;sync_filled<=0;end
        else begin
            sync_ptr<=sync_ptr==SYNC_DELAY-2?13'd0:sync_ptr+1'b1;
            if(sync_filled<SYNC_DELAY) sync_filled<=sync_filled+1'b1;
        end
    end
    assign {O_vsync,O_hsync}=sync_filled>=SYNC_DELAY ? sync_q : 2'b00;
endmodule
`endif
