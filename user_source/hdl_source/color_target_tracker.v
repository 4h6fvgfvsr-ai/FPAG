`ifndef COLOR_TARGET_TRACKER_V
`define COLOR_TARGET_TRACKER_V
// Streaming RGB888 color localization, independently written for this pipeline.
// Reference: JdeRobot/FPGA-robotics color_proc.v (color threshold + frame statistics).
// ROI search/track/reacquire concept:
// https://blog.csdn.net/jiejiemcu/article/details/80579893
// Unlike its coarse horizontal bins, this module computes exact 2-D centroids.
// Legacy statistics or external compact grid-blob statistics are selectable.
// No external SRAM/full RGB framebuffer. RGB path is combinational (no latency).
module color_target_tracker #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [7:0] COLOR_MIN = 8'd100,
    parameter [7:0] COLOR_MARGIN = 8'd40,
    parameter [7:0] GREEN_MIN = COLOR_MIN,
    parameter [7:0] GREEN_MARGIN = COLOR_MARGIN,
    parameter integer GREEN_HUE_NUM = 1,
    parameter integer GREEN_HUE_DEN = 3,
    parameter [7:0] RED_MIN = COLOR_MIN,
    parameter [7:0] RED_MARGIN = COLOR_MARGIN,
    parameter integer RED_HUE_NUM = 1,
    parameter integer RED_HUE_DEN = 3,
    parameter [7:0] BLUE_MIN = COLOR_MIN,
    parameter [7:0] BLUE_MARGIN = COLOR_MARGIN,
    parameter integer BLUE_HUE_NUM = 1,
    parameter integer BLUE_HUE_DEN = 3,
    parameter integer HSV_SAT_SHIFT = 2,
    parameter [7:0] STRONG_MIN = 8'd48,
    parameter [7:0] STRONG_MARGIN = 8'd16,
    parameter [7:0] GREEN_STRONG_MIN = STRONG_MIN,
    parameter [7:0] GREEN_STRONG_MARGIN = STRONG_MARGIN,
    parameter [7:0] BLUE_STRONG_MIN = STRONG_MIN,
    parameter [7:0] BLUE_STRONG_MARGIN = STRONG_MARGIN,
    parameter integer STRONG_SAT_SHIFT = 2,
    parameter integer GREEN_STRONG_SAT_SHIFT = STRONG_SAT_SHIFT,
    parameter integer BLOB_ENABLE = 0,
    parameter integer BLOB_ACQUIRE_INPUT_ENABLE = 0,
    parameter integer MIN_PIXELS = 256,
    parameter integer BOX_THICKNESS = 2,
    parameter integer CROSS_RADIUS = 8,
    // Legacy defaults remain selectable; the 103 top enables the new features.
    parameter integer HSV_ENABLE = 0,
    parameter integer TRACK_ROI = 0,
    parameter integer EDGE_ACQUIRE_MARGIN = 0,
    parameter integer ROI_MARGIN = 64,
    parameter integer MIN_H_RUN = 1,
    parameter integer MIN_AREA = 0,
    parameter integer MIN_FILL_RATIO = 0,
    parameter integer STAT_PIPELINE = 0,
    // Legacy defaults keep the previous single-frame behavior selectable.
    parameter integer STABLE_ENABLE = 0,
    parameter integer KEEP_MIN_PIXELS = MIN_PIXELS,
    parameter integer CONFIRM_FRAMES = 2,
    parameter integer HOLD_FRAMES = 3,
    parameter integer ASSOC_DISTANCE = 96,
    parameter integer COORD_DEADBAND = 2,
    parameter integer SMOOTH_LIMIT = 8,
    parameter integer SMOOTH_SHIFT = 2,
    parameter integer SUM_WIDTH = $clog2(IMG_WIDTH * IMG_HEIGHT + 1) + 12
)(
    input wire I_clk, I_rst_n,
    input wire I_frame_start, I_line_end, I_de,
    input wire [23:0] I_rgb,
    input wire [1:0] I_color, // 0: disabled, 1: red, 2: green, 3: blue
    // Shared grid detector result. Unused when BLOB_ENABLE=0.
    input wire I_blob_result, I_blob_match,
    input wire I_blob_acquire,
    input wire [11:0] I_blob_center_x, I_blob_center_y,
    input wire [11:0] I_blob_left, I_blob_right, I_blob_top, I_blob_bottom,
    input wire [SUM_WIDTH-1:0] I_blob_pixel_count,
    // Aligned classifier tags for the shared detector (before run filtering).
    output wire O_stat_start, O_stat_end, O_stat_de,
    output wire [11:0] O_stat_x, O_stat_y,
    output wire O_stat_hit, O_stat_strong_hit,
    output wire [23:0] O_rgb,
    output wire O_overlay_hit,
    output reg O_valid, O_update,
    output reg O_measured,
    output reg [1:0] O_color,
    output reg [11:0] O_center_x, O_center_y,
    output reg [11:0] O_left, O_right, O_top, O_bottom,
    output reg [SUM_WIDTH-1:0] O_pixel_count
);
    reg [11:0] x, y;
    reg have_frame, aborted;
    reg [1:0] frame_color;
    wire [11:0] px = I_frame_start ? 12'd0 : x;
    wire [11:0] py = I_frame_start ? 12'd0 : y;
    wire [1:0] active_color = I_frame_start ? I_color : frame_color;
    wire frame_ok = (have_frame || I_frame_start) &&
                    (I_frame_start || !aborted) && (I_color == active_color);
    wire red_hit = I_rgb[23:16] >= COLOR_MIN &&
                   {1'b0,I_rgb[23:16]} >= ({1'b0,I_rgb[15:8]} + {1'b0,COLOR_MARGIN}) &&
                   {1'b0,I_rgb[23:16]} >= ({1'b0,I_rgb[7:0]} + {1'b0,COLOR_MARGIN});
    wire green_hit = I_rgb[15:8] >= COLOR_MIN &&
                   {1'b0,I_rgb[15:8]} >= ({1'b0,I_rgb[23:16]} + {1'b0,COLOR_MARGIN}) &&
                   {1'b0,I_rgb[15:8]} >= ({1'b0,I_rgb[7:0]} + {1'b0,COLOR_MARGIN});
    wire blue_hit = I_rgb[7:0] >= COLOR_MIN &&
                   {1'b0,I_rgb[7:0]} >= ({1'b0,I_rgb[23:16]} + {1'b0,COLOR_MARGIN}) &&
                   {1'b0,I_rgb[7:0]} >= ({1'b0,I_rgb[15:8]} + {1'b0,COLOR_MARGIN});
    // Division-free configurable HSV sectors (legacy default +/-20 degrees).
    // Require per-color minimum value/chroma and configurable saturation, rather than a fixed
    // difference between the dominant channel and BOTH other channels.
    wire [7:0] rr=I_rgb[23:16], gg=I_rgb[15:8], bb=I_rgb[7:0];
    wire [7:0] max_rg = rr >= gg ? rr : gg;
    wire [7:0] min_rg = rr <= gg ? rr : gg;
    wire [7:0] value = max_rg >= bb ? max_rg : bb;
    wire [7:0] minimum = min_rg <= bb ? min_rg : bb;
    wire [8:0] chroma = {1'b0,value} - {1'b0,minimum};
    wire [8:0] gb_diff = gg >= bb ? {1'b0,gg}-{1'b0,bb} : {1'b0,bb}-{1'b0,gg};
    wire [8:0] rb_diff = rr >= bb ? {1'b0,rr}-{1'b0,bb} : {1'b0,bb}-{1'b0,rr};
    wire [8:0] rg_diff = rr >= gg ? {1'b0,rr}-{1'b0,gg} : {1'b0,gg}-{1'b0,rr};
    wire [7:0] selected_min = active_color==1 ? RED_MIN : active_color==2 ? GREEN_MIN : BLUE_MIN;
    wire [7:0] selected_margin = active_color==1 ? RED_MARGIN : active_color==2 ? GREEN_MARGIN : BLUE_MARGIN;
    // Widen BEFORE shifting: a 9-bit shift would wrap on saturated colors.
    wire [16:0] weak_chroma_scaled = {8'd0,chroma} << HSV_SAT_SHIFT;
    wire [16:0] strong_chroma_scaled = {8'd0,chroma} << STRONG_SAT_SHIFT;
    wire [16:0] green_strong_chroma_scaled = {8'd0,chroma} << GREEN_STRONG_SAT_SHIFT;
    wire [16:0] selected_strong_chroma_scaled = active_color==2 ? green_strong_chroma_scaled : strong_chroma_scaled;
    wire hsv_quality = value >= selected_min && chroma >= selected_margin &&
                       chroma != 0 && weak_chroma_scaled >= {9'd0,value};
    wire [7:0] selected_strong_min = active_color==2 ? GREEN_STRONG_MIN : active_color==3 ? BLUE_STRONG_MIN : STRONG_MIN;
    wire [7:0] selected_strong_margin = active_color==2 ? GREEN_STRONG_MARGIN : active_color==3 ? BLUE_STRONG_MARGIN : STRONG_MARGIN;
    wire strong_quality = value >= selected_strong_min && chroma >= selected_strong_margin &&
                          chroma != 0 && selected_strong_chroma_scaled >= {9'd0,value};
    wire hsv_hit = hsv_quality &&
        ((active_color==1 && rr==value && (gb_diff*RED_HUE_DEN <= chroma*RED_HUE_NUM)) ||
         (active_color==2 && gg==value && (rb_diff*GREEN_HUE_DEN <= chroma*GREEN_HUE_NUM)) ||
         (active_color==3 && bb==value && (rg_diff*BLUE_HUE_DEN <= chroma*BLUE_HUE_NUM)));
    wire rgb_hit = (active_color==1 && red_hit) ||
                   (active_color==2 && green_hit) || (active_color==3 && blue_hit);
    reg [11:0] roi_left, roi_right, roi_top, roi_bottom;
    wire [11:0] roi_next_left, roi_next_right, roi_next_top, roi_next_bottom;
    wire [11:0] search_l = I_frame_start ? roi_next_left : roi_left;
    wire [11:0] search_r = I_frame_start ? roi_next_right : roi_right;
    wire [11:0] search_t = I_frame_start ? roi_next_top : roi_top;
    wire [11:0] search_b = I_frame_start ? roi_next_bottom : roi_bottom;
    wire source_hit = frame_ok && I_de && px<IMG_WIDTH && py<IMG_HEIGHT &&
        (!TRACK_ROI || (px>=search_l && px<=search_r && py>=search_t && py<=search_b)) &&
        (HSV_ENABLE ? hsv_hit : rgb_hit);
    // Lens shading may leave a coherent peripheral target above the HSV
    // color floor but below the extra absolute strong-core floor. Adapt only
    // inside the configured edge band. The strong saturation requirement
    // still applies: a weak warm/gray strip must not become a color core just
    // because it touches an edge. Hue, connected shape and confirmation also
    // remain mandatory; no part of the image is masked.
    wire peripheral_pixel=EDGE_ACQUIRE_MARGIN>0 &&
        (px<EDGE_ACQUIRE_MARGIN || py<EDGE_ACQUIRE_MARGIN ||
         px>=IMG_WIDTH-EDGE_ACQUIRE_MARGIN || py>=IMG_HEIGHT-EDGE_ACQUIRE_MARGIN);
    wire strong_saturation_ok = chroma != 0 &&
        selected_strong_chroma_scaled >= {9'd0,value};
    wire source_strong_hit = source_hit && (strong_quality ||
        (BLOB_ENABLE && HSV_ENABLE && peripheral_pixel && strong_saturation_ok));

    // Register only the statistics branch. The displayed RGB and its timing
    // never acquire this latency. Tags include frame/line boundaries and color.
    wire st_de, st_start, st_end, st_hit, st_strong_hit, st_ok, st_have;
    wire [11:0] st_x, st_y;
    wire [1:0] st_color;
    generate if (STAT_PIPELINE) begin : statistic_tags
        reg de_q, start_q, end_q, hit_q, strong_q, ok_q, have_q;
        reg [11:0] x_q, y_q;
        reg [1:0] color_q;
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                de_q<=0; start_q<=0; end_q<=0; hit_q<=0; strong_q<=0; ok_q<=0; have_q<=0;
                x_q<=0; y_q<=0; color_q<=0;
            end else begin
                de_q<=I_de; start_q<=I_frame_start; end_q<=I_line_end;
                hit_q<=source_hit; strong_q<=source_strong_hit;
                ok_q<=frame_ok; have_q<=have_frame || I_frame_start;
                x_q<=px; y_q<=py; color_q<=active_color;
            end
        end
        assign st_de=de_q; assign st_start=start_q; assign st_end=end_q;
        assign st_hit=hit_q; assign st_strong_hit=strong_q;
        assign st_ok=ok_q; assign st_have=have_q;
        assign st_x=x_q; assign st_y=y_q; assign st_color=color_q;
    end else begin : legacy_statistics
        assign st_de=I_de; assign st_start=I_frame_start; assign st_end=I_line_end;
        assign st_hit=source_hit; assign st_ok=frame_ok; assign st_have=have_frame || I_frame_start;
        assign st_strong_hit=source_strong_hit;
        assign st_x=px; assign st_y=py; assign st_color=active_color;
    end endgenerate
    assign O_stat_start=st_start; assign O_stat_end=st_end; assign O_stat_de=st_de;
    assign O_stat_x=st_x; assign O_stat_y=st_y;
    assign O_stat_hit=st_hit; assign O_stat_strong_hit=st_strong_hit;
    localparam integer RUN_BITS = MIN_H_RUN>1 ? $clog2(MIN_H_RUN+1) : 1;
    reg [RUN_BITS-1:0] run_length;
    wire [RUN_BITS-1:0] base_run = st_start ? 0 : run_length;
    wire hit = st_hit && base_run >= MIN_H_RUN-1;
    wire burst = hit && base_run == MIN_H_RUN-1;
    // On reaching the minimum run length, include ALL pixels in that run.
    // No left-edge bias is introduced by dropping its first two pixels.
    wire [SUM_WIDTH-1:0] add_count = hit ? (burst ? MIN_H_RUN : 1) : 0;
    wire [SUM_WIDTH-1:0] add_x = !hit ? 0 : burst
        ? MIN_H_RUN*st_x - (MIN_H_RUN*(MIN_H_RUN-1)/2) : st_x;
    wire [SUM_WIDTH-1:0] add_y = !hit ? 0 : burst ? MIN_H_RUN*st_y : st_y;
    wire [11:0] hit_left = burst ? st_x-(MIN_H_RUN-1) : st_x;
    wire last_pixel = st_ok && st_de && st_x==IMG_WIDTH-1 && st_y==IMG_HEIGHT-1;
    reg [SUM_WIDTH-1:0] count, sum_x, sum_y;
    reg [11:0] left, right, top, bottom;
    wire [SUM_WIDTH-1:0] base_count = st_start ? 0 : count;
    wire [SUM_WIDTH-1:0] next_count = base_count + add_count;
    wire [SUM_WIDTH-1:0] next_sum_x = (st_start ? 0 : sum_x) + add_x;
    wire [SUM_WIDTH-1:0] next_sum_y = (st_start ? 0 : sum_y) + add_y;
    wire [11:0] base_left = st_start ? IMG_WIDTH-1 : left;
    wire [11:0] base_right = st_start ? 12'd0 : right;
    wire [11:0] base_top = st_start ? IMG_HEIGHT-1 : top;
    wire [11:0] base_bottom = st_start ? 12'd0 : bottom;
    wire [11:0] next_left = hit && hit_left < base_left ? hit_left : base_left;
    wire [11:0] next_right = hit && st_x > base_right ? st_x : base_right;
    wire [11:0] next_top = hit && st_y < base_top ? st_y : base_top;
    wire [11:0] next_bottom = hit && st_y > base_bottom ? st_y : base_bottom;

    // Two restoring dividers share their denominator and bit counter. They
    // finish in SUM_WIDTH clocks after the last active pixel, not per pixel.
    reg busy, ready, pending_valid,pending_acquire;
    reg result_sent;
    localparam integer DETECT_MIN_PIXELS = STABLE_ENABLE ? KEEP_MIN_PIXELS : MIN_PIXELS;
    reg [1:0] pending_color;
    reg [11:0] pending_left, pending_right, pending_top, pending_bottom;
    reg [11:0] pending_x, pending_y;
    reg [SUM_WIDTH-1:0] denominator, dividend_x, dividend_y;
    reg [SUM_WIDTH-1:0] quotient_x, quotient_y, remainder_x, remainder_y;
    reg [7:0] bits_left;
    reg [1:0] qualification;
    reg [25:0] pending_area;
    wire [12:0] pending_width = {1'b0,pending_right}-{1'b0,pending_left}+13'd1;
    wire [12:0] pending_height = {1'b0,pending_bottom}-{1'b0,pending_top}+13'd1;
    wire [SUM_WIDTH+7:0] filled_area_limit = denominator*MIN_FILL_RATIO;
    wire [SUM_WIDTH:0] shift_x = {remainder_x, dividend_x[SUM_WIDTH-1]};
    wire [SUM_WIDTH:0] shift_y = {remainder_y, dividend_y[SUM_WIDTH-1]};
    wire bit_x = shift_x >= {1'b0,denominator};
    wire bit_y = shift_y >= {1'b0,denominator};
    wire [SUM_WIDTH:0] rem_x_next = bit_x ? shift_x - {1'b0,denominator} : shift_x;
    wire [SUM_WIDTH:0] rem_y_next = bit_y ? shift_y - {1'b0,denominator} : shift_y;
    wire [SUM_WIDTH-1:0] quo_x_next = {quotient_x[SUM_WIDTH-2:0],bit_x};
    wire [SUM_WIDTH-1:0] quo_y_next = {quotient_y[SUM_WIDTH-2:0],bit_y};

    // Complete raw measurements feed the temporal filter ONCE in blanking.
    // Filtering never enters the per-pixel classifier or RGB overlay path.
    wire result_event = STABLE_ENABLE && ready && !result_sent && !busy && qualification==0;
    wire peripheral_result=EDGE_ACQUIRE_MARGIN>0 &&
        (pending_left<EDGE_ACQUIRE_MARGIN || pending_top<EDGE_ACQUIRE_MARGIN ||
         pending_right>=IMG_WIDTH-EDGE_ACQUIRE_MARGIN || pending_bottom>=IMG_HEIGHT-EDGE_ACQUIRE_MARGIN);
    wire stable_ready, stable_valid, stable_measured;
    wire [1:0] stable_color;
    wire [11:0] stable_x,stable_y,stable_left,stable_right,stable_top,stable_bottom;
    wire [SUM_WIDTH-1:0] stable_count;
    generate if(STABLE_ENABLE) begin : temporal_tracking
        color_track_temporal #(.SUM_WIDTH(SUM_WIDTH),.CONFIRM_FRAMES(CONFIRM_FRAMES),
            .HOLD_FRAMES(HOLD_FRAMES),.ASSOC_DISTANCE(ASSOC_DISTANCE),
            .COORD_DEADBAND(COORD_DEADBAND),.SMOOTH_LIMIT(SMOOTH_LIMIT),.SMOOTH_SHIFT(SMOOTH_SHIFT)) filter (
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_frame_start(I_frame_start),.I_color(I_color),
            .I_result(result_event),.I_result_color(pending_color),
            .I_match(pending_valid),.I_strong(pending_valid &&
                ((BLOB_ENABLE && BLOB_ACQUIRE_INPUT_ENABLE) ? pending_acquire :
                 denominator>=(peripheral_result ? KEEP_MIN_PIXELS:MIN_PIXELS))),
            .I_center_x(pending_x),.I_center_y(pending_y),.I_left(pending_left),.I_right(pending_right),
            .I_top(pending_top),.I_bottom(pending_bottom),.I_pixel_count(denominator),
            .O_ready(stable_ready),.O_valid(stable_valid),.O_measured(stable_measured),.O_color(stable_color),
            .O_center_x(stable_x),.O_center_y(stable_y),.O_left(stable_left),.O_right(stable_right),
            .O_top(stable_top),.O_bottom(stable_bottom),.O_pixel_count(stable_count));
    end else begin : legacy_publication
        assign stable_ready=ready;assign stable_valid=pending_valid;assign stable_color=pending_color;
        assign stable_measured=pending_valid;
        assign stable_x=pending_x;assign stable_y=pending_y;
        assign stable_left=pending_left;assign stable_right=pending_right;
        assign stable_top=pending_top;assign stable_bottom=pending_bottom;assign stable_count=denominator;
    end endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x <= 0; y <= 0; have_frame <= 0; aborted <= 0; frame_color <= 0;
            count <= 0; sum_x <= 0; sum_y <= 0;
            left <= IMG_WIDTH-1; right <= 0; top <= IMG_HEIGHT-1; bottom <= 0;
            run_length <= 0;
        end else begin
            if(I_frame_start) begin
                have_frame <= 1; aborted <= 0; frame_color <= I_color;
                x <= (I_de && !I_line_end) ? 12'd1 : 12'd0;
                y <= (I_de && I_line_end) ? 12'd1 : 12'd0;
            end else begin
                if(I_color != frame_color) aborted <= 1;
                if(I_de && have_frame) begin
                    if(I_line_end) begin x <= 0; y <= y + 1'b1; end
                    else x <= x + 1'b1;
                end
            end
            if(st_start || st_de) begin
                count <= next_count; sum_x <= next_sum_x; sum_y <= next_sum_y;
                left <= next_left; right <= next_right;
                top <= next_top; bottom <= next_bottom;
                if(!st_hit || st_end) run_length <= 0;
                else if(base_run < MIN_H_RUN) run_length <= base_run+1'b1;
                else run_length <= base_run;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            busy <= 0; ready <= 0; pending_valid <= 0; pending_acquire<=0; pending_color <= 0;
            result_sent <= 0;
            pending_left <= 0; pending_right <= 0; pending_top <= 0; pending_bottom <= 0;
            pending_x <= 0; pending_y <= 0;
            denominator <= 0; dividend_x <= 0; dividend_y <= 0;
            quotient_x <= 0; quotient_y <= 0; remainder_x <= 0; remainder_y <= 0;
            bits_left <= 0;
            qualification <= 0; pending_area <= 0;
            O_valid <= 0; O_update <= 0; O_measured <= 0; O_color <= 0;
            O_center_x <= 0; O_center_y <= 0;
            O_left <= 0; O_right <= 0; O_top <= 0; O_bottom <= 0; O_pixel_count <= 0;
        end else begin
            O_update <= 0;
            if(result_event) result_sent <= 1;
            // Shape checks run in blanking on REGISTERED frame statistics.
            // They do not lengthen the per-pixel color-to-accumulator path.
            if(!BLOB_ENABLE && qualification==1) begin
                pending_area <= pending_width*pending_height;
                qualification <= 2;
            end else if(!BLOB_ENABLE && qualification==2) begin
                pending_valid <= pending_valid && pending_area>=MIN_AREA &&
                    (MIN_FILL_RATIO==0 || filled_area_limit>=pending_area);
                qualification <= 0;
                if(!busy) ready <= 1;
            end
            // Never display stale coordinates for another color.
            if(I_color != O_color || I_color == 0) begin O_valid <= 0; O_measured <= 0; end
            // Publish ONLY at frame boundaries to avoid a tearing overlay.
            if(I_frame_start && ready) begin
                ready <= 0;
            end
            if(I_frame_start && stable_ready) begin
                O_update <= 1;
                O_valid <= stable_valid && stable_color == I_color && I_color != 0;
                // Held overlay boxes are never fresh actuator measurements.
                O_measured <= stable_measured && stable_color == I_color && I_color != 0;
                O_color <= stable_color;
                O_left <= stable_left; O_right <= stable_right;
                O_top <= stable_top; O_bottom <= stable_bottom;
                O_center_x <= stable_x; O_center_y <= stable_y;
                O_pixel_count <= stable_count;
            end
            if(!BLOB_ENABLE && busy) begin
                dividend_x <= dividend_x << 1; dividend_y <= dividend_y << 1;
                remainder_x <= rem_x_next[SUM_WIDTH-1:0];
                remainder_y <= rem_y_next[SUM_WIDTH-1:0];
                quotient_x <= quo_x_next; quotient_y <= quo_y_next;
                bits_left <= bits_left - 1'b1;
                if(bits_left == 1) begin
                    busy <= 0; ready <= 1;
                    pending_x <= quo_x_next[11:0]; pending_y <= quo_y_next[11:0];
                end
            end
            // Include the last pixel in count, sums and extrema.
            // Invalid/aborted frames publish an invalid result too.
            if(!BLOB_ENABLE && st_de && st_have && st_x == IMG_WIDTH-1 && st_y == IMG_HEIGHT-1) begin
                result_sent <= 0;
                pending_color <= st_color;
                pending_left <= next_left; pending_right <= next_right;
                pending_top <= next_top; pending_bottom <= next_bottom;
                denominator <= next_count;
                quotient_x <= 0; quotient_y <= 0; remainder_x <= 0; remainder_y <= 0;
                pending_valid <= last_pixel && st_color != 0 && next_count >= DETECT_MIN_PIXELS && next_count != 0;
                qualification <= (MIN_AREA!=0 || MIN_FILL_RATIO!=0) ? 1 : 0;
                if(last_pixel && st_color != 0 && next_count >= DETECT_MIN_PIXELS && next_count != 0) begin
                    dividend_x <= next_sum_x; dividend_y <= next_sum_y;
                    bits_left <= SUM_WIDTH; busy <= 1; ready <= 0;
                end else begin
                    pending_x <= 0; pending_y <= 0; busy <= 0;
                    ready <= (MIN_AREA==0 && MIN_FILL_RATIO==0);
                end
            end
            // A complete independent-region result is already shape-qualified.
            // Publish through the SAME frame-boundary/temporal path as legacy.
            if(BLOB_ENABLE && I_blob_result && I_color!=0) begin
                busy<=0; qualification<=0; ready<=1; result_sent<=0;
                pending_valid<=I_blob_match; pending_acquire<=I_blob_acquire; pending_color<=I_color;
                pending_x<=I_blob_center_x; pending_y<=I_blob_center_y;
                pending_left<=I_blob_left; pending_right<=I_blob_right;
                pending_top<=I_blob_top; pending_bottom<=I_blob_bottom;
                denominator<=I_blob_pixel_count;
            end
        end
    end

    // Bypass pending results on the first active pixel so that this pixel
    // sees the same box as the rest of the newly starting frame.
    wire use_pending = I_frame_start && stable_ready;
    wire draw_valid = I_rst_n && I_de && I_color != 0 && (use_pending
        ? stable_valid && stable_color == I_color : O_valid && O_color == I_color);
    wire [11:0] dl = use_pending ? stable_left : O_left;
    wire [11:0] dr = use_pending ? stable_right : O_right;
    wire [11:0] dt = use_pending ? stable_top : O_top;
    wire [11:0] db = use_pending ? stable_bottom : O_bottom;
    wire [11:0] cx = use_pending ? stable_x : O_center_x;
    wire [11:0] cy = use_pending ? stable_y : O_center_y;
    // Search follows the latest RAW box, never a held/smoothed display box.
    // First failed measurement therefore triggers full search immediately,
    // even while the previous display is allowed to coast for a few frames.
    reg raw_roi_valid;
    reg [1:0] raw_roi_color;
    reg [11:0] raw_roi_left,raw_roi_right,raw_roi_top,raw_roi_bottom;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            raw_roi_valid<=0;raw_roi_color<=0;
            raw_roi_left<=0;raw_roi_right<=0;raw_roi_top<=0;raw_roi_bottom<=0;
        end else if(I_frame_start) begin
            raw_roi_valid<=ready && pending_valid && pending_color==I_color && I_color!=0 &&
                (!STABLE_ENABLE || (stable_ready && stable_measured));
            raw_roi_color<=pending_color;
            raw_roi_left<=pending_left;raw_roi_right<=pending_right;
            raw_roi_top<=pending_top;raw_roi_bottom<=pending_bottom;
        end else if(I_color!=raw_roi_color || I_color==0) raw_roi_valid<=0;
    end
    wire raw_follow = I_frame_start ? ready && pending_valid && pending_color==I_color &&
                        stable_ready && stable_measured :
                      raw_roi_valid && raw_roi_color==I_color;
    wire follow_target = TRACK_ROI && I_color!=0 && (STABLE_ENABLE ? raw_follow :
        (use_pending ? pending_valid && pending_color==I_color : O_valid && O_color==I_color));
    wire [11:0] rl = STABLE_ENABLE ? (I_frame_start ? pending_left : raw_roi_left) : dl;
    wire [11:0] rr_box = STABLE_ENABLE ? (I_frame_start ? pending_right : raw_roi_right) : dr;
    wire [11:0] rt = STABLE_ENABLE ? (I_frame_start ? pending_top : raw_roi_top) : dt;
    wire [11:0] rb_box = STABLE_ENABLE ? (I_frame_start ? pending_bottom : raw_roi_bottom) : db;
    wire [12:0] expanded_r = {1'b0,rr_box}+ROI_MARGIN;
    wire [12:0] expanded_b = {1'b0,rb_box}+ROI_MARGIN;
    assign roi_next_left = !follow_target ? 0 : rl>ROI_MARGIN ? rl-ROI_MARGIN : 0;
    assign roi_next_top = !follow_target ? 0 : rt>ROI_MARGIN ? rt-ROI_MARGIN : 0;
    assign roi_next_right = !follow_target || expanded_r>=IMG_WIDTH ? IMG_WIDTH-1 : expanded_r[11:0];
    assign roi_next_bottom = !follow_target || expanded_b>=IMG_HEIGHT ? IMG_HEIGHT-1 : expanded_b[11:0];
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            roi_left<=0; roi_right<=IMG_WIDTH-1; roi_top<=0; roi_bottom<=IMG_HEIGHT-1;
        end else if(I_frame_start) begin
            roi_left<=roi_next_left; roi_right<=roi_next_right;
            roi_top<=roi_next_top; roi_bottom<=roi_next_bottom;
        end
    end
    wire inside_box = px >= dl && px <= dr && py >= dt && py <= db;
    wire box_hit = inside_box && ((px-dl) < BOX_THICKNESS || (dr-px) < BOX_THICKNESS ||
                                  (py-dt) < BOX_THICKNESS || (db-py) < BOX_THICKNESS);
    wire [11:0] dx = px >= cx ? px-cx : cx-px;
    wire [11:0] dy = py >= cy ? py-cy : cy-py;
    wire cross_hit = (dx <= CROSS_RADIUS && py == cy) || (dy <= CROSS_RADIUS && px == cx);
    wire [23:0] box_color = I_color == 1 ? 24'hff3030 :
                            I_color == 2 ? 24'h30ff30 : 24'h3080ff;
    assign O_rgb = !draw_valid ? I_rgb : cross_hit ? 24'hffff00 : box_hit ? box_color : I_rgb;
    assign O_overlay_hit = draw_valid && (cross_hit || box_hit);
endmodule
`endif
