`ifndef COLOR_RGB_TRACKER_V
`define COLOR_RGB_TRACKER_V
// Simultaneous red/green/blue localization on the SAME original RGB stream.
// Three independent sets of frame statistics; no manual color selection.
module color_rgb_tracker #(
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
    parameter integer GREEN_HSV_SAT_SHIFT = HSV_SAT_SHIFT,
    parameter integer BLUE_HSV_SAT_SHIFT = HSV_SAT_SHIFT,
    parameter [7:0] STRONG_MIN = 8'd48,
    parameter [7:0] STRONG_MARGIN = 8'd16,
    parameter [7:0] GREEN_STRONG_MIN = STRONG_MIN,
    parameter [7:0] GREEN_STRONG_MARGIN = STRONG_MARGIN,
    parameter [7:0] BLUE_STRONG_MIN = STRONG_MIN,
    parameter [7:0] BLUE_STRONG_MARGIN = STRONG_MARGIN,
    parameter integer STRONG_SAT_SHIFT = 2,
    parameter integer RED_STRONG_SAT_SHIFT = STRONG_SAT_SHIFT,
    parameter integer BLUE_STRONG_SAT_SHIFT = STRONG_SAT_SHIFT,
    parameter integer GREEN_STRONG_SAT_SHIFT = STRONG_SAT_SHIFT,
    parameter integer BLOB_ENABLE = 0,
    parameter integer BLOB_CELL_SHIFT = 5,
    parameter integer BLOB_CORE_MIN_PIXELS = 16,
    parameter integer BLOB_WEAK_CELL_MIN_PIXELS = 64,
    parameter integer BLOB_HALO_MIN_PIXELS = 4,
    parameter integer BLOB_MIN_STRONG_PIXELS = 0,
    parameter integer BLOB_STRONG_FILL_RATIO = 0,
    parameter integer BLOB_WEAK_FALLBACK_ENABLE = 1,
    parameter [2:0] BLOB_WEAK_FALLBACK_COLOR_MASK = 3'b111,
    parameter integer BLOB_WEAK_FALLBACK_MAX_AREA = 0,
    parameter integer BLOB_MAX_ASPECT_RATIO = 0,
    parameter integer BLOB_TIGHT_BOUNDS = 0,
    parameter integer BLOB_CORE_QUALITY_ENABLE = 0,
    parameter integer BLOB_MAX_PROCESS_CYCLES = 45000,
    parameter integer EDGE_ACQUIRE_MARGIN = 0,
    parameter integer MIN_PIXELS = 256,
    parameter integer BOX_THICKNESS = 2,
    parameter integer CROSS_RADIUS = 8,
    parameter integer HSV_ENABLE = 0,
    parameter integer TRACK_ROI = 0,
    parameter integer ROI_MARGIN = 64,
    parameter integer MIN_H_RUN = 1,
    parameter integer MIN_AREA = 0,
    parameter integer MIN_FILL_RATIO = 0,
    parameter integer STAT_PIPELINE = 0,
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
    input wire I_frame_start, I_line_end, I_de, I_enable,
    input wire [23:0] I_rgb,
    output wire [23:0] O_rgb,
    // Index 0 = red, 1 = green, 2 = blue. Packed coordinates use 12 bits each.
    output wire [2:0] O_valid, O_update,
    output wire [2:0] O_measured,
    output wire [35:0] O_center_x, O_center_y,
    output wire [35:0] O_left, O_right, O_top, O_bottom,
    // Unsmooth, fresh strong-core result for dimensional measurement.
    output wire O_measure_result,O_measure_match,
    output wire [1:0] O_measure_color,
    output wire [47:0] O_measure_bounds,
    output wire [3*SUM_WIDTH-1:0] O_pixel_count
);
    wire [23:0] marked_rgb [0:2];
    wire [2:0] overlay_hit;
    wire [2:0] stat_start,stat_end,stat_de,stat_hit,stat_strong;
    wire [11:0] stat_x[0:2],stat_y[0:2];
    wire blob_result,blob_match,blob_acquire;
    wire [1:0] blob_color;
    wire [11:0] blob_x,blob_y,blob_left,blob_right,blob_top,blob_bottom;
    wire [SUM_WIDTH-1:0] blob_count;
    generate if(BLOB_ENABLE) begin : shared_blob_detector
        color_blob_grid_rgb #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.SUM_WIDTH(SUM_WIDTH),
            .CELL_SHIFT(BLOB_CELL_SHIFT),.MIN_H_RUN(MIN_H_RUN),
            .MIN_PIXELS(MIN_PIXELS),.KEEP_MIN_PIXELS(KEEP_MIN_PIXELS),
            .MIN_AREA(MIN_AREA),.MIN_FILL_RATIO(MIN_FILL_RATIO),
            .CORE_MIN_PIXELS(BLOB_CORE_MIN_PIXELS),.WEAK_CELL_MIN_PIXELS(BLOB_WEAK_CELL_MIN_PIXELS),
            .HALO_MIN_PIXELS(BLOB_HALO_MIN_PIXELS),.MIN_STRONG_PIXELS(BLOB_MIN_STRONG_PIXELS),
            .STRONG_FILL_RATIO(BLOB_STRONG_FILL_RATIO),.WEAK_FALLBACK_ENABLE(BLOB_WEAK_FALLBACK_ENABLE),
            .WEAK_FALLBACK_COLOR_MASK(BLOB_WEAK_FALLBACK_COLOR_MASK),
            .WEAK_FALLBACK_MAX_AREA(BLOB_WEAK_FALLBACK_MAX_AREA),.MAX_ASPECT_RATIO(BLOB_MAX_ASPECT_RATIO),
            .TIGHT_BOUNDS(BLOB_TIGHT_BOUNDS),.CORE_QUALITY_ENABLE(BLOB_CORE_QUALITY_ENABLE),
            .MAX_PROCESS_CYCLES(BLOB_MAX_PROCESS_CYCLES),.EDGE_ACQUIRE_MARGIN(EDGE_ACQUIRE_MARGIN)) detector (
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable),
            .I_frame_start(stat_start[0]),.I_line_end(stat_end[0]),.I_de(stat_de[0]),
            .I_x(stat_x[0]),.I_y(stat_y[0]),.I_hit(stat_hit),.I_strong_hit(stat_strong),
            .I_locked(O_valid & {3{I_enable}}),
            .O_result(blob_result),.O_color(blob_color),.O_match(blob_match),.O_acquire(blob_acquire),
            .O_center_x(blob_x),.O_center_y(blob_y),.O_left(blob_left),.O_right(blob_right),
            .O_top(blob_top),.O_bottom(blob_bottom),.O_pixel_count(blob_count));
    end else begin : legacy_blob_inputs
        assign blob_result=0;assign blob_match=0;assign blob_acquire=0;assign blob_color=0;
        assign blob_x=0;assign blob_y=0;assign blob_left=0;assign blob_right=0;
        assign blob_top=0;assign blob_bottom=0;assign blob_count=0;
    end endgenerate
    assign O_measure_result=blob_result;
    assign O_measure_match=blob_match;
    assign O_measure_color=blob_color;
    assign O_measure_bounds={blob_bottom,blob_top,blob_right,blob_left};
    genvar i;
    generate for(i=0;i<3;i=i+1) begin : colors
        localparam [1:0] SELECT_COLOR = i+1;
        color_target_tracker #(
            .IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT),
            .COLOR_MIN(COLOR_MIN), .COLOR_MARGIN(COLOR_MARGIN),
            .GREEN_MIN(GREEN_MIN), .GREEN_MARGIN(GREEN_MARGIN),
            .GREEN_HUE_NUM(GREEN_HUE_NUM), .GREEN_HUE_DEN(GREEN_HUE_DEN),
            .RED_MIN(RED_MIN),.RED_MARGIN(RED_MARGIN),.RED_HUE_NUM(RED_HUE_NUM),.RED_HUE_DEN(RED_HUE_DEN),
            .BLUE_MIN(BLUE_MIN),.BLUE_MARGIN(BLUE_MARGIN),.BLUE_HUE_NUM(BLUE_HUE_NUM),.BLUE_HUE_DEN(BLUE_HUE_DEN),
            .HSV_SAT_SHIFT(i==1 ? GREEN_HSV_SAT_SHIFT : i==2 ? BLUE_HSV_SAT_SHIFT : HSV_SAT_SHIFT),
            .STRONG_MIN(STRONG_MIN),.STRONG_MARGIN(STRONG_MARGIN),
            .GREEN_STRONG_MIN(GREEN_STRONG_MIN),.GREEN_STRONG_MARGIN(GREEN_STRONG_MARGIN),
            .BLUE_STRONG_MIN(BLUE_STRONG_MIN),.BLUE_STRONG_MARGIN(BLUE_STRONG_MARGIN),
            .STRONG_SAT_SHIFT(i==0 ? RED_STRONG_SAT_SHIFT : i==2 ? BLUE_STRONG_SAT_SHIFT : STRONG_SAT_SHIFT),
            .GREEN_STRONG_SAT_SHIFT(GREEN_STRONG_SAT_SHIFT),.BLOB_ENABLE(BLOB_ENABLE),
            .BLOB_ACQUIRE_INPUT_ENABLE(1),
            .MIN_PIXELS(MIN_PIXELS), .BOX_THICKNESS(BOX_THICKNESS),
            .CROSS_RADIUS(CROSS_RADIUS), .SUM_WIDTH(SUM_WIDTH),
            .HSV_ENABLE(HSV_ENABLE), .TRACK_ROI(TRACK_ROI), .ROI_MARGIN(ROI_MARGIN),.EDGE_ACQUIRE_MARGIN(EDGE_ACQUIRE_MARGIN),
            .MIN_H_RUN(MIN_H_RUN), .MIN_AREA(MIN_AREA), .MIN_FILL_RATIO(MIN_FILL_RATIO),
            .STAT_PIPELINE(STAT_PIPELINE),.STABLE_ENABLE(STABLE_ENABLE),.KEEP_MIN_PIXELS(KEEP_MIN_PIXELS),
            .CONFIRM_FRAMES(CONFIRM_FRAMES),.HOLD_FRAMES(HOLD_FRAMES),.ASSOC_DISTANCE(ASSOC_DISTANCE),
            .COORD_DEADBAND(COORD_DEADBAND),.SMOOTH_LIMIT(SMOOTH_LIMIT),.SMOOTH_SHIFT(SMOOTH_SHIFT)
        )tracker(
            .I_clk(I_clk), .I_rst_n(I_rst_n), .I_frame_start(I_frame_start),
            .I_line_end(I_line_end), .I_de(I_de), .I_rgb(I_rgb),
            .I_color(I_enable ? SELECT_COLOR : 2'd0),
            .I_blob_result(blob_result && blob_color==SELECT_COLOR),.I_blob_match(blob_match),.I_blob_acquire(blob_acquire),
            .I_blob_center_x(blob_x),.I_blob_center_y(blob_y),
            .I_blob_left(blob_left),.I_blob_right(blob_right),.I_blob_top(blob_top),.I_blob_bottom(blob_bottom),
            .I_blob_pixel_count(blob_count),
            .O_stat_start(stat_start[i]),.O_stat_end(stat_end[i]),.O_stat_de(stat_de[i]),
            .O_stat_x(stat_x[i]),.O_stat_y(stat_y[i]),.O_stat_hit(stat_hit[i]),.O_stat_strong_hit(stat_strong[i]),
            .O_rgb(marked_rgb[i]), .O_overlay_hit(overlay_hit[i]),
            .O_valid(O_valid[i]), .O_update(O_update[i]), .O_measured(O_measured[i]), .O_color(),
            .O_center_x(O_center_x[i*12 +: 12]), .O_center_y(O_center_y[i*12 +: 12]),
            .O_left(O_left[i*12 +: 12]), .O_right(O_right[i*12 +: 12]),
            .O_top(O_top[i*12 +: 12]), .O_bottom(O_bottom[i*12 +: 12]),
            .O_pixel_count(O_pixel_count[i*SUM_WIDTH +: SUM_WIDTH])
        );
    end endgenerate
    // Overlap priority is deterministic: red, then green, then blue.
    // Every detector reads the ORIGINAL RGB, never another detector's marks.
    assign O_rgb = !I_enable ? I_rgb : overlay_hit[0] ? marked_rgb[0] :
                   overlay_hit[1] ? marked_rgb[1] : overlay_hit[2] ? marked_rgb[2] : I_rgb;
endmodule
`endif
