// Real-time RGB888 video style and enhancement processor.
//
// Display modes (advance on every I_mode_next pulse):
//   0: original video
//   1: grayscale
//   2: binary image
//   3: Sobel edge extraction
//   4: color cartoon (2x2 smoothing, saturated palette, black Sobel edges)
//   5: original video for simultaneous RGB localization overlay at top level
//
// The reference edge_detect_96 uses Y = R/4 + G/2 + B/4 and averages adjacent
// pixels before Sobel. Keep this interface at one pixel per clock so the
// original-video mode and HDMI timing do not change.
// Mode 3 uses |Gx| + |Gy|. Mode 4 thresholds the two axes independently,
// following Cornell ECE 5760 Real-time Cartoonifier (Fall 2010).
// Reference: https://people.ece.cornell.edu/land/courses/ece5760/FinalProjects/
// f2010/kaf42_jay29_teg25/teg25_jay29_kaf42/code/DE2_TV.v
// The SRAM-dependent temporal average is omitted. A local 2x2 box filter
// supplies spatial smoothing using only the line memories below.

module video_style_processor #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [7:0]   BINARY_THRESHOLD = 8'd96,
    parameter [10:0]  SOBEL_THRESHOLD  = 11'd80,
    // Cornell's 10-bit threshold 192 scales to 48 on an 8-bit RGB bus.
    parameter [9:0]   CARTOON_EDGE_THRESHOLD = 10'd48,
    parameter [4:0]   CARTOON_RED_MASK   = 5'b11100,
    parameter [4:0]   CARTOON_GREEN_MASK = 5'b11110,
    parameter [4:0]   CARTOON_BLUE_MASK  = 5'b11100,
    parameter         CARTOON_BRIGHT    = 1'b0,
    // Enable the reference-slide style; 0 retains the legacy Cornell palette.
    parameter         CARTOON_ENHANCE   = 1'b0,
    parameter integer CARTOON_PALETTE_BITS = 5
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_frame_start,
    input  wire        I_line_end,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire        I_mode_next,
    input  wire        I_mode_original,
    input  wire        I_mode_set,
    input  wire [2:0]  I_mode_value,
    output reg  [23:0] O_rgb,
    output wire [2:0]  O_mode
);

    localparam [2:0] MODE_ORIGINAL = 3'd0;
    localparam [2:0] MODE_GRAY     = 3'd1;
    localparam [2:0] MODE_BINARY   = 3'd2;
    localparam [2:0] MODE_EDGE     = 3'd3;
    localparam [2:0] MODE_CARTOON  = 3'd4;
    localparam [2:0] MODE_COLOR_LAST = 3'd5;
    localparam [31:0] FRAME_PIXEL_COUNT = IMG_WIDTH * IMG_HEIGHT;

    reg [2:0] S_mode_active;
    reg S_mode_pending;
    reg [2:0] S_mode_requested;
    reg [11:0] S_x;
    reg [11:0] S_y;
    wire [11:0] S_mem_x;

    // Packed Cornell pixel: {intensity/4 (6 bits), R5, G5, B5}.
    // Intensity is computed from each channel's top four bits, as in DE2_TV.
    // Its bottom two bits are always zero, so only six bits need storage.
    // Pack common line addresses into synchronous embedded RAMs. Row 2
    // needs only grayscale and cartoon intensity, not its RGB555 channels.
    reg [28:0] S_line_1 [0:IMG_WIDTH-1];
    reg [13:0] S_line_2 [0:IMG_WIDTH-1];
    reg [28:0] S_line_1_q;
    reg [13:0] S_line_2_q;
    reg [11:0] S_prefetch_x;
    reg S_prefetch_valid;
    wire [11:0] S_read_x = I_frame_start ?
        (I_de && !I_line_end ? 12'd1 : 12'd0) :
        (I_de ? (I_line_end ? 12'd0 : S_mem_x+12'd1) : S_mem_x);
    wire S_read_enable = !S_prefetch_valid || S_read_x != S_prefetch_x;

    reg [7:0] S_p11;
    reg [7:0] S_p12;
    reg [7:0] S_p21;
    reg [7:0] S_p22;
    reg [7:0] S_p31;
    reg [7:0] S_p32;
    reg [20:0] S_c_p11;
    reg [20:0] S_c_p12;
    reg [20:0] S_c_p21;
    reg [20:0] S_c_p22;
    reg [20:0] S_c_p31;
    reg [20:0] S_c_p32;
    reg [7:0] S_gray_previous;
    reg [31:0] S_frame_gray_sum;
    reg [7:0]  S_mean_threshold;
    reg [31:0] S_mean_dividend;
    reg [31:0] S_mean_div_quotient;
    reg [31:0] S_mean_div_remainder;
    reg [5:0]  S_mean_div_count;
    reg        S_mean_div_active;

    wire [7:0]  S_gray;
    wire [7:0]  S_edge_gray;
    wire [32:0] S_mean_remainder_shift;
    wire        S_mean_quotient_bit;
    wire [32:0] S_mean_remainder_next;
    wire [31:0] S_mean_quotient_next;
    wire        S_last_pixel_of_frame;
    wire [7:0]  S_line_1_current;
    wire [7:0]  S_line_2_current;
    wire [20:0] S_cartoon_color_in;
    wire [20:0] S_c_line_1_current;
    wire [20:0] S_c_line_2_current;
    wire        S_window_valid;

    wire [10:0] S_gx_positive;
    wire [10:0] S_gx_negative;
    wire [10:0] S_gy_positive;
    wire [10:0] S_gy_negative;
    wire [10:0] S_gx_abs;
    wire [10:0] S_gy_abs;
    wire [10:0] S_gradient;
    wire        S_edge;
    wire [7:0] S_c_gx_positive;
    wire [7:0] S_c_gx_negative;
    wire [7:0] S_c_gy_positive;
    wire [7:0] S_c_gy_negative;
    wire [7:0] S_c_gx_abs;
    wire [7:0] S_c_gy_abs;
    wire [5:0] S_box_r_top;
    wire [5:0] S_box_r_bot;
    wire [6:0] S_box_r_sum;
    wire [5:0] S_box_g_top;
    wire [5:0] S_box_g_bot;
    wire [6:0] S_box_g_sum;
    wire [5:0] S_box_b_top;
    wire [5:0] S_box_b_bot;
    wire [6:0] S_box_b_sum;
    wire [4:0] S_cartoon_r;
    wire [4:0] S_cartoon_g;
    wire [4:0] S_cartoon_b;
    wire [23:0] S_cartoon_color_24;
    wire        S_cartoon_edge;
    wire [23:0] S_cartoon_rgb;
    wire [6:0]  S_cartoon_luma_sum;
    wire [4:0]  S_cartoon_luma;

    function [7:0] F_rgb_to_gray;
        input [23:0] I_pixel;
        begin
            F_rgb_to_gray = {2'b0, I_pixel[23:18]}
                          + {1'b0, I_pixel[15:9]}
                          + {2'b0, I_pixel[7:2]};
        end
    endfunction

    function [7:0] F_saturated_palette;
        input [4:0] channel;
        input [4:0] luma;
        reg signed [6:0] difference;
        reg signed [6:0] saturated;
        reg [4:0] limited;
        begin
            // Fixed-point 1.5x saturation, without multipliers or frame RAM.
            // Signed arithmetic and clamping keep highlights from wrapping.
            difference = $signed({2'b00,channel}) - $signed({2'b00,luma});
            saturated = $signed({2'b00,channel}) + (difference >>> 1);
            if(saturated < 0) limited = 5'd0;
            else if(saturated > 31) limited = 5'd31;
            else limited = saturated[4:0];
            // Fine 32-level palette retains the illustrated example's detail.
            // Equal full-range channels preserve gray, black and white clouds.
            if(CARTOON_PALETTE_BITS==4)
                F_saturated_palette = {limited[4:1],limited[4:1]};
            else F_saturated_palette = {limited,limited[4:2]};
        end
    endfunction

    function [5:0] F_cartoon_intensity;
        input [23:0] I_pixel;
        begin
            // (R/4 + G/2 + B/4)/4 after clearing each channel's low 4 bits.
            F_cartoon_intensity = {2'd0, I_pixel[23:20]}
                                + {1'b0, I_pixel[15:12], 1'b0}
                                + {2'd0, I_pixel[7:4]};
        end
    endfunction

    function [7:0] F_cartoon_quantize;
        input [4:0] I_channel;
        input [4:0] I_mask;
        begin
            // Cornell appends five ones on its RGB10 VGA bus. On RGB8 this
            // becomes three ones. BRIGHT also fills the discarded mask bits.
            F_cartoon_quantize = {((I_channel & I_mask)
                                | (CARTOON_BRIGHT ? ~I_mask : 5'd0)), 3'b111};
        end
    endfunction

    assign O_mode = S_mode_active;

    // I_frame_start can coincide with the first active pixel. Force that
    // pixel to address column zero before the coordinate register resets.
    assign S_mem_x = I_frame_start ? 12'd0 : S_x;
    assign S_gray = F_rgb_to_gray(I_rgb);
    // The reference processes two-pixel averages. At one pixel per clock a
    // sliding two-pixel average gives similar noise suppression without
    // halving horizontal resolution or delaying the HDMI pixel stream.
    assign S_edge_gray = (S_mem_x == 12'd0) ? S_gray
                       : ({1'b0, S_gray_previous} + {1'b0, S_gray}) >> 1;

    assign S_last_pixel_of_frame = I_de && I_line_end &&
                                   (S_y == IMG_HEIGHT - 1);
    assign S_mean_remainder_shift = {S_mean_div_remainder, S_mean_dividend[31]};
    assign S_mean_quotient_bit =
        (S_mean_remainder_shift >= {1'b0, FRAME_PIXEL_COUNT});
    assign S_mean_remainder_next = S_mean_quotient_bit
        ? S_mean_remainder_shift - {1'b0, FRAME_PIXEL_COUNT}
        : S_mean_remainder_shift;
    assign S_mean_quotient_next = {S_mean_div_quotient[30:0],
                                   S_mean_quotient_bit};

    // The mean threshold is computed from one complete frame and used on the
    // next frame. The first frame uses BINARY_THRESHOLD until a mean is ready.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_gray_sum <= 32'd0;
        end else if(I_de) begin
            if(I_frame_start)
                S_frame_gray_sum <= {24'd0, S_gray};
            else if(S_last_pixel_of_frame)
                S_frame_gray_sum <= 32'd0;
            else
                S_frame_gray_sum <= S_frame_gray_sum + S_gray;
        end
    end

    // A 32-cycle restoring divide computes sum / (width * height). It runs
    // in the inactive-video interval after the frame's final active pixel.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_mean_threshold      <= BINARY_THRESHOLD;
            S_mean_dividend       <= 32'd0;
            S_mean_div_quotient   <= 32'd0;
            S_mean_div_remainder  <= 32'd0;
            S_mean_div_count      <= 6'd0;
            S_mean_div_active     <= 1'b0;
        end else if(S_last_pixel_of_frame) begin
            // Include the final pixel, accumulated on this same clock edge.
            S_mean_dividend       <= S_frame_gray_sum + S_gray;
            S_mean_div_quotient   <= 32'd0;
            S_mean_div_remainder  <= 32'd0;
            S_mean_div_count      <= 6'd0;
            S_mean_div_active     <= 1'b1;
        end else if(S_mean_div_active) begin
            S_mean_dividend       <= {S_mean_dividend[30:0], 1'b0};
            S_mean_div_quotient   <= S_mean_quotient_next;
            S_mean_div_remainder  <= S_mean_remainder_next[31:0];
            if(S_mean_div_count == 6'd31) begin
                S_mean_div_active <= 1'b0;
                S_mean_threshold  <= S_mean_quotient_next[7:0];
            end else begin
                S_mean_div_count <= S_mean_div_count + 1'b1;
            end
        end
    end

    assign S_line_1_current = (S_mem_x < IMG_WIDTH) ? S_line_1_q[28:21] : 8'd0;
    assign S_line_2_current = (S_mem_x < IMG_WIDTH) ? S_line_2_q[13:6] : 8'd0;
    assign S_cartoon_color_in = {F_cartoon_intensity(I_rgb),
                                I_rgb[23:19], I_rgb[15:11], I_rgb[7:3]};
    assign S_c_line_1_current = (S_mem_x < IMG_WIDTH)
                              ? S_line_1_q[20:0] : 21'd0;
    assign S_c_line_2_current = (S_mem_x < IMG_WIDTH)
                              ? {S_line_2_q[5:0],15'd0} : 21'd0;

    // Window center is (x-1,y-1).
    //   p11 p12 line_2[x]
    //   p21 p22 line_1[x]
    //   p31 p32 gray(x,y)
    assign S_window_valid = I_de && !I_frame_start &&
                            (S_x >= 12'd2) && (S_y >= 12'd2) &&
                            (S_x < IMG_WIDTH);

    assign S_gx_positive = {3'd0, S_line_2_current}
                         + ({3'd0, S_line_1_current} << 1)
                         + {3'd0, S_edge_gray};
    assign S_gx_negative = {3'd0, S_p11}
                         + ({3'd0, S_p21} << 1)
                         + {3'd0, S_p31};

    assign S_gy_positive = {3'd0, S_p11}
                         + ({3'd0, S_p12} << 1)
                         + {3'd0, S_line_2_current};
    assign S_gy_negative = {3'd0, S_p31}
                         + ({3'd0, S_p32} << 1)
                         + {3'd0, S_edge_gray};

    assign S_gx_abs = (S_gx_positive >= S_gx_negative)
                    ? (S_gx_positive - S_gx_negative)
                    : (S_gx_negative - S_gx_positive);
    assign S_gy_abs = (S_gy_positive >= S_gy_negative)
                    ? (S_gy_positive - S_gy_negative)
                    : (S_gy_negative - S_gy_positive);
    assign S_gradient = S_gx_abs + S_gy_abs;
    assign S_edge = S_window_valid && (S_gradient >= SOBEL_THRESHOLD);

    // Cornell thresholds |Gx| and |Gy| separately, then ORs the results.
    // Use full unsigned weighted sums to avoid the reference's signed wrap
    // on high-contrast input. The stored intensity is one quarter of RGB8 Y.
    assign S_c_gx_positive = {2'd0, S_c_line_2_current[20:15]}
                           + {1'b0, S_c_line_1_current[20:15], 1'b0}
                           + {2'd0, S_cartoon_color_in[20:15]};
    assign S_c_gx_negative = {2'd0, S_c_p11[20:15]}
                           + {1'b0, S_c_p21[20:15], 1'b0}
                           + {2'd0, S_c_p31[20:15]};
    assign S_c_gy_positive = {2'd0, S_c_p11[20:15]}
                           + {1'b0, S_c_p12[20:15], 1'b0}
                           + {2'd0, S_c_line_2_current[20:15]};
    assign S_c_gy_negative = {2'd0, S_c_p31[20:15]}
                           + {1'b0, S_c_p32[20:15], 1'b0}
                           + {2'd0, S_cartoon_color_in[20:15]};
    assign S_c_gx_abs = (S_c_gx_positive >= S_c_gx_negative)
                       ? S_c_gx_positive - S_c_gx_negative
                       : S_c_gx_negative - S_c_gx_positive;
    assign S_c_gy_abs = (S_c_gy_positive >= S_c_gy_negative)
                       ? S_c_gy_positive - S_c_gy_negative
                       : S_c_gy_negative - S_c_gy_positive;
    assign S_cartoon_edge = S_window_valid &&
                          (({S_c_gx_abs, 2'd0} > CARTOON_EDGE_THRESHOLD) ||
                           ({S_c_gy_abs, 2'd0} > CARTOON_EDGE_THRESHOLD));

    // SRAM-free adaptation: average a sliding 2x2 neighborhood starting at
    // the Sobel center (x-1,y-1), then reduce each RGB555 channel with a mask.
    // There is no history of previous frames and no added clock latency.
    assign S_box_r_top = {1'b0, S_c_p22[14:10]}
                        + {1'b0, S_c_line_1_current[14:10]};
    assign S_box_r_bot = {1'b0, S_c_p32[14:10]}
                        + {1'b0, S_cartoon_color_in[14:10]};
    assign S_box_r_sum = {1'b0, S_box_r_top} + {1'b0, S_box_r_bot};
    assign S_box_g_top = {1'b0, S_c_p22[9:5]}
                        + {1'b0, S_c_line_1_current[9:5]};
    assign S_box_g_bot = {1'b0, S_c_p32[9:5]}
                        + {1'b0, S_cartoon_color_in[9:5]};
    assign S_box_g_sum = {1'b0, S_box_g_top} + {1'b0, S_box_g_bot};
    assign S_box_b_top = {1'b0, S_c_p22[4:0]}
                        + {1'b0, S_c_line_1_current[4:0]};
    assign S_box_b_bot = {1'b0, S_c_p32[4:0]}
                        + {1'b0, S_cartoon_color_in[4:0]};
    assign S_box_b_sum = {1'b0, S_box_b_top} + {1'b0, S_box_b_bot};
    // Quantize the current input on borders until a complete window exists.
    assign S_cartoon_r = S_window_valid ? S_box_r_sum[6:2] : I_rgb[23:19];
    assign S_cartoon_g = S_window_valid ? S_box_g_sum[6:2] : I_rgb[15:11];
    assign S_cartoon_b = S_window_valid ? S_box_b_sum[6:2] : I_rgb[7:3];
    assign S_cartoon_luma_sum = {2'd0,S_cartoon_r}
                              + {1'b0,S_cartoon_g,1'b0}
                              + {2'd0,S_cartoon_b};
    assign S_cartoon_luma = S_cartoon_luma_sum[6:2];
    assign S_cartoon_color_24 = CARTOON_ENHANCE ? {
        F_saturated_palette(S_cartoon_r,S_cartoon_luma),
        F_saturated_palette(S_cartoon_g,S_cartoon_luma),
        F_saturated_palette(S_cartoon_b,S_cartoon_luma)} : {
        F_cartoon_quantize(S_cartoon_r, CARTOON_RED_MASK),
        F_cartoon_quantize(S_cartoon_g, CARTOON_GREEN_MASK),
        F_cartoon_quantize(S_cartoon_b, CARTOON_BLUE_MASK)};
    assign S_cartoon_rgb = S_cartoon_edge ? 24'h000000 : S_cartoon_color_24;
    // Local keys retain immediate response and cancel any remote request.
    // A remote selection takes effect at SOF, keeping each frame in one mode.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_mode_active <= MODE_ORIGINAL;
            S_mode_pending <= 1'b0;
            S_mode_requested <= MODE_ORIGINAL;
        end
        else if(I_mode_original) begin
            S_mode_active <= MODE_ORIGINAL;
            S_mode_pending <= 1'b0;
        end
        else if(I_mode_next) begin
                S_mode_pending <= 1'b0;
                if(S_mode_active == MODE_COLOR_LAST)
                    S_mode_active <= MODE_ORIGINAL;
                else
                    S_mode_active <= S_mode_active + 3'd1;
        end
        else if(I_mode_set && I_mode_value <= MODE_COLOR_LAST) begin
            S_mode_requested <= I_mode_value;
            S_mode_pending <= !I_frame_start;
            if(I_frame_start) S_mode_active <= I_mode_value;
        end
        else if(I_frame_start && S_mode_pending) begin
            S_mode_active <= S_mode_requested;
            S_mode_pending <= 1'b0;
        end
    end

    // Active-video coordinates. I_line_end belongs to the final line pixel.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else begin
            if(I_frame_start) begin
                S_x <= I_de ? 12'd1 : 12'd0;
                S_y <= 12'd0;
            end
            else if(I_de) begin
                if(I_line_end) begin
                    S_x <= 12'd0;
                    S_y <= S_y + 12'd1;
                end
                else begin
                    S_x <= S_x + 12'd1;
                end
            end
        end
    end

    // Memories are not cleared on reset; S_window_valid suppresses stale data
    // until two new rows have been received.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_gray_previous <= 8'd0;
        end else if(I_de && (S_mem_x < IMG_WIDTH)) begin
            S_gray_previous <= S_gray;
        end
    end

    // Look ahead one address, matching the coordinate increment on this
    // edge. Current-pixel output and all existing video timing stay intact.
    // A DE pause holds both the coordinate and the already prefetched data.
    always @(posedge I_clk) begin
        if(S_read_enable && S_read_x < IMG_WIDTH) begin
            S_line_1_q <= S_line_1[S_read_x];
            S_line_2_q <= S_line_2[S_read_x];
        end
        if(I_de && S_mem_x < IMG_WIDTH) begin
            S_line_1[S_mem_x] <= {S_edge_gray,S_cartoon_color_in};
            S_line_2[S_mem_x] <= {S_line_1_current,S_c_line_1_current[20:15]};
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin S_prefetch_x<=0; S_prefetch_valid<=0; end
        else begin S_prefetch_x<=S_read_x; S_prefetch_valid<=1; end
    end

    // Horizontal taps for the three rows of the 3x3 window.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_p11 <= 8'd0;
            S_p12 <= 8'd0;
            S_p21 <= 8'd0;
            S_p22 <= 8'd0;
            S_p31 <= 8'd0;
            S_p32 <= 8'd0;
        end
        else if(I_frame_start || (I_de && I_line_end)) begin
            S_p11 <= 8'd0;
            S_p12 <= 8'd0;
            S_p21 <= 8'd0;
            S_p22 <= 8'd0;
            S_p31 <= 8'd0;
            S_p32 <= 8'd0;
        end
        else if(I_de) begin
            S_p11 <= S_p12;
            S_p12 <= S_line_2_current;
            S_p21 <= S_p22;
            S_p22 <= S_line_1_current;
            S_p31 <= S_p32;
            S_p32 <= S_edge_gray;
        end
    end

    // Horizontal taps for the packed Cornell intensity/RGB555 window. Memories
    // intentionally not reset; S_window_valid hides stale data during the
    // first two rows after reset, just as in the Sobel path above.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_c_p11 <= 21'd0;
            S_c_p12 <= 21'd0;
            S_c_p21 <= 21'd0;
            S_c_p22 <= 21'd0;
            S_c_p31 <= 21'd0;
            S_c_p32 <= 21'd0;
        end
        else if(I_frame_start || (I_de && I_line_end)) begin
            S_c_p11 <= 21'd0;
            S_c_p12 <= 21'd0;
            S_c_p21 <= 21'd0;
            S_c_p22 <= 21'd0;
            S_c_p31 <= 21'd0;
            S_c_p32 <= 21'd0;
        end
        else if(I_de) begin
            S_c_p11 <= S_c_p12;
            S_c_p12 <= S_c_line_2_current;
            S_c_p21 <= S_c_p22;
            S_c_p22 <= S_c_line_1_current;
            S_c_p31 <= S_c_p32;
            S_c_p32 <= S_cartoon_color_in;
        end
    end

    // Original/binary/edge use the exact same no-extra-latency output path.
    always @(*) begin
        case(S_mode_active)
            MODE_GRAY:
                O_rgb = {S_gray, S_gray, S_gray};

            MODE_BINARY:
                O_rgb = (S_gray > S_mean_threshold) ? 24'hffffff : 24'h000000;

            MODE_EDGE:
                O_rgb = S_edge ? 24'hffffff : 24'h000000;

            MODE_CARTOON:
                O_rgb = S_cartoon_rgb;

            default:
                O_rgb = I_rgb;
        endcase
    end

endmodule
