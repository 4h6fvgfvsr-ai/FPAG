module hdmi_mixer #(
    parameter H_OFFSET=128, V_OFFSET=60, IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter integer DEBUG_MODE=0, FPS_ENABLE=0, SERVO_ENABLE=0, SERVO_COLOR=0,
    parameter integer TELEMETRY_ENABLE=0, MOTION_ENABLE=0, BUFFER_HEALTH_ENABLE=0,
    parameter integer THRESHOLD_ENABLE=0, PERFORMANCE_ENABLE=0, SIZE_ENABLE=0,
    parameter integer LATENCY_ENABLE=1, THREE_THRESHOLD_ONLY=0, MOTION_SERVO_ENABLE=0,
    parameter integer CONTEXT_DISPLAY=0
)(
    input wire I_clk, I_rst_n,
    input wire I_video_vsync, I_video_hsync, I_video_de, I_video_user, I_video_last,
    input wire [3:0] I_debug_status,
    input wire [2:0] I_style_mode, I_targets_valid,
    input wire [11:0] I_fps_bcd,
    input wire [1:0] I_servo_state, // 0 OFF, 1 LOST, 2 ON, 3 LOCK
    input wire [35:0] I_targets_left, I_targets_right, I_targets_top, I_targets_bottom,
    input wire I_latency_valid,
    input wire [23:0] I_latency_ticks, // 100 us per tick; invalid/out-of-range shows dashes
    input wire [15:0] I_source_frame_id,
    input wire I_motion_valid,I_motion_scene_change,
    input wire [15:0] I_motion_cells,I_motion_events,
    input wire [11:0] I_motion_left,I_motion_right,I_motion_top,I_motion_bottom,
    input wire [3:0] I_buffer_faults,
    input wire I_buffer_ready,
    input wire I_binary_auto,
    input wire [7:0] I_binary_threshold,I_binary_effective,I_sobel_weak,I_sobel_strong,
    input wire [7:0] I_cartoon_threshold,I_range_threshold,I_motion_threshold,
    input wire I_perf_resolution_valid,I_perf_rate_valid,I_perf_pixels_valid,I_perf_drop_valid,
    input wire [15:0] I_perf_width,I_perf_height,I_perf_fps_x100,I_perf_drop_x100,
    input wire [31:0] I_perf_pixels_per_second,
    input wire [2:0] I_size_valid,
    input wire [47:0] I_size_length_x10,I_size_width_x10,
    output wire O_video_rd_en,
    input wire [23:0] I_video_rd_data,
    output reg O_hdmi_vsync, O_hdmi_hsync, O_hdmi_de,
    output reg [23:0] O_hdmi_data
);
    // All paths use three clocks: glyph address / glyph pixel / composition.
    // The logo independently uses the same first two clocks for its raster ROM.
    localparam integer LOGO_W=160, LOGO_H=160;
    localparam integer CENTER_W=288, CENTER_H=32;
    localparam integer CENTER_X=(IMG_WIDTH-CENTER_W)/2, CENTER_Y=IMG_HEIGHT/6;
    localparam integer STATUS_X=16, STATUS_Y=IMG_HEIGHT-40, LEGEND_Y=IMG_HEIGHT-80;
    localparam integer FPS_W=CONTEXT_DISPLAY ? 288:144, FPS_X=IMG_WIDTH-FPS_W-16, FPS_Y=16;
    localparam integer SERVO_W=288,SERVO_X=IMG_WIDTH-SERVO_W-16,SERVO_Y=56;
    localparam integer LATENCY_Y=CONTEXT_DISPLAY ? 56:96,SOURCE_Y=128,COORDINATE_Y=CONTEXT_DISPLAY ? 88:160;
    localparam integer BUFFER_Y=192;
    localparam integer THRESHOLD_Y=CONTEXT_DISPLAY ? 56:224,THRESHOLD_H=160;
    localparam integer PERFORMANCE_W=352,PERFORMANCE_X=IMG_WIDTH-PERFORMANCE_W-16;
    localparam integer PERFORMANCE_Y=400,PERFORMANCE_H=96;
    localparam integer MOTION_LEGEND_W=432;
    localparam integer SIZE_Y=512,SIZE_H=96;
    localparam [23:0] CENTER_COLOR=24'hfff200;
    reg [11:0] S_x, S_y;
    wire [11:0] S_pixel_x=I_video_user ? 12'd0 : S_x;
    wire [11:0] S_pixel_y=I_video_user ? 12'd0 : S_y;
    reg [11:0] S_x_1d, S_x_2d;
    reg [23:0] S_video_data_1d, S_video_data_2d;
    reg S_video_vsync_1d, S_video_vsync_2d, S_video_hsync_1d, S_video_hsync_2d;
    reg S_video_de_1d, S_video_de_2d, S_logo_window_1d, S_logo_window_2d;
    wire [7:0] S_logo_alpha;
    wire [23:0] S_logo_color;
    reg [31:0] S_frame_cnt;
    reg [3:0] S_cnt_d0, S_cnt_d1, S_cnt_d2, S_cnt_d3, S_cnt_d4, S_cnt_d5;
    reg [11:0] S_fps_display;
    reg [1:0] S_servo_display;
    reg S_servo_motion_display;
    reg [7:0] S_char, S_char_1d;
    reg [3:0] S_row, S_col, S_row_1d, S_col_1d;
    reg S_panel, S_panel_1d, S_panel_2d, S_text_on_2d;
    reg [23:0] S_text_color, S_text_color_1d, S_text_color_2d;
    wire [15:0] S_font_bits;
    // Punctuation patterns share the current glyph path; no second font.
    wire [15:0] S_glyph_bits=S_char_1d==8'h2d ?
        ((S_row_1d==7 || S_row_1d==8)?16'h7c00:16'd0) :
        (S_char_1d==8'h2e ? ((S_row_1d==13 || S_row_1d==14)?16'h1000:16'd0):
         S_char_1d==8'h25 ? F_percent_row(S_row_1d) :
         S_char_1d==8'h2f ? F_slash_row(S_row_1d) : S_font_bits);
    reg S_snapshot_pending,S_latency_display_valid,S_source_display_valid,S_coordinate_display_valid;
    reg S_motion_display_valid,S_motion_scene_display;
    reg [3:0] S_buffer_faults_display;
    reg S_buffer_ready_display;
    wire [19:0] S_latency_decimal,S_source_decimal,S_coordinate_x_decimal,S_coordinate_y_decimal;
    wire [19:0] S_motion_decimal,S_events_decimal;
    wire [83:0] S_threshold_digits;
    wire [19:0] S_perf_width_digits,S_perf_height_digits,S_perf_fps_digits,S_perf_drop_digits;
    wire [39:0] S_perf_pixels_digits;
    wire [3:0] S_perf_valid;
    wire S_binary_auto_display;
    wire [119:0] S_size_digits;
    reg [1:0] S_size_sof_delay;
    reg [2:0] S_size_display_valid;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin S_size_sof_delay<=0;S_size_display_valid<=0;end
        else begin
            S_size_sof_delay<={S_size_sof_delay[0],I_video_user};
            if(S_size_sof_delay[1]) S_size_display_valid<=SIZE_ENABLE ? I_size_valid:3'd0;
        end
    end
    generate if(SIZE_ENABLE)begin:g_size_display
        // Engine publishes SOF+1; snapshot at SOF+2, well before row512.
        osd_decimal_snapshot u_size_snapshot(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_sof(S_size_sof_delay[0]),
            .I_latency(I_size_length_x10[15:0]),.I_source_id(I_size_width_x10[15:0]),
            .I_x(I_size_length_x10[31:16]),.I_y(I_size_width_x10[31:16]),
            .I_motion(I_size_length_x10[47:32]),.I_events(I_size_width_x10[47:32]),
            .O_latency(S_size_digits[19:0]),.O_source_id(S_size_digits[39:20]),
            .O_x(S_size_digits[59:40]),.O_y(S_size_digits[79:60]),
            .O_motion(S_size_digits[99:80]),.O_events(S_size_digits[119:100]));
    end else begin:g_no_size_display
        assign S_size_digits=0;
    end endgenerate
    wire S_coordinate_motion=MOTION_SERVO_ENABLE && I_style_mode==3'd6;
    wire [12:0] S_coordinate_x_sum=S_coordinate_motion ? ({1'b0,I_motion_left}+{1'b0,I_motion_right}) :
                                  {1'b0,I_targets_left[SERVO_COLOR*12 +:12]}+
                                  {1'b0,I_targets_right[SERVO_COLOR*12 +:12]};
    wire [12:0] S_coordinate_y_sum=S_coordinate_motion ? ({1'b0,I_motion_top}+{1'b0,I_motion_bottom}) :
                                  {1'b0,I_targets_top[SERVO_COLOR*12 +:12]}+
                                  {1'b0,I_targets_bottom[SERVO_COLOR*12 +:12]};
    reg S_motion_box,S_motion_box_1d,S_motion_box_2d;
    reg [11:0] S_local_x, S_local_y, S_target_x, S_target_y, S_target_w, S_box_left, S_box_top;
    reg [4:0] S_char_index;
    integer k;
    assign O_video_rd_en=I_video_de;
    osd_char_lib u_osd_char_lib(
        .I_char(S_char_1d), .I_row(S_row_1d), .O_row_bits(S_font_bits)
    );
    generate if(PERFORMANCE_ENABLE) begin: g_performance_display
        osd_performance_snapshot u_performance_snapshot(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_sof(I_video_user),
            .I_valid({I_perf_drop_valid,I_perf_pixels_valid,I_perf_rate_valid,I_perf_resolution_valid}),
            .I_width(I_perf_width),.I_height(I_perf_height),.I_fps_x100(I_perf_fps_x100),
            .I_pixels_per_second(I_perf_pixels_per_second),.I_drop_x100(I_perf_drop_x100),
            .O_valid(S_perf_valid),.O_width(S_perf_width_digits),.O_height(S_perf_height_digits),
            .O_fps(S_perf_fps_digits),.O_pixels(S_perf_pixels_digits),.O_drop(S_perf_drop_digits));
    end else begin: g_no_performance_display
        assign S_perf_valid=4'd0;
        assign S_perf_width_digits=20'd0;assign S_perf_height_digits=20'd0;
        assign S_perf_fps_digits=20'd0;assign S_perf_pixels_digits=40'd0;assign S_perf_drop_digits=20'd0;
    end endgenerate
    generate if(THRESHOLD_ENABLE) begin: g_threshold_display
        osd_threshold_snapshot #(.FIELDS(THREE_THRESHOLD_ONLY ? 5:7)) u_threshold_snapshot(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_sof(I_video_user),.I_auto(I_binary_auto),
            .I_values({I_motion_threshold,I_range_threshold,I_cartoon_threshold,
                       I_sobel_strong,I_sobel_weak,I_binary_effective,I_binary_threshold}),
            .O_auto(S_binary_auto_display),.O_digits(S_threshold_digits));
    end else begin: g_no_threshold_display
        assign S_binary_auto_display=1'b1;
        assign S_threshold_digits=84'd0;
    end endgenerate
    generate if(TELEMETRY_ENABLE || MOTION_ENABLE) begin: g_decimal_telemetry
        osd_decimal_snapshot u_decimal_snapshot(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_sof(I_video_user),
            .I_latency(LATENCY_ENABLE ? I_latency_ticks[15:0] : 16'd0),.I_source_id(CONTEXT_DISPLAY ? 16'd0:I_source_frame_id),
            .I_x({4'd0,S_coordinate_x_sum[12:1]}),.I_y({4'd0,S_coordinate_y_sum[12:1]}),
            .I_motion(I_motion_cells),.I_events(I_motion_events),
            .O_latency(S_latency_decimal),.O_source_id(S_source_decimal),
            .O_x(S_coordinate_x_decimal),.O_y(S_coordinate_y_decimal),
            .O_motion(S_motion_decimal),.O_events(S_events_decimal));
    end else begin: g_no_decimal_telemetry
        assign S_latency_decimal=0;assign S_source_decimal=0;
        assign S_coordinate_x_decimal=0;assign S_coordinate_y_decimal=0;
        assign S_motion_decimal=0;assign S_events_decimal=0;
    end endgenerate
    // One-clock-after-SOF capture observes same-edge tracker/meter NBA commits.
    // Conversion takes 96 clocks, well before panels beginning at row 96.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_snapshot_pending<=0;S_latency_display_valid<=0;S_source_display_valid<=0;
            S_coordinate_display_valid<=0;S_motion_display_valid<=0;S_motion_scene_display<=0;
            S_buffer_faults_display<=0;S_buffer_ready_display<=0;
        end else begin
            S_snapshot_pending<=I_video_user;
            if(S_snapshot_pending) begin
                S_latency_display_valid<=TELEMETRY_ENABLE && LATENCY_ENABLE && I_latency_valid && I_latency_ticks<=24'd65535;
                S_source_display_valid<=TELEMETRY_ENABLE && I_latency_valid;
                S_coordinate_display_valid<=TELEMETRY_ENABLE && (S_coordinate_motion ? I_motion_valid:I_targets_valid[SERVO_COLOR]);
                S_motion_display_valid<=MOTION_ENABLE && I_motion_valid;
                S_motion_scene_display<=MOTION_ENABLE && I_motion_scene_change;
                S_buffer_faults_display<=BUFFER_HEALTH_ENABLE ? I_buffer_faults : 4'd0;
                S_buffer_ready_display<=BUFFER_HEALTH_ENABLE && I_buffer_ready;
            end
        end
    end
    anlogic_logo_rom u_anlogic_logo_rom(
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_x(S_pixel_x<LOGO_W && S_pixel_y<LOGO_H ? S_pixel_x[8:0] : 9'd511),
        .I_y(S_pixel_x<LOGO_W && S_pixel_y<LOGO_H ? S_pixel_y[8:0] : 9'd511),
        .O_alpha(S_logo_alpha), .O_color(S_logo_color)
    );
    function [7:0] F_color_char;
        input [1:0] color_index;
        input [3:0] char_index;
        begin
            F_color_char=8'h20;
            case(color_index)
                0:case(char_index) 0:F_color_char="R"; 1:F_color_char="E"; 2:F_color_char="D"; endcase
                1:case(char_index) 0:F_color_char="G"; 1:F_color_char="R"; 2:F_color_char="E";
                    3:F_color_char="E"; 4:F_color_char="N"; endcase
                2:case(char_index) 0:F_color_char="B"; 1:F_color_char="L"; 2:F_color_char="U";
                    3:F_color_char="E"; endcase
            endcase
        end
    endfunction
    function [7:0] F_servo_char;
        input [4:0] index;
        input [1:0] state;
        input motion_mode;
        reg [31:0] word;
        begin
            case(state) 0:word="OFF ";1:word="LOST";2:word="ON  ";default:word="LOCK";endcase
            if(SIZE_ENABLE && state!=0)word="FIXD";
            F_servo_char=" ";
            case(index)
                0:F_servo_char="T";1:F_servo_char="R";2:F_servo_char="A";
                3:F_servo_char="C";4:F_servo_char="K";5:F_servo_char=":";
                7:F_servo_char=motion_mode ? "M":F_color_char(SERVO_COLOR,0);
                8:F_servo_char=motion_mode ? "O":F_color_char(SERVO_COLOR,1);
                9:F_servo_char=motion_mode ? "T":F_color_char(SERVO_COLOR,2);
                10:F_servo_char=motion_mode ? "I":F_color_char(SERVO_COLOR,3);
                11:F_servo_char=motion_mode ? "O":F_color_char(SERVO_COLOR,4);
                12:F_servo_char=motion_mode ? "N":" ";
                13:F_servo_char=word[31:24];14:F_servo_char=word[23:16];
                15:F_servo_char=word[15:8];16:F_servo_char=word[7:0];
                default:F_servo_char=" ";
            endcase
        end
    endfunction
    function [7:0] F_fps_char;
        input [4:0] index;
        input [11:0] bcd;
        begin
            case(index)
                0:F_fps_char="F";
                1:F_fps_char="P";
                2:F_fps_char="S";
                3:F_fps_char=":";4:F_fps_char=" ";
                5:F_fps_char=bcd[11:8]==0 ? " " : 8'h30+bcd[11:8];
                6:F_fps_char=bcd[11:4]==0 ? " " : 8'h30+bcd[7:4];
                7:F_fps_char=8'h30+bcd[3:0];
                default:F_fps_char=" ";
            endcase
        end
    endfunction
    function [15:0] F_percent_row;
        input [3:0] row;
        begin
            case(row)
                1,2:F_percent_row=16'h6200;
                3,4:F_percent_row=16'h6400;
                5,6:F_percent_row=16'h0800;
                7,8:F_percent_row=16'h1000;
                9,10:F_percent_row=16'h2000;
                11,12:F_percent_row=16'h4c00;
                13,14:F_percent_row=16'h8c00;
                default:F_percent_row=16'h0000;
            endcase
        end
    endfunction
    function [15:0] F_slash_row;
        input [3:0] row;
        begin
            case(row)
                1,2:F_slash_row=16'h0200;
                3,4:F_slash_row=16'h0400;
                5,6:F_slash_row=16'h0800;
                7,8:F_slash_row=16'h1000;
                9,10:F_slash_row=16'h2000;
                11,12:F_slash_row=16'h4000;
                13,14:F_slash_row=16'h8000;
                default:F_slash_row=16'h0000;
            endcase
        end
    endfunction
    function [7:0] F_performance_char;
        input [1:0] line;
        input [4:0] index;
        begin
            F_performance_char=" ";
            case(line)
                0:case(index) // Four decimal digits per measured dimension.
                    0:F_performance_char="R";1:F_performance_char="E";2:F_performance_char="S";
                    3:F_performance_char=":";
                    4,5,6,7:F_performance_char=F_decimal_char(S_perf_width_digits,index-3,S_perf_valid[0]);
                    8:F_performance_char="X";
                    9,10,11,12:F_performance_char=F_decimal_char(S_perf_height_digits,index-8,S_perf_valid[0]);
                endcase
                1:case(index) // Truncate sub-1000 pixels/s: four whole MP/s digits and three decimals.
                    0:F_performance_char="P";1:F_performance_char="I";2:F_performance_char="X";
                    3:F_performance_char=":";
                    4,5,6,7:F_performance_char=S_perf_valid[2] ?
                        8'h30+S_perf_pixels_digits[(13-index)*4 +:4] : 8'h2d;
                    8:F_performance_char=".";
                    9,10,11:F_performance_char=S_perf_valid[2] ?
                        8'h30+S_perf_pixels_digits[(14-index)*4 +:4] : 8'h2d;
                    12:F_performance_char="M";13:F_performance_char="P";
                    14:F_performance_char="/";15:F_performance_char="S";
                endcase
                2:case(index)
                    0:F_performance_char="D";1:F_performance_char="R";2:F_performance_char="O";
                    3:F_performance_char="P";4:F_performance_char=":";
                    5,6,7:F_performance_char=F_decimal_char(S_perf_drop_digits,index-5,S_perf_valid[3]);
                    8:F_performance_char=".";
                    9,10:F_performance_char=F_decimal_char(S_perf_drop_digits,index-6,S_perf_valid[3]);
                    11:F_performance_char="%";
                endcase
            endcase
        end
    endfunction
    function [7:0] F_decimal_char;
        input [19:0] bcd;
        input [2:0] digit;
        input valid;
        begin
            F_decimal_char=valid ? 8'h30+bcd[(4-digit)*4 +:4] : 8'h2d;
        end
    endfunction
    function [7:0] F_threshold_digit;
        input [11:0] bcd;
        input [4:0] digit;
        begin F_threshold_digit=8'h30+bcd[(2-digit)*4 +:4];end
    endfunction
    function [7:0] F_threshold_char;
        input [2:0] line;
        input [4:0] index;
        begin
            F_threshold_char=" ";
            case(line)
                0:case(index)
                    0:F_threshold_char="B";1:F_threshold_char="I";2:F_threshold_char="N";3:F_threshold_char=":";
                    4:F_threshold_char=S_binary_auto_display ? "A":"M";
                    5:F_threshold_char=S_binary_auto_display ? "U":"A";
                    6:F_threshold_char=S_binary_auto_display ? "T":"N";
                    7:F_threshold_char=S_binary_auto_display ? "O":" ";
                    9:F_threshold_char="E";10,11:F_threshold_char="F";12:F_threshold_char=":";
                    13,14,15:F_threshold_char=F_threshold_digit(S_threshold_digits[23:12],index-13);
                endcase
                1:case(index)
                    0:F_threshold_char="B";1:F_threshold_char="I";2:F_threshold_char="N";
                    4:F_threshold_char="S";5:F_threshold_char="E";6:F_threshold_char="T";7:F_threshold_char=":";
                    8,9,10:F_threshold_char=F_threshold_digit(S_threshold_digits[11:0],index-8);
                endcase
                2:case(index)
                    0,12:F_threshold_char="S";1:F_threshold_char="O";2:F_threshold_char="B";
                    3:F_threshold_char="E";4:F_threshold_char="L";6:F_threshold_char="W";
                    7,13:F_threshold_char=":";
                    8,9,10:F_threshold_char=F_threshold_digit(S_threshold_digits[35:24],index-8);
                    14,15,16:F_threshold_char=F_threshold_digit(S_threshold_digits[47:36],index-14);
                endcase
                3:case(index)
                    0:F_threshold_char="C";1:F_threshold_char="A";2,9:F_threshold_char="R";
                    3:F_threshold_char="T";4,12:F_threshold_char=":";
                    5,6,7:F_threshold_char=F_threshold_digit(S_threshold_digits[59:48],index-5);
                    10:F_threshold_char="N";11:F_threshold_char="G";
                    13,14,15:F_threshold_char=F_threshold_digit(S_threshold_digits[71:60],index-13);
                endcase
                4:case(index)
                    0:F_threshold_char="M";1,4:F_threshold_char="O";2:F_threshold_char="T";
                    3:F_threshold_char="I";5:F_threshold_char="N";6:F_threshold_char=":";
                    7,8,9:F_threshold_char=F_threshold_digit(S_threshold_digits[83:72],index-7);
                endcase
            endcase
            if(THREE_THRESHOLD_ONLY && (line==4 || (line==3 && index>=9)))
                F_threshold_char=" ";
        end
    endfunction
    function [7:0] F_latency_char;
        input [4:0] index;
        begin
            F_latency_char=" ";
            case(index)
                0:F_latency_char="L";1:F_latency_char="A";2:F_latency_char="T";3:F_latency_char=":";
                5,6,7,8:F_latency_char=F_decimal_char(S_latency_decimal,index-5,S_latency_display_valid);
                9:F_latency_char=".";
                10:F_latency_char=F_decimal_char(S_latency_decimal,4,S_latency_display_valid);
                11:F_latency_char="M";12:F_latency_char="S";
            endcase
        end
    endfunction
    function [7:0] F_source_char;
        input [4:0] index;
        begin
            F_source_char=" ";
            case(index)
                0:F_source_char="S";1:F_source_char="R";2:F_source_char="C";3:F_source_char=":";
                5,6,7,8,9:F_source_char=F_decimal_char(S_source_decimal,index-5,S_source_display_valid);
            endcase
        end
    endfunction
    function [7:0] F_coordinate_char;
        input [4:0] index;
        begin
            F_coordinate_char=" ";
            case(index)
                0:F_coordinate_char="X";1:F_coordinate_char=":";
                2,3,4,5:F_coordinate_char=F_decimal_char(S_coordinate_x_decimal,index-1,S_coordinate_display_valid);
                7:F_coordinate_char="Y";8:F_coordinate_char=":";
                9,10,11,12:F_coordinate_char=F_decimal_char(S_coordinate_y_decimal,index-8,S_coordinate_display_valid);
            endcase
        end
    endfunction
    function [7:0] F_motion_char;
        input [4:0] index;
        begin
            F_motion_char=" ";
            case(index)
                0:F_motion_char="C";1:F_motion_char="H";2:F_motion_char="A";3:F_motion_char="N";
                4:F_motion_char="G";5:F_motion_char="E";6:F_motion_char="D";7:F_motion_char=":";
                8,9,10,11,12:F_motion_char=F_decimal_char(S_motion_decimal,index-8,S_motion_display_valid);
                14:F_motion_char="E";15:F_motion_char="V";16:F_motion_char="E";17:F_motion_char="N";
                18:F_motion_char="T";19:F_motion_char="S";20:F_motion_char=":";
                21,22,23,24,25:F_motion_char=F_decimal_char(S_events_decimal,index-21,S_motion_display_valid);
            endcase
        end
    endfunction
    function [7:0] F_size_char;
        input [1:0] line;
        input [4:0] index;
        reg [19:0] length_digits,width_digits;
        reg valid;
        begin
            length_digits=S_size_digits[line*40 +:20];
            width_digits=S_size_digits[line*40+20 +:20];
            valid=S_size_display_valid[line];
            F_size_char=" ";
            case(index)
                0:case(line)0:F_size_char="R";1:F_size_char="G";2:F_size_char="B";endcase
                2:F_size_char="L";9:F_size_char="W";
                3,4,5:F_size_char=valid ? 8'h30+length_digits[(6-index)*4 +:4]:8'h2d;
                6,13:F_size_char=".";
                7:F_size_char=valid ? 8'h30+length_digits[3:0]:8'h2d;
                10,11,12:F_size_char=valid ? 8'h30+width_digits[(13-index)*4 +:4]:8'h2d;
                14:F_size_char=valid ? 8'h30+width_digits[3:0]:8'h2d;
                15,16:F_size_char="M";
            endcase
        end
    endfunction
    function [7:0] F_light_char;
        input [4:0] index;
        reg [87:0] text;
        begin
            text="LIGHTCHANGE";
            F_light_char=index<11 ? text[(10-index)*8 +:8] : 8'h20;
        end
    endfunction
    function [7:0] F_buffer_char;
        input [4:0] index;
        begin
            F_buffer_char=" ";
            case(index)
                0:F_buffer_char="B";1:F_buffer_char="U";2:F_buffer_char="F";3:F_buffer_char=":";
                5:begin
                    if(S_buffer_faults_display!=0) F_buffer_char="E";
                    else F_buffer_char=S_buffer_ready_display ? "O" : "W";
                end
                6:begin
                    if(S_buffer_faults_display!=0)
                        F_buffer_char=S_buffer_faults_display<10 ? 8'h30+S_buffer_faults_display : 8'h41+S_buffer_faults_display-10;
                    else F_buffer_char=S_buffer_ready_display ? "K" : "A";
                end
                7:if(S_buffer_faults_display==0 && !S_buffer_ready_display) F_buffer_char="I";
                8:if(S_buffer_faults_display==0 && !S_buffer_ready_display) F_buffer_char="T";
            endcase
        end
    endfunction
    function [23:0] F_color_rgb;
        input [1:0] color_index;
        begin
            case(color_index)
                0:F_color_rgb=24'hff4040;
                1:F_color_rgb=24'h40ff80;
                default:F_color_rgb=24'h4080ff;
            endcase
        end
    endfunction
    function [7:0] F_status_char;
        input [4:0] index;
        input [2:0] mode;
        reg [87:0] name;
        begin
            case(mode)
                0:name="ORIGINAL   "; 1:name="GRAYSCALE  "; 2:name="BINARY     ";
                3:name="SOBEL      "; 4:name="CARTOON    "; 5:name="COLOR TRACK";
                6:name="MOTION     ";
                default:name="ORIGINAL   ";
            endcase
            case(index)
                0:F_status_char="A"; 1:F_status_char="L"; 2:F_status_char="G";
                3:F_status_char="O"; 4:F_status_char="R"; 5:F_status_char="I";
                6:F_status_char="T"; 7:F_status_char="H"; 8:F_status_char="M";
                9:F_status_char=":"; 10:F_status_char=" ";
                11:F_status_char=name[87:80]; 12:F_status_char=name[79:72];
                13:F_status_char=name[71:64]; 14:F_status_char=name[63:56];
                15:F_status_char=name[55:48]; 16:F_status_char=name[47:40];
                17:F_status_char=name[39:32]; 18:F_status_char=name[31:24];
                19:F_status_char=name[23:16]; 20:F_status_char=name[15:8]; 21:F_status_char=name[7:0];
                default:F_status_char=" ";
            endcase
        end
    endfunction
    function [7:0] F_blend_channel;
        input [7:0] background, foreground;
        input [7:0] alpha;
        reg signed [8:0] delta;
        reg signed [17:0] product, weighted;
        reg [17:0] folded;
        begin
            // Rounded (foreground*alpha + background*(255-alpha))/255.
            // One signed9x9 multiply per channel; divide255 is a byte fold.
            // For weighted in0..65152, (n+1+(n>>8))>>8 is exact floor(n/255).
            delta=$signed({1'b0,foreground})-$signed({1'b0,background});
            product=delta*$signed({1'b0,alpha});
            weighted=$signed({1'b0,background,8'b0})-$signed({9'd0,background})+
                     product+18'sd127;
            folded=weighted+18'd1+{10'd0,weighted[15:8]};
            F_blend_channel=folded[15:8];
        end
    endfunction
    // SOF can be blank or on active pixel (0,0). The next active pixel is (1,0).
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin S_x<=0; S_y<=0; end
        else if(I_video_user) begin
            if(I_video_de) begin
                S_x<=I_video_last ? 12'd0 : 12'd1;
                S_y<=I_video_last ? 12'd1 : 12'd0;
            end else begin S_x<=0; S_y<=0; end
        end else if(I_video_de) begin
            if(I_video_last) begin S_x<=0; S_y<=S_y+12'd1; end
            else S_x<=S_x+12'd1;
        end
    end
    // BCD carry has no pixel-rate decimal division.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin S_fps_display<=0;S_servo_display<=0;S_servo_motion_display<=0;end
        else if(I_video_user) begin
            if(FPS_ENABLE) S_fps_display<=I_fps_bcd;
            if(SERVO_ENABLE) S_servo_display<=I_servo_state;
            S_servo_motion_display<=MOTION_SERVO_ENABLE && I_style_mode==6;
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_cnt<=0; S_cnt_d0<=0; S_cnt_d1<=0; S_cnt_d2<=0;
            S_cnt_d3<=0; S_cnt_d4<=0; S_cnt_d5<=0;
        end else if(I_video_user) begin
            S_frame_cnt<=S_frame_cnt+32'd1;
            if(S_cnt_d0!=9) S_cnt_d0<=S_cnt_d0+4'd1;
            else begin
                S_cnt_d0<=0;
                if(S_cnt_d1!=9) S_cnt_d1<=S_cnt_d1+4'd1;
                else begin
                    S_cnt_d1<=0;
                    if(S_cnt_d2!=9) S_cnt_d2<=S_cnt_d2+4'd1;
                    else begin
                        S_cnt_d2<=0;
                        if(S_cnt_d3!=9) S_cnt_d3<=S_cnt_d3+4'd1;
                        else begin
                            S_cnt_d3<=0;
                            if(S_cnt_d4!=9) S_cnt_d4<=S_cnt_d4+4'd1;
                            else begin
                                S_cnt_d4<=0; S_cnt_d5<=S_cnt_d5==9 ? 4'd0 : S_cnt_d5+4'd1;
                            end
                        end
                    end
                end
            end
        end
    end
    // Glyph selection is registered before font lookup and bit selection.
    always @(*) begin
        S_char=8'h20; S_row=0; S_col=0; S_panel=0; S_text_color=24'hffffff;
        S_local_x=0; S_local_y=0; S_char_index=0;
        S_target_x=0; S_target_y=0; S_target_w=0; S_box_left=0; S_box_top=0;
        S_motion_box=0;
        if(I_video_de) begin
            if(IMG_WIDTH>=CENTER_W && IMG_HEIGHT>=CENTER_H &&
               S_pixel_x>=CENTER_X && S_pixel_x<CENTER_X+CENTER_W &&
               S_pixel_y>=CENTER_Y && S_pixel_y<CENTER_Y+CENTER_H) begin
                S_panel=1; S_text_color=CENTER_COLOR;
                S_local_x=S_pixel_x-CENTER_X; S_local_y=S_pixel_y-CENTER_Y;
                S_row=S_local_y[4:1];
                if(S_local_x>=16 && S_local_x<80) begin
                    S_char=S_local_x<48 ? 8'h80 : 8'h81;
                    S_local_x=S_local_x-12'd16; S_col=S_local_x[4:1];
                    // Keep the original Chinese bitmap exactly. Its 11 ink
                    // rows are vertically centered inside the 32-pixel group.
                    if(S_local_y>=5 && S_local_y<27) begin
                        S_local_y=S_local_y-12'd5;
                        S_row={1'b0,S_local_y[4:1]};
                    end else S_char=" ";
                end else if(S_local_x>=96 && S_local_x<160) begin
                    S_local_x=S_local_x-12'd96;
                    case(S_local_x[5:4]) 0:S_char="C"; 1:S_char="N"; 2:S_char="T"; 3:S_char=":"; endcase
                    S_col={1'b0,S_local_x[3:1]};
                end else if(S_local_x>=176 && S_local_x<272) begin
                    S_local_x=S_local_x-12'd176;
                    case(S_local_x[6:4])
                        0:S_char=8'h30+S_cnt_d5; 1:S_char=8'h30+S_cnt_d4;
                        2:S_char=8'h30+S_cnt_d3; 3:S_char=8'h30+S_cnt_d2;
                        4:S_char=8'h30+S_cnt_d1; 5:S_char=8'h30+S_cnt_d0;
                        default:S_char=" ";
                    endcase
                    S_col={1'b0,S_local_x[3:1]};
                end
            end
            if(IMG_HEIGHT>=40 && S_pixel_x>=STATUS_X && S_pixel_x<STATUS_X+400 &&
               S_pixel_y>=STATUS_Y && S_pixel_y<STATUS_Y+32) begin
                S_panel=1; S_text_color=24'hffffff;
                S_local_x=S_pixel_x-STATUS_X-12'd8; S_local_y=S_pixel_y-STATUS_Y;
                S_char_index=S_local_x[8:4]; S_char=F_status_char(S_char_index,I_style_mode);
                S_row=S_local_y[4:1]; S_col={1'b0,S_local_x[3:1]};
                if(S_pixel_x<STATUS_X+8) S_char=" ";
            end
            if(I_style_mode==5) begin
                // This legend remains visible when no objects are detected.
                if(IMG_HEIGHT>=80 && S_pixel_x>=16 && S_pixel_x<304 &&
                   S_pixel_y>=LEGEND_Y && S_pixel_y<LEGEND_Y+32) begin
                    S_panel=1; S_char=" ";
                    S_local_x=S_pixel_x-12'd24; S_local_y=S_pixel_y-LEGEND_Y;
                    S_row=S_local_y[4:1]; S_col={1'b0,S_local_x[3:1]};
                    if(S_pixel_x>=24 && S_pixel_x<72) begin
                        S_char=F_color_char(0,S_local_x[7:4]); S_text_color=F_color_rgb(0);
                    end else if(S_pixel_x>=88 && S_pixel_x<168) begin
                        S_local_x=S_pixel_x-12'd88;
                        S_char=F_color_char(1,S_local_x[7:4]); S_text_color=F_color_rgb(1);
                    end else if(S_pixel_x>=184 && S_pixel_x<248) begin
                        S_local_x=S_pixel_x-12'd184;
                        S_char=F_color_char(2,S_local_x[7:4]); S_text_color=F_color_rgb(2);
                    end
                    S_col={1'b0,S_local_x[3:1]};
                end
                // Tracker outputs are stable for the frame. Direct reads allow
                // first-SOF NBA updates without delaying captions by one frame.
                for(k=0;k<3;k=k+1) begin
                    S_box_left=I_targets_left[k*12 +: 12]; S_box_top=I_targets_top[k*12 +: 12];
                    case(k) 0:S_target_w=32; 1:S_target_w=48; default:S_target_w=40; endcase
                    S_target_x=S_box_left>IMG_WIDTH-S_target_w ? IMG_WIDTH-S_target_w : S_box_left;
                    S_target_y=S_box_top>=26 ? S_box_top-12'd26 : S_box_top+12'd4;
                    if(S_target_y>IMG_HEIGHT-24) S_target_y=IMG_HEIGHT-24;
                    if(I_targets_valid[k] && IMG_WIDTH>=48 && IMG_HEIGHT>=24 &&
                       S_pixel_x>=S_target_x && S_pixel_x<S_target_x+S_target_w &&
                       S_pixel_y>=S_target_y && S_pixel_y<S_target_y+24) begin
                        S_panel=1; S_text_color=F_color_rgb(k);
                        S_local_x=S_pixel_x-S_target_x-12'd4; S_local_y=S_pixel_y-S_target_y-12'd4;
                        S_char=F_color_char(k,S_local_x[6:3]);
                        S_col={1'b0,S_local_x[2:0]}; S_row=S_local_y[3:0];
                        if(S_pixel_x<S_target_x+4 || S_pixel_x>=S_target_x+S_target_w-4 ||
                           S_pixel_y<S_target_y+4 || S_pixel_y>=S_target_y+20) S_char=" ";
                    end
                end
            end
            if(SERVO_ENABLE && (!CONTEXT_DISPLAY || I_style_mode==5 ||
               (MOTION_SERVO_ENABLE && I_style_mode==6)) && IMG_WIDTH>=SERVO_W+32 && IMG_HEIGHT>=SERVO_Y+32 &&
               S_pixel_x>=SERVO_X && S_pixel_x<SERVO_X+SERVO_W &&
               S_pixel_y>=SERVO_Y && S_pixel_y<SERVO_Y+32) begin
                S_panel=1;
                case(S_servo_display)
                    0:S_text_color=24'hffffff;1:S_text_color=24'hffc040;
                    2:S_text_color=S_servo_motion_display ? 24'hffff00:F_color_rgb(SERVO_COLOR);default:S_text_color=24'h40ff80;
                endcase
                S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-SERVO_Y;
                S_char=F_servo_char(S_local_x[8:4],S_servo_display,S_servo_motion_display);
                S_col={1'b0,S_local_x[3:1]};S_row=S_local_y[4:1];
                if(S_pixel_x<SERVO_X+8) S_char=" ";
            end
            if(SIZE_ENABLE && I_style_mode==5 && IMG_WIDTH>=896 && IMG_HEIGHT>=SIZE_Y+SIZE_H &&
               S_pixel_x>=SERVO_X && S_pixel_x<SERVO_X+SERVO_W &&
               S_pixel_y>=SIZE_Y && S_pixel_y<SIZE_Y+SIZE_H)begin
                S_panel=1;S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-SIZE_Y;
                S_text_color=F_color_rgb(S_local_y[6:5]);
                S_char=F_size_char(S_local_y[6:5],S_local_x[8:4]);
                S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                if(S_pixel_x<SERVO_X+8)S_char=" ";
            end
            // 896 pixels keeps these panels to the right of the centered mark.
            if(TELEMETRY_ENABLE && IMG_WIDTH>=896 && IMG_HEIGHT>=COORDINATE_Y+32 &&
               S_pixel_x>=SERVO_X && S_pixel_x<SERVO_X+SERVO_W) begin
                if(LATENCY_ENABLE && (!CONTEXT_DISPLAY || I_style_mode==0) && S_pixel_y>=LATENCY_Y && S_pixel_y<LATENCY_Y+32) begin
                    S_panel=1;S_text_color=24'hc8e4ff;
                    S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-LATENCY_Y;
                    S_char=F_latency_char(S_local_x[8:4]);S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                    if(S_pixel_x<SERVO_X+8) S_char=" ";
                end
                if(!CONTEXT_DISPLAY && S_pixel_y>=SOURCE_Y && S_pixel_y<SOURCE_Y+32) begin
                    S_panel=1;S_text_color=24'hc8e4ff;
                    S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-SOURCE_Y;
                    S_char=F_source_char(S_local_x[8:4]);S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                    if(S_pixel_x<SERVO_X+8) S_char=" ";
                end
                if(I_style_mode==5 && S_pixel_y>=COORDINATE_Y && S_pixel_y<COORDINATE_Y+32) begin
                    S_panel=1;S_text_color=F_color_rgb(SERVO_COLOR);
                    S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-COORDINATE_Y;
                    S_char=F_coordinate_char(S_local_x[8:4]);S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                    if(S_pixel_x<SERVO_X+8) S_char=" ";
                end
            end
            if(MOTION_ENABLE && I_style_mode==6) begin
                if(I_motion_valid && !I_motion_scene_change && !I_video_user &&
                   I_motion_left<=I_motion_right && I_motion_top<=I_motion_bottom &&
                   S_pixel_x>=I_motion_left && S_pixel_x<=I_motion_right &&
                   S_pixel_y>=I_motion_top && S_pixel_y<=I_motion_bottom) begin
                    S_motion_box=(S_pixel_x-I_motion_left<2 || I_motion_right-S_pixel_x<2 ||
                                  S_pixel_y-I_motion_top<2 || I_motion_bottom-S_pixel_y<2);
                end
                if(IMG_WIDTH>=MOTION_LEGEND_W+32 && IMG_HEIGHT>=80 &&
                   S_pixel_x>=16 && S_pixel_x<16+MOTION_LEGEND_W &&
                   S_pixel_y>=LEGEND_Y && S_pixel_y<LEGEND_Y+32) begin
                    S_panel=1;S_text_color=24'hffe04a;
                    S_local_x=S_pixel_x-12'd24;S_local_y=S_pixel_y-LEGEND_Y;
                    S_char=F_motion_char(S_local_x[8:4]);S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                    if(S_pixel_x<24) S_char=" ";
                end
                if(S_motion_scene_display && IMG_WIDTH>=896 && IMG_HEIGHT>=COORDINATE_Y+32 &&
                   S_pixel_x>=SERVO_X && S_pixel_x<SERVO_X+SERVO_W &&
                   S_pixel_y>=COORDINATE_Y && S_pixel_y<COORDINATE_Y+32) begin
                    S_panel=1;S_text_color=24'hffc040;
                    S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-COORDINATE_Y;
                    S_char=F_light_char(S_local_x[8:4]);S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                    if(S_pixel_x<SERVO_X+8) S_char=" ";
                end
            end
            if(BUFFER_HEALTH_ENABLE && !CONTEXT_DISPLAY && IMG_WIDTH>=896 && IMG_HEIGHT>=BUFFER_Y+32 &&
               S_pixel_x>=SERVO_X && S_pixel_x<SERVO_X+SERVO_W &&
               S_pixel_y>=BUFFER_Y && S_pixel_y<BUFFER_Y+32) begin
                S_panel=1;
                S_text_color=S_buffer_faults_display!=0 ? 24'hff6060 :
                             (S_buffer_ready_display ? 24'h60ffb0 : 24'hc8e4ff);
                S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-BUFFER_Y;
                S_char=F_buffer_char(S_local_x[8:4]);S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                if(S_pixel_x<SERVO_X+8) S_char=" ";
            end
            if(THRESHOLD_ENABLE && (!CONTEXT_DISPLAY ||
               (I_style_mode>=2 && I_style_mode<=4)) && IMG_WIDTH>=896 && IMG_HEIGHT>=THRESHOLD_Y+THRESHOLD_H &&
               S_pixel_x>=SERVO_X && S_pixel_x<SERVO_X+SERVO_W &&
               S_pixel_y>=THRESHOLD_Y && S_pixel_y<THRESHOLD_Y+THRESHOLD_H) begin
                S_panel=1;S_text_color=24'hb8f4ff;
                S_local_x=S_pixel_x-SERVO_X-12'd8;S_local_y=S_pixel_y-THRESHOLD_Y;
                if(CONTEXT_DISPLAY) begin
                    if(I_style_mode==2 && S_local_y<64)
                        S_char=F_threshold_char(S_local_y[7:5],S_local_x[8:4]);
                    else if(I_style_mode==3 && S_local_y<32)
                        S_char=F_threshold_char(3'd2,S_local_x[8:4]);
                    else if(I_style_mode==4 && S_local_y<32)
                        S_char=F_threshold_char(3'd3,S_local_x[8:4]);
                    else begin S_char=" ";S_panel=0;end
                end else S_char=F_threshold_char(S_local_y[7:5],S_local_x[8:4]);
                S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                if(S_pixel_x<SERVO_X+8 || S_pixel_x>=SERVO_X+SERVO_W-8) S_char=" ";
            end
            if(PERFORMANCE_ENABLE && IMG_WIDTH>=PERFORMANCE_W+32 &&
               IMG_HEIGHT>=PERFORMANCE_Y+PERFORMANCE_H &&
               S_pixel_x>=PERFORMANCE_X && S_pixel_x<PERFORMANCE_X+PERFORMANCE_W &&
               S_pixel_y>=PERFORMANCE_Y && S_pixel_y<PERFORMANCE_Y+PERFORMANCE_H) begin
                S_panel=1;S_text_color=24'hb8f4ff;
                S_local_x=S_pixel_x-PERFORMANCE_X-12'd8;S_local_y=S_pixel_y-PERFORMANCE_Y;
                S_char=F_performance_char(S_local_y[6:5],S_local_x[8:4]);
                S_row=S_local_y[4:1];S_col={1'b0,S_local_x[3:1]};
                if(S_pixel_x<PERFORMANCE_X+8 || S_pixel_x>=PERFORMANCE_X+PERFORMANCE_W-8) S_char=" ";
            end
            // Reserve the upper-right panel even when a target label crosses
            // it. Frame-boundary latching prevents digit tearing.
            if(FPS_ENABLE && IMG_WIDTH>=FPS_W+32 && IMG_HEIGHT>=FPS_Y+32 &&
               S_pixel_x>=FPS_X && S_pixel_x<FPS_X+FPS_W &&
               S_pixel_y>=FPS_Y && S_pixel_y<FPS_Y+32) begin
                S_panel=1;S_text_color=24'hffffff;
                S_local_x=S_pixel_x-FPS_X-12'd8;S_local_y=S_pixel_y-FPS_Y;
                S_char=F_fps_char(S_local_x[8:4],S_fps_display);
                S_col={1'b0,S_local_x[3:1]};S_row=S_local_y[4:1];
                if(S_pixel_x<FPS_X+8) S_char=" ";
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d<=0; S_x_2d<=0; S_video_data_1d<=0; S_video_data_2d<=0;
            S_video_vsync_1d<=0; S_video_vsync_2d<=0; S_video_hsync_1d<=0; S_video_hsync_2d<=0;
            S_video_de_1d<=0; S_video_de_2d<=0; S_logo_window_1d<=0; S_logo_window_2d<=0;
            S_char_1d<=8'h20; S_row_1d<=0; S_col_1d<=0; S_panel_1d<=0; S_panel_2d<=0;
            S_text_on_2d<=0; S_text_color_1d<=0; S_text_color_2d<=0;
            S_motion_box_1d<=0;S_motion_box_2d<=0;
            O_hdmi_vsync<=0; O_hdmi_hsync<=0; O_hdmi_de<=0; O_hdmi_data<=0;
        end else begin
            S_x_1d<=S_pixel_x; S_x_2d<=S_x_1d;
            S_video_data_1d<=I_video_rd_data; S_video_data_2d<=S_video_data_1d;
            S_video_vsync_1d<=I_video_vsync; S_video_vsync_2d<=S_video_vsync_1d;
            S_video_hsync_1d<=I_video_hsync; S_video_hsync_2d<=S_video_hsync_1d;
            S_video_de_1d<=I_video_de; S_video_de_2d<=S_video_de_1d;
            S_logo_window_1d<=I_video_de && S_pixel_x<LOGO_W && S_pixel_y<LOGO_H;
            S_logo_window_2d<=S_logo_window_1d;
            S_char_1d<=S_char; S_row_1d<=S_row; S_col_1d<=S_col;
            S_panel_1d<=S_panel; S_panel_2d<=S_panel_1d;
            S_motion_box_1d<=S_motion_box;S_motion_box_2d<=S_motion_box_1d;
            S_text_color_1d<=S_text_color; S_text_color_2d<=S_text_color_1d;
            S_text_on_2d<=S_glyph_bits[15-S_col_1d];
            O_hdmi_vsync<=S_video_vsync_2d; O_hdmi_hsync<=S_video_hsync_2d; O_hdmi_de<=S_video_de_2d;
            if(!S_video_de_2d) O_hdmi_data<=DEBUG_MODE==0 ? S_video_data_2d : 24'd0;
            else if(DEBUG_MODE==1) begin
                case(S_x_2d[11:7])
                    0:O_hdmi_data<=24'hff0000; 1:O_hdmi_data<=24'h00ff00; 2:O_hdmi_data<=24'h0000ff;
                    3:O_hdmi_data<=24'hffff00; 4:O_hdmi_data<=24'h00ffff; 5:O_hdmi_data<=24'hff00ff;
                    6:O_hdmi_data<=24'hffffff; default:O_hdmi_data<=24'h202020;
                endcase
            end else if(DEBUG_MODE==2) begin
                case(I_debug_status)
                    0:O_hdmi_data<=0; 1:O_hdmi_data<=24'hff0000; 3:O_hdmi_data<=24'hffff00;
                    7:O_hdmi_data<=24'h00ff00; 15:O_hdmi_data<=S_video_data_2d; default:O_hdmi_data<=24'h0000ff;
                endcase
            end else if(DEBUG_MODE==3) begin
                if(S_x_2d<256) O_hdmi_data<=I_debug_status[0] ? 24'hff8000 : 24'h201000;
                else if(S_x_2d<512) O_hdmi_data<=I_debug_status[1] ? 24'hffff00 : 24'h202000;
                else if(S_x_2d<768) O_hdmi_data<=I_debug_status[2] ? 24'hff0000 : 24'h200000;
                else O_hdmi_data<=I_debug_status[3] ? 24'h00ffff : 24'h002020;
            // Only glyph ink covers the picture; spaces and padding stay
            // transparent to video, the logo and the motion-box overlay.
            end else if(S_panel_2d && S_text_on_2d) O_hdmi_data<=S_text_color_2d;
            else if(S_logo_window_2d) begin
                O_hdmi_data<={F_blend_channel(S_video_data_2d[23:16],S_logo_color[23:16],S_logo_alpha),
                             F_blend_channel(S_video_data_2d[15:8],S_logo_color[15:8],S_logo_alpha),
                             F_blend_channel(S_video_data_2d[7:0],S_logo_color[7:0],S_logo_alpha)};
            end else if(S_motion_box_2d) O_hdmi_data<=24'hffe04a;
            else O_hdmi_data<=S_video_data_2d;
        end
    end
endmodule

// Five values share one 32-clock binary-to-BCD converter. No pixel-rate
// divide/multiply is needed, including MP/s: omit the bottom three BCD digits.
// Snapshot one clock after SOF to see same-edge meter updates; commit all
// digits/valid bits atomically after 160 clocks, before raster row400.
module osd_performance_snapshot(
    input wire I_clk,I_rst_n,I_sof,
    input wire [3:0] I_valid,
    input wire [15:0] I_width,I_height,I_fps_x100,I_drop_x100,
    input wire [31:0] I_pixels_per_second,
    output reg [3:0] O_valid,
    output reg [19:0] O_width,O_height,O_fps,O_drop,
    output reg [39:0] O_pixels
);
    reg pending,busy;
    reg [2:0] field;
    reg [4:0] count;
    reg [3:0] valid;
    reg [159:0] values;
    reg [31:0] binary;
    reg [39:0] bcd,adjusted;
    reg [19:0] width_digits,height_digits,fps_digits;
    reg [39:0] pixels_digits;
    wire [39:0] shifted={adjusted[38:0],binary[31]};
    integer digit;
    always @(*) begin
        adjusted=bcd;
        for(digit=0;digit<10;digit=digit+1)
            if(bcd[digit*4 +:4]>=5) adjusted[digit*4 +:4]=bcd[digit*4 +:4]+4'd3;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            pending<=0;busy<=0;field<=0;count<=0;valid<=0;values<=0;binary<=0;bcd<=0;
            width_digits<=0;height_digits<=0;fps_digits<=0;pixels_digits<=0;
            O_valid<=0;O_width<=0;O_height<=0;O_fps<=0;O_pixels<=0;O_drop<=0;
        end else begin
            pending<=I_sof;
            if(pending) begin
                // A new SOF also restarts an unfinished conversion after a
                // malformed/abnormally short frame; stale fields never mix.
                busy<=1;field<=0;count<=0;binary<={16'd0,I_width};bcd<=0;
                values<={16'd0,I_drop_x100,I_pixels_per_second,16'd0,I_fps_x100,
                         16'd0,I_height,16'd0,I_width};
                valid<={I_valid[3] && I_drop_x100<=16'd10000,I_valid[2:1],
                        I_valid[0] && I_width!=0 && I_height!=0 &&
                        I_width<=16'd9999 && I_height<=16'd9999};
            end else if(busy) begin
                binary<={binary[30:0],1'b0};bcd<=shifted;
                if(count==31) begin
                    case(field)
                        0:width_digits<=shifted[19:0];
                        1:height_digits<=shifted[19:0];
                        2:fps_digits<=shifted[19:0];
                        3:pixels_digits<=shifted;
                        4:begin
                            O_width<=width_digits;O_height<=height_digits;O_fps<=fps_digits;
                            O_pixels<=pixels_digits;O_drop<=shifted[19:0];O_valid<=valid;
                        end
                    endcase
                    if(field==4) busy<=0;
                    else begin
                        field<=field+1'b1;binary<=values[(field+1'b1)*32 +:32];bcd<=0;
                    end
                    count<=0;
                end else count<=count+1'b1;
            end
        end
    end
endmodule

// Seven 8-bit values share one eight-clock double-dabble converter. Conversion
// finishes in57 clocks, before the threshold panel beginning on raster row224.
// Capture one clock after SOF so upstream frame-boundary NBA commits are seen.
module osd_threshold_snapshot #(parameter integer FIELDS=7)(
    input wire I_clk,I_rst_n,I_sof,I_auto,
    input wire [55:0] I_values,
    output reg O_auto,
    output reg [83:0] O_digits
);
    reg pending,busy;
    reg [2:0] field,count;
    reg [FIELDS*8-1:0] values;
    reg [7:0] binary;
    reg [11:0] bcd,adjusted;
    wire [11:0] shifted={adjusted[10:0],binary[7]};
    integer digit;
    always @(*) begin
        adjusted=bcd;
        for(digit=0;digit<3;digit=digit+1)
            if(bcd[digit*4 +:4]>=5) adjusted[digit*4 +:4]=bcd[digit*4 +:4]+4'd3;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            pending<=0;busy<=0;field<=0;count<=0;values<=0;
            binary<=0;bcd<=0;O_digits<=0;O_auto<=1;
        end else begin
            if(I_sof) pending<=1;
            if(pending && !busy) begin
                pending<=0;busy<=1;field<=0;count<=0;values<=I_values[FIELDS*8-1:0];
                binary<=I_values[7:0];bcd<=0;O_auto<=I_auto;
            end else if(busy) begin
                binary<={binary[6:0],1'b0};bcd<=shifted;
                if(count==7) begin
                    O_digits[field*12 +:12]<=shifted;
                    if(field==FIELDS-1) busy<=0;
                    else begin field<=field+1'b1;binary<=values[(field+1'b1)*8 +:8];bcd<=0;end
                    count<=0;
                end else count<=count+1'b1;
            end
        end
    end
endmodule
