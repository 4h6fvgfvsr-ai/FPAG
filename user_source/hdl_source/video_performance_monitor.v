`ifndef VIDEO_PERFORMANCE_MONITOR_V
`define VIDEO_PERFORMANCE_MONITOR_V
// Passive processed-display telemetry, one active pixel per I_de clock.
// All inputs are synchronous to I_clk. CLK_HZ is the actual pixel clock:
// this project's HDMI PLL is nominally 75 MHz, not 74.25 MHz. Its timing
// report rounds 13.333333 ns to 13.333 ns; that does not change the clock.
// FPS includes repeated display frames. PPS counts actual DE transfers in an
// exact one-second window. Neither metric claims unique camera throughput.
// Drop percentage measures missing capture IDs BETWEEN observed valid delivered
// IDs. Startup/in-flight frames and intervals with invalid metadata are excluded.
// Repeats are ignored; invalid/stale/backward observations start a new session.
module video_performance_monitor #(
    parameter integer CLK_HZ=75000000,
    parameter integer NO_SIGNAL_CYCLES=CLK_HZ
)(
    input wire I_clk,I_rst_n,
    input wire I_de,I_user,I_last,
    input wire I_source_update,I_source_valid,
    input wire [15:0] I_source_frame_id,
    input wire I_capture_fresh,
    output reg O_resolution_valid,
    output reg [15:0] O_width,O_height,
    output reg O_rate_valid,
    output reg [15:0] O_fps_x100,
    output reg O_pixels_valid,
    output reg [31:0] O_pixels_per_second,
    output reg O_drop_valid,
    output reg [15:0] O_drop_x100,
    output reg [31:0] O_expected_frames,O_dropped_frames
);
    localparam [47:0] FPS_NUMERATOR=48'd100*CLK_HZ;
    reg [2:0] reset_release; // synthesis keep
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n) reset_release<=0;
        else reset_release<={reset_release[1:0],1'b1};
    wire reset_n=I_rst_n && reset_release[2];
    wire sof=I_de && I_user;

    reg have_sof,frame_active,frame_bad;
    reg [31:0] sof_age,window_cycles,window_pixels;
    reg [15:0] line_pixels,frame_width,frame_lines;
    reg fps_pending;
    reg [31:0] fps_period,fps_epoch;
    reg source_baseline,drop_pending;
    reg [15:0] previous_source_id;
    reg [31:0] drop_epoch,source_age;
    wire [15:0] source_delta=I_source_frame_id-previous_source_id;
    wire [32:0] next_expected={1'b0,O_expected_frames}+{17'd0,source_delta};

    // A single shared restoring divider uses one 33-bit subtraction per clock.
    // Latest pending samples are retained if updates arrive while it is busy.
    reg div_busy,div_is_drop;
    reg [5:0] div_step;
    reg [47:0] div_numerator,div_quotient;
    reg [31:0] div_denominator,div_epoch;
    reg [32:0] div_remainder;
    wire [32:0] trial_remainder={div_remainder[31:0],div_numerator[47]};
    wire quotient_bit=trial_remainder>={1'b0,div_denominator};
    wire [47:0] next_quotient={div_quotient[46:0],quotient_bit};

    always @(posedge I_clk or negedge reset_n) begin
        if(!reset_n) begin
            O_resolution_valid<=0;O_width<=0;O_height<=0;
            O_rate_valid<=0;O_fps_x100<=0;
            O_pixels_valid<=0;O_pixels_per_second<=0;
            O_drop_valid<=0;O_drop_x100<=0;
            O_expected_frames<=0;O_dropped_frames<=0;
            have_sof<=0;frame_active<=0;frame_bad<=0;sof_age<=0;
            window_cycles<=0;window_pixels<=0;
            line_pixels<=0;frame_width<=0;frame_lines<=0;
            fps_pending<=0;fps_period<=0;fps_epoch<=0;
            source_baseline<=0;drop_pending<=0;previous_source_id<=0;drop_epoch<=0;source_age<=0;
            div_busy<=0;div_is_drop<=0;div_step<=0;
            div_numerator<=0;div_quotient<=0;div_denominator<=0;
            div_epoch<=0;div_remainder<=0;
        end else begin
            if(div_busy) begin
                div_numerator<={div_numerator[46:0],1'b0};
                div_quotient<=next_quotient;
                div_remainder<=quotient_bit ?
                    trial_remainder-{1'b0,div_denominator}:trial_remainder;
                div_step<=div_step+1'b1;
                if(div_step==47) begin
                    div_busy<=0;
                    if(div_is_drop) begin
                        if(div_epoch==drop_epoch && source_baseline && I_capture_fresh) begin
                            O_drop_x100<=next_quotient[15:0];
                            O_drop_valid<=1;
                        end
                    end else if(div_epoch==fps_epoch && have_sof) begin
                        O_fps_x100<=next_quotient>48'd65535 ? 16'hffff:next_quotient[15:0];
                        O_rate_valid<=1;
                    end
                end
            end else if(drop_pending && source_baseline) begin
                div_busy<=1;div_is_drop<=1;div_step<=0;
                div_numerator<=48'd10000*O_dropped_frames;
                div_denominator<=O_expected_frames;
                div_quotient<=0;div_remainder<=0;div_epoch<=drop_epoch;
                drop_pending<=0;
            end else if(fps_pending && have_sof) begin
                div_busy<=1;div_is_drop<=0;div_step<=0;
                div_numerator<=FPS_NUMERATOR+{17'd0,fps_period[31:1]};
                div_denominator<=fps_period;
                div_quotient<=0;div_remainder<=0;div_epoch<=fps_epoch;
                fps_pending<=0;
            end

            // PPS uses active transfers, including pixels from repeated frames.
            // Start a fresh full window when display SOF first becomes available.
            if(!have_sof) begin
                window_cycles<=sof ? 32'd1:32'd0;
                window_pixels<=sof ? 32'd1:32'd0;
            end else if(window_cycles==CLK_HZ-1) begin
                O_pixels_per_second<=window_pixels+{31'd0,I_de};
                O_pixels_valid<=1;window_cycles<=0;window_pixels<=0;
            end else begin
                window_cycles<=window_cycles+1'b1;
                window_pixels<=window_pixels+{31'd0,I_de};
            end

            if(sof) begin
                if(have_sof) begin fps_pending<=1;fps_period<=sof_age+32'd1;end
                have_sof<=1;sof_age<=0;
                // I_user brackets complete frames, while I_last ends each row.
                // Do not report a resolution from a partial startup frame.
                O_resolution_valid<=frame_active && !frame_bad &&
                    line_pixels==0 && frame_lines!=0 && frame_width!=0;
                if(frame_active && !frame_bad && line_pixels==0 &&
                   frame_lines!=0 && frame_width!=0) begin
                    O_width<=frame_width;O_height<=frame_lines;
                end
                frame_active<=1;frame_bad<=0;
                line_pixels<=I_last ? 16'd0:16'd1;
                frame_width<=I_last ? 16'd1:16'd0;
                frame_lines<=I_last ? 16'd1:16'd0;
            end else begin
                if(have_sof) begin
                    if(sof_age==NO_SIGNAL_CYCLES-1) begin
                        have_sof<=0;frame_active<=0;frame_bad<=0;
                        O_resolution_valid<=0;O_rate_valid<=0;O_fps_x100<=0;
                        O_pixels_valid<=0;O_pixels_per_second<=0;
                        fps_pending<=0;fps_epoch<=fps_epoch+1'b1;
                        window_cycles<=0;window_pixels<=0;
                    end else sof_age<=sof_age+1'b1;
                end
                if(frame_active) begin
                    if(I_de) begin
                        if(line_pixels==16'hffff) frame_bad<=1;
                        else if(I_last) begin
                            if(frame_lines==0) frame_width<=line_pixels+16'd1;
                            else if(frame_width!=line_pixels+16'd1) frame_bad<=1;
                            if(frame_lines==16'hffff) frame_bad<=1;
                            else frame_lines<=frame_lines+1'b1;
                            line_pixels<=0;
                        end else line_pixels<=line_pixels+1'b1;
                    end
                    if((I_last || I_user) && !I_de) frame_bad<=1;
                end
            end

            // This interval is bounded by actually displayed valid source IDs:
            // e.g. 100,100,101,104 gives expected=4, dropped=2 => 50.00%.
            // Invalid metadata means the interval is unknown, not zero loss.
            if(!source_baseline || I_source_update) source_age<=0;
            else if(source_age<NO_SIGNAL_CYCLES-1) source_age<=source_age+1'b1;
            if(!I_capture_fresh || (I_source_update && !I_source_valid) ||
               (source_baseline && !I_source_update && source_age==NO_SIGNAL_CYCLES-1)) begin
                if(source_baseline) drop_epoch<=drop_epoch+1'b1;
                source_baseline<=0;drop_pending<=0;O_drop_valid<=0;source_age<=0;
                O_drop_x100<=0;O_expected_frames<=0;O_dropped_frames<=0;
            end else if(I_source_update) begin
                if(!source_baseline) begin
                    source_baseline<=1;previous_source_id<=I_source_frame_id;
                    O_drop_valid<=0;
                end else if(source_delta!=0) begin
                    previous_source_id<=I_source_frame_id;
                    // Half-range rule accepts wrap 65535->0 but cannot mistake
                    // a backward/out-of-order ID or counter overflow for loss.
                    if(source_delta[15] || next_expected[32]) begin
                        drop_epoch<=drop_epoch+1'b1;drop_pending<=0;
                        O_drop_valid<=0;O_drop_x100<=0;
                        O_expected_frames<=0;O_dropped_frames<=0;
                    end else begin
                        O_expected_frames<=next_expected[31:0];
                        O_dropped_frames<=O_dropped_frames+{16'd0,source_delta}-32'd1;
                        drop_pending<=1;
                    end
                end
            end
        end
    end
    // synthesis translate_off
    initial if(CLK_HZ<2 || NO_SIGNAL_CYCLES<2)
        $fatal(1,"Invalid video performance monitor clock/timeout parameters");
    // synthesis translate_on
endmodule
`endif
