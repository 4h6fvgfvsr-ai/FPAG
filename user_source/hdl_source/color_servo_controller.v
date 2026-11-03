`ifndef COLOR_SERVO_CONTROLLER_V
`define COLOR_SERVO_CONTROLLER_V

// Two positional SG90 channels; every input must be synchronous to I_clk.
// PWM is always present, including while tracking is disabled: the last pulse
// width is held. A disable, recenter or invalid update clears the stored target.
// Commands change only at the rising edge of a complete 20 ms PWM frame.
// A valid update on that edge is used from the following PWM frame. An invalid
// update on that edge immediately inhibits motion. Timeout also inhibits motion
// on its expiry edge. I_recenter takes priority over tracking and I_enable.
module color_servo_controller #(
    parameter integer CLK_HZ = 50000000,
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer PWM_PERIOD_US = 20000,
    parameter integer PAN_CENTER_US = 1500,
    parameter integer TILT_CENTER_US = 1500,
    parameter integer PAN_MIN_US = 1100,
    parameter integer PAN_MAX_US = 1900,
    parameter integer TILT_MIN_US = 1200,
    parameter integer TILT_MAX_US = 1800,
    parameter integer DEADZONE_X = 32,
    parameter integer DEADZONE_Y = 24,
    parameter integer STEP_SHIFT = 4,
    parameter integer MAX_STEP_US = 20,
    parameter integer TARGET_TIMEOUT_MS = 200,
    parameter integer PAN_REVERSE = 0,
    parameter integer TILT_REVERSE = 0
) (
    input wire I_clk,
    input wire I_rst_n,
    input wire I_enable,
    input wire I_recenter,
    input wire I_target_update,
    input wire I_target_valid,
    input wire [11:0] I_target_x,
    input wire [11:0] I_target_y,
    output reg O_pan_pwm,
    output reg O_tilt_pwm,
    output wire [11:0] O_pan_us,
    output wire [11:0] O_tilt_us,
    output wire O_tracking,
    output wire O_centered
);

    function integer bit_width;
        input integer count;
        integer value;
        begin
            value = count - 1;
            bit_width = 0;
            while (value > 0) begin
                bit_width = bit_width + 1;
                value = value >> 1;
            end
            if (bit_width < 1) bit_width = 1;
        end
    endfunction

    // Constant elaboration arithmetic only. No generated clocks, divider or
    // multiplier datapath. CLK_HZ must be an integer multiple of 1 MHz.
    localparam integer TICKS_PER_US = CLK_HZ / 1000000;
    localparam integer TICK_BITS = bit_width(TICKS_PER_US);
    localparam integer PERIOD_BITS = bit_width(PWM_PERIOD_US);
    localparam integer TIMEOUT_TICKS = (CLK_HZ / 1000) * TARGET_TIMEOUT_MS;
    localparam integer AGE_BITS = bit_width(TIMEOUT_TICKS + 1);
    localparam integer CENTER_X = IMG_WIDTH / 2;
    localparam integer CENTER_Y = IMG_HEIGHT / 2;
    localparam integer PAN_RESET_US = (PAN_CENTER_US < PAN_MIN_US) ? PAN_MIN_US :
                                    (PAN_CENTER_US > PAN_MAX_US) ? PAN_MAX_US : PAN_CENTER_US;
    localparam integer TILT_RESET_US = (TILT_CENTER_US < TILT_MIN_US) ? TILT_MIN_US :
                                     (TILT_CENTER_US > TILT_MAX_US) ? TILT_MAX_US : TILT_CENTER_US;

    reg [TICK_BITS-1:0] tick_div;
    reg [PERIOD_BITS-1:0] phase_us;
    reg first_frame;
    reg [11:0] pan_us;
    reg [11:0] tilt_us;
    reg [11:0] target_x;
    reg [11:0] target_y;
    reg target_valid;
    reg [AGE_BITS-1:0] target_age;

    wire us_tick = (tick_div == TICKS_PER_US - 1);
    wire frame_boundary = first_frame ||
                          (us_tick && (phase_us == PWM_PERIOD_US - 1));
    wire input_valid = I_target_valid && (I_target_x < IMG_WIDTH) &&
                       (I_target_y < IMG_HEIGHT);
    wire invalid_update = I_target_update && !input_valid;
    wire target_fresh = target_valid && (target_age < TIMEOUT_TICKS);
    // An old sample reaching its expiry on this edge cannot move the camera.
    wire motion_fresh = target_valid && (target_age < TIMEOUT_TICKS - 1);
    wire [12:0] error_x = (target_x >= CENTER_X) ? target_x - CENTER_X : CENTER_X - target_x;
    wire [12:0] error_y = (target_y >= CENTER_Y) ? target_y - CENTER_Y : CENTER_Y - target_y;
    wire pan_increase = (target_x >= CENTER_X) ^ (PAN_REVERSE != 0);
    wire tilt_increase = (target_y >= CENTER_Y) ^ (TILT_REVERSE != 0);

    assign O_pan_us = pan_us;
    assign O_tilt_us = tilt_us;
    assign O_tracking = I_rst_n && I_enable && !I_recenter &&
                        target_fresh && !invalid_update;
    assign O_centered = O_tracking && (error_x <= DEADZONE_X) &&
                        (error_y <= DEADZONE_Y);

    function [11:0] proportional_step;
        input [12:0] error;
        input integer deadzone;
        reg [12:0] step;
        begin
            if (error <= deadzone) proportional_step = 0;
            else begin
                step = (error - deadzone) >> STEP_SHIFT;
                if (step == 0) proportional_step = 1;
                else if (step > MAX_STEP_US) proportional_step = MAX_STEP_US;
                else proportional_step = step[11:0];
            end
        end
    endfunction

    function [11:0] move_limited;
        input [11:0] position;
        input [11:0] step;
        input increase;
        input integer minimum;
        input integer maximum;
        reg [12:0] sum;
        begin
            if (increase) begin
                sum = {1'b0, position} + {1'b0, step};
                if (sum > maximum) move_limited = maximum;
                else move_limited = sum[11:0];
            end else begin
                if ({1'b0, position} <= minimum + {1'b0, step})
                    move_limited = minimum;
                else move_limited = position - step;
            end
        end
    endfunction

    function [11:0] move_to_center;
        input [11:0] position;
        input integer center;
        reg [12:0] distance;
        begin
            if (position >= center) begin
                distance = position - center;
                if (distance <= MAX_STEP_US) move_to_center = center;
                else move_to_center = position - MAX_STEP_US;
            end else begin
                distance = center - position;
                if (distance <= MAX_STEP_US) move_to_center = center;
                else move_to_center = position + MAX_STEP_US;
            end
        end
    endfunction

    reg [11:0] next_pan_us;
    reg [11:0] next_tilt_us;
    always @* begin
        next_pan_us = pan_us;
        next_tilt_us = tilt_us;
        if (!first_frame && I_recenter) begin
            next_pan_us = move_to_center(pan_us, PAN_RESET_US);
            next_tilt_us = move_to_center(tilt_us, TILT_RESET_US);
        end else if (!first_frame && I_enable && !I_recenter &&
                     motion_fresh && !invalid_update) begin
            next_pan_us = move_limited(pan_us, proportional_step(error_x, DEADZONE_X),
                                       pan_increase, PAN_MIN_US, PAN_MAX_US);
            next_tilt_us = move_limited(tilt_us, proportional_step(error_y, DEADZONE_Y),
                                        tilt_increase, TILT_MIN_US, TILT_MAX_US);
        end
    end

    // Latest complete source sample wins. Enable/recenter require a fresh
    // update afterwards, so toggling either cannot resume an old target.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            target_x <= CENTER_X;
            target_y <= CENTER_Y;
            target_valid <= 1'b0;
            target_age <= 0;
        end else if (!I_enable || I_recenter) begin
            target_valid <= 1'b0;
            target_age <= 0;
        end else if (I_target_update) begin
            target_valid <= input_valid;
            target_age <= 0;
            if (input_valid) begin
                target_x <= I_target_x;
                target_y <= I_target_y;
            end
        end else if (target_valid && (target_age < TIMEOUT_TICKS)) begin
            target_age <= target_age + 1'b1;
        end
    end

    // Outputs are registered and only change at frame start or their exact
    // pulse end. Mid-pulse target/mode changes cannot shorten a pulse.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tick_div <= 0;
            phase_us <= 0;
            first_frame <= 1'b1;
            pan_us <= PAN_RESET_US;
            tilt_us <= TILT_RESET_US;
            O_pan_pwm <= 1'b0;
            O_tilt_pwm <= 1'b0;
        end else if (frame_boundary) begin
            tick_div <= 0;
            phase_us <= 0;
            first_frame <= 1'b0;
            pan_us <= next_pan_us;
            tilt_us <= next_tilt_us;
            O_pan_pwm <= (next_pan_us != 0);
            O_tilt_pwm <= (next_tilt_us != 0);
        end else if (us_tick) begin
            tick_div <= 0;
            phase_us <= phase_us + 1'b1;
            if (phase_us >= pan_us - 1) O_pan_pwm <= 1'b0;
            if (phase_us >= tilt_us - 1) O_tilt_pwm <= 1'b0;
        end else begin
            tick_div <= tick_div + 1'b1;
        end
    end

    // synthesis translate_off
    initial begin
        if ((CLK_HZ < 1000000) || ((CLK_HZ % 1000000) != 0) ||
            (TARGET_TIMEOUT_MS < 1) || (PWM_PERIOD_US < 2) ||
            (IMG_WIDTH < 1) || (IMG_WIDTH > 4096) ||
            (IMG_HEIGHT < 1) || (IMG_HEIGHT > 4096) ||
            (PAN_MIN_US < 1) || (PAN_MIN_US > PAN_MAX_US) ||
            (TILT_MIN_US < 1) || (TILT_MIN_US > TILT_MAX_US) ||
            (PAN_MAX_US >= PWM_PERIOD_US) || (PAN_MAX_US > 4095) ||
            (TILT_MAX_US >= PWM_PERIOD_US) || (TILT_MAX_US > 4095) ||
            (MAX_STEP_US < 1) || (MAX_STEP_US > 4095) ||
            (DEADZONE_X < 0) || (DEADZONE_Y < 0) || (STEP_SHIFT < 0)) begin
            $display("ERROR: invalid color_servo_controller parameters");
            $finish;
        end
    end
    // synthesis translate_on
endmodule

`endif
