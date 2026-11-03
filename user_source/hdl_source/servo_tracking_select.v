`ifndef SERVO_TRACKING_SELECT_V
`define SERVO_TRACKING_SELECT_V
// Pixel-clock target selection. PWM command feedback has already crossed the
// coherent status mailbox. A motion-mode step blanks detection for 180 ms;
// disabling the detector invalidates its old-camera-pose reference frame.
module servo_tracking_select #(
    parameter integer CLK_HZ=75002000, SETTLE_MS=180,
    parameter integer MODE_SWITCH_CYCLES=16
)(
    input wire I_clk,I_rst_n,
    input wire [2:0] I_mode,
    input wire I_live,
    input wire I_color_update,I_color_valid,
    input wire [11:0] I_color_x,I_color_y,
    input wire I_motion_update,I_motion_valid,I_scene_change,
    input wire [11:0] I_left,I_right,I_top,I_bottom,
    input wire I_status_ready,
    input wire [10:0] I_pan_us,I_tilt_us,
    output wire O_enable,O_update,O_valid,O_motion_enable,
    output wire [11:0] O_x,O_y
);
    localparam integer SETTLE_CYCLES=(CLK_HZ/1000)*SETTLE_MS;
    localparam integer LIMIT=SETTLE_CYCLES>MODE_SWITCH_CYCLES ? SETTLE_CYCLES:MODE_SWITCH_CYCLES;
    localparam integer CB=$clog2(LIMIT+1);
    reg [2:0] mode_q;
    reg [CB-1:0] guard_count;
    reg status_seen;
    reg [10:0] pan_q,tilt_q;
    wire motion_mode=I_mode==3'd6;
    wire mode_change=I_mode!=mode_q;
    wire pose_changed=motion_mode && I_status_ready && status_seen &&
                      (I_pan_us!=pan_q || I_tilt_us!=tilt_q);
    wire ready=I_live && !mode_change && guard_count==0 && !pose_changed;
    wire [12:0] sum_x={1'b0,I_left}+{1'b0,I_right};
    wire [12:0] sum_y={1'b0,I_top}+{1'b0,I_bottom};
    wire motion_valid=I_motion_valid && !I_scene_change &&
        I_left<=I_right && I_top<=I_bottom && I_right<1280 && I_bottom<720;
    assign O_motion_enable=motion_mode && ready && I_status_ready;
    assign O_enable=ready && (I_mode==3'd5 || (motion_mode && I_status_ready));
    assign O_update=O_enable && (motion_mode ? I_motion_update:I_color_update);
    assign O_valid=O_enable && (motion_mode ? motion_valid:I_color_valid);
    assign O_x=motion_mode ? sum_x[12:1]:I_color_x;
    assign O_y=motion_mode ? sum_y[12:1]:I_color_y;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            mode_q<=0;guard_count<=0;status_seen<=0;pan_q<=1500;tilt_q<=1500;
        end else begin
            mode_q<=I_mode;
            if(I_status_ready) begin status_seen<=1;pan_q<=I_pan_us;tilt_q<=I_tilt_us;end
            if(mode_change) guard_count<=motion_mode ? SETTLE_CYCLES:MODE_SWITCH_CYCLES;
            else if(pose_changed) guard_count<=SETTLE_CYCLES;
            else if(guard_count!=0) guard_count<=guard_count-1'b1;
        end
    end
endmodule
`endif
