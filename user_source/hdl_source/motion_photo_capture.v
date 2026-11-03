`ifndef MOTION_PHOTO_CAPTURE_V
`define MOTION_PHOTO_CAPTURE_V
// Same pixel clock as the motion result and SD controller. Only a newly
// committed, valid motion result may consume an MCU request. The countdown
// enforces a minimum interval; it never generates autonomous requests.
module motion_photo_capture #(
    parameter integer CLK_HZ=75002000,
    parameter integer INTERVAL_SECONDS=5
)(
    input wire I_clk,I_rst_n,I_enable,I_source_fresh,
    input wire I_motion_update,I_motion_valid,I_scene_change,
    input wire I_view_active,I_capture_ready,
    input wire I_capture_accepted,I_capture_done,I_capture_failed,
    input wire I_mcu_alive,I_mcu_request,
    output reg O_shutter
);
    localparam integer INTERVAL_TICKS=CLK_HZ*INTERVAL_SECONDS;
    localparam integer TIMER_BITS=$clog2(INTERVAL_TICKS+1)<1 ? 1:$clog2(INTERVAL_TICKS+1);
    localparam IDLE=0,ACK_WAIT=1,SAVE_WAIT=2;
    reg [1:0] state;
    reg [TIMER_BITS-1:0] cooldown;
    reg [2:0] ack_timeout;
    reg request_pending;
    wire request_allowed=I_mcu_alive && (request_pending || I_mcu_request);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state<=IDLE;cooldown<=0;ack_timeout<=0;request_pending<=0;
            O_shutter<=0;
        end else begin
            O_shutter<=0;
            // A CPU request waits for a new motion result, never a stale box.
            // Losing the heartbeat cancels pending work, with no takeover.
            if(!I_mcu_alive || !I_enable || !I_source_fresh ||
               I_scene_change || I_view_active ||
               (I_motion_update && !I_motion_valid)) request_pending<=0;
            else if(I_mcu_request) request_pending<=1;
            if(cooldown!=0) cooldown<=cooldown-1'b1;
            case(state)
                IDLE:if(I_enable && I_source_fresh && I_motion_update &&
                    I_motion_valid && !I_scene_change && !I_view_active &&
                    I_capture_ready && cooldown==0 && request_allowed) begin
                    O_shutter<=1;cooldown<=INTERVAL_TICKS;
                    request_pending<=0;
                    ack_timeout<=0;state<=ACK_WAIT;
                end
                ACK_WAIT:begin
                    if(I_capture_accepted) state<=SAVE_WAIT;
                    else if(ack_timeout==7) state<=IDLE;
                    else ack_timeout<=ack_timeout+1'b1;
                end
                SAVE_WAIT:begin
                    // SD completion follows the final committed photo header.
                    // Allow an already accepted write to finish if the MCU
                    // stops while the SD transaction is in progress.
                    if(I_capture_failed) state<=IDLE;
                    else if(I_capture_done) begin
                        state<=IDLE;
                    end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
`endif
