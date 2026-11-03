`ifndef LOCAL_THRESHOLD_CONTROL_V
`define LOCAL_THRESHOLD_CONTROL_V
`include "threshold_digit_editor.v"
// Local keys edit the binary, Sobel and cartoon parameters. Events retain
// their original mode while earlier changes wait for the displayed frame.
module local_threshold_control #(
    parameter integer CLK_HZ=75002000,
    parameter integer FRAME_TIMEOUT_MS=250
)(
    input wire I_clk,I_rst_n,I_fixed_thresholds,
    input wire [2:0] I_mode,I_digit_press,
    input wire I_binary_auto,
    input wire [7:0] I_binary_threshold,I_binary_effective,
    input wire [7:0] I_sobel_weak,I_sobel_strong,I_cartoon_threshold,
    input wire I_config_ready,I_input_sof,I_output_sof,
    output reg O_config_set,O_restore_defaults,
    output reg [1:0] O_config_target,
    output reg [7:0] O_config_value
);
    localparam integer FRAME_TICKS=(CLK_HZ/1000)*FRAME_TIMEOUT_MS < 1 ?
        1:(CLK_HZ/1000)*FRAME_TIMEOUT_MS;
    localparam integer TIMER_BITS=$clog2(FRAME_TICKS+1)<1 ? 1:$clog2(FRAME_TICKS+1);
    localparam [2:0] IDLE=0,EDIT=1,COMMIT=2,WAIT_INPUT=3,WAIT_OUTPUT=4;
    reg [2:0] state;
    reg [TIMER_BITS-1:0] frame_timer;
    reg fixed_previous,restore_pending;
    wire fixed_transition=I_fixed_thresholds!=fixed_previous;
    wire fixed_entry=I_fixed_thresholds && !fixed_previous;
    reg [7:0] pending[0:8];
    reg pending_found;
    reg [3:0] pending_index;
    reg [1:0] pending_target,pending_digit;
    reg [3:0] event_base;
    wire editor_busy,editor_done;
    wire editable=I_mode>=2 && I_mode<=4;
    wire dequeue=state==IDLE && !restore_pending && !fixed_transition &&
        !I_fixed_thresholds && pending_found && !editor_busy;
    reg editor_start;
    reg [7:0] editor_input,editor_minimum;
    reg [1:0] editor_digit,edit_target;
    wire [7:0] editor_value;
    integer n,q;
    always @* begin
        case(I_mode)
            3'd2:event_base=0;
            3'd3:event_base=3;
            default:event_base=6;
        endcase
        pending_found=0;pending_index=0;pending_target=0;pending_digit=0;
        for(n=0;n<9;n=n+1)
            if(!pending_found && pending[n]!=0) begin
                pending_found=1;pending_index=n;
                pending_target=n/3+1;pending_digit=n%3;
            end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for(q=0;q<9;q=q+1) pending[q]<=0;
        end else if(I_fixed_thresholds || fixed_transition) begin
            for(q=0;q<9;q=q+1) pending[q]<=0;
        end else begin
            for(q=0;q<9;q=q+1) begin
                if(editable && event_base==q-(q%3) && I_digit_press[q%3]) begin
                    if(!(dequeue && pending_index==q) && pending[q]!=255)
                        pending[q]<=pending[q]+1'b1;
                end else if(dequeue && pending_index==q) pending[q]<=pending[q]-1'b1;
            end
        end
    end
    threshold_digit_editor u_threshold_digit_editor(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(editor_start),
        .I_value(editor_input),.I_min(editor_minimum),.I_digit(editor_digit),
        .O_busy(editor_busy),.O_done(editor_done),.O_value(editor_value));
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state<=IDLE;frame_timer<=0;fixed_previous<=1;restore_pending<=1;
            editor_start<=0;editor_input<=0;editor_minimum<=0;editor_digit<=0;edit_target<=0;
            O_config_set<=0;O_restore_defaults<=0;O_config_target<=0;O_config_value<=0;
        end else begin
            fixed_previous<=I_fixed_thresholds;
            editor_start<=0;
            if(state==WAIT_INPUT || state==WAIT_OUTPUT) frame_timer<=frame_timer+1'b1;
            else frame_timer<=0;
            case(state)
                IDLE:if(restore_pending) begin
                    O_restore_defaults<=1;O_config_target<=0;O_config_value<=0;
                    O_config_set<=1;restore_pending<=0;state<=COMMIT;
                end else if(dequeue) begin
                    edit_target<=pending_target;editor_digit<=pending_digit;
                    case(pending_target)
                        1:begin
                            editor_input<=I_binary_auto ? I_binary_effective:I_binary_threshold;
                            editor_minimum<=0;
                        end
                        2:begin editor_input<=I_sobel_strong;editor_minimum<=I_sobel_weak;end
                        default:begin editor_input<=I_cartoon_threshold;editor_minimum<=1;end
                    endcase
                    editor_start<=1;state<=EDIT;
                end
                EDIT:if(editor_done) begin
                    O_restore_defaults<=0;O_config_target<=edit_target;
                    O_config_value<=editor_value;O_config_set<=1;state<=COMMIT;
                end
                // Hold until the mode/configuration arbiter accepts this
                // update; a simultaneous MCU command cannot drop a key event.
                COMMIT:if(I_config_ready) begin O_config_set<=0;state<=WAIT_INPUT;end
                WAIT_INPUT:if(I_input_sof) state<=WAIT_OUTPUT;
                WAIT_OUTPUT:if(I_output_sof) state<=IDLE;
                default:begin state<=IDLE;O_config_set<=0;end
            endcase
            if((state==WAIT_INPUT || state==WAIT_OUTPUT) && frame_timer==FRAME_TICKS-1)
                state<=IDLE;
            // Cancel a queued or in-flight edit when SW4 changes. Restoring
            // fixed defaults has priority and cannot be overwritten by it.
            if(fixed_transition) begin
                state<=IDLE;O_config_set<=0;
                if(fixed_entry) restore_pending<=1;
            end
        end
    end
endmodule
`endif
