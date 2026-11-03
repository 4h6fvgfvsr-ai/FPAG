`ifndef PHOTO_SHUTTER_KEY_V
`define PHOTO_SHUTTER_KEY_V
// I_held has already passed the panel's two-flop synchronizer and debounce.
// Short clicks fire on release; a three-second hold fires once. A held key
// carried into mode0 cannot initialize a card or take an unintended photo.
module photo_shutter_key #(
    parameter integer LONG_PRESS_CYCLES=225006000
)(
    input wire I_clk,I_rst_n,I_held,
    input wire [2:0] I_mode,
    output reg O_shutter,O_initialize
);
    localparam integer COUNT_BITS=(LONG_PRESS_CYCLES<2)?1:$clog2(LONG_PRESS_CYCLES);
    reg [COUNT_BITS-1:0] hold_count;
    reg held_previous,active,hold_fired;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            hold_count<=0;held_previous<=0;active<=0;hold_fired<=0;
            O_shutter<=0;O_initialize<=0;
        end else begin
            O_shutter<=0;O_initialize<=0;held_previous<=I_held;
            if(I_mode!=0) begin
                hold_count<=0;active<=0;hold_fired<=0;
            end else if(!I_held) begin
                if(active && !hold_fired) O_shutter<=1;
                hold_count<=0;active<=0;hold_fired<=0;
            end else if(!held_previous) begin
                active<=1;hold_count<=0;hold_fired<=0;
            end else if(active && !hold_fired) begin
                if(hold_count==LONG_PRESS_CYCLES-1) begin
                    O_shutter<=1;O_initialize<=1;hold_fired<=1;
                end else hold_count<=hold_count+1'b1;
            end
        end
    end
endmodule
`endif
