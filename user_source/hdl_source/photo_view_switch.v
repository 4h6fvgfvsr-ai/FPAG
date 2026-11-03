`ifndef PHOTO_VIEW_SWITCH_V
`define PHOTO_VIEW_SWITCH_V
// SW3 up=live (1), down=SD photos (0), matching the board switch pull-ups.
module photo_view_switch #(
    parameter integer DEBOUNCE_CYCLES=750000,
    parameter integer DUAL_ENABLE=0
)(
    input wire I_clk,I_rst_n,I_sw3,
    output wire O_enable,
    input wire I_sw2,
    output wire O_motion
);
    localparam integer COUNT_BITS=DEBOUNCE_CYCLES<2 ? 1:$clog2(DEBOUNCE_CYCLES);
    (* keep=1,preserve=1,ASYNC_REG="TRUE",syn_preserve=1,syn_keep=1 *) reg sw3_meta,sw3_sync;
    reg stable_value;
    reg [COUNT_BITS-1:0] count;
    (* keep=1,preserve=1,ASYNC_REG="TRUE",syn_preserve=1,syn_keep=1 *) reg sw2_meta,sw2_sync;
    reg stable_sw2;
    reg [COUNT_BITS-1:0] count_sw2;
    assign O_enable=!stable_value || (DUAL_ENABLE && !stable_sw2);
    // SW3 has priority when both switches request the one HDMI viewer.
    assign O_motion=DUAL_ENABLE && stable_value && !stable_sw2;
    generate if(DUAL_ENABLE)begin: dual_switch
        always @(posedge I_clk or negedge I_rst_n)begin
            if(!I_rst_n)begin sw2_meta<=1;sw2_sync<=1;stable_sw2<=1;count_sw2<=0;end
            else begin
                sw2_meta<=I_sw2;sw2_sync<=sw2_meta;
                if(sw2_sync==stable_sw2)count_sw2<=0;
                else if(count_sw2==DEBOUNCE_CYCLES-1)begin stable_sw2<=sw2_sync;count_sw2<=0;end
                else count_sw2<=count_sw2+1'b1;
            end
        end
    end else begin: single_switch
        always @*begin sw2_meta=1;sw2_sync=1;stable_sw2=1;count_sw2=0;end
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin sw3_meta<=1;sw3_sync<=1;stable_value<=1;count<=0;end
        else begin
            sw3_meta<=I_sw3;sw3_sync<=sw3_meta;
            if(sw3_sync==stable_value)count<=0;
            else if(count==DEBOUNCE_CYCLES-1)begin stable_value<=sw3_sync;count<=0;end
            else count<=count+1'b1;
        end
    end
endmodule
`endif
