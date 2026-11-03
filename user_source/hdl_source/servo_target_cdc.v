`ifndef SERVO_TARGET_CDC_V
`define SERVO_TARGET_CDC_V
// Latest-result mailbox. Coordinates stay stable until a synchronized ACK;
// updates while busy replace only a separate pending word, never the mailbox.
// Preserve the CDC registers so retiming/enable extraction cannot bypass a
// synchronizer. Publication waits TWO extra destination clocks after request
// detection: the independently bounded (20 ns) payload path has time to pass
// both data registers even if request propagation is faster than payload.
module servo_target_cdc (
    input wire I_src_clk,I_dst_clk,I_rst_n,
    input wire I_src_enable,I_src_update,I_src_valid,
    input wire [11:0] I_src_x,I_src_y,
    output wire O_dst_rst_n,O_dst_enable,
    output reg O_dst_update,O_dst_valid,
    output reg [11:0] O_dst_x,O_dst_y
);
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [1:0] src_reset; // synthesis keep
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [1:0] dst_reset; // synthesis keep
    always @(posedge I_src_clk or negedge I_rst_n)
        if(!I_rst_n) src_reset<=0;else src_reset<={src_reset[0],1'b1};
    always @(posedge I_dst_clk or negedge I_rst_n)
        if(!I_rst_n) dst_reset<=0;else dst_reset<={dst_reset[0],1'b1};
    assign O_dst_rst_n=dst_reset[1];
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [1:0] enable_sync; // synthesis keep
    always @(posedge I_dst_clk or negedge I_rst_n)
        if(!I_rst_n) enable_sync<=0;
        else if(!dst_reset[1]) enable_sync<=0;
        else enable_sync<={enable_sync[0],I_src_enable};
    assign O_dst_enable=enable_sync[1];
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [25:0] latest_payload; // synthesis keep
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [25:0] stable_payload; // synthesis keep
    reg pending,enable_q;
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg request_toggle; // synthesis keep
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg acknowledge_toggle; // synthesis keep
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [1:0] acknowledge_sync; // synthesis keep
    wire source_event=I_src_update || I_src_enable!=enable_q;
    wire [25:0] incoming_payload={I_src_enable,I_src_enable && I_src_update && I_src_valid,I_src_y,I_src_x};
    always @(posedge I_src_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            latest_payload<=0;stable_payload<=0;pending<=0;request_toggle<=0;
            enable_q<=0;acknowledge_sync<=0;
        end else if(!src_reset[1]) begin
            latest_payload<=0;stable_payload<=0;pending<=0;request_toggle<=0;
            enable_q<=0;acknowledge_sync<=0;
        end else begin
            enable_q<=I_src_enable;
            acknowledge_sync<={acknowledge_sync[0],acknowledge_toggle};
            if(source_event) begin latest_payload<=incoming_payload;pending<=1;end
            if(request_toggle==acknowledge_sync[1] && (pending || source_event)) begin
                stable_payload<=source_event ? incoming_payload : latest_payload;
                request_toggle<=~request_toggle;pending<=0;
            end
        end
    end
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [1:0] request_sync; // synthesis keep
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [25:0] payload_sync_1; // synthesis keep
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0 *)
    reg [25:0] payload_sync_2; // synthesis keep
    reg receive_pending,receive_toggle;
    reg [1:0] receive_settle;
    always @(posedge I_dst_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            request_sync<=0;acknowledge_toggle<=0;payload_sync_1<=0;payload_sync_2<=0;
            O_dst_update<=0;O_dst_valid<=0;O_dst_x<=0;O_dst_y<=0;
            receive_pending<=0;receive_toggle<=0;receive_settle<=0;
        end else if(!dst_reset[1]) begin
            request_sync<=0;acknowledge_toggle<=0;payload_sync_1<=0;payload_sync_2<=0;
            O_dst_update<=0;O_dst_valid<=0;O_dst_x<=0;O_dst_y<=0;
            receive_pending<=0;receive_toggle<=0;receive_settle<=0;
        end else begin
            request_sync<={request_sync[0],request_toggle};
            payload_sync_1<=stable_payload;payload_sync_2<=payload_sync_1;
            O_dst_update<=0;
            if(!enable_sync[1]) O_dst_valid<=0;
            if(receive_pending) begin
                if(receive_settle==1) begin
                    O_dst_x<=payload_sync_2[11:0];O_dst_y<=payload_sync_2[23:12];
                    O_dst_valid<=payload_sync_2[25] && payload_sync_2[24] && enable_sync[1];
                    O_dst_update<=1;acknowledge_toggle<=receive_toggle;
                    receive_pending<=0;receive_settle<=0;
                end else receive_settle<=receive_settle-1'b1;
            end else if(request_sync[1]!=acknowledge_toggle) begin
                receive_pending<=1;receive_settle<=2;
                receive_toggle<=request_sync[1];
            end
        end
    end
endmodule
`endif
