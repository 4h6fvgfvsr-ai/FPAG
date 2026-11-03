`ifndef PHOTO_DDR_SINK_V
`define PHOTO_DDR_SINK_V
// SD RGB888 bytes -> RGB888 DDR page0. A held 128-bit word/address crosses
// through a request/acknowledge mailbox. No frame/sector RAM or DSP is needed.
// Byte input uses ready/valid; SD pauses between bytes under backpressure.
module photo_ddr_sink #(
    parameter integer IMAGE_BYTES=1280*720*3
)(
    input wire I_clk,I_ddr_clk,I_rst_n,I_begin,I_cancel,I_enable,
    input wire I_byte_valid,input wire [7:0] I_byte_data,
    output wire O_byte_ready,output reg O_busy,O_done,O_error,
    input wire I_ddr_ready,I_video_read_busy,
    output wire O_ddr_write,output wire [24:0] O_ddr_addr,
    output wire [127:0] O_ddr_data,output wire O_write_pending
);
    (* syn_preserve=1,async_reg="true" *) reg [1:0] pixel_release,ddr_release;
    wire pixel_reset_n=I_rst_n && pixel_release[1];
    wire ddr_reset_n=I_rst_n && ddr_release[1];
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n)pixel_release<=0;else pixel_release<={pixel_release[0],1'b1};
    always @(posedge I_ddr_clk or negedge I_rst_n)
        if(!I_rst_n)ddr_release<=0;else ddr_release<={ddr_release[0],1'b1};
    reg request_toggle,acknowledge_toggle;
    (* syn_preserve=1,async_reg="true" *) reg [1:0] request_sync,ack_sync,enable_sync;
    reg [127:0] held_data,work_shift;
    reg [24:0] held_addr;
    reg [17:0] word_index;
    reg [21:0] input_bytes;
    reg [3:0] packed_bytes;
    reg work_full,canceling;
    wire mailbox_pending=request_toggle!=ack_sync[1];
    assign O_byte_ready=O_busy && !canceling && !work_full && input_bytes<IMAGE_BYTES;
    assign O_write_pending=enable_sync[1] && request_sync[1]!=acknowledge_toggle;
    assign O_ddr_write=O_write_pending && !I_video_read_busy && I_ddr_ready;
    assign O_ddr_addr=held_addr;
    assign O_ddr_data=held_data;
    always @(posedge I_ddr_clk or negedge ddr_reset_n) begin
        if(!ddr_reset_n)begin request_sync<=0;enable_sync<=0;acknowledge_toggle<=0;end
        else begin
            request_sync<={request_sync[0],request_toggle};
            enable_sync<={enable_sync[0],I_enable};
            if(O_ddr_write)acknowledge_toggle<=request_sync[1];
        end
    end
    always @(posedge I_clk or negedge pixel_reset_n) begin
        if(!pixel_reset_n)ack_sync<=0;
        else ack_sync<={ack_sync[0],acknowledge_toggle};
    end
    always @(posedge I_clk or negedge pixel_reset_n) begin
        if(!pixel_reset_n)begin
            request_toggle<=0;held_data<=0;held_addr<=0;work_shift<=0;
            word_index<=0;input_bytes<=0;packed_bytes<=0;work_full<=0;
            O_busy<=0;O_done<=0;O_error<=0;canceling<=0;
        end else begin
            if(I_begin)begin
                // A prior aborted mailbox may still be draining. Its payload
                // remains untouched until ACK; new words then replace page0.
                O_busy<=1;O_done<=0;O_error<=0;canceling<=0;
                word_index<=0;input_bytes<=0;packed_bytes<=0;work_full<=0;
                work_shift<=0;
            end else if(I_cancel)begin
                canceling<=1;work_full<=0;packed_bytes<=0;
                O_done<=0;
                if(!mailbox_pending)O_busy<=0;
            end else if(canceling)begin
                if(!mailbox_pending)O_busy<=0;
            end else if(O_busy)begin
                if(work_full && !mailbox_pending)begin
                    held_data<=work_shift;held_addr<={4'b0,word_index,3'b0};
                    request_toggle<=!request_toggle;word_index<=word_index+1'b1;
                    work_full<=0;
                end
                if(I_byte_valid && O_byte_ready)begin
                    input_bytes<=input_bytes+1'b1;
                    work_shift<={work_shift[119:0],I_byte_data};
                    if(packed_bytes==15)begin packed_bytes<=0;work_full<=1;end
                    else packed_bytes<=packed_bytes+1'b1;
                end
                if(input_bytes==IMAGE_BYTES &&
                   !work_full && !mailbox_pending)begin
                    O_busy<=0;O_done<=packed_bytes==0;
                    O_error<=packed_bytes!=0;
                end
            end
        end
    end
endmodule
`endif
