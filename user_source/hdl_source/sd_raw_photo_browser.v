// Dedicated raw SD viewer: validated headers, bounded navigation, safe deletion.
// SD rx_valid is an already-accepted byte event; rx_ready only backpressures
// pixel reads. No sector RAM is used. Completed images publish after image_done.
module sd_raw_photo_browser #(
    parameter integer ALBUM_ENABLE=0,
    parameter integer DENSE_NUMBER_ENABLE=0
)(
    input wire I_clk,I_rst_n,I_start,input wire [1:0] I_command,
    input wire I_album_motion,
    // Existing valid photos in the selected album, latched logically at start.
    // Used only for dense display numbering; SD slot/header addresses stay fixed.
    input wire [11:0] I_album_count_bcd,
    output reg O_busy,O_done,O_error,output reg [3:0] O_error_code,
    output reg O_empty,output reg [27:0] O_file_number_bcd,
    output reg O_sd_req,O_sd_write,output reg [31:0] O_sd_lba,
    input wire I_sd_accept,I_sd_done,I_sd_error,I_sd_rx_valid,
    input wire [7:0] I_sd_rx_data,output wire O_sd_rx_ready,
    output wire O_sd_tx_valid,output reg [7:0] O_sd_tx_data,
    input wire I_sd_tx_ready,
    output reg O_image_begin,output reg O_deleted,output wire O_pixel_valid,
    output wire [7:0] O_pixel_data,input wire I_pixel_ready,I_image_done,I_image_error
);
    localparam [3:0] IDLE=0,IO_BEGIN=1,IO_REQUEST=2,IO_TRANSFER=3,
        MOUNT=4,HEAD_CHECK=5,STEP=6,DELETE_CHECK=7,DELETE_DONE=8,
        IMAGE_BEGIN=9,PIXEL_READ=10,PIXEL_DONE=11,IMAGE_WAIT=12,SUCCESS=13,
        EMPTY=14,FAIL=15;
    localparam [1:0] READ_VOLUME=0,READ_HEAD=1,READ_PIXELS=2,WRITE_ZERO=3;
    reg [3:0] state,io_return;
    reg [1:0] operation,command;
    reg [9:0] byte_count;
    reg packet_invalid,reverse,current_valid,album_motion;
    reg [7:0] scan_slot,current_slot,scanned_count;
    reg [20:0] scan_lba,current_lba;
    reg [11:0] scan_bcd,current_bcd;
    reg [12:0] pixel_sector;
    wire [8:0] scan_number={1'b0,scan_slot}+9'd1;
    wire [20:0] stepped_lba=scan_lba+(reverse ? -21'd5401 : 21'd5401);
    wire [20:0] next_pixel_lba=O_sd_lba[20:0]+21'd1;
    function [11:0] increment_bcd;
        input [11:0] n;
        begin
            if(n[3:0]!=9)increment_bcd=n+12'd1;
            else if(n[7:4]!=9)increment_bcd={n[11:4]+8'd1,4'b0};
            else increment_bcd={n[11:8]+4'd1,8'b0};
        end
    endfunction
    function [11:0] decrement_bcd;
        input [11:0] n;
        begin
            if(n[3:0]!=0)decrement_bcd=n-12'd1;
            else if(n[7:4]!=0)decrement_bcd={n[11:4]-8'd1,4'h9};
            else decrement_bcd={n[11:8]-4'd1,8'h99};
        end
    endfunction
    `include "sd_photo_format.vh"
    wire [1:0] photo_source=ALBUM_ENABLE ? (album_motion ? 2'd2:2'd1):2'd0;
    wire [7:0] expected_byte=operation==READ_VOLUME ? volume_byte(byte_count[8:0]) :
        photo_album_byte(byte_count[8:0],scan_number,photo_source);
    wire [7:0] first_slot=(ALBUM_ENABLE && album_motion) ? 8'd128:8'd0;
    wire [7:0] last_slot=ALBUM_ENABLE ? {album_motion,7'h7f}:8'hff;
    wire [20:0] first_lba=(ALBUM_ENABLE && album_motion) ? 21'd693376:21'd2048;
    wire [20:0] last_lba=(ALBUM_ENABLE && !album_motion) ? 21'd687975:21'd1379303;
    wire [7:0] scan_limit=ALBUM_ENABLE ? 8'd127:8'd255;
    // After an image error, drain any remaining SPI bytes without feeding the
    // canceled sink; otherwise its low ready would keep the block busy forever.
    assign O_sd_rx_ready=state==IO_TRANSFER && operation==READ_PIXELS ? I_pixel_ready : 1'b1;
    assign O_pixel_valid=state==IO_TRANSFER && operation==READ_PIXELS && I_sd_rx_valid;
    assign O_pixel_data=I_sd_rx_data;
    assign O_sd_tx_valid=(state==IO_REQUEST || state==IO_TRANSFER) && O_sd_write;
    task read_block;
        input [20:0] lba;input[1:0] kind;input[3:0] destination;
        begin O_sd_lba<={11'b0,lba};O_sd_write<=0;operation<=kind;
            io_return<=destination;state<=IO_BEGIN;end
    endtask
    task write_zero;
        input [20:0] lba;input[3:0] destination;
        begin O_sd_lba<={11'b0,lba};O_sd_write<=1;operation<=WRITE_ZERO;
            io_return<=destination;state<=IO_BEGIN;end
    endtask
    task abort_operation;
        input[3:0] code;
        begin O_sd_req<=0;O_error_code<=code;state<=FAIL;
            if(command==3 && O_sd_write)begin
                current_valid<=0;O_empty<=1;O_file_number_bcd<=0;
            end
        end
    endtask
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            state<=IDLE;io_return<=IDLE;operation<=READ_VOLUME;command<=0;
            byte_count<=0;packet_invalid<=0;reverse<=0;current_valid<=0;album_motion<=0;
            scan_slot<=0;current_slot<=0;scanned_count<=0;
            scan_lba<=2048;current_lba<=2048;scan_bcd<=1;current_bcd<=1;pixel_sector<=0;
            O_busy<=0;O_done<=0;O_error<=0;O_error_code<=0;
            O_empty<=1;O_file_number_bcd<=0;O_sd_req<=0;O_sd_write<=0;
            O_sd_lba<=0;O_sd_tx_data<=0;O_image_begin<=0;O_deleted<=0;
        end else begin
            O_done<=0;O_image_begin<=0;O_deleted<=0;
            case(state)
              IDLE:if(I_start)begin
                  O_busy<=1;O_error<=0;O_error_code<=0;command<=I_command;
                  album_motion<=ALBUM_ENABLE && I_album_motion;
                  reverse<=I_command==1;scanned_count<=0;
                  if(I_command==0 || !current_valid || (ALBUM_ENABLE && I_album_motion!=album_motion))begin
                      current_valid<=0;
                      if(DENSE_NUMBER_ENABLE)reverse<=0;
                      scan_slot<=(ALBUM_ENABLE && I_album_motion) ? 8'd128:8'd0;
                      scan_lba<=(ALBUM_ENABLE && I_album_motion) ? 21'd693376:21'd2048;
                      scan_bcd<=1;
                  end
                  else begin
                      scan_slot<=current_slot;scan_lba<=current_lba;
                      if(!DENSE_NUMBER_ENABLE)scan_bcd<=current_bcd;
                      else if(I_command==1)
                          scan_bcd<=current_bcd>1 ? decrement_bcd(current_bcd):I_album_count_bcd;
                      else if(I_command==2)
                          scan_bcd<=current_bcd<I_album_count_bcd ? increment_bcd(current_bcd):12'h001;
                      // Deletion closes the display-number gap. The successor
                      // keeps this rank; deleting the last photo wraps to001.
                      else scan_bcd<=current_bcd<I_album_count_bcd ? current_bcd:12'h001;
                  end
                  read_block(0,READ_VOLUME,MOUNT);
              end
              IO_BEGIN:begin byte_count<=0;packet_invalid<=0;O_sd_req<=1;state<=IO_REQUEST;end
              IO_REQUEST:if(I_sd_accept)begin O_sd_req<=0;state<=IO_TRANSFER;end
              IO_TRANSFER:begin
                  if(!O_sd_write && I_sd_rx_valid)begin
                      if(byte_count>=512)abort_operation(7);
                      else begin byte_count<=byte_count+1'b1;
                          if(operation!=READ_PIXELS && I_sd_rx_data!=expected_byte)packet_invalid<=1;
                      end
                  end
                  if(O_sd_write && I_sd_tx_ready)begin
                      if(byte_count>=512)abort_operation(7);
                      else begin byte_count<=byte_count+1'b1;O_sd_tx_data<=0;end
                  end
                  if(I_sd_error)abort_operation(1);
                  else if(operation==READ_PIXELS && I_image_error)abort_operation(7);
                  else if(I_sd_done)begin
                      if((byte_count==512 && !(O_sd_write ? I_sd_tx_ready : I_sd_rx_valid)) ||
                         (byte_count==511 && (O_sd_write ? I_sd_tx_ready : I_sd_rx_valid)))state<=io_return;
                      else abort_operation(7);
                  end
              end
              MOUNT:begin
                  if(packet_invalid)abort_operation(11);
                  else if(command==3)begin
                      if(current_valid)read_block(scan_lba,READ_HEAD,DELETE_CHECK);else state<=EMPTY;
                  end else if(command!=0 && current_valid)state<=STEP;
                  else read_block(scan_lba,READ_HEAD,HEAD_CHECK);
              end
              HEAD_CHECK:begin
                  if(!packet_invalid)state<=IMAGE_BEGIN;
                  else if(scanned_count==scan_limit)state<=EMPTY;
                  else begin scanned_count<=scanned_count+1'b1;state<=STEP;end
              end
              STEP:begin
                  if(reverse && scan_slot==first_slot)begin
                      scan_slot<=last_slot;scan_lba<=last_lba;
                      if(!DENSE_NUMBER_ENABLE)scan_bcd<=ALBUM_ENABLE ? 12'h128:12'h256;
                      read_block(last_lba,READ_HEAD,HEAD_CHECK);
                  end else if(!reverse && scan_slot==last_slot)begin
                      scan_slot<=first_slot;scan_lba<=first_lba;
                      if(!DENSE_NUMBER_ENABLE)scan_bcd<=1;
                      read_block(first_lba,READ_HEAD,HEAD_CHECK);
                  end else begin
                      scan_slot<=reverse ? scan_slot-1'b1 : scan_slot+1'b1;
                      scan_lba<=stepped_lba;
                      if(!DENSE_NUMBER_ENABLE)scan_bcd<=reverse ? decrement_bcd(scan_bcd) : increment_bcd(scan_bcd);
                      read_block(stepped_lba,READ_HEAD,HEAD_CHECK);
                  end
              end
              DELETE_CHECK:begin
                  if(!packet_invalid)write_zero(scan_lba,DELETE_DONE);
                  else begin current_valid<=0;O_empty<=1;O_file_number_bcd<=0;
                      reverse<=0;scanned_count<=0;state<=STEP;end
              end
              DELETE_DONE:begin
                  O_deleted<=1;
                  current_valid<=0;O_empty<=1;O_file_number_bcd<=0;
                  reverse<=0;scanned_count<=0;state<=STEP;
              end
              IMAGE_BEGIN:begin
                  O_image_begin<=1;pixel_sector<=0;
                  O_sd_lba<={11'b0,scan_lba}+32'd1;state<=PIXEL_READ;
              end
              PIXEL_READ:read_block(O_sd_lba[20:0],READ_PIXELS,PIXEL_DONE);
              PIXEL_DONE:begin
                  if(pixel_sector==5399)state<=IMAGE_WAIT;
                  else begin pixel_sector<=pixel_sector+1'b1;
                      O_sd_lba<={11'b0,next_pixel_lba};state<=PIXEL_READ;end
              end
              IMAGE_WAIT:begin
                  if(I_image_error)abort_operation(7);
                  else if(I_image_done)begin
                      current_valid<=1;current_slot<=scan_slot;current_lba<=scan_lba;current_bcd<=scan_bcd;
                      O_empty<=0;O_file_number_bcd<={16'b0,scan_bcd};state<=SUCCESS;
                  end
              end
              EMPTY:begin current_valid<=0;O_empty<=1;O_file_number_bcd<=0;state<=SUCCESS;end
              SUCCESS:begin O_busy<=0;O_done<=1;state<=IDLE;end
              FAIL:begin O_busy<=0;O_error<=1;state<=IDLE;end
              default:abort_operation(7);
            endcase
        end
    end
endmodule
