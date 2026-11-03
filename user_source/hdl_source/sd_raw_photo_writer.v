// Dedicated raw SD photograph volume; no filesystem, CPU or internal RAM.
// Sectors: 0 volume header; slot n (zero based) starts at 2048+n*5401.
// PIC2 stores 1280x720 RGB888, 5400 sectors + one custom header.
// All pixel sectors are written before committing the complete DONE header.
// Volume1/PIC1 is not accepted or automatically erased.
// I_initialize is sampled only with I_start. An unknown card is write protected
// unless that sample is high. Initialization invalidates LBA0, clears all 256
// slot headers, and commits the valid volume header last. A valid volume is
// never erased by I_initialize. The caller supplies physical long-press gating.
// Pixel DMA valid/data/ready goes directly to SPI; DMA must sustain byte requests.
module sd_raw_photo_writer #(
    parameter integer ALBUM_ENABLE=0
)(
    input wire I_clk, input wire I_rst_n, input wire I_start,
    input wire I_initialize,input wire I_motion,
    output reg O_busy, output reg O_done, output reg O_error,
    output reg [3:0] O_error_code,
    output reg O_sd_req, output reg O_sd_write, output reg [31:0] O_sd_lba,
    input wire I_sd_accept, input wire I_sd_done, input wire I_sd_error,
    input wire I_sd_rx_valid, input wire [7:0] I_sd_rx_data,
    output wire O_sd_tx_valid, output reg [7:0] O_sd_tx_data,
    input wire I_sd_tx_ready,
    output reg O_pixel_req, output reg [12:0] O_pixel_sector_index,
    input wire I_pixel_valid, input wire [7:0] I_pixel_data, input wire I_pixel_error,
    output wire O_pixel_ready,
    output reg [27:0] O_file_number_bcd
);
    localparam [3:0] IDLE=0,IO_BEGIN=1,IO_REQUEST=2,IO_TRANSFER=3,
        MOUNT=4,SLOT_CHECK=5,ERASE_START=6,ERASE_NEXT=7,VOLUME_FINISHED=8,
        PIXEL_BEGIN=9,PIXEL_WAIT=10,PIXEL_DONE=11,SUCCESS=12,FAIL=13;
    localparam [2:0] READ_VOLUME=0,READ_PHOTO=1,WRITE_ZERO=2,
        WRITE_VOLUME=3,WRITE_PHOTO=4,WRITE_PIXELS=5;
    reg [3:0] state,io_return;
    reg [2:0] operation;
    reg [9:0] byte_count;
    reg [7:0] slot_index;
    reg [20:0] slot_lba;
    reg packet_invalid,allow_initialize,album_motion;
    reg slot_zero,slot_erased;
    wire [8:0] slot_number={1'b0,slot_index}+9'd1;
    wire [20:0] next_slot_lba=slot_lba+21'd5401;
    wire [20:0] next_pixel_lba=O_sd_lba[20:0]+21'd1;

    function [11:0] increment_bcd;
        input [11:0] n;
        begin
            if(n[3:0]!=9)increment_bcd=n+12'd1;
            else if(n[7:4]!=9)increment_bcd={n[11:4]+8'd1,4'b0};
            else increment_bcd={n[11:8]+4'd1,8'b0};
        end
    endfunction
    `include "sd_photo_format.vh"
    wire [7:0] canonical_read_byte=volume_byte(byte_count[8:0]);
    wire [1:0] photo_source=ALBUM_ENABLE ? (album_motion ? 2'd2:2'd1):2'd0;
    assign O_sd_tx_valid=(state==IO_REQUEST || state==IO_TRANSFER) && O_sd_write &&
        (operation!=WRITE_PIXELS || I_pixel_valid);
    wire [7:0] next_write_byte=operation==WRITE_PIXELS ? I_pixel_data :
        operation==WRITE_ZERO ? 8'b0 : operation==WRITE_VOLUME ?
        volume_byte(byte_count[8:0]) : photo_album_byte(byte_count[8:0],slot_number,photo_source);
    assign O_pixel_ready=state==IO_TRANSFER && operation==WRITE_PIXELS && I_sd_tx_ready;

    task read_block;
        input [20:0] lba;input[2:0] kind;input[3:0] destination;
        begin
            O_sd_lba<={11'b0,lba};O_sd_write<=0;operation<=kind;
            io_return<=destination;state<=IO_BEGIN;
        end
    endtask
    task write_block;
        input [20:0] lba;input[2:0] kind;input[3:0] destination;
        begin
            O_sd_lba<={11'b0,lba};O_sd_write<=1;operation<=kind;
            io_return<=destination;state<=IO_BEGIN;
        end
    endtask
    task abort_photo;
        input[3:0] code;
        begin O_sd_req<=0;O_error_code<=code;state<=FAIL;end
    endtask
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            state<=IDLE;io_return<=IDLE;operation<=READ_VOLUME;byte_count<=0;
            slot_index<=0;slot_lba<=2048;packet_invalid<=0;allow_initialize<=0;album_motion<=0;
            slot_zero<=0;slot_erased<=0;
            O_busy<=0;O_done<=0;O_error<=0;O_error_code<=0;
            O_sd_req<=0;O_sd_write<=0;O_sd_lba<=0;O_pixel_req<=0;
            O_pixel_sector_index<=0;O_file_number_bcd<=0;
            O_sd_tx_data<=0;
        end else begin
            O_done<=0;O_pixel_req<=0;
            case(state)
              IDLE:if(I_start)begin
                  O_busy<=1;O_error<=0;O_error_code<=0;allow_initialize<=I_initialize;
                  album_motion<=ALBUM_ENABLE && I_motion;
                  slot_index<=(ALBUM_ENABLE && I_motion) ? 8'd128:8'd0;
                  slot_lba<=(ALBUM_ENABLE && I_motion) ? 21'd693376:21'd2048;
                  O_file_number_bcd<=1;
                  read_block(0,READ_VOLUME,MOUNT);
              end
              IO_BEGIN:begin
                  byte_count<=0;packet_invalid<=0;slot_zero<=1;slot_erased<=1;
                  O_sd_req<=1;state<=IO_REQUEST;
              end
              // The block driver's previous sticky error is cleared by its new
              // accepted start. Observe errors only in IO_TRANSFER afterwards.
              IO_REQUEST:if(I_sd_accept)begin O_sd_req<=0;state<=IO_TRANSFER;end
              IO_TRANSFER:begin
                  if(!O_sd_write && I_sd_rx_valid)begin
                      if(byte_count>=512)abort_photo(7);
                      else begin
                          byte_count<=byte_count+1'b1;
                          if(operation==READ_VOLUME && I_sd_rx_data!=canonical_read_byte)packet_invalid<=1;
                          if(operation==READ_PHOTO)begin
                              if(I_sd_rx_data!=0)slot_zero<=0;
                              if(I_sd_rx_data!=8'hff)slot_erased<=0;
                          end
                      end
                  end
                  if(O_sd_write && I_sd_tx_ready)begin
                      if(!O_sd_tx_valid)abort_photo(9);
                      else if(byte_count>=512)abort_photo(7);
                      else begin byte_count<=byte_count+1'b1;O_sd_tx_data<=next_write_byte;end
                  end
                  if(I_sd_error)abort_photo(1);
                  else if(I_pixel_error && operation==WRITE_PIXELS)abort_photo(8);
                  else if(I_sd_done)begin
                      if((byte_count==512 && !(O_sd_write ? I_sd_tx_ready : I_sd_rx_valid)) ||
                         (byte_count==511 && (O_sd_write ? (I_sd_tx_ready && O_sd_tx_valid) : I_sd_rx_valid)))
                          state<=io_return;
                      else abort_photo(7);
                  end
              end
              MOUNT:begin
                  if(!packet_invalid)read_block(slot_lba,READ_PHOTO,SLOT_CHECK);
                  else if(allow_initialize)write_block(0,WRITE_ZERO,ERASE_START);
                  else abort_photo(11);
              end
              SLOT_CHECK:begin
                  // Only entirely blank/deleted sectors are free. A legacy,
                  // unknown-extension or damaged nonblank header is occupied;
                  // do not destroy an existing photo because validation failed.
                  if(slot_zero || slot_erased)state<=PIXEL_BEGIN;
                  else if(ALBUM_ENABLE ? slot_index[6:0]==127 : slot_index==255)abort_photo(5);
                  else begin
                      slot_index<=slot_index+1'b1;slot_lba<=next_slot_lba;
                      O_file_number_bcd<={16'b0,increment_bcd(O_file_number_bcd[11:0])};
                      read_block(next_slot_lba,READ_PHOTO,SLOT_CHECK);
                  end
              end
              ERASE_START:begin slot_index<=0;slot_lba<=2048;
                  write_block(2048,WRITE_ZERO,ERASE_NEXT);end
              ERASE_NEXT:begin
                  if(slot_index==255)write_block(0,WRITE_VOLUME,VOLUME_FINISHED);
                  else begin slot_index<=slot_index+1'b1;slot_lba<=next_slot_lba;
                      write_block(next_slot_lba,WRITE_ZERO,ERASE_NEXT);end
              end
              VOLUME_FINISHED:begin
                  slot_index<=(ALBUM_ENABLE && album_motion) ? 8'd128:8'd0;
                  slot_lba<=(ALBUM_ENABLE && album_motion) ? 21'd693376:21'd2048;
                  O_file_number_bcd<=1;state<=PIXEL_BEGIN;
              end
              PIXEL_BEGIN:begin
                  O_pixel_sector_index<=0;O_sd_lba<={11'b0,slot_lba}+32'd1;
                  O_pixel_req<=1;state<=PIXEL_WAIT;
              end
              PIXEL_WAIT:begin
                  if(I_pixel_error && !O_pixel_req)abort_photo(8);
                  else if(I_pixel_valid)write_block(O_sd_lba[20:0],WRITE_PIXELS,PIXEL_DONE);
              end
              PIXEL_DONE:begin
                  if(O_pixel_sector_index==5399)write_block(slot_lba,WRITE_PHOTO,SUCCESS);
                  else begin
                      O_pixel_sector_index<=O_pixel_sector_index+1'b1;
                      O_sd_lba<={11'b0,next_pixel_lba};O_pixel_req<=1;state<=PIXEL_WAIT;
                  end
              end
              SUCCESS:begin O_busy<=0;O_done<=1;state<=IDLE;end
              FAIL:begin O_busy<=0;O_error<=1;state<=IDLE;end
              default:abort_photo(7);
            endcase
        end
    end
endmodule
