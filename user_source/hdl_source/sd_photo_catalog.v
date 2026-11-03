`ifndef SD_PHOTO_CATALOG_V
`define SD_PHOTO_CATALOG_V
// Read-only count of fully committed ALB1 records. Publish both counts together
// after checking the volume and all256 headers. No RAM or image data is read.
module sd_photo_catalog(
    input wire I_clk,I_rst_n,I_start,
    output reg O_busy,O_done,O_error,
    output reg [3:0] O_error_code,
    output reg [11:0] O_manual_bcd,O_motion_bcd,
    output reg O_sd_req,output reg [31:0] O_sd_lba,
    input wire I_sd_accept,I_sd_done,I_sd_error,I_sd_rx_valid,
    input wire [7:0] I_sd_rx_data
);
    localparam IDLE=0,BEGIN_READ=1,REQUEST=2,TRANSFER=3,CHECK=4,SUCCESS=5,FAIL=6;
    reg [2:0] state;
    reg volume,invalid;
    reg [7:0] slot;
    reg [9:0] count;
    reg [20:0] lba;
    reg [11:0] manual_work,motion_work;
    `include "sd_photo_format.vh"
    wire [8:0] number={1'b0,slot}+9'd1;
    wire [7:0] expected=volume ? volume_byte(count[8:0]):
        photo_album_byte(count[8:0],number,slot[7] ? 2'd2:2'd1);
    function [11:0] increment_bcd;
        input [11:0] n;
        begin
            if(n[3:0]!=9)increment_bcd=n+12'd1;
            else if(n[7:4]!=9)increment_bcd={n[11:4]+8'd1,4'b0};
            else increment_bcd={n[11:8]+4'd1,8'b0};
        end
    endfunction
    task fail;
        input [3:0] code;
        begin O_sd_req<=0;O_error_code<=code;state<=FAIL;end
    endtask
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            state<=IDLE;volume<=1;invalid<=0;slot<=0;count<=0;lba<=0;
            manual_work<=0;motion_work<=0;O_manual_bcd<=0;O_motion_bcd<=0;
            O_busy<=0;O_done<=0;O_error<=0;O_error_code<=0;O_sd_req<=0;O_sd_lba<=0;
        end else begin
            O_done<=0;
            case(state)
                IDLE:if(I_start)begin
                    O_busy<=1;O_error<=0;O_error_code<=0;volume<=1;
                    slot<=0;lba<=0;manual_work<=0;motion_work<=0;state<=BEGIN_READ;
                end
                BEGIN_READ:begin
                    count<=0;invalid<=0;O_sd_lba<={11'b0,lba};O_sd_req<=1;state<=REQUEST;
                end
                REQUEST:if(I_sd_accept)begin O_sd_req<=0;state<=TRANSFER;end
                TRANSFER:begin
                    if(I_sd_rx_valid)begin
                        if(count>=512)fail(7);
                        else begin count<=count+1'b1;if(I_sd_rx_data!=expected)invalid<=1;end
                    end
                    if(I_sd_error)fail(1);
                    else if(I_sd_done)begin
                        if((count==512 && !I_sd_rx_valid) || (count==511 && I_sd_rx_valid))state<=CHECK;
                        else fail(7);
                    end
                end
                CHECK:begin
                    if(volume)begin
                        if(invalid)fail(11);
                        else begin volume<=0;lba<=2048;state<=BEGIN_READ;end
                    end else begin
                        if(!invalid)begin
                            if(slot[7])motion_work<=increment_bcd(motion_work);
                            else manual_work<=increment_bcd(manual_work);
                        end
                        if(slot==255)state<=SUCCESS;
                        else begin slot<=slot+1'b1;lba<=lba+21'd5401;state<=BEGIN_READ;end
                    end
                end
                SUCCESS:begin
                    O_manual_bcd<=manual_work;O_motion_bcd<=motion_work;
                    O_busy<=0;O_done<=1;state<=IDLE;
                end
                FAIL:begin O_busy<=0;O_error<=1;state<=IDLE;end
                default:fail(7);
            endcase
        end
    end
endmodule
`endif
