// One immutable RGB888 DDR frame -> requested 512-byte RGB888 sector.
// Video reads have priority; a photo burst starts only with no queued video
// returns or active video command burst. Ownership stays through all returns.
// Request payload is held across a toggle handshake. The dual-clock RAM is
// published only after all32 return beats are stored. No new DSP is needed.
module photo_ddr_sector #(
    parameter integer RETURN_TIMEOUT=1048575
)(
    input wire I_pixel_clk,I_ddr_clk,I_rst_n,
    input wire I_request,
    input wire [12:0] I_sector,
    input wire [1:0] I_page,
    output reg O_busy,O_done,O_error,
    output wire O_byte_valid,
    output wire [7:0] O_byte_data,
    input wire I_byte_ready,
    input wire I_video_write_busy,I_video_read_busy,
    input wire I_video_read_en,
    input wire [24:0] I_video_read_addr,
    input wire I_ddr_ready,I_ddr_return_valid,
    input wire [127:0] I_ddr_return_data,
    output wire O_video_ready,O_camera_ready,O_video_return_valid,
    output wire O_ddr_read_en,
    output wire [24:0] O_ddr_read_addr,
    output reg O_owns_ddr
);
    (* syn_preserve=1,async_reg="true" *) reg [1:0] pixel_reset_release,ddr_reset_release;
    wire pixel_reset_n=I_rst_n && pixel_reset_release[1];
    wire ddr_reset_n=I_rst_n && ddr_reset_release[1];
    always @(posedge I_pixel_clk or negedge I_rst_n)
        if(!I_rst_n) pixel_reset_release<=0;
        else pixel_reset_release<={pixel_reset_release[0],1'b1};
    always @(posedge I_ddr_clk or negedge I_rst_n)
        if(!I_rst_n) ddr_reset_release<=0;
        else ddr_reset_release<={ddr_reset_release[0],1'b1};
    reg request_toggle;
    reg [12:0] request_sector;
    reg [1:0] request_page;
    (* syn_preserve=1,async_reg="true" *) reg [1:0] request_sync;
    reg completion_toggle,completion_error;
    (* syn_preserve=1,async_reg="true" *) reg [1:0] completion_sync;
    reg completion_seen;
    reg [9:0] video_pending;
    reg [5:0] issued,returned;
    reg [24:0] photo_addr;
    reg [19:0] timeout_count;
    reg draining,failed;
    (* ram_style="bram" *) reg [127:0] sector_ram[0:31];
    wire photo_read=O_owns_ddr && !draining && issued<32 && I_ddr_ready;
    assign O_video_ready=I_ddr_ready && !O_owns_ddr;
    assign O_camera_ready=O_video_ready;
    assign O_video_return_valid=I_ddr_return_valid && !O_owns_ddr;
    assign O_ddr_read_en=O_owns_ddr ? photo_read : I_video_read_en;
    assign O_ddr_read_addr=O_owns_ddr ? photo_addr : I_video_read_addr;
    function [24:0] page_base(input [1:0] page);
        begin case(page)
            0:page_base=0;1:page_base=7000000;
            2:page_base=14000000;default:page_base=21000000;
        endcase end
    endfunction
    always @(posedge I_ddr_clk or negedge ddr_reset_n) begin
        if(!ddr_reset_n) request_sync<=0;
        else request_sync<={request_sync[0],request_toggle};
    end
    always @(posedge I_pixel_clk or negedge pixel_reset_n) begin
        if(!pixel_reset_n) completion_sync<=0;
        else completion_sync<={completion_sync[0],completion_toggle};
    end
    always @(posedge I_ddr_clk) begin
        if(O_owns_ddr && I_ddr_return_valid && returned<32)
            sector_ram[returned]<=I_ddr_return_data;
    end
    always @(posedge I_ddr_clk or negedge ddr_reset_n) begin
        if(!ddr_reset_n) begin
            video_pending<=0;issued<=0;returned<=0;photo_addr<=0;
            O_owns_ddr<=0;completion_toggle<=0;completion_error<=0;
            timeout_count<=0;draining<=0;failed<=0;
        end else begin
            case({I_video_read_en,O_video_return_valid && video_pending!=0})
                2'b10:video_pending<=video_pending+1'b1;
                2'b01:video_pending<=video_pending-1'b1;
                default:begin end
            endcase
            if(!O_owns_ddr) begin
                if(request_sync[1]!=completion_toggle && video_pending==0 &&
                   !I_video_read_busy && !I_video_write_busy && !I_video_read_en && !I_ddr_return_valid) begin
                    O_owns_ddr<=1;issued<=0;returned<=0;timeout_count<=0;
                    photo_addr<=page_base(request_page)+
                        ({12'd0,request_sector}<<8);
                    draining<=0;failed<=0;
                end
            end else begin
                if(!draining) begin
                    if(timeout_count<RETURN_TIMEOUT) timeout_count<=timeout_count+1'b1;
                    else begin draining<=1;failed<=1;end
                end
                if(photo_read) begin issued<=issued+1'b1;photo_addr<=photo_addr+25'd8;end
                if(I_ddr_return_valid && returned<issued) begin
                    returned<=returned+1'b1;
                    if(returned==31) draining<=1;
                end
                // Keep the owner bit until after the last return has gone low.
                // A timeout drains issued reads before video may resume.
                if(draining && returned==issued && !I_ddr_return_valid) begin
                    O_owns_ddr<=0;
                    completion_error<=failed || returned!=32;
                    completion_toggle<=request_sync[1];
                end
            end
        end
    end
    localparam P_IDLE=0,P_WAIT=1,P_READ=2,P_LOAD=3,P_BYTES=4;
    reg [2:0] pixel_state;
    reg [4:0] word_index;
    reg [127:0] word_read,word_shift;
    reg [3:0] source_byte;
    assign O_byte_valid=pixel_state==P_BYTES;
    assign O_byte_data=word_shift[127:120];
    always @(posedge I_pixel_clk) word_read<=sector_ram[word_index];
    always @(posedge I_pixel_clk or negedge pixel_reset_n) begin
        if(!pixel_reset_n) begin
            request_toggle<=0;request_sector<=0;request_page<=0;completion_seen<=0;
            O_busy<=0;O_done<=0;O_error<=0;pixel_state<=P_IDLE;
            word_index<=0;word_shift<=0;source_byte<=0;
        end else begin
            O_done<=0;O_error<=0;
            case(pixel_state)
                P_IDLE:if(I_request) begin
                    if(I_sector<5400)begin
                        O_busy<=1;request_sector<=I_sector;request_page<=I_page;
                        request_toggle<=!request_toggle;pixel_state<=P_WAIT;
                        word_index<=0;source_byte<=0;
                    end else begin O_error<=1;O_done<=1;end
                end
                P_WAIT:if(completion_sync[1]!=completion_seen)begin
                    completion_seen<=completion_sync[1];
                    if(completion_error)begin O_busy<=0;O_error<=1;O_done<=1;pixel_state<=P_IDLE;end
                    else pixel_state<=P_READ;
                end
                P_READ:pixel_state<=P_LOAD;
                P_LOAD:begin word_shift<=word_read;pixel_state<=P_BYTES;end
                P_BYTES:if(I_byte_ready)begin
                    word_shift<={word_shift[119:0],8'd0};
                    if(source_byte==15)begin
                        source_byte<=0;
                        if(word_index==31)begin O_busy<=0;O_done<=1;pixel_state<=P_IDLE;end
                        else begin word_index<=word_index+1'b1;pixel_state<=P_READ;end
                    end else source_byte<=source_byte+1'b1;
                end
                default:pixel_state<=P_IDLE;
            endcase
        end
    end
endmodule
