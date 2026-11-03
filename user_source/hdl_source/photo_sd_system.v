`ifndef PHOTO_SD_SYSTEM_V
`define PHOTO_SD_SYSTEM_V
`include "sd_spi_block.v"
`include "sd_raw_photo_writer.v"
`include "photo_capture_control.v"
`include "photo_ddr_sector.v"
`include "photo_ddr_sink.v"
`include "sd_raw_photo_browser.v"
`include "sd_photo_catalog.v"
// All raw-card/SD logic is in the pixel domain. DDR transfers use the explicit
// request/completion mailbox and publish a complete sector via dual-clock RAM.
module photo_sd_system #(
    parameter integer BROWSER_ENABLE=0,
    parameter integer REPLAY_BLANKING_CONTROL=0,
    parameter integer MOTION_CAPTURE_ENABLE=0,
    parameter integer ALBUM_CONTROL_ENABLE=0,
    parameter integer SD_INIT_HALF_PERIOD=96,SD_FAST_HALF_PERIOD=4,
    parameter integer SD_FAST_LOW_PERIOD=3,SD_FAST_HIGH_PERIOD=1,
    parameter integer SD_POWER_WAIT_CYCLES=75000
)(
    input wire I_clk,I_ddr_clk,I_rst_n,I_shutter,I_source_fresh,I_initialize,
    input wire [2:0] I_mode,
    input wire I_page_locked,
    input wire [1:0] I_locked_page,
    output wire O_page_hold,
    output wire [2:0] O_status,
    output wire [3:0] O_error_code,
    output wire [27:0] O_file_number_bcd,
    input wire I_sd_miso,
    output wire O_sd_cs_n,O_sd_clk,O_sd_mosi,
    input wire I_video_write_busy,I_video_read_busy,I_video_read_en,
    input wire [24:0] I_video_read_addr,
    input wire I_ddr_ready,I_ddr_return_valid,
    input wire [127:0] I_ddr_return_data,
    output wire O_video_ready,O_camera_ready,O_video_return_valid,
    output wire O_ddr_read_en,
    output wire [24:0] O_ddr_read_addr,
    output wire O_owns_ddr
    ,input wire I_view_enable,I_previous,I_next,I_delete
    ,output reg O_view_active
    ,output wire O_view_valid
    ,output wire O_replay_write,O_replay_pending
    ,output wire [24:0] O_replay_addr
    ,output wire [127:0] O_replay_data
    ,input wire I_replay_ready
    ,output wire O_capture_ready,O_capture_accepted,O_capture_done,O_capture_failed
    ,output wire O_motion_accepted
    ,input wire I_view_motion
    ,output wire [11:0] O_manual_display_bcd,O_motion_display_bcd
    ,output wire [2:0] O_manual_display_status,O_motion_display_status
    ,output wire [3:0] O_manual_display_error,O_motion_display_error
);
    (* syn_preserve=1,async_reg="true" *) reg [1:0] pixel_reset_release;
    wire pixel_reset_n=I_rst_n && pixel_reset_release[1];
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n) pixel_reset_release<=0;
        else pixel_reset_release<={pixel_reset_release[0],1'b1};
    wire sd_initialized,sd_busy,sd_done,sd_error,sd_retry;
    wire [3:0] sd_error_code;
    wire sd_read_valid,sd_write_req;
    wire [7:0] sd_read_data;
    wire file_start,file_busy,file_done,file_error,file_initialize,file_motion;
    wire [3:0] file_error_code;
    wire [27:0] file_number;
    wire capture_request,capture_write,capture_tx_valid;
    wire capture_ready;
    wire [31:0] capture_lba;
    wire [7:0] capture_tx_data;
    wire browse_request,browse_write,browse_tx_valid,browse_rx_ready;
    wire [31:0] browse_lba;
    wire [7:0] browse_tx_data;
    wire catalog_owner,catalog_pending,catalog_request,catalog_done,catalog_error;
    wire [31:0] catalog_lba;
    wire [3:0] catalog_error_code;
    wire [11:0] catalog_manual,catalog_motion;
    wire block_request=catalog_owner ? catalog_request: (O_view_active ? browse_request:capture_request);
    wire block_write=!catalog_owner && (O_view_active ? browse_write:capture_write);
    wire [31:0] block_lba=catalog_owner ? catalog_lba: (O_view_active ? browse_lba:capture_lba);
    wire [7:0] block_tx_data=O_view_active ? browse_tx_data:capture_tx_data;
    wire sd_read_ready=catalog_owner || !O_view_active || browse_rx_ready;
    wire block_accept=block_request && sd_initialized && !sd_busy;
    wire pixel_request,pixel_ready,pixel_valid,dma_busy,dma_done,dma_error;
    wire [12:0] pixel_sector;
    wire [7:0] pixel_data;
    wire [1:0] snapshot_page;
    wire [2:0] capture_status;
    wire [3:0] capture_error;
    wire [27:0] capture_number;
    reg browse_start,view_image_valid,view_job_active,browse_album_motion;
    reg [1:0] browse_command;
    reg [2:0] view_status;
    reg [3:0] view_error;
    wire browse_busy,browse_done,browse_error,browse_empty,browse_deleted;
    wire [3:0] browse_error_code;
    wire [27:0] browse_number;
    wire image_begin,image_byte_valid,image_byte_ready,image_done,sink_busy,sink_error;
    wire [7:0] image_byte;
    wire replay_allowed=!REPLAY_BLANKING_CONTROL || I_replay_ready;
    assign O_status=O_view_active ? view_status:capture_status;
    assign O_error_code=O_view_active ? view_error:capture_error;
    assign O_file_number_bcd=O_view_active ? browse_number:capture_number;
    assign O_view_valid=O_view_active && view_image_valid;
    wire requested_motion=ALBUM_CONTROL_ENABLE ? I_view_motion:(MOTION_CAPTURE_ENABLE && I_mode==6);
    wire [11:0] browse_album_count;
    generate if(ALBUM_CONTROL_ENABLE)begin: album_control
        reg catalog_start,owner,pending,counts_valid,initialized_previous,file_error_previous;
        reg [3:0] scan_error,manual_error,motion_error;
        reg [11:0] manual_count_bcd,motion_count_bcd;
        assign browse_album_count=browse_album_motion ? motion_count_bcd:manual_count_bcd;
        function [11:0] increment;
            input [11:0] n;
            begin
                if(n[3:0]!=9)increment=n+12'd1;
                else if(n[7:4]!=9)increment={n[11:4]+8'd1,4'b0};
                else increment={n[11:8]+4'd1,8'b0};
            end
        endfunction
        function [11:0] decrement;
            input [11:0] n;
            begin
                if(n[3:0]!=0)decrement=n-12'd1;
                else if(n[7:4]!=0)decrement={n[11:4]-8'd1,4'h9};
                else decrement={n[11:8]-4'd1,8'h99};
            end
        endfunction
        assign catalog_owner=owner;
        assign catalog_pending=pending;
        sd_photo_catalog catalog(.I_clk(I_clk),.I_rst_n(pixel_reset_n),.I_start(catalog_start),
            .O_busy(),.O_done(catalog_done),.O_error(catalog_error),.O_error_code(catalog_error_code),
            .O_manual_bcd(catalog_manual),.O_motion_bcd(catalog_motion),
            .O_sd_req(catalog_request),.O_sd_lba(catalog_lba),
            .I_sd_accept(block_accept && owner),.I_sd_done(sd_done && owner),
            .I_sd_error(sd_error && owner),.I_sd_rx_valid(sd_read_valid && owner),.I_sd_rx_data(sd_read_data));
        always @(posedge I_clk or negedge pixel_reset_n)begin
            if(!pixel_reset_n)begin
                catalog_start<=0;owner<=0;pending<=1;counts_valid<=0;initialized_previous<=0;file_error_previous<=0;
                manual_count_bcd<=0;motion_count_bcd<=0;scan_error<=0;manual_error<=0;motion_error<=0;
            end else begin
                catalog_start<=0;initialized_previous<=sd_initialized;file_error_previous<=file_error;
                if(!sd_initialized)counts_valid<=0;
                if(sd_initialized && !initialized_previous)begin pending<=1;counts_valid<=0;scan_error<=0;end
                if(!owner && pending && sd_initialized && !sd_busy && !file_busy && !file_start &&
                   !O_page_hold && capture_status!=2 && !dma_busy && !view_job_active &&
                   !browse_busy && !browse_start && !sink_busy)begin
                    owner<=1;pending<=0;catalog_start<=1;scan_error<=0;
                end
                if(owner && !catalog_start)begin
                    if(catalog_done)begin
                        owner<=0;counts_valid<=1;manual_count_bcd<=catalog_manual;motion_count_bcd<=catalog_motion;
                    end else if(catalog_error)begin
                        owner<=0;counts_valid<=0;scan_error<=catalog_error_code;
                    end
                end
                // Only a committed full header changes the current card count.
                if(file_done)begin
                    if(counts_valid)begin
                        if(file_motion)begin if(motion_count_bcd<12'h128)motion_count_bcd<=increment(motion_count_bcd);end
                        else if(manual_count_bcd<12'h128)manual_count_bcd<=increment(manual_count_bcd);
                    end else pending<=1;
                    if(file_motion)motion_error<=0;else manual_error<=0;
                end
                if(browse_deleted && counts_valid)begin
                    if(browse_album_motion)begin if(motion_count_bcd!=0)motion_count_bcd<=decrement(motion_count_bcd);end
                    else if(manual_count_bcd!=0)manual_count_bcd<=decrement(manual_count_bcd);
                end
                // A failed sector write can leave a partial header: rescan the
                // card instead of guessing the retained count after an error.
                if((file_error && !file_error_previous) || (browse_error && view_job_active))begin
                    if(!pending && !owner)begin pending<=1;counts_valid<=0;end
                end
                if(O_capture_failed)begin
                    if(file_motion)motion_error<=capture_error;else manual_error<=capture_error;
                end
                if(O_capture_accepted)begin
                    if(file_motion)motion_error<=0;else manual_error<=0;
                end
            end
        end
        wire [3:0] base_error=(!sd_initialized && sd_error) ? sd_error_code:scan_error;
        wire [2:0] base_status=!sd_initialized ? (sd_error ? 3'd4:3'd0):(pending || owner) ? 3'd2:
            !counts_valid ? (scan_error!=0 ? 3'd4:3'd2):3'd1;
        assign O_manual_display_bcd=(O_view_active && !browse_album_motion) ? browse_number[11:0]:manual_count_bcd;
        assign O_motion_display_bcd=(O_view_active && browse_album_motion) ? browse_number[11:0]:motion_count_bcd;
        assign O_manual_display_status=O_view_active && !browse_album_motion ? view_status:
            capture_status==2 && !file_motion ? 3'd2:manual_error!=0 ? 3'd4:base_status;
        assign O_motion_display_status=O_view_active && browse_album_motion ? view_status:
            capture_status==2 && file_motion ? 3'd2:motion_error!=0 ? 3'd4:base_status;
        assign O_manual_display_error=O_view_active && !browse_album_motion ? view_error:
            manual_error!=0 ? manual_error:base_error;
        assign O_motion_display_error=O_view_active && browse_album_motion ? view_error:
            motion_error!=0 ? motion_error:base_error;
    end else begin: no_album_control
        assign browse_album_count=0;
        assign catalog_owner=0;assign catalog_pending=0;assign catalog_request=0;
        assign catalog_lba=0;assign catalog_done=0;assign catalog_error=0;assign catalog_error_code=0;
        assign catalog_manual=0;assign catalog_motion=0;
        assign O_manual_display_bcd=0;assign O_motion_display_bcd=0;
        assign O_manual_display_status=1;assign O_motion_display_status=1;
        assign O_manual_display_error=0;assign O_motion_display_error=0;
    end endgenerate
    always @(posedge I_clk or negedge pixel_reset_n) begin
        if(!pixel_reset_n)begin
            O_view_active<=0;browse_start<=0;browse_command<=0;
            view_image_valid<=0;view_status<=1;view_error<=0;view_job_active<=0;browse_album_motion<=0;
        end else begin
            browse_start<=0;
            if(!O_view_active)begin
                if(BROWSER_ENABLE && I_view_enable && !catalog_owner && !catalog_pending && sd_initialized && !sd_busy &&
                   !file_busy && !O_page_hold && capture_status!=2 && !dma_busy)begin
                    O_view_active<=1;browse_start<=1;browse_command<=0;
                    browse_album_motion<=requested_motion;
                    view_status<=2;view_error<=0;view_image_valid<=0;view_job_active<=1;
                end
            end else begin
                if(view_job_active && !browse_start)begin
                    if(browse_error)begin
                        view_status<=4;view_error<=(browse_error_code==1 && sd_error) ? sd_error_code:browse_error_code;
                        view_job_active<=0;view_image_valid<=0;
                    end else if(browse_done)begin
                        view_status<=1;view_image_valid<=!browse_empty;view_job_active<=0;
                    end
                end
                if(!view_job_active && !browse_busy && !sink_busy && !sd_busy && !browse_start && !catalog_owner)begin
                    if(!I_view_enable)begin O_view_active<=0;view_image_valid<=0;end
                    else if(!catalog_pending && ALBUM_CONTROL_ENABLE && requested_motion!=browse_album_motion)begin
                        browse_album_motion<=requested_motion;browse_command<=0;browse_start<=1;
                        view_status<=2;view_error<=0;view_image_valid<=0;view_job_active<=1;
                    end
                    else if(!catalog_pending && (I_previous || I_next || I_delete))begin
                        browse_command<=I_delete ? 2'd3 : I_previous ? 2'd1:2'd2;
                        browse_start<=1;view_status<=2;view_error<=0;
                        view_image_valid<=0;view_job_active<=1;
                    end
                end
            end
        end
    end
    assign O_capture_ready=capture_ready && !O_view_active &&
        !(BROWSER_ENABLE && I_view_enable) && !catalog_owner && !catalog_pending;
    photo_capture_control #(.INITIALIZE_CONTROL(1),.ALLOW_MOTION_SHUTTER(MOTION_CAPTURE_ENABLE)) u_control(
        .I_clk(I_clk),.I_rst_n(pixel_reset_n),
        .I_shutter(I_shutter && !O_view_active && !(BROWSER_ENABLE && I_view_enable) && !catalog_owner && (!catalog_pending || !sd_initialized)),.I_mode(I_mode),
        .I_initialize(I_initialize && !O_view_active && !(BROWSER_ENABLE && I_view_enable) && !catalog_owner && (!catalog_pending || !sd_initialized)),
        .I_source_fresh(I_source_fresh),.I_sd_initialized(sd_initialized),
        .I_sd_busy(sd_busy),.I_sd_error(sd_error),.I_sd_error_code(sd_error_code),
        .I_page_locked(I_page_locked),.I_locked_page(I_locked_page),
        .I_file_busy(file_busy),.I_file_done(file_done),.I_file_error(file_error),
        .I_file_error_code((file_error_code==1 && sd_error) ? sd_error_code:file_error_code),.I_file_number_bcd(file_number),.I_dma_error(dma_error),
        .O_page_hold(O_page_hold),.O_file_start(file_start),.O_sd_retry(sd_retry),
        .O_file_initialize(file_initialize),.O_file_motion(file_motion),
        .O_capture_ready(capture_ready),.O_capture_accepted(O_capture_accepted),
        .O_capture_done(O_capture_done),.O_capture_failed(O_capture_failed),
        .O_motion_accepted(O_motion_accepted),
        .O_page(snapshot_page),.O_status(capture_status),.O_error_code(capture_error),
        .O_file_number_bcd(capture_number));
    // Restore the proven mode0 transport: 75 MHz /(3 low +1 high)=18.75 MHz.
    sd_spi_block #(.READ_FLOW_CONTROL(1),.INIT_HALF_PERIOD(SD_INIT_HALF_PERIOD),.FAST_HALF_PERIOD(SD_FAST_HALF_PERIOD),
        .FAST_LOW_PERIOD(SD_FAST_LOW_PERIOD),.FAST_HIGH_PERIOD(SD_FAST_HIGH_PERIOD),
        .POWER_WAIT_CYCLES(SD_POWER_WAIT_CYCLES)) u_transport(
        .I_clk(I_clk),.I_rst_n(pixel_reset_n),.I_start(sd_retry || block_accept),
        .I_write(block_write),.I_lba(block_lba),.I_write_data(block_tx_data),
        .I_sd_miso(I_sd_miso),.I_read_ready(sd_read_ready),
        .O_sd_cs_n(O_sd_cs_n),.O_sd_clk(O_sd_clk),.O_sd_mosi(O_sd_mosi),
        .O_initialized(sd_initialized),.O_busy(sd_busy),.O_done(sd_done),
        .O_error(sd_error),.O_error_code(sd_error_code),
        .O_read_valid(sd_read_valid),.O_read_data(sd_read_data),.O_write_byte_req(sd_write_req));
    sd_raw_photo_writer #(.ALBUM_ENABLE(1)) u_raw_store(
        .I_clk(I_clk),.I_rst_n(pixel_reset_n),.I_start(file_start),.I_initialize(file_initialize),.I_motion(file_motion),
        .O_busy(file_busy),.O_done(file_done),.O_error(file_error),.O_error_code(file_error_code),
        .O_sd_req(capture_request),.O_sd_write(capture_write),.O_sd_lba(capture_lba),
        .I_sd_accept(block_accept && !O_view_active && !catalog_owner),.I_sd_done(sd_done && !O_view_active && !catalog_owner),
        .I_sd_error(sd_error && !O_view_active && !catalog_owner),
        .I_sd_rx_valid(sd_read_valid && !O_view_active && !catalog_owner),.I_sd_rx_data(sd_read_data),
        .O_sd_tx_valid(capture_tx_valid),.O_sd_tx_data(capture_tx_data),.I_sd_tx_ready(sd_write_req && !O_view_active),
        .O_pixel_req(pixel_request),.O_pixel_sector_index(pixel_sector),
        .I_pixel_valid(pixel_valid),.I_pixel_data(pixel_data),.O_pixel_ready(pixel_ready),
        .I_pixel_error(dma_error),.O_file_number_bcd(file_number));
    sd_raw_photo_browser #(.ALBUM_ENABLE(1),.DENSE_NUMBER_ENABLE(ALBUM_CONTROL_ENABLE)) u_browser(
        .I_clk(I_clk),.I_rst_n(pixel_reset_n),.I_start(browse_start),.I_command(browse_command),.I_album_motion(browse_album_motion),
        .I_album_count_bcd(browse_album_count),
        .O_busy(browse_busy),.O_done(browse_done),.O_error(browse_error),
        .O_error_code(browse_error_code),.O_empty(browse_empty),.O_file_number_bcd(browse_number),
        .O_sd_req(browse_request),.O_sd_write(browse_write),.O_sd_lba(browse_lba),
        .I_sd_accept(block_accept && O_view_active && !catalog_owner),.I_sd_done(sd_done && O_view_active && !catalog_owner),
        .I_sd_error(sd_error && O_view_active && !catalog_owner),.I_sd_rx_valid(sd_read_valid && sd_read_ready && O_view_active && !catalog_owner),
        .I_sd_rx_data(sd_read_data),.O_sd_rx_ready(browse_rx_ready),
        .O_sd_tx_valid(browse_tx_valid),.O_sd_tx_data(browse_tx_data),.I_sd_tx_ready(sd_write_req && O_view_active),
        .O_deleted(browse_deleted),.O_image_begin(image_begin),.O_pixel_valid(image_byte_valid),.O_pixel_data(image_byte),
        .I_pixel_ready(image_byte_ready && replay_allowed),
        .I_image_done(image_done),.I_image_error(sink_error));
    photo_ddr_sink u_replay_sink(
        .I_clk(I_clk),.I_ddr_clk(I_ddr_clk),.I_rst_n(I_rst_n),.I_begin(image_begin),
        .I_cancel(browse_error || (browse_done && browse_empty)),.I_enable(O_view_active),
        .I_byte_valid(image_byte_valid && replay_allowed),.I_byte_data(image_byte),.O_byte_ready(image_byte_ready),
        .O_busy(sink_busy),.O_done(image_done),.O_error(sink_error),
        .I_ddr_ready(I_ddr_ready),.I_video_read_busy(I_video_read_busy),
        .O_ddr_write(O_replay_write),.O_ddr_addr(O_replay_addr),.O_ddr_data(O_replay_data),
        .O_write_pending(O_replay_pending));
    photo_ddr_sector u_sector_dma(
        .I_pixel_clk(I_clk),.I_ddr_clk(I_ddr_clk),.I_rst_n(I_rst_n),
        .I_request(pixel_request),.I_sector(pixel_sector),.I_page(snapshot_page),
        .O_busy(dma_busy),.O_done(dma_done),.O_error(dma_error),
        .O_byte_valid(pixel_valid),.O_byte_data(pixel_data),.I_byte_ready(pixel_ready),
        .I_video_write_busy(I_video_write_busy),.I_video_read_busy(I_video_read_busy),
        .I_video_read_en(I_video_read_en),.I_video_read_addr(I_video_read_addr),
        .I_ddr_ready(I_ddr_ready),.I_ddr_return_valid(I_ddr_return_valid),.I_ddr_return_data(I_ddr_return_data),
        .O_video_ready(O_video_ready),.O_camera_ready(O_camera_ready),.O_video_return_valid(O_video_return_valid),
        .O_ddr_read_en(O_ddr_read_en),.O_ddr_read_addr(O_ddr_read_addr),.O_owns_ddr(O_owns_ddr));
endmodule
`endif
