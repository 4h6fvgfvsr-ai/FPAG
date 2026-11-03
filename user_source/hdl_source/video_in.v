module video_in #(
    parameter integer FRAME_BEATS=1280*720*3/16,
    parameter integer BURST_BEATS=48,
    parameter integer PHOTO_CONTROL=0
)(
    input wire I_rst_n,
    input wire I_camera_clk,I_camera_frame_start,I_camera_valid,
    input wire [127:0] I_camera_data,
    input wire I_mipi_rx_error,I_ddr_clk,I_display_pause,I_video_out_rd_busy,
    output wire O_video_in_wr_busy,
    output reg [1:0] O_video_out_rp,
    output reg O_ddr_user_wr_en,
    output reg [24:0] O_ddr_user_addr,
    output wire [127:0] O_ddr_user_wr_data,
    input wire I_ddr_user_ready,
    output wire O_fifo_full,O_fifo_reset,O_fifo_write,
    input wire I_photo_hold,
    output reg O_photo_locked,
    output reg [1:0] O_photo_page
);
    localparam integer COUNT_BITS=$clog2(FRAME_BEATS+1);
    (* syn_preserve=1,async_reg="true" *) reg [1:0] S_camera_release,S_ddr_release;
    wire S_camera_reset_n=I_rst_n && S_camera_release[1];
    wire S_ddr_reset_n=I_rst_n && S_ddr_release[1];
    always @(posedge I_camera_clk or negedge I_rst_n)
        if(!I_rst_n) S_camera_release<=0;else S_camera_release<={S_camera_release[0],1'b1};
    always @(posedge I_ddr_clk or negedge I_rst_n)
        if(!I_rst_n) S_ddr_release<=0;else S_ddr_release<={S_ddr_release[0],1'b1};
    reg S_camera_frame_start_extend;
    reg S_camera_armed;
    reg [3:0] S_extend_cnt;
    (* syn_preserve=1,async_reg="true" *) reg S_camera_frame_start_extend_1d,S_camera_frame_start_extend_2d;
    reg S_camera_frame_start_extend_3d;
    wire S_frame_start=!S_camera_frame_start_extend_3d && S_camera_frame_start_extend_2d;
    // The production ISP deliberately supplies early SOF, before valid RGB.
    // Retain that lead; flush the previous frame before its new first payload.
    wire S_fifo_rst=!S_camera_reset_n || !S_ddr_reset_n || S_camera_frame_start_extend_3d;
    wire [8:0] S_fifo_rd_num;
    reg [1:0] S_video_in_wp;
    reg S_ddr_wr_valid,S_have_frame;
    reg [7:0] S_ddr_wr_cnt;
    reg [COUNT_BITS-1:0] S_frame_written;
    (* syn_preserve=1,async_reg="true" *) reg [1:0] photo_hold_sync;
    reg [3:0] completed_pages;
    // A released photograph can be many seconds older than the live stream.
    // Each released bank stays display-ineligible until a good complete frame
    // replaces it; a later capture cannot accidentally forget an older bank.
    reg [3:0] released_pages_pending;
    reg [1:0] writing_page;
    reg writing_good;
    wire acquire_photo=PHOTO_CONTROL && photo_hold_sync[1] && !O_photo_locked &&
        completed_pages[O_video_out_rp] && O_video_out_rp!=writing_page;
    wire protect_photo=PHOTO_CONTROL && (O_photo_locked || acquire_photo);
    wire [1:0] protected_page=O_photo_locked ? O_photo_page : O_video_out_rp;
    function [1:0] next_free_page;
        input [1:0] page,locked_page;
        reg [1:0] candidate;
        begin
            candidate=page+1'b1;
            next_free_page=candidate==locked_page ? candidate+1'b1 : candidate;
        end
    endfunction
    wire [1:0] photo_write_page=S_video_in_wp==protected_page ?
        next_free_page(S_video_in_wp,protected_page) : S_video_in_wp;
    wire [1:0] selected_write_page=protect_photo ? photo_write_page :
        I_mipi_rx_error ? S_video_in_wp-1'b1 : S_video_in_wp;
    wire [1:0] desired_read_page=protect_photo ?
        next_free_page(next_free_page(photo_write_page,protected_page),protected_page) : S_video_in_wp+2'd2;
    wire [3:0] live_readable=completed_pages & ~released_pages_pending &
        ~(4'b0001 << selected_write_page) &
        ~(protect_photo ? (4'b0001 << protected_page) : 4'b0000);
    reg [1:0] safe_read_page;
    always @* begin
        // Before the first complete frame there is no valid page to select;
        // retain the reset/current pointer. During normal capture the other
        // completed live banks provide a safe fallback to the nominal ring.
        safe_read_page=O_video_out_rp;
        if(live_readable[desired_read_page]) safe_read_page=desired_read_page;
        else if(live_readable[O_video_out_rp]) safe_read_page=O_video_out_rp;
        else if(live_readable[writing_page]) safe_read_page=writing_page;
        else if(live_readable[0]) safe_read_page=0;
        else if(live_readable[1]) safe_read_page=1;
        else if(live_readable[2]) safe_read_page=2;
        else if(live_readable[3]) safe_read_page=3;
    end
    wire S_fifo_rd_en=S_ddr_wr_valid && I_ddr_user_ready && !S_fifo_rst && !S_frame_start;
    assign O_fifo_reset=S_fifo_rst;
    // If reset releases mid-frame, wait for the next early SOF before writing.
    wire S_fifo_write=I_camera_valid && S_camera_armed && !S_fifo_rst;
    assign O_fifo_write=S_fifo_write;
    // Include the registered last write, preventing read/write command overlap.
    assign O_video_in_wr_busy=S_ddr_wr_valid || O_ddr_user_wr_en;
    w128_d512_fifo U_w128_d512_fifo(
        .rst(S_fifo_rst),.clkw(I_camera_clk),.we(S_fifo_write),.di(I_camera_data),
        .afull(),.full_flag(O_fifo_full),.wrusedw(),
        .clkr(I_ddr_clk),.re(S_fifo_rd_en),.dout(O_ddr_user_wr_data),.rdusedw(S_fifo_rd_num),
        .valid(),.empty_flag(),.aempty());
    function [24:0] page_base;
        input [1:0] page;
        begin case(page)
            0:page_base=25'd0;1:page_base=25'd7000000;
            2:page_base=25'd14000000;default:page_base=25'd21000000;
        endcase end
    endfunction
    always @(posedge I_camera_clk or negedge S_camera_reset_n) begin
        if(!S_camera_reset_n) begin S_camera_frame_start_extend<=0;S_extend_cnt<=0;S_camera_armed<=0;end
        else begin
            if(I_camera_frame_start) S_camera_armed<=1;
            if(I_camera_frame_start) S_camera_frame_start_extend<=1;
            else if(S_extend_cnt==7) S_camera_frame_start_extend<=0;
            if(S_camera_frame_start_extend) S_extend_cnt<=S_extend_cnt+1'b1;
            else S_extend_cnt<=0;
        end
    end
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) begin
            S_camera_frame_start_extend_1d<=0;S_camera_frame_start_extend_2d<=0;S_camera_frame_start_extend_3d<=0;
        end else begin
            S_camera_frame_start_extend_1d<=S_camera_frame_start_extend;
            S_camera_frame_start_extend_2d<=S_camera_frame_start_extend_1d;
            S_camera_frame_start_extend_3d<=S_camera_frame_start_extend_2d;
        end
    end
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) begin
            photo_hold_sync<=0;O_photo_locked<=0;O_photo_page<=0;
            completed_pages<=0;released_pages_pending<=0;writing_page<=0;writing_good<=0;
        end else begin
            photo_hold_sync<=PHOTO_CONTROL ? {photo_hold_sync[0],I_photo_hold} : 2'b0;
            if(O_ddr_user_wr_en && S_frame_written==FRAME_BEATS-1 && writing_good && !I_mipi_rx_error) begin
                completed_pages[writing_page]<=1;
                released_pages_pending[writing_page]<=0;
            end
            if(I_mipi_rx_error) writing_good<=0;
            if(S_frame_start) begin
                writing_page<=selected_write_page;
                writing_good<=!I_mipi_rx_error;
                completed_pages[selected_write_page]<=0;
            end
            if(!photo_hold_sync[1]) begin
                O_photo_locked<=0;
                if(PHOTO_CONTROL && O_photo_locked) begin
                    released_pages_pending[O_photo_page]<=1;
                    completed_pages[O_photo_page]<=0;
                end
            end
            else if(acquire_photo) begin O_photo_locked<=1;O_photo_page<=O_video_out_rp;end
        end
    end
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) begin S_video_in_wp<=0;O_video_out_rp<=0;end
        else if(S_frame_start && !I_display_pause) begin
            if(protect_photo) begin
                // Three live pages continue while the fourth is immutable.
                // Read the preceding completed live page, never the new write.
                if(!I_mipi_rx_error) S_video_in_wp<=next_free_page(photo_write_page,protected_page);
            end else begin
                if(!I_mipi_rx_error) S_video_in_wp<=S_video_in_wp+1'b1;
            end
            O_video_out_rp<=PHOTO_CONTROL ? safe_read_page : S_video_in_wp+2'd2;
        end
    end
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) begin
            S_ddr_wr_valid<=0;S_ddr_wr_cnt<=0;O_ddr_user_wr_en<=0;
            S_have_frame<=0;S_frame_written<=0;O_ddr_user_addr<=0;
        end else begin
            if(S_frame_start || S_fifo_rst) begin
                S_ddr_wr_valid<=0;S_ddr_wr_cnt<=0;O_ddr_user_wr_en<=0;
            end else begin
                O_ddr_user_wr_en<=S_fifo_rd_en;
                if(S_ddr_wr_valid) begin
                    if(I_ddr_user_ready) begin
                        if(S_ddr_wr_cnt==BURST_BEATS-1) begin S_ddr_wr_valid<=0;S_ddr_wr_cnt<=0;end
                        else S_ddr_wr_cnt<=S_ddr_wr_cnt+1'b1;
                    end
                end else if(S_have_frame && S_fifo_rd_num>=BURST_BEATS && !I_video_out_rd_busy &&
                            S_frame_written<=FRAME_BEATS-BURST_BEATS) begin
                    S_ddr_wr_valid<=1;S_ddr_wr_cnt<=0;
                end
            end
            if(S_frame_start) begin
                S_have_frame<=1;S_frame_written<=0;
                O_ddr_user_addr<=page_base(selected_write_page);
            end else if(O_ddr_user_wr_en) begin
                O_ddr_user_addr<=O_ddr_user_addr+25'd8;
                if(S_frame_written<FRAME_BEATS) S_frame_written<=S_frame_written+1'b1;
            end
        end
    end
    // synthesis translate_off
    initial if(BURST_BEATS<1 || BURST_BEATS>240 || FRAME_BEATS<BURST_BEATS || FRAME_BEATS%BURST_BEATS!=0)
        $fatal(1,"Invalid video input frame/burst geometry");
    // synthesis translate_on
endmodule
