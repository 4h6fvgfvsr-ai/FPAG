module video_out #(
    parameter integer FRAME_BEATS=1280*720*3/16,
    parameter integer BURST_BEATS=48,
    parameter integer SAFE_FIFO_WORDS=496,
    parameter integer RESERVE_WORDS=8,
    parameter integer MAX_PENDING_READS=256
)(
    input wire I_rst_n,I_ddr_clk,
    output wire O_video_out_rd_busy,
    input wire I_video_in_wr_busy,
    input wire [1:0] I_video_out_rp,
    output wire O_ddr_user_rd_en,
    output reg [24:0] O_ddr_user_addr,
    input wire I_ddr_user_ready,I_ddr_user_rd_valid,
    input wire [127:0] I_ddr_user_rd_data,
    input wire I_dsi_clk,I_video_vsync,I_video_rd_en,
    output reg [23:0] O_vdieo_data,
    output wire O_fifo_full,O_fifo_empty,O_fifo_reset,O_fifo_read,O_fifo_write
);
    localparam integer COUNT_BITS=$clog2(FRAME_BEATS+1);
    (* syn_preserve=1,async_reg="true" *) reg [1:0] S_ddr_release,S_display_release;
    wire S_ddr_reset_n=I_rst_n && S_ddr_release[1];
    wire S_display_reset_n=I_rst_n && S_display_release[1];
    always @(posedge I_ddr_clk or negedge I_rst_n)
        if(!I_rst_n) S_ddr_release<=0;else S_ddr_release<={S_ddr_release[0],1'b1};
    always @(posedge I_dsi_clk or negedge I_rst_n)
        if(!I_rst_n) S_display_release<=0;else S_display_release<={S_display_release[0],1'b1};
    (* syn_preserve=1,async_reg="true" *) reg S_video_vsync_1d,S_video_vsync_2d;
    reg S_video_vsync_3d;
    wire S_video_frame_start=!S_video_vsync_3d && S_video_vsync_2d;
    reg S_frame_fifo_reset;
    reg [3:0] S_fifo_rst_cnt;
    wire S_fifo_rst=!S_ddr_reset_n || !S_display_reset_n || S_frame_fifo_reset;
    wire [8:0] S_fifo_wr_num;
    wire [127:0] S_fifo_rd_data;
    wire S_fifo_emtpy;
    reg S_ddr_rd_valid,S_have_frame,S_drop_returns;
    reg [7:0] S_ddr_rd_cnt;
    reg [COUNT_BITS-1:0] S_frame_reads;
    reg [9:0] S_reads_pending;
    // Returned occupancy alone cannot protect against queued, late DDR data.
    // Every accepted command consumes credit before its response arrives.
    wire [10:0] S_committed_words={2'b0,S_fifo_wr_num}+{1'b0,S_reads_pending};
    wire S_ddr_rd_trig=S_ddr_reset_n && S_have_frame && !S_drop_returns && !S_fifo_rst &&
        !I_video_in_wr_busy && !S_ddr_rd_valid &&
        S_frame_reads<=FRAME_BEATS-BURST_BEATS &&
        S_reads_pending<=MAX_PENDING_READS-BURST_BEATS &&
        S_committed_words<=SAFE_FIFO_WORDS-BURST_BEATS-RESERVE_WORDS;
    assign O_ddr_user_rd_en=S_ddr_rd_valid && I_ddr_user_ready && !S_fifo_rst &&
        !S_video_frame_start && !S_drop_returns && S_frame_reads<FRAME_BEATS;
    assign O_video_out_rd_busy=S_ddr_rd_trig || S_ddr_rd_valid;
    // At VS discard previous-page responses until its last outstanding read
    // has returned. New prefetch then begins with a clean FIFO and address.
    wire S_fifo_write=I_ddr_user_rd_valid && S_reads_pending!=0 && !S_drop_returns &&
        !S_fifo_rst && !S_video_frame_start;
    reg [3:0] S_fifo_rd_cnt,S_fifo_rd_cnt_1d;
    reg S_video_rd_en_1d;
    reg [127:0] S_fifo_rd_data_1d;
    wire S_fifo_rd_en=S_display_reset_n && !S_fifo_rst && I_video_rd_en &&
        (S_fifo_rd_cnt==0 || S_fifo_rd_cnt==5 || S_fifo_rd_cnt==10);
    assign O_fifo_empty=S_fifo_emtpy;
    assign O_fifo_reset=S_fifo_rst;
    assign O_fifo_read=S_fifo_rd_en;
    assign O_fifo_write=S_fifo_write;
    function [24:0] page_base;
        input [1:0] page;
        begin case(page)
            0:page_base=25'd0;1:page_base=25'd7000000;
            2:page_base=25'd14000000;default:page_base=25'd21000000;
        endcase end
    endfunction
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) begin S_video_vsync_1d<=0;S_video_vsync_2d<=0;S_video_vsync_3d<=0;end
        else begin
            S_video_vsync_1d<=I_video_vsync;S_video_vsync_2d<=S_video_vsync_1d;S_video_vsync_3d<=S_video_vsync_2d;
        end
    end
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) begin S_frame_fifo_reset<=0;S_fifo_rst_cnt<=0;end
        else if(S_video_frame_start) begin S_frame_fifo_reset<=1;S_fifo_rst_cnt<=0;end
        else if(S_frame_fifo_reset) begin
            if(S_fifo_rst_cnt==10) begin S_frame_fifo_reset<=0;S_fifo_rst_cnt<=0;end
            else S_fifo_rst_cnt<=S_fifo_rst_cnt+1'b1;
        end
    end
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) S_reads_pending<=0;
        else case({O_ddr_user_rd_en,I_ddr_user_rd_valid && S_reads_pending!=0})
            2'b10:S_reads_pending<=S_reads_pending+1'b1;
            2'b01:S_reads_pending<=S_reads_pending-1'b1;
            default:S_reads_pending<=S_reads_pending;
        endcase
    end
    always @(posedge I_ddr_clk or negedge S_ddr_reset_n) begin
        if(!S_ddr_reset_n) begin
            S_have_frame<=0;S_drop_returns<=1;S_ddr_rd_valid<=0;S_ddr_rd_cnt<=0;
            S_frame_reads<=0;O_ddr_user_addr<=0;
        end else begin
            if(S_video_frame_start) begin
                S_have_frame<=1;S_drop_returns<=1;S_ddr_rd_valid<=0;S_ddr_rd_cnt<=0;
                S_frame_reads<=0;O_ddr_user_addr<=page_base(I_video_out_rp);
            end else begin
                if(S_drop_returns && S_reads_pending==0 && !I_ddr_user_rd_valid && !S_fifo_rst)
                    S_drop_returns<=0;
                if(S_fifo_rst || S_drop_returns) begin S_ddr_rd_valid<=0;S_ddr_rd_cnt<=0;end
                else if(S_ddr_rd_valid) begin
                    if(O_ddr_user_rd_en) begin
                        if(S_ddr_rd_cnt==BURST_BEATS-1) begin S_ddr_rd_valid<=0;S_ddr_rd_cnt<=0;end
                        else S_ddr_rd_cnt<=S_ddr_rd_cnt+1'b1;
                    end
                end else if(S_ddr_rd_trig) begin S_ddr_rd_valid<=1;S_ddr_rd_cnt<=0;end
                if(O_ddr_user_rd_en) begin
                    S_frame_reads<=S_frame_reads+1'b1;O_ddr_user_addr<=O_ddr_user_addr+25'd8;
                end
            end
        end
    end
    w128_d512_fifo U_w128_d512_fifo(
        .rst(S_fifo_rst),.clkw(I_ddr_clk),.we(S_fifo_write),.di(I_ddr_user_rd_data),
        .wrusedw(S_fifo_wr_num),.afull(),.full_flag(O_fifo_full),
        .clkr(I_dsi_clk),.re(S_fifo_rd_en),.dout(S_fifo_rd_data),.rdusedw(),
        .valid(),.empty_flag(S_fifo_emtpy),.aempty());
    // Preserve the original packed128-to-RGB24 timing and lane ordering.
    always @(posedge I_dsi_clk or negedge S_display_reset_n) begin
        if(!S_display_reset_n) begin
            S_fifo_rd_cnt<=0;S_video_rd_en_1d<=0;S_fifo_rd_cnt_1d<=0;
            S_fifo_rd_data_1d<=0;O_vdieo_data<=0;
        end else begin
            if(I_video_rd_en) S_fifo_rd_cnt<=S_fifo_rd_cnt+1'b1;else S_fifo_rd_cnt<=0;
            S_video_rd_en_1d<=I_video_rd_en;S_fifo_rd_cnt_1d<=S_fifo_rd_cnt;S_fifo_rd_data_1d<=S_fifo_rd_data;
            if(S_video_rd_en_1d) begin
                case(S_fifo_rd_cnt_1d)
                    0:O_vdieo_data<=S_fifo_rd_data[127:104];1:O_vdieo_data<=S_fifo_rd_data[103:80];
                    2:O_vdieo_data<=S_fifo_rd_data[79:56];3:O_vdieo_data<=S_fifo_rd_data[55:32];
                    4:O_vdieo_data<=S_fifo_rd_data[31:8];
                    5:O_vdieo_data<={S_fifo_rd_data_1d[7:0],S_fifo_rd_data[127:112]};
                    6:O_vdieo_data<=S_fifo_rd_data[111:88];7:O_vdieo_data<=S_fifo_rd_data[87:64];
                    8:O_vdieo_data<=S_fifo_rd_data[63:40];9:O_vdieo_data<=S_fifo_rd_data[39:16];
                    10:O_vdieo_data<={S_fifo_rd_data_1d[15:0],S_fifo_rd_data[127:120]};
                    11:O_vdieo_data<=S_fifo_rd_data[119:96];12:O_vdieo_data<=S_fifo_rd_data[95:72];
                    13:O_vdieo_data<=S_fifo_rd_data[71:48];14:O_vdieo_data<=S_fifo_rd_data[47:24];
                    15:O_vdieo_data<=S_fifo_rd_data[23:0];
                endcase
            end else O_vdieo_data<=0;
        end
    end
    // synthesis translate_off
    initial if(BURST_BEATS<1 || BURST_BEATS>240 || FRAME_BEATS<BURST_BEATS ||
               FRAME_BEATS%BURST_BEATS!=0 || SAFE_FIFO_WORDS>510 || SAFE_FIFO_WORDS<BURST_BEATS+RESERVE_WORDS ||
               MAX_PENDING_READS<BURST_BEATS || MAX_PENDING_READS>SAFE_FIFO_WORDS)
        $fatal(1,"Invalid video output frame/burst/credit geometry");
    // synthesis translate_on
endmodule
