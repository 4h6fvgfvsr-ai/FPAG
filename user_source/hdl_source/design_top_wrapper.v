

// Include the added cores from this source directory as well as listing them
// in the project. This keeps legacy/cached TD source lists self-contained.
`include "color_target_tracker.v"
`include "color_track_temporal.v"
`include "color_blob_grid_rgb.v"
`include "color_rgb_tracker.v"
`include "camera_auto_exposure.v"
`include "camera_color_balance.v"
`include "video_noise_reducer.v"
`include "video_cartoon_reference_stream.v"
`include "motion_opening_detector.v"
`include "video_latency_monitor.v"
`include "osd_decimal_snapshot.v"
`include "video_buffer_health.v"
`include "video_frame_rate_meter.v"
`include "servo_target_cdc.v"
`include "color_servo_controller.v"
`include "local_threshold_control.v"
`include "board_threshold_panel.v"
`include "photo_sd_system.v"
`include "photo_shutter_key.v"
`include "photo_view_switch.v"
`include "photo_hdmi_select.v"
`include "motion_photo_capture.v"

module design_top_wrapper #(
    parameter integer SERVO_TARGET_COLOR=0, // 0 red, 1 green, 2 blue; track RED
    parameter integer SERVO_PAN_REVERSE=1,
    parameter integer SERVO_TILT_REVERSE=0
)(
    input wire        I_sys_clk,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    input wire [3:0]  I_button,
    input wire       I_sw4,
    input wire       I_sw3,
    input wire       I_sw2,
    output wire [7:0] O_seg,
    output wire [7:0] O_sel,
    output wire O_servo_pan_pwm,
    output wire O_servo_tilt_pwm,

    // Board USB-UART for hard RISC-V MCU diagnostics.
    input wire I_mcu_uart_rx,
    output wire O_mcu_uart_tx,
    // TF1 socket SPI: CLK A7, CMD A8, DAT0 E8, DAT3 B12.
    input wire I_sd_miso,
    output wire O_sd_clk,O_sd_mosi,O_sd_cs_n,

    // Ethernet is disabled. Keep its active-low reset asserted on the board.
    output wire       O_phy_rst,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;
    // Declare before port connections: TD treats a later initialized wire
    // as a conflicting implicit net and can tie the video reset permanently.
    wire        S_video_ddr_rst_n;
    wire        S_camera_fifo_write;
    wire        S_display_fifo_write;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire S_photo_ddr_read_en,S_photo_video_return_valid,S_photo_camera_ready;
    wire [24:0] S_photo_ddr_read_addr;
    wire S_photo_ddr_owner,S_photo_hold,S_photo_locked;
    wire [1:0] S_photo_locked_page;
    wire [2:0] S_photo_status;
    wire S_motion_auto_shutter,S_photo_capture_ready,S_photo_capture_accepted;
    wire S_photo_capture_done,S_photo_capture_failed;
    wire [3:0] S_photo_error;
    wire [27:0] S_photo_file_bcd;
    wire S_photo_reset_n;
    wire S_camera_write_rst_n;
    wire S_photo_view_enable,S_photo_view_active,S_photo_view_valid,S_photo_view_motion;
    wire [11:0] S_manual_display_bcd,S_motion_display_bcd;
    wire [2:0] S_manual_display_status,S_motion_display_status;
    wire [3:0] S_manual_display_error,S_motion_display_error;
    wire S_photo_replay_ready;
    (* syn_preserve=1,async_reg="true" *) reg [1:0] S_photo_view_ddr_sync;
    wire S_photo_replay_write,S_photo_replay_pending;
    wire [24:0] S_photo_replay_addr;
    wire [127:0] S_photo_replay_data;
    wire S_tx_vsync,S_tx_de;
    wire [23:0] S_tx_rgb;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    wire[23:0]  S_video_clean_data;
    wire        S_clean_vsync, S_clean_hsync, S_clean_de;
    wire        S_clean_user, S_clean_last;
    wire [11:0] S_fps_bcd;
    wire[23:0]  S_video_style_data;
    wire        S_style_next_trig;
    wire[2:0]   S_style_mode;
    wire [2:0] S_digit_press;
    wire [2:0] S_digit_held;
    wire S_photo_shutter,S_photo_initialize;
    wire S_fixed_thresholds;
    reg [7:0] S_panel_value;
    wire S_local_config_set,S_local_restore_defaults;
    wire [1:0] S_local_config_target;
    wire [7:0] S_local_config_value;
    wire S_cfg_binary_auto;
    wire [7:0] S_cfg_binary_threshold,S_cfg_binary_effective;
    wire [7:0] S_cfg_sobel_weak,S_cfg_sobel_strong,S_cfg_cartoon_threshold;
    wire [7:0] S_cfg_range_threshold,S_cfg_motion_threshold;
    wire S_mcu_core_clk,S_mcu_timer_clk,S_mcu_pll_lock,S_mcu_ahb_clk;
    wire [1:0] S_mcu_htrans,S_mcu_hresp;
    wire S_mcu_hwrite,S_mcu_hready;
    wire [30:0] S_mcu_haddr;
    wire [2:0] S_mcu_hsize;
    wire [31:0] S_mcu_hwdata,S_mcu_hrdata;
    reg [7:0] S_mcu_reset_count;
    (* async_reg="true",syn_preserve=1 *) reg [1:0] S_mcu_reset_pipe,S_mcu_apb_reset_pipe;
    wire S_mcu_lock_ready=S_rst_n && S_mcu_pll_lock;
    reg S_mcu_reset_ready;
    wire S_mcu_alive,S_mcu_mode_set,S_mcu_config_set;
    wire [2:0] S_mcu_mode_value;
    wire [1:0] S_mcu_config_target;
    wire [7:0] S_mcu_config_value;
    wire S_mcu_motion_request,S_mcu_manual_shutter;
    reg S_requested_binary_auto;
    reg [7:0] S_requested_binary_threshold,S_requested_sobel_weak,S_requested_sobel_strong;
    reg [7:0] S_requested_cartoon_threshold,S_requested_range_threshold,S_requested_motion_threshold;
    // MCU commands take priority; local updates wait until accepted. Merge
    // only the selected parameter into the current displayed configuration.
    wire S_local_config_valid=S_local_config_set &&
        (S_local_restore_defaults || !S_fixed_thresholds);
    wire S_control_config_set=S_mcu_config_set || S_local_config_valid;
    wire S_control_restore=!S_mcu_config_set && S_local_restore_defaults;
    wire [1:0] S_control_target=S_mcu_config_set ? S_mcu_config_target:S_local_config_target;
    wire [7:0] S_control_value=S_mcu_config_set ? S_mcu_config_value:S_local_config_value;
    wire S_control_binary_auto=S_control_restore ? 1'b0:
        (S_control_target==1 ? 1'b0:S_requested_binary_auto);
    wire [7:0] S_control_binary_threshold=S_control_restore ? 8'd96:
        (S_control_target==1 ? S_control_value:S_requested_binary_threshold);
    wire [7:0] S_control_sobel_weak=S_control_restore ? 8'd6:
        ((S_control_target==2 && S_requested_sobel_weak>S_control_value) ? S_control_value:S_requested_sobel_weak);
    wire [7:0] S_control_sobel_strong=S_control_restore ? 8'd16:
        (S_control_target==2 ? S_control_value:S_requested_sobel_strong);
    wire [7:0] S_control_cartoon_threshold=S_control_restore ? 8'd26:
        (S_control_target==3 ? S_control_value:S_requested_cartoon_threshold);
    wire [7:0] S_control_range_threshold=S_control_restore ? 8'd24:S_requested_range_threshold;
    wire [7:0] S_control_motion_threshold=S_control_restore ? 8'd24:S_requested_motion_threshold;
    // Accumulate accepted updates even before their frame is displayed.
    // This prevents a local edit from replacing another pending MCU setting.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_requested_binary_auto<=0;S_requested_binary_threshold<=96;
            S_requested_sobel_weak<=6;S_requested_sobel_strong<=16;
            S_requested_cartoon_threshold<=26;S_requested_range_threshold<=24;
            S_requested_motion_threshold<=24;
        end else if(S_control_config_set) begin
            S_requested_binary_auto<=S_control_binary_auto;
            S_requested_binary_threshold<=S_control_binary_threshold;
            S_requested_sobel_weak<=S_control_sobel_weak;S_requested_sobel_strong<=S_control_sobel_strong;
            S_requested_cartoon_threshold<=S_control_cartoon_threshold;
            S_requested_range_threshold<=S_control_range_threshold;S_requested_motion_threshold<=S_control_motion_threshold;
        end
    end
    wire[23:0]  S_color_track_data;
    wire[23:0]  S_video_display_data;
    wire        S_track_enable;
    // Packed target order: red, green, blue (12 bits per coordinate).
    wire[2:0]   S_targets_valid, S_targets_update, S_targets_measured;
    wire[35:0]  S_targets_x, S_targets_y;
    wire[35:0]  S_targets_left, S_targets_right, S_targets_top, S_targets_bottom;
    wire[95:0]  S_targets_pixels;
    wire S_camera_capture_fresh;
    (* syn_preserve=1, async_reg="true" *) reg [1:0] S_motion_capture_fresh_sync;
    wire S_latency_valid,S_latency_repeat,S_latency_new,S_latency_update;
    wire [23:0] S_latency_ticks;
    wire [15:0] S_source_frame_id;
    wire S_motion_valid,S_motion_update,S_motion_scene_change;
    wire [11:0] S_motion_left,S_motion_right,S_motion_top,S_motion_bottom;
    wire [15:0] S_motion_cells,S_motion_events;
    wire S_camera_fifo_full,S_camera_fifo_reset;
    wire S_display_fifo_full,S_display_fifo_empty,S_display_fifo_reset,S_display_fifo_read;
    wire S_command_fifo_full,S_command_fifo_write;
    wire [3:0] S_buffer_faults;
    wire S_servo_rst_n,S_servo_mode,S_servo_update,S_servo_valid;
    wire S_servo_source_enable,S_servo_source_update,S_servo_source_valid;
    wire S_motion_detection_enable;
    wire [11:0] S_servo_source_x,S_servo_source_y;
    wire [11:0] S_servo_x,S_servo_y;
    wire S_servo_tracking,S_servo_centered;
    reg [1:0] S_servo_tracking_sync,S_servo_centered_sync;
    wire [1:0] S_servo_display_state = !S_track_enable ? 2'd0 :
        !S_servo_tracking_sync[1] ? 2'd1 : S_servo_centered_sync[1] ? 2'd3 : 2'd2;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    // Documented PH1P35 clocks: core300, timer100, AHB150, APB50 MHz.
    // MCU resets are local and cannot reset the running camera/DDR pipeline.
    MCU_PLL u_MCU_PLL(.refclk(I_sys_clk),.reset(1'b0),
        .clk0_out(S_mcu_core_clk),.clk1_out(S_mcu_timer_clk),.lock(S_mcu_pll_lock));
    always @(posedge S_mcu_timer_clk or negedge S_mcu_lock_ready)
        if(!S_mcu_lock_ready) begin S_mcu_reset_count<=0;S_mcu_reset_ready<=0;end
        else if(&S_mcu_reset_count) S_mcu_reset_ready<=1;
        else S_mcu_reset_count<=S_mcu_reset_count+1'b1;
    always @(posedge S_mcu_core_clk or negedge S_mcu_reset_ready)
        if(!S_mcu_reset_ready) S_mcu_reset_pipe<=0;
        else S_mcu_reset_pipe<={S_mcu_reset_pipe[0],1'b1};
    always @(posedge I_sys_clk or negedge S_mcu_reset_ready)
        if(!S_mcu_reset_ready) S_mcu_apb_reset_pipe<=0;
        else S_mcu_apb_reset_pipe<={S_mcu_apb_reset_pipe[0],1'b1};
    CameraMCU u_camera_mcu(
        .core_clk(S_mcu_core_clk),.timer_clk(S_mcu_timer_clk),
        .core_reset(!S_mcu_reset_pipe[1]),.por_reset(!S_mcu_reset_pipe[1]),.core_sysrst(),
        .uart0_rx(I_mcu_uart_rx),.uart0_tx(O_mcu_uart_tx),
        .ahb_clk(S_mcu_ahb_clk),.htrans(S_mcu_htrans),.hwrite(S_mcu_hwrite),
        .haddr(S_mcu_haddr),.hsize(S_mcu_hsize),.hburst(),.hprot(),.hmastlock(),
        .hwdata(S_mcu_hwdata),.hrdata(S_mcu_hrdata),.hresp(S_mcu_hresp),.hready(S_mcu_hready),
        .apb_clk(I_sys_clk),.apb_rst(!S_mcu_apb_reset_pipe[1]),
        .paddr(),.psel(),.penable(),.pwrite(),.pwdata(),.pstrobe(),.pprot(),
        .prdata(32'b0),.pready(1'b1),.pslverr(1'b0),.nmi(1'b0),.clic_irq(23'b0));
    camera_mcu_control #(.VIDEO_CLK_HZ(75002000),.WATCHDOG_MS(500)) u_camera_mcu_control(
        .I_bus_clk(S_mcu_ahb_clk),.I_bus_rst_n(S_mcu_reset_pipe[1]),
        .I_htrans(S_mcu_htrans),.I_hwrite(S_mcu_hwrite),.I_haddr(S_mcu_haddr),
        .I_hsize(S_mcu_hsize),.I_hwdata(S_mcu_hwdata),
        .O_hrdata(S_mcu_hrdata),.O_hresp(S_mcu_hresp),.O_hready(S_mcu_hready),
        .I_video_clk(S_hdmi_pixel_clk),.I_video_rst_n(S_hdmi_rst_n),
        .I_frame_start(S_hdmi_user && S_hdmi_de),.I_mode(S_style_mode),
        .I_source_fresh(S_motion_capture_fresh_sync[1]),.I_motion_update(S_motion_update),
        .I_motion_valid(S_motion_valid),.I_scene_change(S_motion_scene_change),
        .I_view_active(S_photo_view_enable || S_photo_view_active),.I_capture_ready(S_photo_capture_ready),
        .I_capture_accepted(S_photo_capture_accepted),.I_capture_failed(S_photo_capture_failed),
        .I_fixed_thresholds(S_fixed_thresholds),
        .I_thresholds({S_cfg_motion_threshold,S_cfg_cartoon_threshold,S_cfg_sobel_strong,S_cfg_binary_threshold}),
        .I_counts({4'b0,S_motion_display_bcd,4'b0,S_manual_display_bcd}),
        .O_alive(S_mcu_alive),.O_mode_set(S_mcu_mode_set),.O_mode_value(S_mcu_mode_value),
        .O_config_set(S_mcu_config_set),.O_config_target(S_mcu_config_target),.O_config_value(S_mcu_config_value),
        .O_motion_request(S_mcu_motion_request),.O_manual_shutter(S_mcu_manual_shutter));

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };
	
	assign O_cam_rst 	= 1'b1;
    wire S_camera_config_reset_n;
    // Dedicated clock output; start sensor configuration only after XCLK
    // has been stable for at least 32768 cycles (1.376 ms at 23.81 MHz).
    camera_sensor_clock u_camera_clock (
        .I_clk(S_24m_clk), .I_locked(S_pll_lock),
        .O_sensor_clk(O_cam_24m), .O_config_reset_n(S_camera_config_reset_n)
    );
	
	
	
	
    // R10 used to be I_rst_n, but is physically the Ethernet TX_CTL pin.
    // Do not leave an implicit, undriven reset net after removing that port.
    // PLL reset is active HIGH: keep it inactive, as in the Ethernet PLL.
    // Device startup initializes the PLL; downstream reset still follows lock.
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset       ( 1'b0              ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  // Independent RAW brightness observer; existing ISP/style/tracking unchanged.
  // Start at the original 4x setting; automatically adjust within 1x..8x.
  camera_auto_exposure #(
      .IMG_WIDTH(1280), .IMG_HEIGHT(720), .TARGET(128), .TOLERANCE(12),
      .CENTER_METER(1), .VERY_DARK(64), .VERY_BRIGHT(208),
      .MIN_EXPOSURE(3), .MAX_EXPOSURE(1990), .INITIAL_EXPOSURE(1990),
      .MIN_GAIN(16), .MAX_GAIN(112), .INITIAL_GAIN(80), .SETTLE_FRAMES(3)
  ) u_camera_auto_exposure (
      .I_pixel_clk(S_csi_rx_clk), .I_cfg_clk(S_24m_clk), .I_rst_n(S_rst_n),
      .I_frame_start(S_raw_tuser), .I_valid(S_raw_tvalid), .I_raw10(S_raw_tdata),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );
  
    uicfgcs500_720p #(
      .CLK_DIV(24000000 / 100000 - 1)
  ) u_uicfgcs500 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_camera_config_reset_n),  // wait for stable sensor XCLK
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done)  //AE配置完成
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );


    video_in #(.PHOTO_CONTROL(1)) u_video_in(
        .I_rst_n              ( S_camera_write_rst_n ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy || S_photo_ddr_owner ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_photo_camera_ready ),
        .O_fifo_full(S_camera_fifo_full),.O_fifo_reset(S_camera_fifo_reset),
        .O_fifo_write(S_camera_fifo_write),
        .I_photo_hold(S_photo_hold),.O_photo_locked(S_photo_locked),
        .O_photo_page(S_photo_locked_page)
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en || S_photo_replay_write;

    assign S_ddr_user_rd_en    = S_photo_ddr_read_en;

    assign S_ddr_user_addr     = S_photo_replay_write ? S_photo_replay_addr : S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_photo_ddr_read_en ? S_photo_ddr_read_addr : 'd0;

    assign S_ddr_user_wr_data  = S_photo_replay_write ? S_photo_replay_data:S_vi_ddr_wr_data;


    // Video commands and FIFOs stay reset until DDR training is complete.
    // Each video module synchronizes release in its own clock domains.
    assign S_video_ddr_rst_n=S_rst_n && S_init_calib_complete;
    assign S_camera_write_rst_n=S_video_ddr_rst_n && !S_photo_view_active;
    always @(posedge S_ddr_clk or negedge S_video_ddr_rst_n)
        if(!S_video_ddr_rst_n)S_photo_view_ddr_sync<=0;
        else S_photo_view_ddr_sync<={S_photo_view_ddr_sync[0],S_photo_view_active};
    assign S_video_out_rst_n = S_video_ddr_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy || S_photo_ddr_owner || S_photo_replay_pending ),
        .I_video_out_rp      ( S_photo_view_ddr_sync[1] ? 2'd0:S_video_out_rp ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_vo_ddr_rd_ready    ),
        .I_ddr_user_rd_valid ( S_photo_video_return_valid ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      ),
        .O_fifo_full(S_display_fifo_full),.O_fifo_empty(S_display_fifo_empty),
        .O_fifo_reset(S_display_fifo_reset),.O_fifo_read(S_display_fifo_read),
        .O_fifo_write(S_display_fifo_write)
    );


    assign S_photo_reset_n=S_video_ddr_rst_n && S_hdmi_rst_n;
    motion_photo_capture #(.CLK_HZ(75002000),.INTERVAL_SECONDS(5)) u_motion_photo_capture(
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_photo_reset_n),.I_enable(S_style_mode==6),
        .I_source_fresh(S_motion_capture_fresh_sync[1]),
        .I_motion_update(S_motion_update),.I_motion_valid(S_motion_valid),.I_scene_change(S_motion_scene_change),
        .I_view_active(S_photo_view_enable || S_photo_view_active),
        .I_capture_ready(S_photo_capture_ready),.I_capture_accepted(S_photo_capture_accepted),
        .I_capture_done(S_photo_capture_done),.I_capture_failed(S_photo_capture_failed),
        .I_mcu_alive(S_mcu_alive),.I_mcu_request(S_mcu_motion_request),
        .O_shutter(S_motion_auto_shutter));
    photo_sd_system #(.BROWSER_ENABLE(1),.REPLAY_BLANKING_CONTROL(1),.MOTION_CAPTURE_ENABLE(1),.ALBUM_CONTROL_ENABLE(1)) u_photo_sd_system(
        .I_clk(S_hdmi_pixel_clk),.I_ddr_clk(S_ddr_clk),.I_rst_n(S_photo_reset_n),
        .I_shutter(S_style_mode==6 ? S_motion_auto_shutter:(S_photo_shutter || S_mcu_manual_shutter)),
        .O_capture_ready(S_photo_capture_ready),.O_motion_accepted(S_photo_capture_accepted),
        .O_capture_done(S_photo_capture_done),.O_capture_failed(S_photo_capture_failed),
        .I_initialize(S_photo_initialize),.I_mode(S_style_mode),
        .I_view_enable(S_photo_view_enable),.I_view_motion(S_photo_view_motion),
        .O_manual_display_bcd(S_manual_display_bcd),.O_motion_display_bcd(S_motion_display_bcd),
        .O_manual_display_status(S_manual_display_status),.O_motion_display_status(S_motion_display_status),
        .O_manual_display_error(S_manual_display_error),.O_motion_display_error(S_motion_display_error),.I_previous(S_digit_press[0]),
        .I_next(S_digit_press[1]),.I_delete(S_digit_press[2]),
        .O_view_active(S_photo_view_active),.O_view_valid(S_photo_view_valid),
        .O_replay_write(S_photo_replay_write),.O_replay_pending(S_photo_replay_pending),
        .O_replay_addr(S_photo_replay_addr),.O_replay_data(S_photo_replay_data),
        .I_replay_ready(S_photo_replay_ready),
        .I_source_fresh(S_motion_capture_fresh_sync[1]),
        .I_page_locked(S_photo_locked),.I_locked_page(S_photo_locked_page),.O_page_hold(S_photo_hold),
        .O_status(S_photo_status),.O_error_code(S_photo_error),.O_file_number_bcd(S_photo_file_bcd),
        .I_sd_miso(I_sd_miso),.O_sd_cs_n(O_sd_cs_n),.O_sd_clk(O_sd_clk),.O_sd_mosi(O_sd_mosi),
        .I_video_write_busy(S_video_in_wr_busy),.I_video_read_busy(S_video_out_rd_busy),
        .I_video_read_en(S_vo_ddr_rd_en),.I_video_read_addr(S_vo_ddr_rd_addr),
        .I_ddr_ready(S_ddr_user_ready),.I_ddr_return_valid(S_ddr_user_rd_valid),
        .I_ddr_return_data(S_ddr_user_rd_data),
        .O_video_ready(S_vo_ddr_rd_ready),.O_camera_ready(S_photo_camera_ready),
        .O_video_return_valid(S_photo_video_return_valid),.O_ddr_read_en(S_photo_ddr_read_en),
        .O_ddr_read_addr(S_photo_ddr_read_addr),.O_owns_ddr(S_photo_ddr_owner));

    local_threshold_control #(.CLK_HZ(75002000)) u_local_threshold_control(
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),
        .I_fixed_thresholds(S_fixed_thresholds),.I_mode(S_style_mode),
        .I_digit_press((S_photo_view_enable || S_photo_view_active) ? 3'b0:S_digit_press),
        .I_binary_auto(S_cfg_binary_auto),.I_binary_threshold(S_cfg_binary_threshold),
        .I_binary_effective(S_cfg_binary_effective),.I_sobel_weak(S_cfg_sobel_weak),
        .I_sobel_strong(S_cfg_sobel_strong),.I_cartoon_threshold(S_cfg_cartoon_threshold),
        .I_config_ready(!S_mcu_config_set),.I_input_sof(S_hdmi_user && S_hdmi_de),
        .I_output_sof(S_clean_user && S_clean_de),
        .O_config_set(S_local_config_set),.O_restore_defaults(S_local_restore_defaults),
        .O_config_target(S_local_config_target),.O_config_value(S_local_config_value));

    // Board push-buttons are active-low (10K pull-up, pressed to GND).
    // KEY1/D5 advances modes. KEY2/3/4 edit the hundreds/tens/ones digits.
    key_press_debounce #(
        .STABLE_CYCLES ( 750000 ),
        .COUNTER_WIDTH ( 20     )
    )u_style_next_key(
        .I_clk         ( S_hdmi_pixel_clk  ),
        .I_rst_n       ( S_hdmi_rst_n       ),
        .I_key_n       ( I_button[0]        ),
        .O_press_pulse ( S_style_next_trig  )
    );

    // Edit the canonical parameter associated with the displayed-frame mode.
    always @* begin
        case(S_style_mode)
            3'd2:S_panel_value=S_cfg_binary_auto ? S_cfg_binary_effective:S_cfg_binary_threshold;
            3'd3:S_panel_value=S_cfg_sobel_strong;
            3'd4:S_panel_value=S_cfg_cartoon_threshold;
            default:S_panel_value=0;
        endcase
    end
    photo_view_switch #(.DUAL_ENABLE(1)) u_photo_view_switch(
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),.I_sw3(I_sw3),.I_sw2(I_sw2),
        .O_enable(S_photo_view_enable),.O_motion(S_photo_view_motion));
    photo_shutter_key u_photo_key(
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),
        .I_held(S_digit_held[2]),.I_mode((S_photo_view_enable || S_photo_view_active) ? 3'd7:S_style_mode),
        .O_shutter(S_photo_shutter),.O_initialize(S_photo_initialize));
    board_threshold_panel #(.CLK_HZ(75002000),.DEBOUNCE_CYCLES(750000),.PHOTO_ENABLE(1),.THRESHOLD_VALID_ENABLE(1),.CAPTURE_COUNT_ENABLE(0),.DUAL_PHOTO_ENABLE(1),
        .SCAN_HZ(1000),.BLANK_CYCLES(150),.DIGIT_ACTIVE_LOW(1),.SEGMENT_ACTIVE_LOW(1))
    u_threshold_panel(
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),
        .I_keys_n(I_button[3:1]),.I_sw4_n(I_sw4),.I_value(S_panel_value),
        .I_threshold_valid(S_style_mode>=2 && S_style_mode<=4),
        .I_photo_mode(S_style_mode==0 || S_style_mode==6 || S_photo_view_enable || S_photo_view_active),.I_photo_status(S_photo_status),
        .I_photo_error(S_photo_error),.I_photo_number_bcd(S_photo_file_bcd[11:0]),
        .I_capture_count_bcd(16'b0),
        .I_manual_display_bcd(S_manual_display_bcd),.I_motion_display_bcd(S_motion_display_bcd),
        .I_manual_display_status(S_manual_display_status),.I_motion_display_status(S_motion_display_status),
        .I_manual_display_error(S_manual_display_error),.I_motion_display_error(S_motion_display_error),
        .O_digit_held(S_digit_held),
        .O_digit_press(S_digit_press),.O_fixed_thresholds(S_fixed_thresholds),
        .O_seg(O_seg),.O_sel(O_sel));

    video_frame_rate_meter #(.FRAME_BEATS(1280*720*3/16),.REF_CLK_HZ(50000000))
    u_video_frame_rate_meter (
        .I_frame_clk(S_csi_rx_clk),.I_ref_clk(I_sys_clk),
        .I_display_clk(S_hdmi_pixel_clk),.I_rst_n(S_rst_n),
        .I_frame_start(S_ISP_O_tuser),.I_frame_valid(S_ISP_O_tvalid),
        .O_fps_bcd(S_fps_bcd),.O_capture_fresh(S_camera_capture_fresh)
    );

    // Keep DDR reads on the original VTC. The denoiser delays RGB and all
    // downstream video controls together; it does not change pixel count.
    assign S_hdmi_window_rd_en = S_hdmi_de;
    // Centered reference-style stream replaces both old spatial processors.
    // 4970 clocks at75.002MHz (~66.3us), constant throughput onepixel/clock.
    video_cartoon_reference_stream #(
        .IMG_WIDTH(1280), .IMG_HEIGHT(720), .H_TOTAL(1650), .LAST_MODE(6),
        .RUNTIME_CONTROL(1), .ORIGINAL_NATURAL(1), .IMAGE_QUALITY_ENABLE(1), .CARTOON_FLAT_STYLE(1),
        .VIVID_COLOR_ENABLE(1),.PHOTO_VIEW_ENABLE(1)
    ) u_video_cartoon_reference_stream (
        .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
        .I_vsync(S_hdmi_vsync), .I_hsync(S_hdmi_hsync),
        .I_de(S_hdmi_de), .I_user(S_hdmi_user), .I_last(S_hdmi_last),
        .I_rgb(S_video_rd_data),.I_photo_view(S_photo_view_active),
        .I_mode_next(S_style_next_trig && !S_mcu_mode_set), .I_mode_original(1'b0),
        .I_mode_set(S_mcu_mode_set),
        .I_mode_value(S_mcu_mode_value),
        .I_config_set(S_control_config_set),.I_binary_auto(S_control_binary_auto),
        .I_binary_threshold(S_control_binary_threshold),
        .I_sobel_weak(S_control_sobel_weak),.I_sobel_strong(S_control_sobel_strong),
        .I_cartoon_threshold(S_control_cartoon_threshold),
        .I_range_threshold(S_control_range_threshold),.I_motion_threshold(S_control_motion_threshold),
        .O_binary_auto(S_cfg_binary_auto),.O_binary_threshold(S_cfg_binary_threshold),
        .O_binary_effective(S_cfg_binary_effective),
        .O_sobel_weak(S_cfg_sobel_weak),.O_sobel_strong(S_cfg_sobel_strong),
        .O_cartoon_threshold(S_cfg_cartoon_threshold),
        .O_range_threshold(S_cfg_range_threshold),.O_motion_threshold(S_cfg_motion_threshold),
        .O_vsync(S_clean_vsync), .O_hsync(S_clean_hsync),
        .O_de(S_clean_de), .O_user(S_clean_user), .O_last(S_clean_last),
        .O_rgb(S_video_style_data), .O_mode(S_style_mode)
    );
    // COLORTRACK consumes the aligned toned original in mode5.
    assign S_video_clean_data = S_video_style_data;

    // Only camera IDs remain: bind complete real DDR pages and preserve
    // duplicate rejection. Camera writer resets also invalidate every tag.
    video_latency_monitor #(.TIMESTAMP_BITS(16)) u_video_latency_monitor (
        .I_ref_clk(I_sys_clk),
        .I_capture_clk(S_csi_rx_clk),
        .I_ddr_clk(S_ddr_clk),.I_display_clk(S_hdmi_pixel_clk),.I_rst_n(S_camera_write_rst_n && S_camera_capture_fresh),
        .I_capture_start(S_raw_tuser && S_raw_tvalid),
        .I_wr_en(S_vi_ddr_wr_en),.I_wr_addr(S_vi_ddr_wr_addr),
        .I_rd_en(S_vo_ddr_rd_en),.I_rd_addr(S_vo_ddr_rd_addr),
        .I_display_de(S_clean_de),.I_display_user(S_clean_user),
        .O_valid(S_latency_valid),.O_latency_ticks(S_latency_ticks),
        .O_frame_id(S_source_frame_id),.O_repeat(S_latency_repeat),
        .O_frame_new(S_latency_new),.O_update(S_latency_update)
    );
    // Mode6: gray frame difference + directional erosion + dilation.
    // No stationary hold. Motion-mode servo steps reset the old-pose reference.
    // Repeated DDR pages retain the previous true-source result; a new static
    // camera frame clears it. This avoids repeat-frame flicker without a hold.
    motion_opening_detector #(.RUNTIME_CONTROL(1),.SOURCE_TAG_CONTROL(1)) u_motion_opening_detector (
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),
        .I_de(S_clean_de),.I_user(S_clean_user),.I_last(S_clean_last),
        .I_rgb(S_video_style_data),.I_enable(S_motion_detection_enable),
        .I_difference_threshold(S_cfg_motion_threshold),
        .I_source_update(S_latency_update),.I_source_valid(S_latency_valid),
        .I_source_frame_id(S_source_frame_id),
        .I_source_fresh(S_motion_capture_fresh_sync[1]),.O_update(S_motion_update),
        .O_valid(S_motion_valid),.O_scene_change(S_motion_scene_change),
        .O_left(S_motion_left),.O_right(S_motion_right),
        .O_top(S_motion_top),.O_bottom(S_motion_bottom),
        .O_changed_cells(S_motion_cells),.O_event_count(S_motion_events)
    );
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) S_motion_capture_fresh_sync<=2'b00;
        else S_motion_capture_fresh_sync<={S_motion_capture_fresh_sync[0],S_camera_capture_fresh};
    end
    video_buffer_health u_video_buffer_health (
        .I_capture_clk(S_csi_rx_clk),.I_ddr_clk(S_ddr_clk),
        .I_display_clk(S_hdmi_pixel_clk),.I_rst_n(S_rst_n),
        .I_capture_write(S_camera_fifo_write),.I_capture_full(S_camera_fifo_full),
        .I_capture_reset(S_camera_fifo_reset),
        .I_ddr_video_write(S_display_fifo_write),.I_ddr_video_full(S_display_fifo_full),
        .I_ddr_video_reset(S_display_fifo_reset),
        .I_ddr_command_write(S_command_fifo_write),.I_ddr_command_full(S_command_fifo_full),
        .I_display_read(S_display_fifo_read),.I_display_empty(S_display_fifo_empty),
        .I_display_monitor_enable(S_latency_valid && S_motion_capture_fresh_sync[1] && !S_display_fifo_reset),
        .O_faults(S_buffer_faults)
    );

    // Mode5 contains the centered, denoised, toned original. Tracking/OSD
    // are composed afterward; other modes leave target detection disabled.
    assign S_track_enable = (S_style_mode == 3'd5);
    assign S_video_display_data = S_track_enable ? S_color_track_data : S_video_style_data;
    color_rgb_tracker #(
        .IMG_WIDTH(1280), .IMG_HEIGHT(720),
        // Restore the earlier red S>=25% core sensitivity. Reject long thin
        // components by shape instead of discarding desaturated red balls.
        // Blue first uses its S>=50% core to resist computer-surface casts;
        // if none qualifies, recover compact weak-blue components bounded
        // to one quarter of the frame. No screen-edge exclusion is applied.
        .COLOR_MIN(8'd64), .COLOR_MARGIN(8'd24), .MIN_PIXELS(768),
        .GREEN_MIN(8'd6), .GREEN_MARGIN(8'd2), .GREEN_HUE_NUM(15), .GREEN_HUE_DEN(16),
        .RED_MIN(8'd16),.RED_MARGIN(8'd6),.RED_HUE_NUM(1),.RED_HUE_DEN(3),
        .BLUE_MIN(8'd8),.BLUE_MARGIN(8'd3),.BLUE_HUE_NUM(15),.BLUE_HUE_DEN(16),
        .HSV_SAT_SHIFT(2),.STRONG_MIN(8'd16),.STRONG_MARGIN(8'd6),.STRONG_SAT_SHIFT(2),
        .GREEN_HSV_SAT_SHIFT(3),.BLUE_HSV_SAT_SHIFT(3),.RED_STRONG_SAT_SHIFT(2),
        .BLUE_STRONG_SAT_SHIFT(1),
        .GREEN_STRONG_SAT_SHIFT(2),
        .GREEN_STRONG_MIN(8'd8),.GREEN_STRONG_MARGIN(8'd2),
        .BLUE_STRONG_MIN(8'd10),.BLUE_STRONG_MARGIN(8'd3),
        .BLOB_ENABLE(1),.BLOB_CELL_SHIFT(5),.BLOB_CORE_MIN_PIXELS(32),
        .BLOB_WEAK_CELL_MIN_PIXELS(64),.BLOB_HALO_MIN_PIXELS(4),.BLOB_MAX_PROCESS_CYCLES(45000),
        .BLOB_MIN_STRONG_PIXELS(128),.BLOB_STRONG_FILL_RATIO(4),.BLOB_WEAK_FALLBACK_ENABLE(1),
        .BLOB_WEAK_FALLBACK_COLOR_MASK(3'b100),.BLOB_WEAK_FALLBACK_MAX_AREA(1280*720/4),
        .BLOB_MAX_ASPECT_RATIO(4),
        .BLOB_TIGHT_BOUNDS(1),.BLOB_CORE_QUALITY_ENABLE(1),.EDGE_ACQUIRE_MARGIN(96),
        .HSV_ENABLE(1), .TRACK_ROI(0), .ROI_MARGIN(96), .MIN_H_RUN(3),
        .MIN_AREA(384), .MIN_FILL_RATIO(2), .STAT_PIPELINE(1),
        .STABLE_ENABLE(1),.KEEP_MIN_PIXELS(384),.CONFIRM_FRAMES(2),.HOLD_FRAMES(3),
        // Blob halo eligibility can move an edge by a 32-pixel cell. Smooth
        // these changes as well as small centroid changes, while keeping
        // distant target association and fresh-measurement servo gating intact.
        .ASSOC_DISTANCE(96),.COORD_DEADBAND(1),.SMOOTH_LIMIT(64),.SMOOTH_SHIFT(2)
    )u_color_rgb_tracker(
        .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
        .I_frame_start(S_clean_user), .I_line_end(S_clean_last),
        .I_de(S_clean_de), .I_rgb(S_video_clean_data), .I_enable(S_track_enable),
        .O_rgb(S_color_track_data), .O_valid(S_targets_valid), .O_update(S_targets_update),
        .O_measured(S_targets_measured),
        .O_center_x(S_targets_x), .O_center_y(S_targets_y),
        .O_left(S_targets_left), .O_right(S_targets_right),
        .O_top(S_targets_top), .O_bottom(S_targets_bottom), .O_pixel_count(S_targets_pixels),
        .O_measure_result(),.O_measure_match(),
        .O_measure_color(),.O_measure_bounds()
    );

    // The mailbox carries only newly measured coordinates. Camera freshness
    // expires even if HDMI keeps displaying and detecting an old DDR frame.
    // A photo request inhibits motion before DDR ownership changes. Returning
    // to live video requires a newly tagged camera page, never the saved photo.
    // Disabling tracking retains the last positional PWM in the controller.
    // Only RED color mode drives the actuators. MOTION remains display-only.
    assign S_servo_source_enable=S_track_enable && !S_photo_view_enable &&
        !S_photo_view_active && S_motion_capture_fresh_sync[1] && S_latency_valid;
    assign S_servo_source_update=S_targets_update[SERVO_TARGET_COLOR];
    assign S_servo_source_valid=S_targets_valid[SERVO_TARGET_COLOR] &&
        S_targets_measured[SERVO_TARGET_COLOR];
    assign S_servo_source_x=S_targets_x[SERVO_TARGET_COLOR*12 +:12];
    assign S_servo_source_y=S_targets_y[SERVO_TARGET_COLOR*12 +:12];
    assign S_motion_detection_enable=S_style_mode==3'd6;
    servo_target_cdc u_servo_target_cdc (
        .I_src_clk(S_hdmi_pixel_clk),.I_dst_clk(I_sys_clk),.I_rst_n(S_hdmi_rst_n),
        .I_src_enable(S_servo_source_enable),.I_src_update(S_servo_source_update),
        .I_src_valid(S_servo_source_valid),
        .I_src_x(S_servo_source_x),
        .I_src_y(S_servo_source_y),
        .O_dst_rst_n(S_servo_rst_n),.O_dst_enable(S_servo_mode),
        .O_dst_update(S_servo_update),.O_dst_valid(S_servo_valid),
        .O_dst_x(S_servo_x),.O_dst_y(S_servo_y)
    );
    color_servo_controller #(
        .CLK_HZ(50000000),.IMG_WIDTH(1280),.IMG_HEIGHT(720),
        .PAN_MIN_US(1100),.PAN_MAX_US(1900),
        .TILT_MIN_US(1200),.TILT_MAX_US(1800),
        .DEADZONE_X(32),.DEADZONE_Y(24),.MAX_STEP_US(20),
        .PAN_REVERSE(SERVO_PAN_REVERSE),.TILT_REVERSE(SERVO_TILT_REVERSE)
    ) u_color_servo_controller (
        .I_clk(I_sys_clk),.I_rst_n(S_servo_rst_n),
        .I_enable(S_servo_mode && S_camera_capture_fresh),.I_recenter(1'b0),
        .I_target_update(S_servo_update),.I_target_valid(S_servo_valid),
        .I_target_x(S_servo_x),.I_target_y(S_servo_y),
        .O_pan_pwm(O_servo_pan_pwm),.O_tilt_pwm(O_servo_tilt_pwm),
        .O_pan_us(),.O_tilt_us(),
        .O_tracking(S_servo_tracking),.O_centered(S_servo_centered)
    );
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin S_servo_tracking_sync<=0;S_servo_centered_sync<=0;end
        else begin
            S_servo_tracking_sync<={S_servo_tracking_sync[0],S_servo_tracking};
            S_servo_centered_sync<={S_servo_centered_sync[0],S_servo_centered};
        end
    end

    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  ),
        .O_cmd_fifo_full         ( S_command_fifo_full    ),
        .O_cmd_fifo_write        ( S_command_fifo_write   )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    ),
        .FPS_ENABLE ( 1    ),
        .SERVO_ENABLE(1),.SERVO_COLOR(SERVO_TARGET_COLOR),.MOTION_SERVO_ENABLE(0),
        .TELEMETRY_ENABLE(1),.LATENCY_ENABLE(1),.MOTION_ENABLE(1),.BUFFER_HEALTH_ENABLE(1),.THRESHOLD_ENABLE(1),
        .PERFORMANCE_ENABLE(0),.SIZE_ENABLE(0),.THREE_THRESHOLD_ONLY(1),.CONTEXT_DISPLAY(1)
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_clean_vsync      ),
        .I_video_hsync   ( S_clean_hsync      ),
        .I_video_de      ( S_clean_de         ),
        .I_video_user    ( S_clean_user       ),
        .I_video_last    ( S_clean_last       ),
        .I_debug_status  ( S_hdmi_debug_status),
        .I_style_mode    ( S_style_mode       ),
        .I_binary_auto(S_cfg_binary_auto),.I_binary_threshold(S_cfg_binary_threshold),
        .I_binary_effective(S_cfg_binary_effective),
        .I_sobel_weak(S_cfg_sobel_weak),.I_sobel_strong(S_cfg_sobel_strong),
        .I_cartoon_threshold(S_cfg_cartoon_threshold),
        .I_range_threshold(S_cfg_range_threshold),.I_motion_threshold(S_cfg_motion_threshold),
        .I_fps_bcd       ( S_fps_bcd          ),
        .I_servo_state   ( S_servo_display_state ),
        .I_latency_valid(S_latency_valid),.I_latency_ticks(S_latency_ticks),
        .I_source_frame_id(S_source_frame_id),
        .I_perf_resolution_valid(1'b0),
        .I_perf_width(16'd0),.I_perf_height(16'd0),
        .I_perf_rate_valid(1'b0),.I_perf_fps_x100(16'd0),
        .I_perf_pixels_valid(1'b0),.I_perf_pixels_per_second(32'd0),
        .I_perf_drop_valid(1'b0),.I_perf_drop_x100(16'd0),
        .I_motion_valid(S_motion_valid),.I_motion_scene_change(S_motion_scene_change),
        .I_motion_cells(S_motion_cells),.I_motion_events(S_motion_events),
        .I_motion_left(S_motion_left),.I_motion_right(S_motion_right),
        .I_motion_top(S_motion_top),.I_motion_bottom(S_motion_bottom),
        .I_buffer_faults(S_buffer_faults),
        .I_buffer_ready(S_latency_valid && S_motion_capture_fresh_sync[1]),
        .I_size_valid(3'b0),.I_size_length_x10(48'd0),.I_size_width_x10(48'd0),
        .I_targets_valid ( S_targets_valid    ),
        .I_targets_left  ( S_targets_left     ),
        .I_targets_right ( S_targets_right    ),
        .I_targets_top   ( S_targets_top      ),
        .I_targets_bottom( S_targets_bottom   ),
        .O_video_rd_en   (                   ),
        .I_video_rd_data ( S_video_display_data ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    // The unused YT8511 cannot negotiate or transmit while held in reset.
    // No network PLL, MAC, packet buffer or video tap is instantiated.
    assign O_phy_rst = 1'b0;

    photo_hdmi_select #(.RAW_CONTROL_DELAY(0)) u_photo_hdmi_select(
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),
        .I_view_active(S_photo_view_active),.I_view_valid(S_photo_view_valid),
        .I_raw_vsync(S_clean_vsync),.I_raw_de(S_clean_de),.I_raw_rgb(S_video_style_data),
        .I_live_vsync(S_hdmi_out_vsync),.I_live_de(S_hdmi_out_de),.I_live_rgb(S_hdmi_out_data),
        .O_vsync(S_tx_vsync),.O_de(S_tx_de),.O_rgb(S_tx_rgb),
        .O_replay_ready(S_photo_replay_ready));
    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_tx_vsync        ),
        .I_video_in_de      ( S_tx_de           ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_tx_rgb          ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule

// Keep the camera XCLK on the HDIO clock input, with an actual PLL-derived
// asynchronous reset. Data inputs are constants; there is no fabric clock
// used as IO data and no extra generated clock domain.
module camera_sensor_clock (
    input wire I_clk, input wire I_locked,
    output wire O_sensor_clk, output wire O_config_reset_n
);
    reg [15:0] startup_count;
    always @(posedge I_clk or negedge I_locked)
        if (!I_locked) startup_count <= 16'd0;
        else if (!startup_count[15]) startup_count <= startup_count + 1'b1;
    assign O_config_reset_n = startup_count[15];
    PH1P_LOGIC_HD_ODDR #(.ASYNCRST("ENABLE")) u_camera_clock_out (
        .clk(I_clk), .rst(!I_locked), .d0(1'b1), .d1(1'b0), .q(O_sensor_clk)
    );
endmodule
