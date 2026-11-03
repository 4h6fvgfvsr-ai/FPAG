// PH1P35 hard MCU AHB control/status mailbox. No image buffers or inferred RAM.
// AHB is the dedicated PL master window: CPU 0x80000000 => I_haddr == 0.
module camera_mcu_control #(
    parameter integer VIDEO_CLK_HZ = 74250000,
    parameter integer WATCHDOG_MS = 500
) (
    input wire I_bus_clk, input wire I_bus_rst_n,
    input wire [1:0] I_htrans, input wire I_hwrite,
    input wire [30:0] I_haddr, input wire [2:0] I_hsize,
    input wire [31:0] I_hwdata,
    output reg [31:0] O_hrdata, output wire [1:0] O_hresp,
    output wire O_hready,
    input wire I_video_clk, input wire I_video_rst_n,
    input wire I_frame_start,
    input wire [2:0] I_mode,
    input wire I_source_fresh, input wire I_motion_update,
    input wire I_motion_valid, input wire I_scene_change,
    input wire I_view_active, input wire I_capture_ready,
    input wire I_capture_accepted, input wire I_capture_failed,
    input wire I_fixed_thresholds,
    input wire [31:0] I_thresholds, input wire [31:0] I_counts,
    output reg O_alive,
    output reg O_mode_set, output reg [2:0] O_mode_value,
    output reg O_config_set, output reg [1:0] O_config_target,
    output reg [7:0] O_config_value,
    output reg O_motion_request, output reg O_manual_shutter
);
    function integer count_width;
        input integer n;
        integer v;
        begin
            count_width = 1;
            v = n - 1;
            while (v > 1) begin count_width = count_width + 1; v = v >> 1; end
        end
    endfunction
    localparam integer WATCHDOG_CYCLES = (VIDEO_CLK_HZ / 1000) * WATCHDOG_MS;
    localparam integer WATCHDOG_WIDTH = count_width(WATCHDOG_CYCLES);
    localparam [WATCHDOG_WIDTH-1:0] WATCHDOG_LAST = WATCHDOG_CYCLES - 1;
    localparam [7:0] RESULT_OK = 0, RESULT_OPCODE = 1, RESULT_ARGUMENT = 2,
                     RESULT_FIXED = 3, RESULT_BLOCKED = 4, RESULT_RESET = 5;

    // Either side resetting cancels the transaction on both sides. Deassertion
    // is synchronized independently, so stale toggles cannot become commands.
    wire bridge_reset_n = I_bus_rst_n & I_video_rst_n;
    (* ASYNC_REG = "TRUE" *) reg [1:0] bus_reset_pipe;
    (* ASYNC_REG = "TRUE" *) reg [1:0] video_reset_pipe;
    always @(posedge I_bus_clk or negedge bridge_reset_n)
        if (!bridge_reset_n) bus_reset_pipe <= 0;
        else bus_reset_pipe <= {bus_reset_pipe[0], 1'b1};
    always @(posedge I_video_clk or negedge bridge_reset_n)
        if (!bridge_reset_n) video_reset_pipe <= 0;
        else video_reset_pipe <= {video_reset_pipe[0], 1'b1};

    reg data_valid, data_write;
    // Decode during the AHB address phase. Registered one-hot read selects
    // avoid a wide address check + mux on the 150MHz -> 300MHz return path.
    reg [7:0] data_read_select;
    (* keep = 1 *) reg data_high_bad;
    reg data_control_bad;
    // Five inputs form one LUT. The following bank select needs only this
    // flag, HTRANS, HWRITE and three index bits: another single LUT.
    (* keep = 1 *) wire word_aligned = (I_haddr[1:0] == 0) &&
                                      (I_hsize == 3'b010);
    wire control_bad = !word_aligned ||
                       (I_hwrite && I_haddr[4:2] != 3'd2);
    reg error_final;
    reg bus_busy, command_request_bus;
    reg [31:0] command_payload_bus;
    reg command_ack_video;
    (* ASYNC_REG = "TRUE" *) reg [1:0] command_ack_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] command_request_sync;
    reg ack_guard_bus, request_guard_video, pending_video;
    reg [31:0] command_payload_video;
    reg [10:0] flags_hold_video, flags_snapshot_bus;
    reg [31:0] thresholds_hold_video, thresholds_snapshot_bus;
    reg [31:0] counts_hold_video, counts_snapshot_bus;
    reg [31:0] capture_hold_video, capture_snapshot_bus;
    reg [15:0] completed_sequence_video, completed_sequence_bus;
    reg [7:0] result_hold_video, result_snapshot_bus;
    reg [15:0] accepted_count_video, failed_count_video;
    reg [WATCHDOG_WIDTH-1:0] watchdog_count_video;
    reg motion_valid_recent;
    reg [7:0] command_result;
    wire motion_now = (I_mode == 3'd6) && I_source_fresh &&
                      !I_scene_change &&
                      (I_motion_update ? I_motion_valid : motion_valid_recent);
    wire transfer_invalid = data_valid &&
        (data_high_bad || data_control_bad ||
         (data_write && (bus_busy || !bus_reset_pipe[1])));
    wire transfer_error = error_final || transfer_invalid;
    // Preserve the original two-cycle AHB ERROR response and zero-wait reads.
    assign O_hresp = transfer_error ? 2'b01 : 2'b00;
    assign O_hready = !transfer_error || error_final;
    // Preserve four small mux cuts. Each pair fits in one LUT4, and the
    // final OR plus high-address qualification fits in one LUT5. High
    // address validation is registered separately, never in the bank-select
    // input cone from the 300MHz MCU to the 150MHz AHB clock.
    (* keep = 1 *) wire [31:0] read_pair0 =
        ({32{data_read_select[0]}} & 32'h43414d52) |
        ({32{data_read_select[1]}} & {31'b0, bus_busy});
    (* keep = 1 *) wire [31:0] read_pair1 =
        ({32{data_read_select[2]}} & command_payload_bus) |
        ({32{data_read_select[3]}} & {21'b0, flags_snapshot_bus});
    (* keep = 1 *) wire [31:0] read_pair2 =
        ({32{data_read_select[4]}} & thresholds_snapshot_bus) |
        ({32{data_read_select[5]}} & counts_snapshot_bus);
    (* keep = 1 *) wire [31:0] read_pair3 =
        ({32{data_read_select[6]}} & capture_snapshot_bus) |
        ({32{data_read_select[7]}} & {8'b0, result_snapshot_bus, completed_sequence_bus});
    always @* O_hrdata = (read_pair0 | read_pair1 | read_pair2 | read_pair3) &
                         {32{!data_high_bad}};

    always @(posedge I_bus_clk or negedge bridge_reset_n) begin
        if (!bridge_reset_n) begin
            data_valid <= 0; data_write <= 0; data_read_select <= 0; data_high_bad <= 0; data_control_bad <= 0;
            error_final <= 0; bus_busy <= 0; command_request_bus <= 0;
            command_payload_bus <= 0; command_ack_sync <= 0; ack_guard_bus <= 0;
            flags_snapshot_bus <= 0; thresholds_snapshot_bus <= 0;
            counts_snapshot_bus <= 0; capture_snapshot_bus <= 0;
            completed_sequence_bus <= 0; result_snapshot_bus <= RESULT_RESET;
        end else if (!bus_reset_pipe[1]) begin
            data_valid <= 0; data_write <= 0; data_read_select <= 0; data_high_bad <= 0; data_control_bad <= 0;
            error_final <= 0; bus_busy <= 0; command_request_bus <= 0;
            command_payload_bus <= 0; command_ack_sync <= 0; ack_guard_bus <= 0;
            flags_snapshot_bus <= 0; thresholds_snapshot_bus <= 0;
            counts_snapshot_bus <= 0; capture_snapshot_bus <= 0;
            completed_sequence_bus <= 0; result_snapshot_bus <= RESULT_RESET;
        end else begin
            command_ack_sync <= {command_ack_sync[0], command_ack_video};
            if (bus_busy && command_ack_sync[1] == command_request_bus) begin
                if (!ack_guard_bus) ack_guard_bus <= 1;
                else begin
                    // Response fields are held until the next command completes.
                    // req/ack synchronizers + this guard allow the whole bus to settle.
                    flags_snapshot_bus <= flags_hold_video;
                    thresholds_snapshot_bus <= thresholds_hold_video;
                    counts_snapshot_bus <= counts_hold_video;
                    capture_snapshot_bus <= capture_hold_video;
                    completed_sequence_bus <= completed_sequence_video;
                    result_snapshot_bus <= result_hold_video;
                    bus_busy <= 0; ack_guard_bus <= 0;
                end
            end else ack_guard_bus <= 0;

            if (O_hready) begin
                data_valid <= I_htrans[1];
                data_write <= I_hwrite;
                data_high_bad <= (I_haddr[30:5] != 0);
                data_control_bad <= control_bad;
                data_read_select <= (I_htrans[1] && !I_hwrite && word_aligned) ?
                                    (8'b1 << I_haddr[4:2]) : 8'b0;
                error_final <= 0;
                if (data_valid && data_write && !transfer_error) begin
                    command_payload_bus <= I_hwdata;
                    command_request_bus <= !command_request_bus;
                    bus_busy <= 1; ack_guard_bus <= 0;
                end
            end else if (transfer_error) error_final <= 1;
        end
    end

    // Decode only the already captured mailbox. Reserved bits must be zero.
    // Mode and threshold commands have no image-side effects outside a frame edge.
    always @* begin
        command_result = RESULT_OK;
        case (command_payload_video[7:0])
            8'd1: if (command_payload_video[31:8] != 0)
                      command_result = RESULT_ARGUMENT;
            8'd2: if (command_payload_video[31:11] != 0 ||
                      command_payload_video[10:8] > 6)
                      command_result = RESULT_ARGUMENT;
            8'd3: if (command_payload_video[31:8] != 0)
                      command_result = RESULT_ARGUMENT;
                  else if (!motion_now || I_view_active || !I_capture_ready)
                      command_result = RESULT_BLOCKED;
            8'd4: if (command_payload_video[31:24] != 0 ||
                      command_payload_video[15:8] < 1 ||
                      command_payload_video[15:8] > 3)
                      command_result = RESULT_ARGUMENT;
                  else if (I_fixed_thresholds) command_result = RESULT_FIXED;
            8'd5: if (command_payload_video[31:8] != 0)
                      command_result = RESULT_ARGUMENT;
                  else if (I_mode != 0 || !I_source_fresh ||
                           I_scene_change || I_view_active || !I_capture_ready)
                      command_result = RESULT_BLOCKED;
            default: command_result = RESULT_OPCODE;
        endcase
    end
    always @(posedge I_video_clk or negedge bridge_reset_n) begin
        if (!bridge_reset_n) begin
            command_request_sync <= 0; request_guard_video <= 0;
            command_ack_video <= 0; pending_video <= 0;
            command_payload_video <= 0; flags_hold_video <= 0;
            thresholds_hold_video <= 0; counts_hold_video <= 0;
            capture_hold_video <= 0; completed_sequence_video <= 0;
            result_hold_video <= RESULT_RESET; accepted_count_video <= 0;
            failed_count_video <= 0; watchdog_count_video <= 0;
            motion_valid_recent <= 0; O_alive <= 0;
            O_mode_set <= 0; O_mode_value <= 0; O_config_set <= 0;
            O_config_target <= 0; O_config_value <= 0;
            O_motion_request <= 0; O_manual_shutter <= 0;
        end else if (!video_reset_pipe[1]) begin
            command_request_sync <= 0; request_guard_video <= 0;
            command_ack_video <= 0; pending_video <= 0;
            command_payload_video <= 0; flags_hold_video <= 0;
            thresholds_hold_video <= 0; counts_hold_video <= 0;
            capture_hold_video <= 0; completed_sequence_video <= 0;
            result_hold_video <= RESULT_RESET; accepted_count_video <= 0;
            failed_count_video <= 0; watchdog_count_video <= 0;
            motion_valid_recent <= 0; O_alive <= 0;
            O_mode_set <= 0; O_mode_value <= 0; O_config_set <= 0;
            O_config_target <= 0; O_config_value <= 0;
            O_motion_request <= 0; O_manual_shutter <= 0;
        end else begin
            command_request_sync <= {command_request_sync[0], command_request_bus};
            O_mode_set <= 0; O_config_set <= 0;
            O_motion_request <= 0; O_manual_shutter <= 0;
            if (!I_source_fresh || I_scene_change || I_mode != 6)
                motion_valid_recent <= 0;
            else if (I_motion_update) motion_valid_recent <= I_motion_valid;
            if (I_capture_accepted) accepted_count_video <= accepted_count_video + 1'b1;
            if (I_capture_failed) failed_count_video <= failed_count_video + 1'b1;

            if (O_alive) begin
                if (watchdog_count_video == WATCHDOG_LAST) begin
                    watchdog_count_video <= 0; O_alive <= 0;
                end else watchdog_count_video <= watchdog_count_video + 1'b1;
            end else watchdog_count_video <= 0;

            if (command_request_sync[1] != command_ack_video && !pending_video) begin
                if (!request_guard_video) request_guard_video <= 1;
                else begin
                    command_payload_video <= command_payload_bus;
                    pending_video <= 1; request_guard_video <= 0;
                end
            end else request_guard_video <= 0;

            if (pending_video && I_frame_start) begin
                if (command_result == RESULT_OK) begin
                    case (command_payload_video[7:0])
                        1: begin O_alive <= 1; watchdog_count_video <= 0; end
                        2: begin O_mode_set <= 1;
                                 O_mode_value <= command_payload_video[10:8]; end
                        3: O_motion_request <= 1;
                        4: begin O_config_set <= 1;
                                 O_config_target <= command_payload_video[9:8];
                                 O_config_value <= command_payload_video[23:16]; end
                        5: O_manual_shutter <= 1;
                    endcase
                end
                flags_hold_video <= {I_mode,
                    ((command_payload_video[7:0] == 1 && command_result == RESULT_OK) ||
                     (O_alive && watchdog_count_video != WATCHDOG_LAST)),
                    I_fixed_thresholds, I_scene_change, I_capture_ready,
                    I_view_active, motion_now, I_source_fresh, (I_mode == 6)};
                thresholds_hold_video <= I_thresholds;
                counts_hold_video <= I_counts;
                capture_hold_video <= {
                    (failed_count_video + {15'b0, I_capture_failed}),
                    (accepted_count_video + {15'b0, I_capture_accepted})};
                completed_sequence_video <= completed_sequence_video + 1'b1;
                result_hold_video <= command_result;
                command_ack_video <= command_request_sync[1];
                pending_video <= 0;
            end
        end
    end
endmodule
