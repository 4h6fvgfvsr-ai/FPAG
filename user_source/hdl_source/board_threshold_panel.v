`ifndef BOARD_THRESHOLD_PANEL_V
`define BOARD_THRESHOLD_PANEL_V
// HX1P35B local threshold panel, clocked entirely in the HDMI pixel domain.
// KEY2/KEY3/KEY4 increment the hundreds/tens/units respectively in the owner.
// SW4 up = 1 = fixed thresholds; down = 0 = adjustable thresholds.
// DIG[0..7] = a,b,c,d,e,f,g,DP; SEL0..SEL2 = leftmost three digits.
// Optional SEL4..SEL7 show committed motion snapshots; SEL3 separates groups.
// The manual's text says SEL is active high, but its PNP9012 high-side
// schematic implies active-low FPGA controls. Both polarities are selectable.
module board_threshold_panel #(
    parameter integer CLK_HZ = 75002000,
    parameter integer DEBOUNCE_CYCLES = 750000,
    parameter integer SCAN_HZ = 1000,
    parameter integer BLANK_CYCLES = 150,
    parameter integer DIGIT_ACTIVE_LOW = 1,
    parameter integer SEGMENT_ACTIVE_LOW = 1,
    parameter integer PHOTO_ENABLE = 0,
    parameter integer THRESHOLD_VALID_ENABLE = 0,
    parameter integer CAPTURE_COUNT_ENABLE = 0,
    parameter integer DUAL_PHOTO_ENABLE = 0
) (
    input wire I_clk,
    input wire I_rst_n,
    input wire [2:0] I_keys_n,
    input wire I_sw4_n,
    input wire [7:0] I_value,
    input wire I_threshold_valid,
    input wire I_photo_mode,
    input wire [2:0] I_photo_status,
    input wire [3:0] I_photo_error,
    input wire [11:0] I_photo_number_bcd,
    input wire [15:0] I_capture_count_bcd,
    input wire [11:0] I_manual_display_bcd,I_motion_display_bcd,
    input wire [2:0] I_manual_display_status,I_motion_display_status,
    input wire [3:0] I_manual_display_error,I_motion_display_error,
    output reg [2:0] O_digit_press,
    output wire [2:0] O_digit_held,
    output wire O_fixed_thresholds,
    output reg [7:0] O_seg,
    output reg [7:0] O_sel
);
    function integer counter_width;
        input integer limit;
        integer n;
        begin
            n = limit - 1;
            counter_width = 0;
            while (n > 0) begin
                counter_width = counter_width + 1;
                n = n >> 1;
            end
            if (counter_width < 1) counter_width = 1;
        end
    endfunction

    // SCAN_HZ is the refresh rate of a complete three/seven-digit frame. Constant
    // divisions only size a counter; the data path contains no multiplier.
    localparam integer DB_TICKS = (DEBOUNCE_CYCLES < 1) ? 1 : DEBOUNCE_CYCLES;
    localparam integer DB_WIDTH = counter_width(DB_TICKS);
    localparam integer SCAN_DIGITS = DUAL_PHOTO_ENABLE ? 6:CAPTURE_COUNT_ENABLE ? 7 : 3;
    localparam integer SLOT_RAW = CLK_HZ / ((SCAN_HZ < 1 ? 1 : SCAN_HZ) * SCAN_DIGITS);
    localparam integer SLOT_TICKS = (SLOT_RAW < 2) ? 2 : SLOT_RAW;
    localparam integer SLOT_WIDTH = counter_width(SLOT_TICKS);
    localparam integer BLANK_TICKS = (BLANK_CYCLES < 1) ? 1 :
        ((BLANK_CYCLES >= SLOT_TICKS) ? SLOT_TICKS - 1 : BLANK_CYCLES);
    localparam [7:0] SEG_OFF = SEGMENT_ACTIVE_LOW ? 8'hff : 8'h00;
    localparam [7:0] SEL_OFF = DIGIT_ACTIVE_LOW ? 8'hff : 8'h00;

    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0, ASYNC_REG="TRUE", syn_preserve=1, syn_keep=1 *) reg [2:0] keys_meta /* synthesis keep */;
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0, ASYNC_REG="TRUE", syn_preserve=1, syn_keep=1 *) reg [2:0] keys_sync /* synthesis keep */;
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0, ASYNC_REG="TRUE", syn_preserve=1, syn_keep=1 *) reg sw4_meta /* synthesis keep */;
    (* keep=1, preserve=1, allow_forward_retiming=0, allow_backward_retiming=0, ASYNC_REG="TRUE", syn_preserve=1, syn_keep=1 *) reg sw4_sync /* synthesis keep */;
    reg [2:0] keys_stable;
    reg sw4_stable;
    reg [DB_WIDTH-1:0] key_count [0:2];
    reg [DB_WIDTH-1:0] sw4_count;
    integer key_index;

    assign O_fixed_thresholds = sw4_stable;
    assign O_digit_held = ~keys_stable;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            keys_meta <= 3'b111;
            keys_sync <= 3'b111;
            sw4_meta <= 1'b1;
            sw4_sync <= 1'b1;
        end else begin
            keys_meta <= I_keys_n;
            keys_sync <= keys_meta;
            sw4_meta <= I_sw4_n;
            sw4_sync <= sw4_meta;
        end
    end

    // A mismatch must remain stable for the full interval in either direction.
    // Release is debounced too, so bounce cannot re-arm an already held key.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            keys_stable <= 3'b111;
            sw4_stable <= 1'b1;
            sw4_count <= 0;
            O_digit_press <= 3'b000;
            for (key_index = 0; key_index < 3; key_index = key_index + 1)
                key_count[key_index] <= 0;
        end else begin
            O_digit_press <= 3'b000;
            for (key_index = 0; key_index < 3; key_index = key_index + 1) begin
                if (keys_sync[key_index] == keys_stable[key_index]) begin
                    key_count[key_index] <= 0;
                end else if (key_count[key_index] == DB_TICKS - 1) begin
                    key_count[key_index] <= 0;
                    keys_stable[key_index] <= keys_sync[key_index];
                    if (!keys_sync[key_index]) O_digit_press[key_index] <= 1'b1;
                end else begin
                    key_count[key_index] <= key_count[key_index] + 1'b1;
                end
            end
            if (sw4_sync == sw4_stable) begin
                sw4_count <= 0;
            end else if (sw4_count == DB_TICKS - 1) begin
                sw4_count <= 0;
                sw4_stable <= sw4_sync;
            end else begin
                sw4_count <= sw4_count + 1'b1;
            end
        end
    end

    // Eight-cycle shift/add-3 conversion, independent of the scanner. Publishing
    // only completed results and taking a scan-frame snapshot avoids torn digits.
    reg [7:0] bcd_input;
    reg [7:0] bcd_binary_work;
    reg [11:0] bcd_work;
    reg [11:0] bcd_adjusted;
    reg [11:0] bcd_ready;
    reg [2:0] bcd_step;
    reg bcd_busy;
    integer nibble_index;
    always @* begin
        bcd_adjusted = bcd_work;
        for (nibble_index = 0; nibble_index < 3; nibble_index = nibble_index + 1)
            if (bcd_work[nibble_index*4 +: 4] >= 5)
                bcd_adjusted[nibble_index*4 +: 4] =
                    bcd_work[nibble_index*4 +: 4] + 4'd3;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            bcd_input <= 0;
            bcd_binary_work <= 0;
            bcd_work <= 0;
            bcd_ready <= 0;
            bcd_step <= 0;
            bcd_busy <= 1'b0;
        end else if (!bcd_busy) begin
            if (I_value != bcd_input) begin
                bcd_input <= I_value;
                bcd_binary_work <= I_value;
                bcd_work <= 0;
                bcd_step <= 0;
                bcd_busy <= 1'b1;
            end
        end else begin
            bcd_work <= {bcd_adjusted[10:0], bcd_binary_work[7]};
            bcd_binary_work <= {bcd_binary_work[6:0], 1'b0};
            if (bcd_step == 7) begin
                bcd_ready <= {bcd_adjusted[10:0], bcd_binary_work[7]};
                bcd_busy <= 1'b0;
            end else begin
                bcd_step <= bcd_step + 1'b1;
            end
        end
    end

    function [7:0] decimal_segments;
        input [3:0] digit;
        begin
            case (digit)
                0: decimal_segments = 8'h3f;
                1: decimal_segments = 8'h06;
                2: decimal_segments = 8'h5b;
                3: decimal_segments = 8'h4f;
                4: decimal_segments = 8'h66;
                5: decimal_segments = 8'h6d;
                6: decimal_segments = 8'h7d;
                7: decimal_segments = 8'h07;
                8: decimal_segments = 8'h7f;
                9: decimal_segments = 8'h6f;
                4'ha: decimal_segments = 8'h40; // -
                4'hb: decimal_segments = 8'h6d; // S
                4'hc: decimal_segments = 8'h5e; // d
                4'he: decimal_segments = 8'h79; // E
                default: decimal_segments = 8'h00;
            endcase
        end
    endfunction

    reg [SLOT_WIDTH-1:0] slot_count;
    reg [2:0] scan_index;
    reg [11:0] frame_bcd;
    reg [15:0] frame_count_bcd;
    reg [3:0] scan_digit;
    // Toggle every quarter second using the existing scan-frame tick rather
    // than adding a 25-bit pixel-clock divider. One complete blink is ~2 Hz.
    localparam integer BLINK_FRAMES = SCAN_HZ < 4 ? 1 : SCAN_HZ / 4;
    localparam integer BLINK_WIDTH = counter_width(BLINK_FRAMES);
    reg [BLINK_WIDTH-1:0] blink_count;
    reg blink_on;
    reg [11:0] display_ready;
    wire [11:0] motion_display_ready;
    function [11:0] photo_digits;
        input [2:0] status;
        input [3:0] error_code;
        input [11:0] number;
        input blink;
        reg [3:0] tens,units;
        begin
            tens=error_code>=10 ? 4'd1:4'd0;
            units=error_code>=10 ? error_code-4'd10:error_code;
            case(status)
                0:photo_digits=12'hbcd;
                2:photo_digits=blink ? 12'haaa:12'hddd;
                4:photo_digits={4'he,tens,units};
                default:photo_digits=number;
            endcase
        end
    endfunction
    assign motion_display_ready=photo_digits(I_motion_display_status,I_motion_display_error,I_motion_display_bcd,blink_on);
    wire [3:0] error_units = I_photo_error >= 10 ? I_photo_error - 4'd10 : I_photo_error;
    wire [3:0] error_tens = I_photo_error >= 10 ? 4'd1 : 4'd0;
    always @* begin
        display_ready = THRESHOLD_VALID_ENABLE && !I_threshold_valid ? 12'haaa : bcd_ready;
        if (PHOTO_ENABLE && I_photo_mode) begin
            case (I_photo_status)
                0: display_ready = 12'hbcd; // "Sd "
                2: display_ready = blink_on ? 12'haaa : 12'hddd;
                4: display_ready = {4'he,error_tens,error_units};
                default: display_ready = I_photo_number_bcd;
            endcase
        end
        if(DUAL_PHOTO_ENABLE)display_ready=photo_digits(I_manual_display_status,I_manual_display_error,I_manual_display_bcd,blink_on);
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin blink_count <= 0; blink_on <= 1; end
        else if (DUAL_PHOTO_ENABLE ? (I_manual_display_status!=2 && I_motion_display_status!=2):
                     (!PHOTO_ENABLE || !I_photo_mode || I_photo_status != 2)) begin
            blink_count <= 0; blink_on <= 1;
        end else if (slot_count == 0 && scan_index == 0) begin
            if (blink_count == BLINK_FRAMES-1) begin blink_count <= 0; blink_on <= ~blink_on; end
            else blink_count <= blink_count + 1'b1;
        end
    end
    wire [7:0] segment_on = decimal_segments(scan_digit);
    // Leave SEL3 blank between the original left three and right four digits.
    wire [2:0] physical_index = scan_index<3 ? scan_index:scan_index+(DUAL_PHOTO_ENABLE ? 3'd2:3'd1);
    wire [7:0] selected_digit = 8'b00000001 << physical_index;
    always @* begin
        case (scan_index)
            0: scan_digit = frame_bcd[11:8];
            1: scan_digit = frame_bcd[7:4];
            2: scan_digit = frame_bcd[3:0];
            3: scan_digit = DUAL_PHOTO_ENABLE ? frame_count_bcd[11:8]:frame_count_bcd[15:12];
            4: scan_digit = DUAL_PHOTO_ENABLE ? frame_count_bcd[7:4]:frame_count_bcd[11:8];
            5: scan_digit = DUAL_PHOTO_ENABLE ? frame_count_bcd[3:0]:frame_count_bcd[7:4];
            default: scan_digit = frame_count_bcd[3:0];
        endcase
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            slot_count <= 0;
            scan_index <= 0;
            frame_bcd <= 0;
            frame_count_bcd <= 0;
            O_seg <= SEG_OFF;
            O_sel <= SEL_OFF;
        end else begin
            if (slot_count == 0) begin
                O_sel <= SEL_OFF;
                O_seg <= SEG_OFF;
                if (scan_index == 0) begin
                    frame_bcd <= display_ready;
                    if(DUAL_PHOTO_ENABLE)frame_count_bcd<={4'b0,motion_display_ready};
                    else if(CAPTURE_COUNT_ENABLE) frame_count_bcd <= I_capture_count_bcd;
                end
            end else if (slot_count == BLANK_TICKS) begin
                O_seg <= SEGMENT_ACTIVE_LOW ? ~segment_on : segment_on;
                O_sel <= DIGIT_ACTIVE_LOW ? ~selected_digit : selected_digit;
            end
            if (slot_count == SLOT_TICKS - 1) begin
                slot_count <= 0;
                scan_index <= (scan_index == SCAN_DIGITS-1) ? 0 : scan_index + 1'b1;
            end else begin
                slot_count <= slot_count + 1'b1;
            end
        end
    end
endmodule
`endif
