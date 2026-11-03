// Coarse FPGA frame-difference detector: default 1280x720 -> 80x45 cells.
// Every cell samples the CENTER pixel of its 16x16 region, not an area average.
// Gray = rounded (R + 2*G + B)/4. One pixel/clock; no back-pressure.
// Previous frame is a synchronous 3600x9 RAM (8-bit gray + committed epoch).
// No RAM reset/clear sweep is required; a full frame warms the reference.
// I_de qualifies a pixel. I_user marks its first pixel; I_last its line end.
// I_source_fresh is a held camera-running/timeout level, NOT a per-frame pulse.
// Low enable/fresh immediately removes the result and discards the reference.
// A fresh camera may repeat DDR frames: identical frames yield zero motion.
// Only a COMPLETE, correctly framed image commits bbox/count/status.
// Premature SOF, short/long lines, or source invalidation abort the reference.
// >=35% changed cells signals scene/exposure change and suppresses motion.
// This is coarse motion localization, not full-resolution object segmentation.
`ifndef MOTION_FRAME_DETECTOR_V
`define MOTION_FRAME_DETECTOR_V

module motion_frame_detector #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer DIFFERENCE_THRESHOLD = 24,
    parameter integer MIN_CHANGED_CELLS = 8,
    parameter integer SCENE_CHANGE_PERCENT = 35,
    parameter integer RUNTIME_CONTROL = 0
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_de,
    input wire I_user,
    input wire I_last,
    input wire [23:0] I_rgb,
    input wire I_enable,
    input wire I_source_fresh,
    input wire [7:0] I_difference_threshold,
    output reg O_update,
    output reg O_valid,
    output reg O_scene_change,
    output reg [11:0] O_left,
    output reg [11:0] O_right,
    output reg [11:0] O_top,
    output reg [11:0] O_bottom,
    output reg [15:0] O_changed_cells,
    output reg [15:0] O_event_count
);
    localparam integer GRID_WIDTH = IMG_WIDTH/16;
    localparam integer GRID_HEIGHT = IMG_HEIGHT/16;
    localparam integer GRID_CELLS = GRID_WIDTH*GRID_HEIGHT;
    localparam integer PIXELS = IMG_WIDTH*IMG_HEIGHT;
    localparam integer INDEX_BITS = GRID_CELLS < 2 ? 1 : $clog2(GRID_CELLS+1);
    localparam integer COL_BITS = GRID_WIDTH < 2 ? 1 : $clog2(GRID_WIDTH);
    localparam integer ROW_BITS = GRID_HEIGHT < 2 ? 1 : $clog2(GRID_HEIGHT);
    localparam integer COUNT_BITS = PIXELS < 2 ? 1 : $clog2(PIXELS+1);
    localparam integer SCENE_CELLS = (GRID_CELLS*SCENE_CHANGE_PERCENT+99)/100;
    // Supported dimensions are multiples of 16, from 16 to 4096 on each axis.
    // The default 3600x9 implementation fits two 18-Kbit ERAMs.
    (* ram_style = "bram" *) reg [8:0] previous_grid [0:GRID_CELLS-1];
    reg [8:0] previous_read;
    reg frame_active, previous_valid, previous_epoch;
    reg frame_reference_valid, frame_reference_epoch, frame_epoch;
    reg frame_bad_reference;
    reg [11:0] pixel_x, pixel_y;
    reg [COUNT_BITS-1:0] pixel_count;
    reg [INDEX_BITS-1:0] sample_index, changed_count;
    reg [COL_BITS-1:0] bbox_left, bbox_right;
    reg [ROW_BITS-1:0] bbox_top, bbox_bottom;
    reg sample_pending, sample_reference_valid;
    reg [7:0] sampled_gray;
    reg [COL_BITS-1:0] sampled_col;
    reg [ROW_BITS-1:0] sampled_row;
    reg [7:0] frame_difference_threshold;
    wire [31:0] difference_threshold=RUNTIME_CONTROL ?
        {24'd0,frame_difference_threshold} : DIFFERENCE_THRESHOLD;
    // Configuration is already output-frame aligned by the style stream.
    // SOF capture keeps one complete detector frame on the same threshold.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) frame_difference_threshold<=DIFFERENCE_THRESHOLD[7:0];
        else if(I_de && I_user) frame_difference_threshold<=I_difference_threshold;
    end

    wire [9:0] gray_sum = {2'b0,I_rgb[23:16]} + {1'b0,I_rgb[15:8],1'b0} +
                           {2'b0,I_rgb[7:0]} + 10'd2;
    wire [7:0] gray = gray_sum[9:2];
    wire line_is_correct = (I_last == (pixel_x == IMG_WIDTH-1));
    wire sample_now = I_de && !I_user && I_enable && I_source_fresh &&
                      frame_active && line_is_correct &&
                      pixel_x[3:0] == 4'd8 && pixel_y[3:0] == 4'd8 &&
                      sample_index < GRID_CELLS;
    wire [7:0] difference = sampled_gray >= previous_read[7:0] ?
                           sampled_gray-previous_read[7:0] :
                           previous_read[7:0]-sampled_gray;
    // Read-before-write synchronous RAM; no asynchronous read or memory reset.
    always @(posedge I_clk) begin
        if(sample_now) begin
            previous_read <= previous_grid[sample_index];
            previous_grid[sample_index] <= {frame_epoch,gray};
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            frame_active<=0; previous_valid<=0; previous_epoch<=0;
            frame_reference_valid<=0; frame_reference_epoch<=0; frame_epoch<=0;
            frame_bad_reference<=0; pixel_x<=0;pixel_y<=0;pixel_count<=0;
            sample_index<=0; changed_count<=0;
            bbox_left<=0;bbox_right<=0;bbox_top<=0;bbox_bottom<=0;
            sample_pending<=0;sample_reference_valid<=0;sampled_gray<=0;
            sampled_col<=0;sampled_row<=0;
            O_update<=0;O_valid<=0;O_scene_change<=0;
            O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
            O_changed_cells<=0;O_event_count<=0;
        end else begin
            O_update<=0;
            sample_pending<=sample_now;
            if(sample_now) begin
                sampled_gray<=gray;
                sampled_col<=pixel_x[11:4];
                sampled_row<=pixel_y[11:4];
                sample_reference_valid<=frame_reference_valid;
                sample_index<=sample_index+1'b1;
            end
            if(!I_enable || !I_source_fresh) begin
                frame_active<=0;previous_valid<=0;frame_reference_valid<=0;
                frame_bad_reference<=0;sample_pending<=0;sample_reference_valid<=0;
                pixel_x<=0;pixel_y<=0;pixel_count<=0;
                sample_index<=0;changed_count<=0;
                O_valid<=0;O_scene_change<=0;O_changed_cells<=0;
                O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                // Event count is a session counter and survives disable/unplug.
            end else begin
                if(sample_pending && frame_active && sample_reference_valid) begin
                    if(previous_read[8] != frame_reference_epoch)
                        frame_bad_reference<=1;
                    else if(difference >= difference_threshold) begin
                        changed_count<=changed_count+1'b1;
                        if(changed_count == 0 || sampled_col < bbox_left) bbox_left<=sampled_col;
                        if(changed_count == 0 || sampled_col > bbox_right) bbox_right<=sampled_col;
                        if(changed_count == 0 || sampled_row < bbox_top) bbox_top<=sampled_row;
                        if(changed_count == 0 || sampled_row > bbox_bottom) bbox_bottom<=sampled_row;
                    end
                end
                if(I_de) begin
                    if(I_user) begin
                        // SOF consumes pixel (0,0); first sample is at (8,8).
                        frame_active<=1;pixel_x<=1;pixel_y<=0;pixel_count<=1;
                        sample_index<=0;changed_count<=0;
                        frame_epoch<=!previous_epoch;
                        frame_reference_epoch<=previous_epoch;
                        frame_bad_reference<=0;
                        sample_pending<=0;sample_reference_valid<=0;
                        bbox_left<=0;bbox_right<=0;bbox_top<=0;bbox_bottom<=0;
                        if(frame_active || I_last) begin
                            // A new SOF before frame completion rejects partial RAM contents.
                            previous_valid<=0;frame_reference_valid<=0;
                            O_valid<=0;O_scene_change<=0;O_changed_cells<=0;
                            O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                            if(I_last) frame_active<=0;
                        end else
                            frame_reference_valid<=previous_valid;
                    end else if(frame_active) begin
                        if(!line_is_correct) begin
                            frame_active<=0;previous_valid<=0;frame_reference_valid<=0;
                            sample_pending<=0;
                            O_valid<=0;O_scene_change<=0;O_changed_cells<=0;
                            O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                        end else if(I_last && pixel_y == IMG_HEIGHT-1) begin
                            frame_active<=0;sample_pending<=0;
                            O_update<=1;
                            if(pixel_count == PIXELS-1 && sample_index == GRID_CELLS) begin
                                // Full current frame becomes the next reference even after
                                // warmup or an intentionally rejected global scene change.
                                previous_valid<=1;previous_epoch<=frame_epoch;
                                if(frame_reference_valid && !frame_bad_reference) begin
                                    O_changed_cells<=changed_count;
                                    O_scene_change<=changed_count >= SCENE_CELLS;
                                    if(changed_count >= MIN_CHANGED_CELLS && changed_count < SCENE_CELLS) begin
                                        O_valid<=1;
                                        O_left<={bbox_left,4'b0};
                                        O_right<={bbox_right,4'b1111};
                                        O_top<={bbox_top,4'b0};
                                        O_bottom<={bbox_bottom,4'b1111};
                                        if(!O_valid && O_event_count != 16'hffff)
                                            O_event_count<=O_event_count+1'b1;
                                    end else begin
                                        O_valid<=0;
                                        O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                                    end
                                end else begin
                                    O_valid<=0;O_scene_change<=0;O_changed_cells<=0;
                                    O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                                end
                            end else begin
                                previous_valid<=0;frame_reference_valid<=0;
                                O_valid<=0;O_scene_change<=0;O_changed_cells<=0;
                                O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                            end
                        end else begin
                            pixel_count<=pixel_count+1'b1;
                            if(I_last) begin pixel_x<=0;pixel_y<=pixel_y+1'b1;end
                            else pixel_x<=pixel_x+1'b1;
                        end
                    end else begin
                        // Valid pixels without SOF cannot form a usable reference.
                        previous_valid<=0;
                        O_valid<=0;O_scene_change<=0;O_changed_cells<=0;
                        O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;
                    end
                end
            end
        end
    end
endmodule
`endif

