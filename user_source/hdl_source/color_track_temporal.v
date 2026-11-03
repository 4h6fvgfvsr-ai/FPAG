`ifndef COLOR_TRACK_TEMPORAL_V
`define COLOR_TRACK_TEMPORAL_V
// Frame-result stabilizer. It never touches the streaming RGB/data timing.
// I_result is one pulse in blanking AFTER raw centroid/shape qualification.
// O_ready marks an atomic prepared result and is consumed by I_frame_start.
// Keep the search ROI based on fresh raw detection, not on this held overlay.
module color_track_temporal #(
    parameter integer SUM_WIDTH = 32,
    parameter integer CONFIRM_FRAMES = 2,
    parameter integer HOLD_FRAMES = 3,
    parameter integer ASSOC_DISTANCE = 96,
    parameter integer COORD_DEADBAND = 2,
    parameter integer SMOOTH_LIMIT = 8,
    parameter integer SMOOTH_SHIFT = 2
)(
    input wire I_clk, I_rst_n, I_frame_start,
    input wire [1:0] I_color,
    input wire I_result,
    input wire [1:0] I_result_color,
    input wire I_match, I_strong,
    input wire [11:0] I_center_x, I_center_y,
    input wire [11:0] I_left, I_right, I_top, I_bottom,
    input wire [SUM_WIDTH-1:0] I_pixel_count,
    output wire O_ready, O_valid, O_measured,
    output reg [1:0] O_color,
    output reg [11:0] O_center_x, O_center_y,
    output reg [11:0] O_left, O_right, O_top, O_bottom,
    output reg [SUM_WIDTH-1:0] O_pixel_count
);
    localparam integer CONFIRM_BITS = CONFIRM_FRAMES > 1 ? $clog2(CONFIRM_FRAMES+1) : 1;
    localparam integer MISS_BITS = HOLD_FRAMES > 0 ? $clog2(HOLD_FRAMES+2) : 1;
    localparam [1:0] IDLE=0, FILTER=1, COMMIT=2;
    reg [1:0] selected_color, state;
    reg ready_q, valid_q, measured_q, locked;
    reg deferred_ready_q;
    reg frame_started, frame_result_seen;
    reg [CONFIRM_BITS-1:0] candidate_hits;
    reg [MISS_BITS-1:0] misses;
    reg [11:0] track_x, track_y, candidate_x, candidate_y;
    reg snap_coordinates;
    reg [2:0] coord_index;
    reg [11:0] raw_x, raw_y, raw_left, raw_right, raw_top, raw_bottom;
    reg [11:0] filt_x, filt_y, filt_left, filt_right, filt_top, filt_bottom;
    reg [SUM_WIDTH-1:0] accepted_count;

    // Mode changes suppress stale prepared results even before the next clock.
    assign O_ready = I_rst_n && I_color!=0 && I_color==selected_color && ready_q;
    assign O_valid = I_rst_n && I_color!=0 && I_color==selected_color && valid_q;
    assign O_measured = I_rst_n && I_color!=0 && I_color==selected_color && valid_q && measured_q;

    // A missing result is charged once at the next frame boundary. Its prepared
    // miss result is consumed at the following boundary, like ordinary results.
    // This prevents a truncated/stalled statistics stream retaining a box forever.
    wire missing_result = I_frame_start && frame_started && !frame_result_seen && !I_result && !deferred_ready_q;
    wire result_event = state==IDLE && (I_result || missing_result) &&
                        (!frame_result_seen || I_frame_start || missing_result);
    wire result_match = I_result && I_result_color==I_color && I_match;
    wire result_strong = result_match && I_strong;
    wire [11:0] track_dx = I_center_x>=track_x ? I_center_x-track_x : track_x-I_center_x;
    wire [11:0] track_dy = I_center_y>=track_y ? I_center_y-track_y : track_y-I_center_y;
    wire [11:0] candidate_dx = I_center_x>=candidate_x ? I_center_x-candidate_x : candidate_x-I_center_x;
    wire [11:0] candidate_dy = I_center_y>=candidate_y ? I_center_y-candidate_y : candidate_y-I_center_y;
    wire near_track = track_dx<=ASSOC_DISTANCE && track_dy<=ASSOC_DISTANCE;
    wire near_candidate = candidate_dx<=ASSOC_DISTANCE && candidate_dy<=ASSOC_DISTANCE;
    wire confirmed_candidate = result_strong && (CONFIRM_FRAMES<=1 ||
        (candidate_hits!=0 && near_candidate && candidate_hits>=CONFIRM_FRAMES-1));
    wire accept_result = result_match && ((locked && near_track) || confirmed_candidate);

    // One shared 12-bit difference/move datapath visits six coordinates.
    // It is evaluated only in FILTER, well before the next display boundary.
    reg [11:0] old_coordinate, new_coordinate;
    always @* begin
        case(coord_index)
            0: begin old_coordinate=O_left; new_coordinate=raw_left; end
            1: begin old_coordinate=O_right; new_coordinate=raw_right; end
            2: begin old_coordinate=O_top; new_coordinate=raw_top; end
            3: begin old_coordinate=O_bottom; new_coordinate=raw_bottom; end
            4: begin old_coordinate=O_center_x; new_coordinate=raw_x; end
            default: begin old_coordinate=O_center_y; new_coordinate=raw_y; end
        endcase
    end
    wire increasing = new_coordinate>=old_coordinate;
    wire [11:0] coordinate_delta = increasing ? new_coordinate-old_coordinate : old_coordinate-new_coordinate;
    wire [11:0] shifted_step = coordinate_delta >> SMOOTH_SHIFT;
    wire [11:0] coordinate_step = shifted_step==0 ? 12'd1 : shifted_step;
    wire [11:0] moved_coordinate = increasing ? old_coordinate+coordinate_step : old_coordinate-coordinate_step;
    wire [11:0] filtered_coordinate = snap_coordinates ? new_coordinate :
        coordinate_delta<=COORD_DEADBAND ? old_coordinate :
        coordinate_delta<=SMOOTH_LIMIT ? moved_coordinate : new_coordinate;

    // Independent smoothing must not invert bounds or put a centroid outside.
    // Malformed incoming bounds are also sorted rather than drawing a wrap box.
    wire [11:0] safe_raw_left = raw_left<=raw_right ? raw_left : raw_right;
    wire [11:0] safe_raw_right = raw_left<=raw_right ? raw_right : raw_left;
    wire [11:0] safe_raw_top = raw_top<=raw_bottom ? raw_top : raw_bottom;
    wire [11:0] safe_raw_bottom = raw_top<=raw_bottom ? raw_bottom : raw_top;
    wire [11:0] final_left = filt_left<=filt_right ? filt_left : safe_raw_left;
    wire [11:0] final_right = filt_left<=filt_right ? filt_right : safe_raw_right;
    wire [11:0] final_top = filt_top<=filt_bottom ? filt_top : safe_raw_top;
    wire [11:0] final_bottom = filt_top<=filt_bottom ? filt_bottom : safe_raw_bottom;
    wire [11:0] final_x = filt_x<final_left ? final_left : filt_x>final_right ? final_right : filt_x;
    wire [11:0] final_y = filt_y<final_top ? final_top : filt_y>final_bottom ? final_bottom : filt_y;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            selected_color<=0; state<=IDLE; ready_q<=0; valid_q<=0; measured_q<=0; locked<=0;
            deferred_ready_q<=0;
            frame_started<=0; frame_result_seen<=0; candidate_hits<=0; misses<=0;
            track_x<=0; track_y<=0; candidate_x<=0; candidate_y<=0;
            snap_coordinates<=0; coord_index<=0;
            raw_x<=0; raw_y<=0; raw_left<=0; raw_right<=0; raw_top<=0; raw_bottom<=0;
            filt_x<=0; filt_y<=0; filt_left<=0; filt_right<=0; filt_top<=0; filt_bottom<=0;
            accepted_count<=0; O_color<=0; O_center_x<=0; O_center_y<=0;
            O_left<=0; O_right<=0; O_top<=0; O_bottom<=0; O_pixel_count<=0;
        end else if(I_color==0 || I_color!=selected_color) begin
            selected_color<=I_color; state<=IDLE; ready_q<=0; valid_q<=0; measured_q<=0; locked<=0;
            deferred_ready_q<=0;
            frame_started<=0; frame_result_seen<=0; candidate_hits<=0; misses<=0;
            track_x<=0; track_y<=0; candidate_x<=0; candidate_y<=0;
            snap_coordinates<=0; coord_index<=0; accepted_count<=0;
            O_color<=I_color; O_center_x<=0; O_center_y<=0;
            O_left<=0; O_right<=0; O_top<=0; O_bottom<=0; O_pixel_count<=0;
        end else begin
            // A synthetic miss is detected AT SOF, after the parent has sampled
            // its preceding prepared result. Do not expose the new result during
            // that same SOF: the first-pixel bypass must match the parent frame.
            if(deferred_ready_q && !I_frame_start) begin
                ready_q<=1;
                deferred_ready_q<=0;
            end
            if(I_frame_start) begin
                ready_q<=0;
                frame_started<=1;
                frame_result_seen<=0;
            end
            if(result_event) begin
                // A normal result belongs to this frame. A synthetic miss at
                // frame_start belongs to the preceding frame, not the new one.
                if(I_result) frame_result_seen<=1;
                ready_q<=0;
                deferred_ready_q<=0;
                O_color<=I_color;
                if(accept_result) begin
                    snap_coordinates<=!locked || !near_track;
                    locked<=1; misses<=0; candidate_hits<=0;
                    track_x<=I_center_x; track_y<=I_center_y;
                    raw_x<=I_center_x; raw_y<=I_center_y;
                    raw_left<=I_left; raw_right<=I_right; raw_top<=I_top; raw_bottom<=I_bottom;
                    accepted_count<=I_pixel_count;
                    coord_index<=0;
                    state<=FILTER;
                end else begin
                    // Weak/invalid frames interrupt the strong acquisition run.
                    if(result_strong) begin
                        candidate_x<=I_center_x; candidate_y<=I_center_y;
                        if(candidate_hits!=0 && near_candidate) candidate_hits<=candidate_hits+1'b1;
                        else candidate_hits<=1;
                    end else candidate_hits<=0;
                    O_pixel_count<=0;
                    measured_q<=0;
                    ready_q<=!missing_result;
                    deferred_ready_q<=missing_result;
                    if(locked && misses<HOLD_FRAMES) begin
                        misses<=misses+1'b1;
                        valid_q<=1;
                    end else begin
                        locked<=0; misses<=0; valid_q<=0;
                    end
                end
            end else if(state==FILTER) begin
                case(coord_index)
                    0: filt_left<=filtered_coordinate;
                    1: filt_right<=filtered_coordinate;
                    2: filt_top<=filtered_coordinate;
                    3: filt_bottom<=filtered_coordinate;
                    4: filt_x<=filtered_coordinate;
                    5: filt_y<=filtered_coordinate;
                endcase
                if(coord_index==5) state<=COMMIT;
                else coord_index<=coord_index+1'b1;
            end else if(state==COMMIT) begin
                O_left<=final_left; O_right<=final_right;
                O_top<=final_top; O_bottom<=final_bottom;
                O_center_x<=final_x; O_center_y<=final_y;
                O_pixel_count<=accepted_count;
                valid_q<=1; measured_q<=1; ready_q<=1; state<=IDLE;
            end
        end
    end
endmodule
`endif
