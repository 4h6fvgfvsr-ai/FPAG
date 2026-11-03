`ifndef CAMERA_AUTO_EXPOSURE_V
`define CAMERA_AUTO_EXPOSURE_V
// SC500CS software AEC/AGC. Independent RAW10 observation branch: never
// modifies pixels, ready/valid, ISP, color tracking, HDMI or Ethernet.
// SC500CS V1.8 sections 2.2.1/2.2.2: exposure first, N+2-frame latency;
// VTS=1000 -> maximum exposure=2*1000-10=1990 half-line units.
module camera_auto_exposure #(
    parameter integer IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter integer TARGET=96, TOLERANCE=8,
    parameter integer CENTER_METER=0,
    parameter integer VERY_DARK=48, VERY_BRIGHT=176,
    parameter integer MIN_EXPOSURE=3, MAX_EXPOSURE=1990,
    parameter integer INITIAL_EXPOSURE=1990,
    // Existing gain-table interface: index=O_ag-16; 16=1x,80=4x,112=8x.
    parameter integer MIN_GAIN=16, MAX_GAIN=112, INITIAL_GAIN=80,
    parameter integer SETTLE_FRAMES=3
)(
    input wire I_pixel_clk, I_cfg_clk, I_rst_n,
    input wire I_frame_start, I_valid,
    input wire [39:0] I_raw10,
    input wire I_cam_cfg_done, I_ae_cfg_done,
    output reg O_ae_req,
    output reg [15:0] O_ae, O_ag
);
    localparam integer FRAME_BEATS=IMG_WIDTH*IMG_HEIGHT/4;
    localparam integer LINE_BEATS=IMG_WIDTH/4;
    // Optional central half-width/half-height window excludes corner lamps.
    // Full-frame beat counting still rejects truncated/oversized input frames.
    localparam integer METER_BEATS=CENTER_METER ? FRAME_BEATS/4 : FRAME_BEATS;
    localparam integer COUNT_BITS=$clog2(FRAME_BEATS+2);
    localparam integer SUM_BITS=$clog2(FRAME_BEATS*255+1);
    localparam integer WAIT_BITS=SETTLE_FRAMES>0 ? $clog2(SETTLE_FRAMES+1) : 1;
    localparam [SUM_BITS-1:0] LOW_TOTAL=(TARGET-TOLERANCE)*METER_BEATS;
    localparam [SUM_BITS-1:0] HIGH_TOTAL=(TARGET+TOLERANCE)*METER_BEATS;
    localparam [SUM_BITS-1:0] DARK_TOTAL=VERY_DARK*METER_BEATS;
    localparam [SUM_BITS-1:0] BRIGHT_TOTAL=VERY_BRIGHT*METER_BEATS;
    localparam [2:0] HOLD=0, DARK=1, BRIGHT=2, FAR_DARK=3, FAR_BRIGHT=4;

    // Asynchronous assertion, synchronous release in both domains.
    reg [1:0] pixel_release, cfg_release;
    always @(posedge I_pixel_clk or negedge I_rst_n)
        if(!I_rst_n) pixel_release<=0;
        else pixel_release<={pixel_release[0],1'b1};
    always @(posedge I_cfg_clk or negedge I_rst_n)
        if(!I_rst_n) cfg_release<=0;
        else cfg_release<={cfg_release[0],1'b1};
    wire pixel_rst_n=pixel_release[1], cfg_rst_n=cfg_release[1];

    // Average four Bayer samples and convert RAW10 -> 8-bit metering scale.
    // Over a complete Bayer frame this weights green twice as heavily as R/B.
    wire [10:0] pair0={1'b0,I_raw10[9:0]}+{1'b0,I_raw10[19:10]};
    wire [10:0] pair1={1'b0,I_raw10[29:20]}+{1'b0,I_raw10[39:30]};
    wire [11:0] raw_total={1'b0,pair0}+{1'b0,pair1};
    reg valid_q, start_q;
    reg [7:0] level_q;
    always @(posedge I_pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin valid_q<=0; start_q<=0; level_q<=0; end
        else begin valid_q<=I_valid; start_q<=I_frame_start; level_q<=raw_total[11:4]; end
    end

    reg have_frame, report_req, report_ack;
    reg [COUNT_BITS-1:0] pixel_beats;
    reg [SUM_BITS-1:0] frame_sum;
    reg [$clog2(LINE_BEATS+1)-1:0] beat_x;
    reg [$clog2(IMG_HEIGHT+1)-1:0] beat_y;
    wire in_meter=!CENTER_METER || (!start_q &&
        beat_x>=LINE_BEATS/4 && beat_x<LINE_BEATS*3/4 &&
        beat_y>=IMG_HEIGHT/4 && beat_y<IMG_HEIGHT*3/4);
    reg [2:0] report_code;
    reg [1:0] ack_sync, req_sync;
    // Bundled-data mailbox. report_code stays stable until a two-stage ACK
    // returns; no separately synchronized multi-bit sum/mean can tear.
    always @(posedge I_pixel_clk or negedge pixel_rst_n)
        if(!pixel_rst_n) ack_sync<=0;
        else ack_sync<={ack_sync[0],report_ack};
    always @(posedge I_cfg_clk or negedge cfg_rst_n)
        if(!cfg_rst_n) req_sync<=0;
        else req_sync<={req_sync[0],report_req};

    always @(posedge I_pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            have_frame<=0; pixel_beats<=0; frame_sum<=0;
            beat_x<=0; beat_y<=0;
            report_req<=0; report_code<=HOLD;
        end else if(start_q) begin
            // Publish the PREVIOUS complete frame just after a new frame starts.
            // Reject partial frames and excess data. Include the first beat
            // whether frame_start occurs in blanking or on its first pixel.
            if(have_frame && pixel_beats==FRAME_BEATS && report_req==ack_sync[1]) begin
                report_req<=!report_req;
                if(frame_sum<DARK_TOTAL) report_code<=FAR_DARK;
                else if(frame_sum<LOW_TOTAL) report_code<=DARK;
                else if(frame_sum>BRIGHT_TOTAL) report_code<=FAR_BRIGHT;
                else if(frame_sum>HIGH_TOTAL) report_code<=BRIGHT;
                else report_code<=HOLD;
            end
            have_frame<=1;
            pixel_beats<=valid_q ? 1 : 0;
            frame_sum<=valid_q && !CENTER_METER ? level_q : 0;
            beat_x<=valid_q ? 1 : 0; beat_y<=0;
        end else if(valid_q && have_frame) begin
            // Saturate at expected_count+1 to make overflow unambiguously invalid.
            if(pixel_beats<FRAME_BEATS+1) pixel_beats<=pixel_beats+1'b1;
            if(pixel_beats<FRAME_BEATS && in_meter) frame_sum<=frame_sum+level_q;
            if(beat_x==LINE_BEATS-1) begin beat_x<=0; beat_y<=beat_y+1'b1; end
            else beat_x<=beat_x+1'b1;
        end
    end

    reg [1:0] write_state;
    reg [WAIT_BITS-1:0] settle_left;
    wire new_report=req_sync[1]!=report_ack;
    wire large_error=report_code==FAR_DARK || report_code==FAR_BRIGHT;
    wire [15:0] shifted_step=large_error ? O_ae>>2 : O_ae>>3;
    wire [15:0] exposure_step=shifted_step==0 ? 16'd1 : shifted_step;
    wire [15:0] gain_step=large_error ? 16'd8 : 16'd4;
    wire [16:0] increased_exposure={1'b0,O_ae}+{1'b0,exposure_step};
    wire [16:0] increased_gain={1'b0,O_ag}+{1'b0,gain_step};
    reg [15:0] next_exposure, next_gain;
    always @(*) begin
        next_exposure=O_ae; next_gain=O_ag;
        if(report_code==DARK || report_code==FAR_DARK) begin
            if(O_ae<MAX_EXPOSURE)
                next_exposure=increased_exposure>MAX_EXPOSURE ? MAX_EXPOSURE : increased_exposure[15:0];
            else if(O_ag<MAX_GAIN)
                next_gain=increased_gain>MAX_GAIN ? MAX_GAIN : increased_gain[15:0];
        end else if(report_code==BRIGHT || report_code==FAR_BRIGHT) begin
            if(O_ag>MIN_GAIN)
                next_gain=O_ag-MIN_GAIN<gain_step ? MIN_GAIN : O_ag-gain_step;
            else if(O_ae>MIN_EXPOSURE)
                next_exposure=O_ae-MIN_EXPOSURE<exposure_step ? MIN_EXPOSURE : O_ae-exposure_step;
        end
    end

    always @(posedge I_cfg_clk or negedge cfg_rst_n) begin
        if(!cfg_rst_n) begin
            O_ae<=INITIAL_EXPOSURE; O_ag<=INITIAL_GAIN; O_ae_req<=0;
            write_state<=0; settle_left<=SETTLE_FRAMES; report_ack<=0;
        end else begin
            O_ae_req<=0;
            if(new_report) report_ack<=req_sync[1];
            if(!I_cam_cfg_done) begin
                O_ae<=INITIAL_EXPOSURE; O_ag<=INITIAL_GAIN;
                write_state<=0; settle_left<=SETTLE_FRAMES;
            end else begin
                // Keep both settings frozen throughout all five I2C writes.
                // Do not accept another change before the transaction completes.
                case(write_state)
                    1: if(!I_ae_cfg_done) write_state<=2;
                    2: if(I_ae_cfg_done) begin write_state<=0; settle_left<=SETTLE_FRAMES; end
                    default: if(new_report && I_ae_cfg_done) begin
                        if(settle_left!=0) settle_left<=settle_left-1'b1;
                        else if(next_exposure!=O_ae || next_gain!=O_ag) begin
                            O_ae<=next_exposure; O_ag<=next_gain;
                            O_ae_req<=1; write_state<=1;
                        end
                    end
                endcase
            end
        end
    end

    // synthesis translate_off
    initial begin
        if(IMG_WIDTH%4!=0 || FRAME_BEATS<1 || TARGET<=TOLERANCE || TARGET+TOLERANCE>255 ||
            VERY_DARK>TARGET-TOLERANCE || VERY_BRIGHT<TARGET+TOLERANCE ||
            INITIAL_EXPOSURE<MIN_EXPOSURE || INITIAL_EXPOSURE>MAX_EXPOSURE ||
            INITIAL_GAIN<MIN_GAIN || INITIAL_GAIN>MAX_GAIN || SETTLE_FRAMES<0 ||
            (CENTER_METER && (IMG_WIDTH%16!=0 || IMG_HEIGHT%4!=0)))
            $fatal(1,"Invalid automatic exposure parameters");
    end
    // synthesis translate_on
endmodule
`endif
