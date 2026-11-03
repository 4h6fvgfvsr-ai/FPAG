`ifndef VIDEO_LATENCY_MONITOR_V
`define VIDEO_LATENCY_MONITOR_V
// Passive acquisition-to-processed-HDMI measurement. It never gates video.
// I_capture_start must identify the first available camera RAW payload, not
// exposure start. A 50 MHz reference produces 100 us Gray timestamp ticks.
// Metadata is attached to the actual DDR write page, accepted only after an
// entire contiguous frame, then selected by the actual first DDR read address.
// Exposure, sensor transport before this marker, and panel scanout are excluded.
module video_latency_monitor #(
    parameter integer REF_CLK_HZ=50000000,
    parameter integer TIMESTAMP_BITS=24,
    parameter integer FRAME_BEATS=1280*720*3/16,
    parameter integer MAX_AGE_TICKS=10000,
    parameter [24:0] PAGE0=25'd0, PAGE1=25'd7000000,
    parameter [24:0] PAGE2=25'd14000000, PAGE3=25'd21000000
)(
    input wire I_ref_clk,I_capture_clk,I_ddr_clk,I_display_clk,I_rst_n,
    input wire I_capture_start,
    input wire I_wr_en,I_rd_en,
    input wire [24:0] I_wr_addr,I_rd_addr,
    input wire I_display_de,I_display_user,
    output reg O_valid,
    output reg [23:0] O_latency_ticks,
    output reg [15:0] O_frame_id,
    output reg O_repeat,O_frame_new,
    output reg O_update
);
    localparam integer TICK_CYCLES=REF_CLK_HZ/10000;
    localparam integer DIV_BITS=TICK_CYCLES>1?$clog2(TICK_CYCLES):1;
    localparam integer COUNT_BITS=$clog2(FRAME_BEATS+2);
    reg [2:0] ref_release,capture_release,ddr_release,display_release; // synthesis keep
    // Assert immediately from the common reset; each domain releases its state
    // only after three local rising edges, even if another clock starts earlier.
    always @(posedge I_ref_clk or negedge I_rst_n)
        if(!I_rst_n) ref_release<=0;else ref_release<={ref_release[1:0],1'b1};
    always @(posedge I_capture_clk or negedge I_rst_n)
        if(!I_rst_n) capture_release<=0;else capture_release<={capture_release[1:0],1'b1};
    always @(posedge I_ddr_clk or negedge I_rst_n)
        if(!I_rst_n) ddr_release<=0;else ddr_release<={ddr_release[1:0],1'b1};
    always @(posedge I_display_clk or negedge I_rst_n)
        if(!I_rst_n) display_release<=0;else display_release<={display_release[1:0],1'b1};
    wire ref_reset_n=I_rst_n && ref_release[2];
    wire capture_reset_n=I_rst_n && capture_release[2];
    wire ddr_reset_n=I_rst_n && ddr_release[2];
    wire display_reset_n=I_rst_n && display_release[2];
    reg [DIV_BITS-1:0] tick_div;
    reg [TIMESTAMP_BITS-1:0] reference_tick,reference_gray;
    wire [TIMESTAMP_BITS-1:0] next_tick=reference_tick+24'd1;
    always @(posedge I_ref_clk or negedge ref_reset_n) begin
        if(!ref_reset_n) begin tick_div<=0;reference_tick<=0;reference_gray<=0;end
        else if(tick_div==TICK_CYCLES-1) begin
            tick_div<=0;reference_tick<=next_tick;
            reference_gray<=next_tick^(next_tick>>1);
        end else tick_div<=tick_div+1'b1;
    end
    function [TIMESTAMP_BITS-1:0] decode_tick;
        input [TIMESTAMP_BITS-1:0] gray;
        integer i;
        begin
            for(i=0;i<TIMESTAMP_BITS;i=i+1) decode_tick[i]=^(gray>>i);
        end
    endfunction
    function [15:0] decode_id;
        input [15:0] gray;
        integer i;
        begin
            for(i=0;i<16;i=i+1) decode_id[i]=^(gray>>i);
        end
    endfunction
    function [2:0] page_at_base;
        input [24:0] address;
        begin
            case(address)
                PAGE0:page_at_base=3'b100;
                PAGE1:page_at_base=3'b101;
                PAGE2:page_at_base=3'b110;
                PAGE3:page_at_base=3'b111;
                default:page_at_base=3'b000;
            endcase
        end
    endfunction
    wire [2:0] write_base=page_at_base(I_wr_addr);
    wire [2:0] read_base=page_at_base(I_rd_addr);
    // Corrupt writes can jump into a different already complete bank. Invalidate
    // that actual bank too, rather than only poisoning the expected write page.
    wire [1:0] actual_write_page=I_wr_addr<PAGE1?2'd0:
        (I_wr_addr<PAGE2?2'd1:(I_wr_addr<PAGE3?2'd2:2'd3));

    // Capture mailbox: stable payload, request/acknowledge toggles. If capture
    // events arrive faster than the handshake, retain the latest pending event.
    // The separate Gray frame counter prevents binding writes to an older ID.
    reg [TIMESTAMP_BITS-1:0] capture_tick_1,capture_tick_2; // synthesis keep
    reg [15:0] capture_id,capture_id_gray;
    wire [15:0] capture_next=capture_id+16'd1;
    reg [TIMESTAMP_BITS+15:0] capture_payload,capture_pending_payload;
    reg capture_request,capture_pending;
    reg [1:0] capture_ack_sync; // synthesis keep
    reg capture_ack;
    wire capture_available=capture_ack_sync[1]==capture_request;
    always @(posedge I_capture_clk or negedge capture_reset_n) begin
        if(!capture_reset_n) begin
            capture_tick_1<=0;capture_tick_2<=0;capture_id<=0;capture_id_gray<=0;
            capture_payload<=0;capture_pending_payload<=0;
            capture_request<=0;capture_pending<=0;capture_ack_sync<=0;
        end else begin
            capture_tick_1<=reference_gray;capture_tick_2<=capture_tick_1;
            capture_ack_sync<={capture_ack_sync[0],capture_ack};
            if(I_capture_start) begin
                capture_id<=capture_next;
                capture_id_gray<=capture_next^(capture_next>>1);
                if(capture_available) begin
                    capture_payload<={capture_next,decode_tick(capture_tick_2)};
                    capture_request<=~capture_request;capture_pending<=0;
                end else begin
                    capture_pending_payload<={capture_next,decode_tick(capture_tick_2)};
                    capture_pending<=1;
                end
            end else if(capture_available && capture_pending) begin
                capture_payload<=capture_pending_payload;
                capture_request<=~capture_request;capture_pending<=0;
            end
        end
    end
    reg [1:0] capture_req_sync; // synthesis keep
    reg [15:0] capture_id_1,capture_id_2; // synthesis keep
    reg [TIMESTAMP_BITS-1:0] ddr_tick_1,ddr_tick_2; // synthesis keep
    reg capture_seen,capture_wait,capture_metadata_valid;
    reg [TIMESTAMP_BITS+15:0] capture_metadata;
    wire [15:0] current_capture_id=decode_id(capture_id_2);
    wire [TIMESTAMP_BITS-1:0] ddr_now=decode_tick(ddr_tick_2);
    wire [TIMESTAMP_BITS-1:0] capture_age=ddr_now-capture_metadata[TIMESTAMP_BITS-1:0];
    always @(posedge I_ddr_clk or negedge ddr_reset_n) begin
        if(!ddr_reset_n) begin
            capture_req_sync<=0;capture_id_1<=0;capture_id_2<=0;
            ddr_tick_1<=0;ddr_tick_2<=0;capture_seen<=0;capture_wait<=0;
            capture_ack<=0;capture_metadata<=0;capture_metadata_valid<=0;
        end else begin
            capture_req_sync<={capture_req_sync[0],capture_request};
            capture_id_1<=capture_id_gray;capture_id_2<=capture_id_1;
            ddr_tick_1<=reference_gray;ddr_tick_2<=ddr_tick_1;
            // Expiry is sticky until a new capture mailbox arrives; an old
            // timestamp cannot become recent again at the24-bit timer wrap.
            if(capture_metadata_valid && capture_age>MAX_AGE_TICKS)
                capture_metadata_valid<=0;
            if(capture_req_sync[1]!=capture_seen) begin
                capture_seen<=capture_req_sync[1];capture_wait<=1;
            end else if(capture_wait) begin
                capture_metadata<=capture_payload;capture_metadata_valid<=1;
                capture_ack<=capture_seen;capture_wait<=0;
            end
        end
    end

    reg [TIMESTAMP_BITS+15:0] page_metadata[0:3];
    reg [3:0] page_valid;
    reg writing,writing_bad;
    reg [1:0] writing_page;
    reg [15:0] writing_id;
    reg [COUNT_BITS-1:0] write_count;
    reg [24:0] expected_address;
    reg [1:0] expiry_page;
    wire [TIMESTAMP_BITS-1:0] page_age=ddr_now-page_metadata[expiry_page][TIMESTAMP_BITS-1:0];
    wire new_binding_valid=capture_metadata_valid &&
        capture_metadata[TIMESTAMP_BITS+15:TIMESTAMP_BITS]==current_capture_id &&
        capture_age<=MAX_AGE_TICKS;
    integer p;
    always @(posedge I_ddr_clk or negedge ddr_reset_n) begin
        if(!ddr_reset_n) begin
            page_valid<=0;writing<=0;writing_bad<=1;writing_page<=0;
            writing_id<=0;write_count<=0;expected_address<=0;
            expiry_page<=0;
            for(p=0;p<4;p=p+1) page_metadata[p]<=0;
        end else begin
            // One shared age comparison checks all four banks every four DDR
            // clocks. Clearing validity before timer wrap prevents resurrection.
            expiry_page<=expiry_page+1'b1;
            if(page_valid[expiry_page] && page_age>MAX_AGE_TICKS)
                page_valid[expiry_page]<=0;
            if(writing && write_count<FRAME_BEATS &&
               writing_id!=current_capture_id) begin
                writing_bad<=1;page_valid[writing_page]<=0;
            end
            if(I_wr_en) begin
                if(write_base[2]) begin
                    writing<=1;writing_page<=write_base[1:0];
                    writing_id<=capture_metadata[TIMESTAMP_BITS+15:TIMESTAMP_BITS];
                    writing_bad<=!new_binding_valid;
                    write_count<=1;expected_address<=I_wr_addr+25'd8;
                    page_metadata[write_base[1:0]]<=capture_metadata;
                    page_valid[write_base[1:0]]<=FRAME_BEATS==1 && new_binding_valid;
                end else if(writing) begin
                    expected_address<=expected_address+25'd8;
                    if(write_count<FRAME_BEATS+1) write_count<=write_count+1'b1;
                    if(I_wr_addr!=expected_address || writing_id!=current_capture_id ||
                       write_count>=FRAME_BEATS) begin
                        writing_bad<=1;page_valid[writing_page]<=0;
                        page_valid[actual_write_page]<=0;
                    end else if(write_count==FRAME_BEATS-1)
                        page_valid[writing_page]<=!writing_bad;
                end else page_valid[actual_write_page]<=0;
            end
        end
    end

    // Read mailbox binds metadata to the bank actually addressed by video_out.
    // Selecting an incomplete/replaced page produces an explicit invalid sample.
    reg read_active;
    reg [1:0] reading_page;
    reg [TIMESTAMP_BITS+16:0] read_payload,read_pending_payload;
    reg read_request,read_pending;
    reg [1:0] read_ack_sync; // synthesis keep
    reg read_ack;
    wire read_available=read_ack_sync[1]==read_request;
    wire read_selection=I_rd_en && read_base[2];
    wire read_overwrite=I_wr_en && read_active && actual_write_page==reading_page;
    wire [TIMESTAMP_BITS+16:0] selected_metadata={page_valid[read_base[1:0]] &&
        !(I_wr_en && actual_write_page==read_base[1:0]),
        page_metadata[read_base[1:0]]};
    wire [TIMESTAMP_BITS+16:0] next_read_payload=read_selection ? selected_metadata :
        {1'b0,page_metadata[reading_page]};
    always @(posedge I_ddr_clk or negedge ddr_reset_n) begin
        if(!ddr_reset_n) begin
            read_active<=0;reading_page<=0;read_payload<=0;read_pending_payload<=0;
            read_request<=0;read_pending<=0;read_ack_sync<=0;
        end else begin
            read_ack_sync<={read_ack_sync[0],read_ack};
            if(read_selection || read_overwrite) begin
                if(read_selection) begin read_active<=1;reading_page<=read_base[1:0];end
                if(read_available) begin
                    read_payload<=next_read_payload;read_request<=~read_request;read_pending<=0;
                end else begin read_pending_payload<=next_read_payload;read_pending<=1;end
            end else if(read_available && read_pending) begin
                read_payload<=read_pending_payload;read_request<=~read_request;read_pending<=0;
            end
        end
    end
    reg [1:0] read_req_sync; // synthesis keep
    reg [TIMESTAMP_BITS-1:0] display_tick_1,display_tick_2; // synthesis keep
    reg read_seen,read_wait,read_fresh;
    reg [TIMESTAMP_BITS+16:0] display_metadata;
    reg previous_valid;
    reg [15:0] previous_id;
    wire [TIMESTAMP_BITS-1:0] display_now=decode_tick(display_tick_2);
    wire [TIMESTAMP_BITS-1:0] elapsed=display_now-display_metadata[TIMESTAMP_BITS-1:0];
    wire sample_valid=read_fresh && display_metadata[TIMESTAMP_BITS+16] && elapsed<=MAX_AGE_TICKS;
    always @(posedge I_display_clk or negedge display_reset_n) begin
        if(!display_reset_n) begin
            read_req_sync<=0;display_tick_1<=0;display_tick_2<=0;
            read_seen<=0;read_wait<=0;read_ack<=0;read_fresh<=0;
            display_metadata<=0;previous_valid<=0;previous_id<=0;
            O_valid<=0;O_latency_ticks<=0;O_frame_id<=0;
            O_repeat<=0;O_frame_new<=0;O_update<=0;
        end else begin
            read_req_sync<={read_req_sync[0],read_request};
            display_tick_1<=reference_gray;display_tick_2<=display_tick_1;
            O_update<=0;O_frame_new<=0;
            if(read_req_sync[1]!=read_seen) begin
                read_seen<=read_req_sync[1];read_wait<=1;
            end else if(read_wait) begin
                display_metadata<=read_payload;read_ack<=read_seen;
                read_wait<=0;read_fresh<=1;
                if(!read_payload[TIMESTAMP_BITS+16]) O_valid<=0;
            end
            if(I_display_de && I_display_user) begin
                read_fresh<=0;O_update<=1;O_valid<=sample_valid;
                O_repeat<=sample_valid && previous_valid &&
                    display_metadata[TIMESTAMP_BITS+15:TIMESTAMP_BITS]==previous_id;
                O_frame_new<=sample_valid && (!previous_valid ||
                    display_metadata[TIMESTAMP_BITS+15:TIMESTAMP_BITS]!=previous_id);
                if(sample_valid) begin
                    O_latency_ticks<=elapsed;O_frame_id<=display_metadata[TIMESTAMP_BITS+15:TIMESTAMP_BITS];
                    previous_id<=display_metadata[TIMESTAMP_BITS+15:TIMESTAMP_BITS];previous_valid<=1;
                end else previous_valid<=0;
            end
        end
    end
    // synthesis translate_off
    initial if(TICK_CYCLES<1 || REF_CLK_HZ%10000!=0 || FRAME_BEATS<1 ||
               MAX_AGE_TICKS<1 || MAX_AGE_TICKS>=(1<<(TIMESTAMP_BITS-1)) || TIMESTAMP_BITS<16 || TIMESTAMP_BITS>24 ||
               PAGE0!=0 || PAGE1<=PAGE0 || PAGE2<=PAGE1 || PAGE3<=PAGE2)
        $fatal(1,"Invalid latency-monitor tick/frame parameters");
    // synthesis translate_on
endmodule
`endif
