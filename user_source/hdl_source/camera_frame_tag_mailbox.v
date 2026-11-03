`ifndef CAMERA_FRAME_TAG_MAILBOX_V
`define CAMERA_FRAME_TAG_MAILBOX_V
// Camera IDs follow the actual complete DDR write bank and the actual first
// display read address. No timestamp, latency, output FPS or drop calculation.
// A stream reset or freshness timeout clears all three domains together; local
// three-clock releases prevent old mailbox/bank state surviving a restart.
module camera_frame_tag_mailbox #(
    parameter integer FRAME_BEATS=1280*720*3/16,
    parameter [24:0] PAGE0=25'd0,PAGE1=25'd7000000,
    parameter [24:0] PAGE2=25'd14000000,PAGE3=25'd21000000
)(
    input wire I_capture_clk,I_ddr_clk,I_display_clk,I_rst_n,I_source_fresh,
    input wire I_capture_start,I_wr_en,I_rd_en,
    input wire [24:0] I_wr_addr,I_rd_addr,
    input wire I_display_de,I_display_user,
    output reg O_valid,
    output reg [15:0] O_frame_id,
    output reg O_repeat,O_frame_new,O_update
);
    localparam integer COUNT_BITS=$clog2(FRAME_BEATS+2);
    wire stream_reset_n=I_rst_n && I_source_fresh;
    reg [2:0] capture_release,ddr_release,display_release; // synthesis keep
    always @(posedge I_capture_clk or negedge stream_reset_n)
        if(!stream_reset_n)capture_release<=0;else capture_release<={capture_release[1:0],1'b1};
    always @(posedge I_ddr_clk or negedge stream_reset_n)
        if(!stream_reset_n)ddr_release<=0;else ddr_release<={ddr_release[1:0],1'b1};
    always @(posedge I_display_clk or negedge stream_reset_n)
        if(!stream_reset_n)display_release<=0;else display_release<={display_release[1:0],1'b1};
    wire capture_reset_n=stream_reset_n && capture_release[2];
    wire ddr_reset_n=stream_reset_n && ddr_release[2];
    wire display_reset_n=stream_reset_n && display_release[2];
    function [15:0] decode_id;
        input [15:0] gray;integer i;
        begin for(i=0;i<16;i=i+1)decode_id[i]=^(gray>>i);end
    endfunction
    function [2:0] page_at_base;
        input [24:0] address;
        begin case(address)
            PAGE0:page_at_base=3'b100;PAGE1:page_at_base=3'b101;
            PAGE2:page_at_base=3'b110;PAGE3:page_at_base=3'b111;
            default:page_at_base=3'b000;
        endcase end
    endfunction
    wire [2:0] write_base=page_at_base(I_wr_addr),read_base=page_at_base(I_rd_addr);
    wire [1:0] actual_write_page=I_wr_addr<PAGE1 ? 2'd0:
        (I_wr_addr<PAGE2 ? 2'd1:(I_wr_addr<PAGE3 ? 2'd2:2'd3));
    reg [15:0] capture_id,capture_id_gray; // synthesis keep
    wire [15:0] capture_next=capture_id+16'd1;
    reg [15:0] capture_payload,capture_pending_payload; // synthesis keep
    reg capture_request,capture_pending,capture_ack;
    reg [1:0] capture_ack_sync; // synthesis keep
    wire capture_available=capture_ack_sync[1]==capture_request;
    always @(posedge I_capture_clk or negedge capture_reset_n)begin
        if(!capture_reset_n)begin
            capture_id<=0;capture_id_gray<=0;capture_payload<=0;capture_pending_payload<=0;
            capture_request<=0;capture_pending<=0;capture_ack_sync<=0;
        end else begin
            capture_ack_sync<={capture_ack_sync[0],capture_ack};
            if(I_capture_start)begin
                capture_id<=capture_next;capture_id_gray<=capture_next^(capture_next>>1);
                if(capture_available)begin capture_payload<=capture_next;capture_request<=~capture_request;capture_pending<=0;end
                else begin capture_pending_payload<=capture_next;capture_pending<=1;end
            end else if(capture_available && capture_pending)begin
                capture_payload<=capture_pending_payload;capture_request<=~capture_request;capture_pending<=0;
            end
        end
    end
    reg [1:0] capture_req_sync; // synthesis keep
    reg [15:0] capture_id_1,capture_id_2; // synthesis keep
    reg capture_seen,capture_wait,capture_metadata_valid;
    reg [15:0] capture_metadata; // synthesis keep
    wire [15:0] current_capture_id=decode_id(capture_id_2);
    always @(posedge I_ddr_clk or negedge ddr_reset_n)begin
        if(!ddr_reset_n)begin
            capture_req_sync<=0;capture_id_1<=0;capture_id_2<=0;
            capture_seen<=0;capture_wait<=0;capture_ack<=0;capture_metadata<=0;capture_metadata_valid<=0;
        end else begin
            capture_req_sync<={capture_req_sync[0],capture_request};
            capture_id_1<=capture_id_gray;capture_id_2<=capture_id_1;
            if(capture_req_sync[1]!=capture_seen)begin capture_seen<=capture_req_sync[1];capture_wait<=1;end
            else if(capture_wait)begin
                capture_metadata<=capture_payload;capture_metadata_valid<=1;
                capture_ack<=capture_seen;capture_wait<=0;
            end
        end
    end
    reg [15:0] page_metadata[0:3];
    reg [3:0] page_valid;
    reg writing,writing_bad;
    reg [1:0] writing_page;
    reg [15:0] writing_id;
    reg [COUNT_BITS-1:0] write_count;
    reg [24:0] expected_address;
    wire new_binding_valid=capture_metadata_valid && capture_metadata==current_capture_id;
    integer p;
    always @(posedge I_ddr_clk or negedge ddr_reset_n)begin
        if(!ddr_reset_n)begin
            page_valid<=0;writing<=0;writing_bad<=1;writing_page<=0;writing_id<=0;
            write_count<=0;expected_address<=0;for(p=0;p<4;p=p+1)page_metadata[p]<=0;
        end else begin
            if(writing && write_count<FRAME_BEATS && writing_id!=current_capture_id)begin
                writing_bad<=1;page_valid[writing_page]<=0;
            end
            if(I_wr_en)begin
                if(write_base[2])begin
                    writing<=1;writing_page<=write_base[1:0];writing_id<=capture_metadata;
                    writing_bad<=!new_binding_valid;write_count<=1;expected_address<=I_wr_addr+25'd8;
                    page_metadata[write_base[1:0]]<=capture_metadata;
                    page_valid[write_base[1:0]]<=FRAME_BEATS==1 && new_binding_valid;
                end else if(writing)begin
                    expected_address<=expected_address+25'd8;
                    if(write_count<FRAME_BEATS+1)write_count<=write_count+1'b1;
                    if(I_wr_addr!=expected_address || writing_id!=current_capture_id || write_count>=FRAME_BEATS)begin
                        writing_bad<=1;page_valid[writing_page]<=0;page_valid[actual_write_page]<=0;
                    end else if(write_count==FRAME_BEATS-1)page_valid[writing_page]<=!writing_bad;
                end else page_valid[actual_write_page]<=0;
            end
        end
    end
    reg read_active;
    reg [1:0] reading_page;
    reg [16:0] read_payload,read_pending_payload; // synthesis keep
    reg read_request,read_pending,read_ack;
    reg [1:0] read_ack_sync; // synthesis keep
    wire read_available=read_ack_sync[1]==read_request;
    wire read_selection=I_rd_en && read_base[2];
    wire read_overwrite=I_wr_en && read_active && actual_write_page==reading_page;
    wire [16:0] selected_metadata={page_valid[read_base[1:0]] &&
        !(I_wr_en && actual_write_page==read_base[1:0]),page_metadata[read_base[1:0]]};
    wire [16:0] next_read_payload=read_selection ? selected_metadata : {1'b0,page_metadata[reading_page]};
    always @(posedge I_ddr_clk or negedge ddr_reset_n)begin
        if(!ddr_reset_n)begin
            read_active<=0;reading_page<=0;read_payload<=0;read_pending_payload<=0;
            read_request<=0;read_pending<=0;read_ack_sync<=0;
        end else begin
            read_ack_sync<={read_ack_sync[0],read_ack};
            if(read_selection || read_overwrite)begin
                if(read_selection)begin read_active<=1;reading_page<=read_base[1:0];end
                if(read_available)begin read_payload<=next_read_payload;read_request<=~read_request;read_pending<=0;end
                else begin read_pending_payload<=next_read_payload;read_pending<=1;end
            end else if(read_available && read_pending)begin
                read_payload<=read_pending_payload;read_request<=~read_request;read_pending<=0;
            end
        end
    end
    reg [1:0] read_req_sync; // synthesis keep
    reg read_seen,read_wait,read_fresh;
    reg [16:0] display_metadata; // synthesis keep
    reg previous_valid;
    reg [15:0] previous_id;
    wire sample_valid=read_fresh && display_metadata[16];
    always @(posedge I_display_clk or negedge display_reset_n)begin
        if(!display_reset_n)begin
            read_req_sync<=0;read_seen<=0;read_wait<=0;read_ack<=0;read_fresh<=0;
            display_metadata<=0;previous_valid<=0;previous_id<=0;
            O_valid<=0;O_frame_id<=0;O_repeat<=0;O_frame_new<=0;O_update<=0;
        end else begin
            read_req_sync<={read_req_sync[0],read_request};O_update<=0;O_frame_new<=0;
            if(read_req_sync[1]!=read_seen)begin read_seen<=read_req_sync[1];read_wait<=1;end
            else if(read_wait)begin
                display_metadata<=read_payload;read_ack<=read_seen;read_wait<=0;read_fresh<=1;
                if(!read_payload[16])O_valid<=0;
            end
            if(I_display_de && I_display_user)begin
                read_fresh<=0;O_update<=1;O_valid<=sample_valid;
                O_repeat<=sample_valid && previous_valid && display_metadata[15:0]==previous_id;
                O_frame_new<=sample_valid && (!previous_valid || display_metadata[15:0]!=previous_id);
                if(sample_valid)begin O_frame_id<=display_metadata[15:0];previous_id<=display_metadata[15:0];previous_valid<=1;end
                else previous_valid<=0;
            end
        end
    end
    // synthesis translate_off
    initial if(FRAME_BEATS<1 || PAGE0!=0 || PAGE1<=PAGE0 || PAGE2<=PAGE1 || PAGE3<=PAGE2)
        $fatal(1,"Invalid frame tag bank/frame parameters");
    // synthesis translate_on
endmodule
`endif
