// K4 is a shutter only in original mode0; optional motion shutter uses mode6.
// Explicit card initialization remains mode0 only. Busy requests are ignored.
// Hold one fully completed DDR page until photo data/header commit has finished.
// An uninitialized card gets an initialization retry, never a blind sector write.
module photo_capture_control #(
    parameter integer INITIALIZE_CONTROL=0,
    parameter integer ALLOW_MOTION_SHUTTER=0,
    parameter integer LOCK_TIMEOUT=33554431,
    parameter integer SAVED_DISPLAY_TICKS=150004000
)(
    input wire I_clk,I_rst_n,I_shutter,I_source_fresh,I_initialize,
    input wire [2:0] I_mode,
    input wire I_sd_initialized,I_sd_busy,I_sd_error,
    input wire [3:0] I_sd_error_code,
    input wire I_page_locked,
    input wire [1:0] I_locked_page,
    input wire I_file_busy,I_file_done,I_file_error,
    input wire [3:0] I_file_error_code,
    input wire [27:0] I_file_number_bcd,
    input wire I_dma_error,
    output reg O_page_hold,O_file_start,O_sd_retry,O_file_initialize,O_file_motion,
    output reg [1:0] O_page,
    output reg [2:0] O_status,
    output reg [3:0] O_error_code,
    output reg [27:0] O_file_number_bcd,
    output wire O_capture_ready,
    output reg O_capture_accepted,O_capture_done,O_capture_failed,
    output reg O_motion_accepted
);
    // Status:0 SD initializing;1 ready;2 lock/save;3 saved;4 error.
    localparam IDLE=0,LOCK=1,SETTLE=2,SAVING=3,RETRY=4,START_WAIT=5;
    reg [2:0] state;
    reg retry_seen_busy;
    reg [27:0] timer;
    (* syn_preserve=1,async_reg="true" *) reg [1:0] lock_sync;
    assign O_capture_ready=state==IDLE && I_sd_initialized && !I_sd_busy &&
        !I_file_busy && !lock_sync[1] && I_source_fresh && O_status!=4;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) lock_sync<=0;
        else lock_sync<={lock_sync[0],I_page_locked};
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state<=IDLE;timer<=0;O_page_hold<=0;O_file_start<=0;O_sd_retry<=0;O_file_initialize<=0;
            O_page<=0;O_status<=0;O_error_code<=0;O_file_number_bcd<=0;retry_seen_busy<=0;
            O_capture_accepted<=0;O_capture_done<=0;O_capture_failed<=0;
            O_motion_accepted<=0;O_file_motion<=0;
        end else begin
            O_file_start<=0;O_sd_retry<=0;
            O_capture_accepted<=0;O_capture_done<=0;O_capture_failed<=0;
            O_motion_accepted<=0;
            case(state)
                IDLE:begin
                    O_page_hold<=0;
                    if(O_status==3) begin
                        if(timer==SAVED_DISPLAY_TICKS-1) begin O_status<=I_sd_initialized ? 1:0;timer<=0;end
                        else timer<=timer+1'b1;
                    end else if(O_status!=4) begin
                        if(I_sd_error && !I_sd_initialized && !I_sd_busy) begin O_status<=4;O_error_code<=I_sd_error_code;end
                        else O_status<=I_sd_initialized ? 1:0;
                    end
                    if((I_shutter && (I_mode==0 || (ALLOW_MOTION_SHUTTER && I_mode==6))) ||
                       (INITIALIZE_CONTROL && I_initialize && I_mode==0)) begin
                        timer<=0;O_error_code<=0;
                        O_file_initialize<=INITIALIZE_CONTROL && I_initialize && I_mode==0;
                        // Hold the accepted request source through page lock,
                        // DMA and final header commit even if the mode changes.
                        O_file_motion<=ALLOW_MOTION_SHUTTER && I_mode==6;
                        if(!I_sd_initialized) begin
                            if(!I_sd_busy) begin O_sd_retry<=1;retry_seen_busy<=0;state<=RETRY;end
                            O_status<=0;
                        end else if(!I_source_fresh) begin O_status<=4;O_error_code<=8;end
                        else if(!I_sd_busy && !I_file_busy && !lock_sync[1]) begin
                            O_page_hold<=1;O_status<=2;state<=LOCK;O_capture_accepted<=1;
                            O_motion_accepted<=ALLOW_MOTION_SHUTTER && I_mode==6;
                        end
                    end
                end
                LOCK:begin
                    if(!I_source_fresh || timer==LOCK_TIMEOUT) begin
                        O_page_hold<=0;O_status<=4;O_error_code<=8;state<=IDLE;O_capture_failed<=1;
                    end else if(lock_sync[1]) begin O_page<=I_locked_page;state<=SETTLE;end
                    else timer<=timer+1'b1;
                end
                SETTLE:begin O_file_start<=1;timer<=0;state<=START_WAIT;end
                START_WAIT:begin
                    // Wait until the new job clears a prior sticky file error.
                    if(I_file_busy) state<=SAVING;
                    else if(timer==LOCK_TIMEOUT) begin
                        O_page_hold<=0;O_status<=4;O_error_code<=9;state<=IDLE;O_capture_failed<=1;
                    end else timer<=timer+1'b1;
                end
                SAVING:begin
                    if(I_file_error || I_dma_error) begin
                        O_page_hold<=0;O_status<=4;
                        O_capture_failed<=1;
                        O_error_code<=I_dma_error ? 4'd9:I_file_error_code;state<=IDLE;
                    end else if(I_file_done) begin
                        O_file_number_bcd<=I_file_number_bcd;
                        O_page_hold<=0;O_status<=3;timer<=0;state<=IDLE;O_capture_done<=1;
                    end
                end
                RETRY:begin
                    if(I_sd_busy) retry_seen_busy<=1;
                    if(retry_seen_busy && !I_sd_busy) begin
                        O_status<=I_sd_initialized ? 1:4;
                        O_error_code<=I_sd_initialized ? 4'd0:I_sd_error_code;
                        state<=IDLE;
                    end
                end
            endcase
        end
    end
endmodule
