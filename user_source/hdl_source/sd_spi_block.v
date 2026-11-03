// SDHC/SDXC SPI single-sector transport, independently implemented for PH1P35.
// All logic uses I_clk; SD_CLK is an output, never an internal fabric clock.
// Default divisors assume 75 MHz: 390.625 kHz during init, 18.75 MHz afterwards.
// Only block-addressed cards are accepted. SDSC/exFAT support is not implied.
// I_start is accepted when !O_busy. If !O_initialized, it retries initialization
// only; the caller must issue a separate sector request after initialization.
// O_write_byte_req pulses once per byte. The caller supplies I_write_data on
// the following clock (synchronous RAM is supported) and retains it until the
// next request. O_done pulses only for a successful complete sector operation.
// O_error/code are sticky until a new request or reset. Error codes:
// 1 initialization/command timeout, 2 not SDHC/SDXC, 3 I/O command response,
// 4 read token/timeout, 5 write data rejected, 6 card busy timeout.
module sd_spi_block #(
    parameter integer READ_FLOW_CONTROL=0,
    parameter integer INIT_HALF_PERIOD=96,
    parameter integer FAST_HALF_PERIOD=2,
    parameter integer FAST_LOW_PERIOD=FAST_HALF_PERIOD,
    parameter integer FAST_HIGH_PERIOD=FAST_HALF_PERIOD,
    parameter integer POWER_WAIT_CYCLES=75000,
    parameter integer CMD_TIMEOUT_BYTES=16,
    parameter integer TOKEN_TIMEOUT_BYTES=262143,
    parameter integer BUSY_TIMEOUT_BYTES=1048575,
    parameter integer INIT_MAX_ATTEMPTS=1024
)(
    input wire I_clk,I_rst_n,I_start,I_write,
    input wire [31:0] I_lba,
    input wire [7:0] I_write_data,
    input wire I_sd_miso,I_read_ready,
    output reg O_sd_cs_n,
    output wire O_sd_clk,
    output reg O_sd_mosi,
    output reg O_initialized,O_busy,O_done,O_error,
    output reg [3:0] O_error_code,
    output reg O_read_valid,
    output reg [7:0] O_read_data,
    output reg O_write_byte_req
);
    localparam [5:0] POWER=0,PRE=1,PRE_WAIT=2,CMD0=3,RES0=4,
        CMD8=5,RES8=6,R7=7,R7_WAIT=8,CMD55=9,RES55=10,
        ACMD41=11,RES41=12,INIT_DELAY=13,CMD58=14,RES58=15,
        OCR=16,OCR_WAIT=17,IDLE=18,GAP=19,GAP_WAIT=20,
        CMD_SEND=21,CMD_SEND_WAIT=22,R1=23,R1_WAIT=24,
        IO_RES=25,READ_TOKEN=26,READ_TOKEN_WAIT=27,
        READ_DATA=28,READ_DATA_WAIT=29,READ_CRC=30,READ_CRC_WAIT=31,
        WRITE_LEAD=32,WRITE_LEAD_WAIT=33,WRITE_TOKEN=34,WRITE_TOKEN_WAIT=35,
        WRITE_REQ=36,WRITE_DELAY=37,WRITE_DATA=38,WRITE_DATA_WAIT=39,
        WRITE_CRC=40,WRITE_CRC_WAIT=41,WRITE_RESP=42,WRITE_RESP_WAIT=43,
        WRITE_BUSY=44,WRITE_BUSY_WAIT=45,FINISH=46,FINISH_WAIT=47,FAILED=48,
        READ_DELIVER=49;
    localparam integer POWER_BITS=(POWER_WAIT_CYCLES<2)?1:$clog2(POWER_WAIT_CYCLES);
    localparam integer ATTEMPT_BITS=(INIT_MAX_ATTEMPTS<2)?1:$clog2(INIT_MAX_ATTEMPTS);
    localparam integer TIMEOUT_MAX=(BUSY_TIMEOUT_BYTES>TOKEN_TIMEOUT_BYTES)?BUSY_TIMEOUT_BYTES:TOKEN_TIMEOUT_BYTES;
    localparam integer TIMEOUT_BITS=(TIMEOUT_MAX<2)?1:$clog2(TIMEOUT_MAX+1);
    reg [5:0] state,return_state,command_number;
    reg [31:0] command_argument;
    reg [2:0] command_index;
    reg [7:0] response;
    reg [POWER_BITS-1:0] power_count;
    reg [ATTEMPT_BITS-1:0] init_attempt;
    reg [TIMEOUT_BITS-1:0] wait_bytes;
    reg [8:0] data_index;
    reg [3:0] pre_index;
    reg [1:0] extra_index;
    reg io_write,r7_bad,ocr_bad,ocr_high_capacity;

    // SPI mode 0. MISO is sampled on rising edges; MOSI changes on falling
    // edges. Independent low/high periods preserve the falling-edge-to-MISO
    // sampling window at faster clocks. Legacy FAST_HALF_PERIOD remains the
    // default for both phases. Init always uses symmetric INIT_HALF_PERIOD.
    // Each byte ends with a full final low period before done.
    reg spi_start,spi_busy,spi_done,spi_sclk;
    reg [7:0] spi_tx,spi_shift_tx,spi_shift_rx,spi_rx;
    reg [7:0] spi_div_count;
    reg [2:0] spi_bit;
    reg spi_last;
    wire [7:0] spi_half_period=O_initialized ?
        (spi_sclk ? FAST_HIGH_PERIOD : FAST_LOW_PERIOD) : INIT_HALF_PERIOD;
    assign O_sd_clk=spi_sclk;
    // A dedicated output register removes the busy/shift mux from the pad
    // path. MOSI still changes on start/falling edges, with identical bits.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            spi_busy<=0;spi_done<=0;spi_sclk<=0;spi_shift_tx<=8'hff;O_sd_mosi<=1'b1;
            spi_shift_rx<=0;spi_rx<=0;spi_div_count<=0;spi_bit<=0;spi_last<=0;
        end else begin
            spi_done<=0;
            if(!spi_busy) begin
                spi_sclk<=0;spi_div_count<=0;
                if(spi_start) begin
                    spi_busy<=1;spi_shift_tx<=spi_tx;spi_shift_rx<=0;O_sd_mosi<=spi_tx[7];
                    spi_bit<=0;spi_last<=0;
                end
            end else if(spi_div_count==spi_half_period-1'b1) begin
                spi_div_count<=0;
                if(spi_last) begin
                    spi_busy<=0;spi_done<=1;spi_rx<=spi_shift_rx;O_sd_mosi<=1'b1;
                end else if(!spi_sclk) begin
                    spi_sclk<=1;spi_shift_rx<={spi_shift_rx[6:0],I_sd_miso};
                end else begin
                    spi_sclk<=0;
                    if(spi_bit==7) spi_last<=1;
                    else begin spi_bit<=spi_bit+1'b1;spi_shift_tx<={spi_shift_tx[6:0],1'b1};O_sd_mosi<=spi_shift_tx[6];end
                end
            end else spi_div_count<=spi_div_count+1'b1;
        end
    end

    task fail;
        input [3:0] code;
        begin
            O_error<=1;O_error_code<=code;O_sd_cs_n<=1;O_busy<=0;
            state<=O_initialized ? IDLE : FAILED;
        end
    endtask

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state<=POWER;return_state<=RES0;command_number<=0;command_argument<=0;
            command_index<=0;response<=8'hff;power_count<=0;init_attempt<=0;
            wait_bytes<=0;data_index<=0;pre_index<=0;extra_index<=0;
            io_write<=0;r7_bad<=0;ocr_bad<=0;ocr_high_capacity<=0;
            spi_start<=0;spi_tx<=8'hff;O_sd_cs_n<=1;O_initialized<=0;
            O_busy<=1;O_done<=0;O_error<=0;O_error_code<=0;
            O_read_valid<=0;O_read_data<=0;O_write_byte_req<=0;
        end else begin
            spi_start<=0;O_done<=0;O_read_valid<=0;O_write_byte_req<=0;
            case(state)
                POWER: begin
                    O_sd_cs_n<=1;
                    if(power_count==POWER_WAIT_CYCLES-1) begin power_count<=0;pre_index<=0;state<=PRE;end
                    else power_count<=power_count+1'b1;
                end
                PRE: begin spi_tx<=8'hff;spi_start<=1;state<=PRE_WAIT;end
                PRE_WAIT: if(spi_done) begin
                    if(pre_index==9) state<=CMD0;
                    else begin pre_index<=pre_index+1'b1;state<=PRE;end
                end
                CMD0: begin command_number<=0;command_argument<=0;return_state<=RES0;state<=GAP;end
                RES0: if(response==8'h01) state<=CMD8;else fail(1);
                CMD8: begin command_number<=8;command_argument<=32'h1aa;return_state<=RES8;state<=GAP;end
                RES8: if(response==8'h01) begin extra_index<=0;r7_bad<=0;state<=R7;end else fail(2);
                R7: begin spi_tx<=8'hff;spi_start<=1;state<=R7_WAIT;end
                R7_WAIT: if(spi_done) begin
                    if(extra_index==3) begin
                        if(r7_bad || spi_rx!=8'haa) fail(2);else state<=CMD55;
                    end else begin
                        if(spi_rx!=((extra_index==2)?8'h01:8'h00)) r7_bad<=1;
                        extra_index<=extra_index+1'b1;state<=R7;
                    end
                end
                CMD55: begin command_number<=55;command_argument<=0;return_state<=RES55;state<=GAP;end
                RES55: if(response==8'h01) state<=ACMD41;else fail(1);
                ACMD41: begin command_number<=41;command_argument<=32'h40000000;return_state<=RES41;state<=GAP;end
                RES41: begin
                    if(response==0) state<=CMD58;
                    else if(response==1 && init_attempt<INIT_MAX_ATTEMPTS-1) begin
                        init_attempt<=init_attempt+1'b1;power_count<=0;O_sd_cs_n<=1;state<=INIT_DELAY;
                    end else fail(1);
                end
                INIT_DELAY: begin
                    if(power_count==POWER_WAIT_CYCLES-1) begin power_count<=0;state<=CMD55;end
                    else power_count<=power_count+1'b1;
                end
                CMD58: begin command_number<=58;command_argument<=0;return_state<=RES58;state<=GAP;end
                RES58: if(response==0) begin extra_index<=0;ocr_bad<=0;ocr_high_capacity<=0;state<=OCR;end else fail(1);
                OCR: begin spi_tx<=8'hff;spi_start<=1;state<=OCR_WAIT;end
                OCR_WAIT: if(spi_done) begin
                    if(extra_index==0) begin ocr_bad<=!spi_rx[7];ocr_high_capacity<=spi_rx[6];end
                    if(extra_index==3) begin
                        if(ocr_bad) fail(1);
                        else if(!ocr_high_capacity) fail(2);
                        else begin O_initialized<=1;O_sd_cs_n<=1;O_busy<=0;state<=IDLE;end
                    end else begin extra_index<=extra_index+1'b1;state<=OCR;end
                end
                IDLE: if(I_start) begin
                    O_busy<=1;O_error<=0;O_error_code<=0;command_argument<=I_lba;
                    command_number<=I_write?24:17;io_write<=I_write;return_state<=IO_RES;state<=GAP;
                end
                FAILED: if(I_start) begin
                    O_error<=0;O_error_code<=0;O_busy<=1;power_count<=0;
                    init_attempt<=0;pre_index<=0;state<=POWER;
                end
                GAP: begin O_sd_cs_n<=1;spi_tx<=8'hff;spi_start<=1;state<=GAP_WAIT;end
                GAP_WAIT: if(spi_done) begin O_sd_cs_n<=0;command_index<=0;state<=CMD_SEND;end
                CMD_SEND: begin
                    case(command_index)
                        0:spi_tx<={2'b01,command_number};
                        1:spi_tx<=command_argument[31:24];2:spi_tx<=command_argument[23:16];
                        3:spi_tx<=command_argument[15:8];4:spi_tx<=command_argument[7:0];
                        default:spi_tx<=(command_number==0)?8'h95:((command_number==8)?8'h87:8'hff);
                    endcase
                    spi_start<=1;state<=CMD_SEND_WAIT;
                end
                CMD_SEND_WAIT: if(spi_done) begin
                    if(command_index==5) begin wait_bytes<=0;state<=R1;end
                    else begin command_index<=command_index+1'b1;state<=CMD_SEND;end
                end
                R1: begin spi_tx<=8'hff;spi_start<=1;state<=R1_WAIT;end
                R1_WAIT: if(spi_done) begin
                    if(!spi_rx[7]) begin response<=spi_rx;state<=return_state;end
                    else if(wait_bytes==CMD_TIMEOUT_BYTES-1) fail(O_initialized?3:1);
                    else begin wait_bytes<=wait_bytes+1'b1;state<=R1;end
                end
                IO_RES: begin
                    wait_bytes<=0;data_index<=0;extra_index<=0;
                    if(response!=0) fail(3);
                    else state<=io_write?WRITE_LEAD:READ_TOKEN;
                end
                READ_TOKEN: begin spi_tx<=8'hff;spi_start<=1;state<=READ_TOKEN_WAIT;end
                READ_TOKEN_WAIT: if(spi_done) begin
                    if(spi_rx==8'hfe) state<=READ_DATA;
                    else if(spi_rx!=8'hff || wait_bytes==TOKEN_TIMEOUT_BYTES-1) fail(4);
                    else begin wait_bytes<=wait_bytes+1'b1;state<=READ_TOKEN;end
                end
                READ_DATA: begin spi_tx<=8'hff;spi_start<=1;state<=READ_DATA_WAIT;end
                READ_DATA_WAIT: if(spi_done) begin
                    O_read_data<=spi_rx;
                    if(READ_FLOW_CONTROL) state<=READ_DELIVER;
                    else begin
                        O_read_valid<=1;
                        if(data_index==511) begin extra_index<=0;state<=READ_CRC;end
                        else begin data_index<=data_index+1'b1;state<=READ_DATA;end
                    end
                end
                // Stop SCLK between bytes while the replay DDR mailbox is full.
                // Publish a normal ready/valid word; it remains stable until
                // accepted. Legacy callers retain the original pulse contract.
                READ_DELIVER:begin
                    O_read_valid<=1;
                    if(O_read_valid && I_read_ready) begin
                        O_read_valid<=0;
                        if(data_index==511) begin extra_index<=0;state<=READ_CRC;end
                        else begin data_index<=data_index+1'b1;state<=READ_DATA;end
                    end
                end
                READ_CRC: begin spi_tx<=8'hff;spi_start<=1;state<=READ_CRC_WAIT;end
                READ_CRC_WAIT: if(spi_done) begin
                    if(extra_index==1) state<=FINISH;
                    else begin extra_index<=extra_index+1'b1;state<=READ_CRC;end
                end
                WRITE_LEAD: begin spi_tx<=8'hff;spi_start<=1;state<=WRITE_LEAD_WAIT;end
                WRITE_LEAD_WAIT: if(spi_done) state<=WRITE_TOKEN;
                WRITE_TOKEN: begin spi_tx<=8'hfe;spi_start<=1;state<=WRITE_TOKEN_WAIT;end
                WRITE_TOKEN_WAIT: if(spi_done) state<=WRITE_REQ;
                WRITE_REQ: begin O_write_byte_req<=1;state<=WRITE_DELAY;end
                WRITE_DELAY: state<=WRITE_DATA;
                WRITE_DATA: begin spi_tx<=I_write_data;spi_start<=1;state<=WRITE_DATA_WAIT;end
                WRITE_DATA_WAIT: if(spi_done) begin
                    if(data_index==511) begin extra_index<=0;state<=WRITE_CRC;end
                    else begin data_index<=data_index+1'b1;state<=WRITE_REQ;end
                end
                WRITE_CRC: begin spi_tx<=8'hff;spi_start<=1;state<=WRITE_CRC_WAIT;end
                WRITE_CRC_WAIT: if(spi_done) begin
                    if(extra_index==1) begin wait_bytes<=0;state<=WRITE_RESP;end
                    else begin extra_index<=extra_index+1'b1;state<=WRITE_CRC;end
                end
                WRITE_RESP: begin spi_tx<=8'hff;spi_start<=1;state<=WRITE_RESP_WAIT;end
                WRITE_RESP_WAIT: if(spi_done) begin
                    if(spi_rx[4:0]==5'b00101) begin wait_bytes<=0;state<=WRITE_BUSY;end
                    else if(spi_rx!=8'hff || wait_bytes==CMD_TIMEOUT_BYTES-1) fail(5);
                    else begin wait_bytes<=wait_bytes+1'b1;state<=WRITE_RESP;end
                end
                WRITE_BUSY: begin spi_tx<=8'hff;spi_start<=1;state<=WRITE_BUSY_WAIT;end
                WRITE_BUSY_WAIT: if(spi_done) begin
                    if(spi_rx==8'hff) state<=FINISH;
                    else if(wait_bytes==BUSY_TIMEOUT_BYTES-1) fail(6);
                    else begin wait_bytes<=wait_bytes+1'b1;state<=WRITE_BUSY;end
                end
                FINISH: begin O_sd_cs_n<=1;spi_tx<=8'hff;spi_start<=1;state<=FINISH_WAIT;end
                FINISH_WAIT: if(spi_done) begin O_busy<=0;O_done<=1;state<=IDLE;end
                default: begin O_initialized<=0;O_busy<=0;O_sd_cs_n<=1;O_error<=1;O_error_code<=1;state<=FAILED;end
            endcase
        end
    end
    // synthesis translate_off
    initial begin
        if(INIT_HALF_PERIOD<1 || INIT_HALF_PERIOD>255 || FAST_HALF_PERIOD<1 || FAST_HALF_PERIOD>255 ||
           FAST_LOW_PERIOD<1 || FAST_LOW_PERIOD>255 || FAST_HIGH_PERIOD<1 || FAST_HIGH_PERIOD>255 ||
           POWER_WAIT_CYCLES<1 || CMD_TIMEOUT_BYTES<1 || TOKEN_TIMEOUT_BYTES<1 || BUSY_TIMEOUT_BYTES<1 || INIT_MAX_ATTEMPTS<1)
            $fatal(1,"Invalid SD SPI clock or timeout parameters");
    end
    // synthesis translate_on
endmodule
