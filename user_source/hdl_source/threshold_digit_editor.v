`ifndef THRESHOLD_DIGIT_EDITOR_V
`define THRESHOLD_DIGIT_EDITOR_V
// One decimal position is advanced independently, skipping values outside the
// legal interval. Eight double-dabble clocks and a short search share a small
// datapath; no multiplication, division or image-stream registers are used.
module threshold_digit_editor(
    input wire I_clk,I_rst_n,I_start,
    input wire [7:0] I_value,I_min,
    input wire [1:0] I_digit, // 0 hundreds, 1 tens, 2 units
    output wire O_busy,
    output reg O_done,
    output reg [7:0] O_value
);
    localparam [1:0] IDLE=0,CONVERT=1,PREPARE=2,SEARCH=3;
    reg [1:0] state,digit;
    reg [7:0] binary,minimum,original;
    reg [11:0] bcd,adjusted;
    reg [2:0] count;
    reg [3:0] trial,attempt;
    reg [3:0] hundreds,tens,units;
    reg [9:0] candidate;
    wire [11:0] shifted={adjusted[10:0],binary[7]};
    assign O_busy=state!=IDLE;
    integer j;
    always @(*) begin
        adjusted=bcd;
        for(j=0;j<3;j=j+1)
            if(bcd[j*4 +:4]>=5) adjusted[j*4 +:4]=bcd[j*4 +:4]+4'd3;
        hundreds=bcd[11:8];tens=bcd[7:4];units=bcd[3:0];
        case(digit)
            0:hundreds=trial;
            1:tens=trial;
            default:units=trial;
        endcase
        candidate=({6'd0,hundreds}<<6)+({6'd0,hundreds}<<5)+
                  ({6'd0,hundreds}<<2)+({6'd0,tens}<<3)+
                  ({6'd0,tens}<<1)+{6'd0,units};
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state<=IDLE;digit<=0;binary<=0;minimum<=0;original<=0;
            bcd<=0;count<=0;trial<=0;attempt<=0;O_done<=0;O_value<=0;
        end else begin
            O_done<=0;
            case(state)
                IDLE:if(I_start) begin
                    binary<=I_value;original<=I_value;minimum<=I_min;
                    digit<=I_digit;count<=0;bcd<=0;state<=CONVERT;
                end
                CONVERT:begin
                    binary<={binary[6:0],1'b0};bcd<=shifted;
                    if(count==7) state<=PREPARE;
                    else count<=count+1'b1;
                end
                PREPARE:begin
                    case(digit)
                        0:trial<=bcd[11:8]==9?4'd0:bcd[11:8]+1'b1;
                        1:trial<=bcd[7:4]==9?4'd0:bcd[7:4]+1'b1;
                        default:trial<=bcd[3:0]==9?4'd0:bcd[3:0]+1'b1;
                    endcase
                    attempt<=0;state<=SEARCH;
                end
                SEARCH:if(candidate<=255 && candidate>={2'd0,minimum}) begin
                    O_value<=candidate[7:0];O_done<=1;state<=IDLE;
                end else if(attempt==9) begin
                    O_value<=original;O_done<=1;state<=IDLE;
                end else begin
                    trial<=trial==9?4'd0:trial+1'b1;attempt<=attempt+1'b1;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
`endif
