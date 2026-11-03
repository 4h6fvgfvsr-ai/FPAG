`ifndef OSD_DECIMAL_SNAPSHOT_V
`define OSD_DECIMAL_SNAPSHOT_V
// Share one16-cycle double-dabble converter across six frame-stable fields.
// Capture oneclock after SOF to observe tracker/latency frame commits.
module osd_decimal_snapshot(
    input wire I_clk,I_rst_n,I_sof,
    input wire [15:0] I_latency,I_source_id,I_x,I_y,I_motion,I_events,
    output reg [19:0] O_latency,O_source_id,O_x,O_y,O_motion,O_events
);
    reg pending,busy;
    reg [2:0] field;
    reg [4:0] count;
    reg [15:0] words[0:5];
    reg [15:0] binary;
    reg [19:0] bcd;
    reg [19:0] adjusted;
    wire [19:0] shifted={adjusted[18:0],binary[15]};
    integer j;
    always @(*) begin
        adjusted=bcd;
        for(j=0;j<5;j=j+1) if(bcd[j*4 +:4]>=5) adjusted[j*4 +:4]=bcd[j*4 +:4]+4'd3;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            pending<=0;busy<=0;field<=0;count<=0;binary<=0;bcd<=0;
            O_latency<=0;O_source_id<=0;O_x<=0;O_y<=0;O_motion<=0;O_events<=0;
            for(j=0;j<6;j=j+1) words[j]<=0;
        end else begin
            if(I_sof) pending<=1;
            if(pending && !busy) begin
                pending<=0;busy<=1;field<=0;count<=0;bcd<=0;
                words[0]<=I_latency;words[1]<=I_source_id;words[2]<=I_x;
                words[3]<=I_y;words[4]<=I_motion;words[5]<=I_events;
                binary<=I_latency;
            end else if(busy) begin
                binary<={binary[14:0],1'b0};bcd<=shifted;
                if(count==15) begin
                    case(field)
                        0:O_latency<=shifted;1:O_source_id<=shifted;
                        2:O_x<=shifted;3:O_y<=shifted;
                        4:O_motion<=shifted;5:O_events<=shifted;
                    endcase
                    if(field==5) busy<=0;
                    else begin field<=field+1'b1;binary<=words[field+1'b1];bcd<=0;end
                    count<=0;
                end else count<=count+1'b1;
            end
        end
    end
endmodule
`endif
