// Independently implemented binary morphology. Zero-padded 3x3 window.
// Raster input must leave >=4 clocks between pixels. Right/bottom padding
// is generated internally, including the last two output rows/columns.
// No line RAM reset: row qualification rejects all stale line contents.
`ifndef MOTION_BINARY_3X3_V
`define MOTION_BINARY_3X3_V
module motion_binary_3x3 #(
    parameter integer WIDTH=320, HEIGHT=180, ERODE=1
)(
    input wire I_clk,I_rst_n,I_abort,I_valid,I_bit,
    input wire [11:0] I_x,I_y,
    output reg O_valid,O_bit,
    output reg [11:0] O_x,O_y
);
    (* ram_style="distributed" *) reg [1:0] lines[0:WIDTH-1];
    reg [2:0] column_1,column_2;
    reg right_pad,flush;
    reg [11:0] right_y,flush_x;
    reg [1:0] pace;
    wire inject=flush && pace==0;
    wire step=I_valid || right_pad || inject;
    wire [11:0] x=I_valid ? I_x : right_pad ? WIDTH : flush_x;
    wire [11:0] y=I_valid ? I_y : right_pad ? right_y : HEIGHT;
    wire bit_in=I_valid && I_bit;
    wire [1:0] line_data=x<WIDTH ? lines[x] : 2'b0;
    wire [2:0] column={y>=2 ? line_data[1] : 1'b0,
                       y>=1 ? line_data[0] : 1'b0,bit_in};
    // Mode2 retains a 3-pixel horizontal OR vertical run. At a 4px sample
    // pitch a strict square erosion would erase slowly moving thin edges.
    wire directional=(&column_1) || (&{column_2[1],column_1[1],column[1]});
    wire filtered=ERODE==2 ? directional : ERODE==1 ?
        &{column_2,column_1,column} : |{column_2,column_1,column};
    always @(posedge I_clk) begin
        if(step && x<WIDTH && y<HEIGHT && !I_abort)
            lines[x]<={y==0 ? 1'b0 : line_data[0],bit_in};
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_valid<=0;O_bit<=0;O_x<=0;O_y<=0;
            column_1<=0;column_2<=0;right_pad<=0;
            right_y<=0;flush<=0;flush_x<=0;pace<=0;
        end else begin
            O_valid<=0;
            if(I_abort) begin
                column_1<=0;column_2<=0;right_pad<=0;flush<=0;pace<=0;
            end else begin
                if(flush && pace!=0) pace<=pace-1'b1;
                if(step) begin
                    column_1<=column;
                    column_2<=x==0 ? 3'b0 : column_1;
                    if(x>=1 && y>=1) begin
                        O_valid<=1;O_bit<=filtered;O_x<=x-1'b1;O_y<=y-1'b1;
                    end
                    if(I_valid && I_x==WIDTH-1) begin
                        right_pad<=1;right_y<=I_y;
                    end else if(right_pad) begin
                        right_pad<=0;
                        if(right_y==HEIGHT-1) begin
                            flush<=1;flush_x<=0;pace<=3;
                        end
                    end else if(inject) begin
                        pace<=3;
                        if(flush_x==WIDTH) begin flush<=0;flush_x<=0;end
                        else flush_x<=flush_x+1'b1;
                    end
                end
            end
        end
    end
endmodule
`endif
