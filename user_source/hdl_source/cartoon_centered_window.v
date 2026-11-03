`ifndef CARTOON_CENTERED_WINDOW_V
`define CARTOON_CENTERED_WINDOW_V
// Centered 3x3 raster window with synchronous RAM and replicated borders.
// Contract: continuous rectangular active rows, H_TOTAL clocks per row;
// I_user coincides with the first pixel, at least one blank row follows.
// Low TOP_W bits alone are retained in row two (center payload is row one).
// Sampled input at edgeN produces O_de after edgeN+H_TOTAL+1.
module cartoon_centered_window #(
    parameter integer IMG_WIDTH=1280, IMG_HEIGHT=720, H_TOTAL=1650,
    parameter integer DATA_W=24, TOP_W=24
)(
    input wire I_clk,I_rst_n,I_de,I_user,I_last,
    input wire [DATA_W-1:0] I_data,
    output reg O_de,O_user,O_last,
    output reg [TOP_W-1:0] O_a,O_b,O_c,
    output reg [DATA_W-1:0] O_d,O_e,O_f,O_g,O_h,O_i
);
    reg [11:0] x,y;
    reg running;
    wire start=I_de && I_user;
    wire [11:0] px=start ? 12'd0 : x;
    wire [11:0] py=start ? 12'd0 : y;
    wire sample=(running || start) && px<=IMG_WIDTH && py<=IMG_HEIGHT;
    reg [DATA_W-1:0] line1[0:IMG_WIDTH-1];
    reg [TOP_W-1:0] line2[0:IMG_WIDTH-1];
    reg [DATA_W-1:0] q1,last0,last1;
    reg [TOP_W-1:0] q2,last2;
    reg [TOP_W-1:0] a,b;
    reg [DATA_W-1:0] d,e,g,h;
    wire [DATA_W-1:0] current=px==IMG_WIDTH ? last0 :
        (py==IMG_HEIGHT ? q1 : I_data);
    wire [DATA_W-1:0] above=px==IMG_WIDTH ? last1 : (py==0 ? current : q1);
    wire [TOP_W-1:0] top=px==IMG_WIDTH ? last2 :
        (py<=1 ? above[TOP_W-1:0] : q2);
    wire [11:0] read_x=sample && px<IMG_WIDTH-1 ? px+12'd1 : 12'd0;
    reg [11:0] read_previous;
    reg read_valid;
    wire read_enable=!read_valid || start || read_x!=read_previous;
    // Prefetch the next column. Right padding uses saved last-column values.
    always @(posedge I_clk) begin
        if(read_enable) begin q1<=line1[read_x];q2<=line2[read_x];end
        if(sample && px<IMG_WIDTH) begin
            line1[px]<=current;
            line2[px]<=above[TOP_W-1:0];
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin x<=0;y<=0;running<=0;read_valid<=0;read_previous<=0;end
        else begin
            read_valid<=1;read_previous<=read_x;
            if(start) begin x<=1;y<=0;running<=1;end
            else if(running) begin
                if(x==H_TOTAL-1) begin x<=0;y<=y+1'b1;end
                else x<=x+1'b1;
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            a<=0;b<=0;d<=0;e<=0;g<=0;h<=0;
            last0<=0;last1<=0;last2<=0;
            O_de<=0;O_user<=0;O_last<=0;
            O_a<=0;O_b<=0;O_c<=0;O_d<=0;O_e<=0;O_f<=0;
            O_g<=0;O_h<=0;O_i<=0;
        end else begin
            O_de<=sample && px>0 && py>0;
            O_user<=sample && px==1 && py==1;
            O_last<=sample && px==IMG_WIDTH && py>0;
            if(sample) begin
                last0<=current;last1<=above;last2<=top;
                if(px==0) begin a<=top;b<=top;d<=above;e<=above;g<=current;h<=current;end
                else begin
                    O_a<=a;O_b<=b;O_c<=top;
                    O_d<=d;O_e<=e;O_f<=above;
                    O_g<=g;O_h<=h;O_i<=current;
                    a<=b;b<=top;d<=e;e<=above;g<=h;h<=current;
                end
            end
        end
    end
endmodule
`endif
