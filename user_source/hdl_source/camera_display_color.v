`ifndef CAMERA_DISPLAY_COLOR_V
`define CAMERA_DISPLAY_COLOR_V
`include "cartoon_original_tone.v"
// Shared live/replay color finish. Four clocks, one RGB pixel every clock.
// Chroma about Rec601 luma grows 1/8, skin-like colors 1/16. Near-neutrals
// and deep shadows bypass chroma. One shared headroom decision backs off
// all three channels together rather than independently clipping a hue.
module camera_display_color(
    input wire I_clk,I_rst_n,input wire [23:0] I_rgb,
    output wire [23:0] O_rgb
);
    reg [23:0] rgb_1,rgb_2;
    reg [7:0] y_1,mx_1,mn_1;
    wire [15:0] red={8'd0,I_rgb[23:16]},green={8'd0,I_rgb[15:8]},blue={8'd0,I_rgb[7:0]};
    // Constant shift/add coefficients consume no additional multipliers.
    wire [15:0] y_sum=(red<<6)+(red<<3)+(red<<2)+red+
        (green<<7)+(green<<4)+(green<<2)+(green<<1)+
        (blue<<4)+(blue<<3)+(blue<<2)+blue+16'd128;
    wire [7:0] mx=I_rgb[23:16]>I_rgb[15:8] ?
        (I_rgb[23:16]>I_rgb[7:0]?I_rgb[23:16]:I_rgb[7:0]):
        (I_rgb[15:8]>I_rgb[7:0]?I_rgb[15:8]:I_rgb[7:0]);
    wire [7:0] mn=I_rgb[23:16]<I_rgb[15:8] ?
        (I_rgb[23:16]<I_rgb[7:0]?I_rgb[23:16]:I_rgb[7:0]):
        (I_rgb[15:8]<I_rgb[7:0]?I_rgb[15:8]:I_rgb[7:0]);
    function signed [10:0] chroma;
        input [7:0] v,y;
        input half_strength;
        reg signed [10:0] d,amount;
        begin
            d=$signed({3'b0,v})-$signed({3'b0,y});
            if(half_strength) amount=d<0 ? -((-d+11'sd8)>>>4):((d+11'sd8)>>>4);
            else amount=d<0 ? -((-d+11'sd4)>>>3):((d+11'sd4)>>>3);
            chroma=$signed({3'b0,v})+amount;
        end
    endfunction
    wire signed [10:0] r8=chroma(rgb_1[23:16],y_1,0),
        g8=chroma(rgb_1[15:8],y_1,0),b8=chroma(rgb_1[7:0],y_1,0);
    wire signed [10:0] r16=chroma(rgb_1[23:16],y_1,1),
        g16=chroma(rgb_1[15:8],y_1,1),b16=chroma(rgb_1[7:0],y_1,1);
    wire safe8=r8>=0 && r8<=255 && g8>=0 && g8<=255 && b8>=0 && b8<=255;
    wire safe16=r16>=0 && r16<=255 && g16>=0 && g16<=255 && b16>=0 && b16<=255;
    wire skin=rgb_1[23:16]>rgb_1[15:8] && rgb_1[15:8]>=rgb_1[7:0];
    wire protect_chroma=(mx_1-mn_1<=16) || mx_1<=24;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin rgb_1<=0;rgb_2<=0;y_1<=0;mx_1<=0;mn_1<=0;end
        else begin
            rgb_1<=I_rgb;y_1<=y_sum[15:8];mx_1<=mx;mn_1<=mn;
            if(protect_chroma)rgb_2<=rgb_1;
            else if(!skin && safe8)rgb_2<={r8[7:0],g8[7:0],b8[7:0]};
            else if(safe16)rgb_2<={r16[7:0],g16[7:0],b16[7:0]};
            else rgb_2<=rgb_1;
        end
    end
    cartoon_original_tone #(.QUALITY_GAMMA(2)) u_tone(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_rgb(rgb_2),.O_rgb(O_rgb));
endmodule
`endif
