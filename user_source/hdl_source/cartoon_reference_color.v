// Streaming fixed-point approximation of cartoon_reference_rgb.m, luma profile.
// One pixel on every clock, no enables and no frame storage.
// Nine register stages (LATENCY = 9): an input sampled at edge N is available
// just after edge N+8. Delay DE/HS/VS/mode alongside by nine registers.
// Tone: exact rounded 256-entry MATLAB knot LUT.
// Saturation: nearest 1/16 chroma/max bin through 3/8; gain <= 96/64.
// Warm-color protection caps gain at 80/64 without affecting strong primaries.
// Midtone lift: 8-bit max LUT, quarter-step colorful weight (<= 26 counts).
// Neutral highlight: same max ramp, quantized saturation gates, Q5 blend <=16/32.
// Palette: rounded 64-level palette, rounded 35-percent blend.
// Edge: root supplies opacity Q8 (0..77), thin-gradient NMS is external.
// No variable divider or general multiplier is used; only fixed shifts/adds.
// I_detail is signed luma-minus-local-lowpass, bounded internally to +/-15.
`ifndef CARTOON_REFERENCE_COLOR_V
`define CARTOON_REFERENCE_COLOR_V

module cartoon_reference_color #(
    parameter integer FLAT_STYLE = 0
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire [23:0] I_rgb,
    input wire signed [8:0] I_detail,
    input wire [7:0] I_edge_strength,
    output reg [23:0] O_rgb
);
    localparam integer LATENCY = 9;
    function [7:0] tone_lut;
        input [7:0] value;
        begin case(value)
            8'd0: tone_lut = 8'd0;
            8'd1: tone_lut = 8'd1;
            8'd2: tone_lut = 8'd1;
            8'd3: tone_lut = 8'd2;
            8'd4: tone_lut = 8'd2;
            8'd5: tone_lut = 8'd3;
            8'd6: tone_lut = 8'd3;
            8'd7: tone_lut = 8'd4;
            8'd8: tone_lut = 8'd4;
            8'd9: tone_lut = 8'd5;
            8'd10: tone_lut = 8'd5;
            8'd11: tone_lut = 8'd6;
            8'd12: tone_lut = 8'd7;
            8'd13: tone_lut = 8'd7;
            8'd14: tone_lut = 8'd8;
            8'd15: tone_lut = 8'd8;
            8'd16: tone_lut = 8'd9;
            8'd17: tone_lut = 8'd9;
            8'd18: tone_lut = 8'd10;
            8'd19: tone_lut = 8'd10;
            8'd20: tone_lut = 8'd11;
            8'd21: tone_lut = 8'd12;
            8'd22: tone_lut = 8'd12;
            8'd23: tone_lut = 8'd13;
            8'd24: tone_lut = 8'd13;
            8'd25: tone_lut = 8'd14;
            8'd26: tone_lut = 8'd14;
            8'd27: tone_lut = 8'd15;
            8'd28: tone_lut = 8'd16;
            8'd29: tone_lut = 8'd17;
            8'd30: tone_lut = 8'd18;
            8'd31: tone_lut = 8'd19;
            8'd32: tone_lut = 8'd20;
            8'd33: tone_lut = 8'd21;
            8'd34: tone_lut = 8'd22;
            8'd35: tone_lut = 8'd23;
            8'd36: tone_lut = 8'd24;
            8'd37: tone_lut = 8'd25;
            8'd38: tone_lut = 8'd26;
            8'd39: tone_lut = 8'd27;
            8'd40: tone_lut = 8'd28;
            8'd41: tone_lut = 8'd29;
            8'd42: tone_lut = 8'd30;
            8'd43: tone_lut = 8'd31;
            8'd44: tone_lut = 8'd32;
            8'd45: tone_lut = 8'd33;
            8'd46: tone_lut = 8'd33;
            8'd47: tone_lut = 8'd34;
            8'd48: tone_lut = 8'd35;
            8'd49: tone_lut = 8'd36;
            8'd50: tone_lut = 8'd37;
            8'd51: tone_lut = 8'd38;
            8'd52: tone_lut = 8'd39;
            8'd53: tone_lut = 8'd40;
            8'd54: tone_lut = 8'd41;
            8'd55: tone_lut = 8'd42;
            8'd56: tone_lut = 8'd43;
            8'd57: tone_lut = 8'd44;
            8'd58: tone_lut = 8'd45;
            8'd59: tone_lut = 8'd46;
            8'd60: tone_lut = 8'd47;
            8'd61: tone_lut = 8'd49;
            8'd62: tone_lut = 8'd50;
            8'd63: tone_lut = 8'd51;
            8'd64: tone_lut = 8'd52;
            8'd65: tone_lut = 8'd53;
            8'd66: tone_lut = 8'd54;
            8'd67: tone_lut = 8'd55;
            8'd68: tone_lut = 8'd56;
            8'd69: tone_lut = 8'd57;
            8'd70: tone_lut = 8'd58;
            8'd71: tone_lut = 8'd59;
            8'd72: tone_lut = 8'd60;
            8'd73: tone_lut = 8'd61;
            8'd74: tone_lut = 8'd62;
            8'd75: tone_lut = 8'd63;
            8'd76: tone_lut = 8'd64;
            8'd77: tone_lut = 8'd65;
            8'd78: tone_lut = 8'd66;
            8'd79: tone_lut = 8'd67;
            8'd80: tone_lut = 8'd68;
            8'd81: tone_lut = 8'd69;
            8'd82: tone_lut = 8'd70;
            8'd83: tone_lut = 8'd71;
            8'd84: tone_lut = 8'd72;
            8'd85: tone_lut = 8'd73;
            8'd86: tone_lut = 8'd74;
            8'd87: tone_lut = 8'd75;
            8'd88: tone_lut = 8'd76;
            8'd89: tone_lut = 8'd77;
            8'd90: tone_lut = 8'd78;
            8'd91: tone_lut = 8'd79;
            8'd92: tone_lut = 8'd80;
            8'd93: tone_lut = 8'd81;
            8'd94: tone_lut = 8'd82;
            8'd95: tone_lut = 8'd83;
            8'd96: tone_lut = 8'd84;
            8'd97: tone_lut = 8'd85;
            8'd98: tone_lut = 8'd86;
            8'd99: tone_lut = 8'd87;
            8'd100: tone_lut = 8'd88;
            8'd101: tone_lut = 8'd89;
            8'd102: tone_lut = 8'd91;
            8'd103: tone_lut = 8'd92;
            8'd104: tone_lut = 8'd93;
            8'd105: tone_lut = 8'd94;
            8'd106: tone_lut = 8'd95;
            8'd107: tone_lut = 8'd96;
            8'd108: tone_lut = 8'd97;
            8'd109: tone_lut = 8'd98;
            8'd110: tone_lut = 8'd99;
            8'd111: tone_lut = 8'd100;
            8'd112: tone_lut = 8'd101;
            8'd113: tone_lut = 8'd102;
            8'd114: tone_lut = 8'd103;
            8'd115: tone_lut = 8'd104;
            8'd116: tone_lut = 8'd105;
            8'd117: tone_lut = 8'd106;
            8'd118: tone_lut = 8'd107;
            8'd119: tone_lut = 8'd108;
            8'd120: tone_lut = 8'd109;
            8'd121: tone_lut = 8'd110;
            8'd122: tone_lut = 8'd111;
            8'd123: tone_lut = 8'd112;
            8'd124: tone_lut = 8'd113;
            8'd125: tone_lut = 8'd114;
            8'd126: tone_lut = 8'd115;
            8'd127: tone_lut = 8'd116;
            8'd128: tone_lut = 8'd117;
            8'd129: tone_lut = 8'd118;
            8'd130: tone_lut = 8'd119;
            8'd131: tone_lut = 8'd120;
            8'd132: tone_lut = 8'd121;
            8'd133: tone_lut = 8'd122;
            8'd134: tone_lut = 8'd123;
            8'd135: tone_lut = 8'd124;
            8'd136: tone_lut = 8'd125;
            8'd137: tone_lut = 8'd126;
            8'd138: tone_lut = 8'd127;
            8'd139: tone_lut = 8'd128;
            8'd140: tone_lut = 8'd129;
            8'd141: tone_lut = 8'd131;
            8'd142: tone_lut = 8'd132;
            8'd143: tone_lut = 8'd133;
            8'd144: tone_lut = 8'd134;
            8'd145: tone_lut = 8'd135;
            8'd146: tone_lut = 8'd136;
            8'd147: tone_lut = 8'd137;
            8'd148: tone_lut = 8'd138;
            8'd149: tone_lut = 8'd139;
            8'd150: tone_lut = 8'd140;
            8'd151: tone_lut = 8'd141;
            8'd152: tone_lut = 8'd142;
            8'd153: tone_lut = 8'd143;
            8'd154: tone_lut = 8'd144;
            8'd155: tone_lut = 8'd145;
            8'd156: tone_lut = 8'd146;
            8'd157: tone_lut = 8'd147;
            8'd158: tone_lut = 8'd149;
            8'd159: tone_lut = 8'd150;
            8'd160: tone_lut = 8'd151;
            8'd161: tone_lut = 8'd152;
            8'd162: tone_lut = 8'd153;
            8'd163: tone_lut = 8'd154;
            8'd164: tone_lut = 8'd155;
            8'd165: tone_lut = 8'd157;
            8'd166: tone_lut = 8'd158;
            8'd167: tone_lut = 8'd159;
            8'd168: tone_lut = 8'd160;
            8'd169: tone_lut = 8'd161;
            8'd170: tone_lut = 8'd162;
            8'd171: tone_lut = 8'd163;
            8'd172: tone_lut = 8'd165;
            8'd173: tone_lut = 8'd166;
            8'd174: tone_lut = 8'd167;
            8'd175: tone_lut = 8'd168;
            8'd176: tone_lut = 8'd169;
            8'd177: tone_lut = 8'd170;
            8'd178: tone_lut = 8'd172;
            8'd179: tone_lut = 8'd173;
            8'd180: tone_lut = 8'd174;
            8'd181: tone_lut = 8'd175;
            8'd182: tone_lut = 8'd176;
            8'd183: tone_lut = 8'd177;
            8'd184: tone_lut = 8'd178;
            8'd185: tone_lut = 8'd180;
            8'd186: tone_lut = 8'd181;
            8'd187: tone_lut = 8'd182;
            8'd188: tone_lut = 8'd183;
            8'd189: tone_lut = 8'd184;
            8'd190: tone_lut = 8'd185;
            8'd191: tone_lut = 8'd187;
            8'd192: tone_lut = 8'd188;
            8'd193: tone_lut = 8'd189;
            8'd194: tone_lut = 8'd190;
            8'd195: tone_lut = 8'd191;
            8'd196: tone_lut = 8'd192;
            8'd197: tone_lut = 8'd193;
            8'd198: tone_lut = 8'd195;
            8'd199: tone_lut = 8'd196;
            8'd200: tone_lut = 8'd197;
            8'd201: tone_lut = 8'd198;
            8'd202: tone_lut = 8'd199;
            8'd203: tone_lut = 8'd200;
            8'd204: tone_lut = 8'd201;
            8'd205: tone_lut = 8'd203;
            8'd206: tone_lut = 8'd204;
            8'd207: tone_lut = 8'd205;
            8'd208: tone_lut = 8'd207;
            8'd209: tone_lut = 8'd208;
            8'd210: tone_lut = 8'd209;
            8'd211: tone_lut = 8'd210;
            8'd212: tone_lut = 8'd212;
            8'd213: tone_lut = 8'd213;
            8'd214: tone_lut = 8'd214;
            8'd215: tone_lut = 8'd215;
            8'd216: tone_lut = 8'd217;
            8'd217: tone_lut = 8'd218;
            8'd218: tone_lut = 8'd219;
            8'd219: tone_lut = 8'd220;
            8'd220: tone_lut = 8'd222;
            8'd221: tone_lut = 8'd223;
            8'd222: tone_lut = 8'd224;
            8'd223: tone_lut = 8'd226;
            8'd224: tone_lut = 8'd227;
            8'd225: tone_lut = 8'd228;
            8'd226: tone_lut = 8'd229;
            8'd227: tone_lut = 8'd231;
            8'd228: tone_lut = 8'd232;
            8'd229: tone_lut = 8'd233;
            8'd230: tone_lut = 8'd234;
            8'd231: tone_lut = 8'd236;
            8'd232: tone_lut = 8'd237;
            8'd233: tone_lut = 8'd238;
            8'd234: tone_lut = 8'd239;
            8'd235: tone_lut = 8'd241;
            8'd236: tone_lut = 8'd242;
            8'd237: tone_lut = 8'd243;
            8'd238: tone_lut = 8'd245;
            8'd239: tone_lut = 8'd246;
            8'd240: tone_lut = 8'd247;
            8'd241: tone_lut = 8'd248;
            8'd242: tone_lut = 8'd250;
            8'd243: tone_lut = 8'd250;
            8'd244: tone_lut = 8'd251;
            8'd245: tone_lut = 8'd251;
            8'd246: tone_lut = 8'd251;
            8'd247: tone_lut = 8'd252;
            8'd248: tone_lut = 8'd252;
            8'd249: tone_lut = 8'd253;
            8'd250: tone_lut = 8'd253;
            8'd251: tone_lut = 8'd253;
            8'd252: tone_lut = 8'd254;
            8'd253: tone_lut = 8'd254;
            8'd254: tone_lut = 8'd255;
            8'd255: tone_lut = 8'd255;
            default: tone_lut = 8'd0;
        endcase end
    endfunction
    function [4:0] lift_lut;
        input [7:0] value;
        begin case(value)
            8'd0: lift_lut = 5'd0;
            8'd1: lift_lut = 5'd0;
            8'd2: lift_lut = 5'd0;
            8'd3: lift_lut = 5'd0;
            8'd4: lift_lut = 5'd0;
            8'd5: lift_lut = 5'd0;
            8'd6: lift_lut = 5'd0;
            8'd7: lift_lut = 5'd0;
            8'd8: lift_lut = 5'd0;
            8'd9: lift_lut = 5'd0;
            8'd10: lift_lut = 5'd0;
            8'd11: lift_lut = 5'd0;
            8'd12: lift_lut = 5'd0;
            8'd13: lift_lut = 5'd0;
            8'd14: lift_lut = 5'd0;
            8'd15: lift_lut = 5'd0;
            8'd16: lift_lut = 5'd0;
            8'd17: lift_lut = 5'd0;
            8'd18: lift_lut = 5'd0;
            8'd19: lift_lut = 5'd0;
            8'd20: lift_lut = 5'd0;
            8'd21: lift_lut = 5'd0;
            8'd22: lift_lut = 5'd0;
            8'd23: lift_lut = 5'd0;
            8'd24: lift_lut = 5'd0;
            8'd25: lift_lut = 5'd0;
            8'd26: lift_lut = 5'd0;
            8'd27: lift_lut = 5'd0;
            8'd28: lift_lut = 5'd0;
            8'd29: lift_lut = 5'd0;
            8'd30: lift_lut = 5'd0;
            8'd31: lift_lut = 5'd0;
            8'd32: lift_lut = 5'd0;
            8'd33: lift_lut = 5'd0;
            8'd34: lift_lut = 5'd0;
            8'd35: lift_lut = 5'd0;
            8'd36: lift_lut = 5'd0;
            8'd37: lift_lut = 5'd0;
            8'd38: lift_lut = 5'd0;
            8'd39: lift_lut = 5'd0;
            8'd40: lift_lut = 5'd1;
            8'd41: lift_lut = 5'd1;
            8'd42: lift_lut = 5'd2;
            8'd43: lift_lut = 5'd2;
            8'd44: lift_lut = 5'd3;
            8'd45: lift_lut = 5'd3;
            8'd46: lift_lut = 5'd4;
            8'd47: lift_lut = 5'd4;
            8'd48: lift_lut = 5'd5;
            8'd49: lift_lut = 5'd5;
            8'd50: lift_lut = 5'd6;
            8'd51: lift_lut = 5'd6;
            8'd52: lift_lut = 5'd7;
            8'd53: lift_lut = 5'd7;
            8'd54: lift_lut = 5'd8;
            8'd55: lift_lut = 5'd8;
            8'd56: lift_lut = 5'd9;
            8'd57: lift_lut = 5'd9;
            8'd58: lift_lut = 5'd10;
            8'd59: lift_lut = 5'd10;
            8'd60: lift_lut = 5'd11;
            8'd61: lift_lut = 5'd11;
            8'd62: lift_lut = 5'd12;
            8'd63: lift_lut = 5'd12;
            8'd64: lift_lut = 5'd13;
            8'd65: lift_lut = 5'd13;
            8'd66: lift_lut = 5'd14;
            8'd67: lift_lut = 5'd14;
            8'd68: lift_lut = 5'd15;
            8'd69: lift_lut = 5'd15;
            8'd70: lift_lut = 5'd16;
            8'd71: lift_lut = 5'd16;
            8'd72: lift_lut = 5'd17;
            8'd73: lift_lut = 5'd17;
            8'd74: lift_lut = 5'd18;
            8'd75: lift_lut = 5'd18;
            8'd76: lift_lut = 5'd19;
            8'd77: lift_lut = 5'd19;
            8'd78: lift_lut = 5'd20;
            8'd79: lift_lut = 5'd20;
            8'd80: lift_lut = 5'd21;
            8'd81: lift_lut = 5'd21;
            8'd82: lift_lut = 5'd22;
            8'd83: lift_lut = 5'd22;
            8'd84: lift_lut = 5'd23;
            8'd85: lift_lut = 5'd23;
            8'd86: lift_lut = 5'd24;
            8'd87: lift_lut = 5'd24;
            8'd88: lift_lut = 5'd25;
            8'd89: lift_lut = 5'd25;
            8'd90: lift_lut = 5'd26;
            8'd91: lift_lut = 5'd26;
            8'd92: lift_lut = 5'd26;
            8'd93: lift_lut = 5'd26;
            8'd94: lift_lut = 5'd26;
            8'd95: lift_lut = 5'd26;
            8'd96: lift_lut = 5'd26;
            8'd97: lift_lut = 5'd26;
            8'd98: lift_lut = 5'd26;
            8'd99: lift_lut = 5'd26;
            8'd100: lift_lut = 5'd26;
            8'd101: lift_lut = 5'd26;
            8'd102: lift_lut = 5'd26;
            8'd103: lift_lut = 5'd26;
            8'd104: lift_lut = 5'd26;
            8'd105: lift_lut = 5'd26;
            8'd106: lift_lut = 5'd26;
            8'd107: lift_lut = 5'd26;
            8'd108: lift_lut = 5'd26;
            8'd109: lift_lut = 5'd26;
            8'd110: lift_lut = 5'd26;
            8'd111: lift_lut = 5'd26;
            8'd112: lift_lut = 5'd26;
            8'd113: lift_lut = 5'd26;
            8'd114: lift_lut = 5'd26;
            8'd115: lift_lut = 5'd26;
            8'd116: lift_lut = 5'd26;
            8'd117: lift_lut = 5'd26;
            8'd118: lift_lut = 5'd26;
            8'd119: lift_lut = 5'd26;
            8'd120: lift_lut = 5'd26;
            8'd121: lift_lut = 5'd26;
            8'd122: lift_lut = 5'd26;
            8'd123: lift_lut = 5'd26;
            8'd124: lift_lut = 5'd26;
            8'd125: lift_lut = 5'd26;
            8'd126: lift_lut = 5'd26;
            8'd127: lift_lut = 5'd26;
            8'd128: lift_lut = 5'd26;
            8'd129: lift_lut = 5'd26;
            8'd130: lift_lut = 5'd26;
            8'd131: lift_lut = 5'd26;
            8'd132: lift_lut = 5'd26;
            8'd133: lift_lut = 5'd26;
            8'd134: lift_lut = 5'd26;
            8'd135: lift_lut = 5'd26;
            8'd136: lift_lut = 5'd26;
            8'd137: lift_lut = 5'd26;
            8'd138: lift_lut = 5'd26;
            8'd139: lift_lut = 5'd26;
            8'd140: lift_lut = 5'd26;
            8'd141: lift_lut = 5'd26;
            8'd142: lift_lut = 5'd26;
            8'd143: lift_lut = 5'd26;
            8'd144: lift_lut = 5'd26;
            8'd145: lift_lut = 5'd26;
            8'd146: lift_lut = 5'd26;
            8'd147: lift_lut = 5'd26;
            8'd148: lift_lut = 5'd26;
            8'd149: lift_lut = 5'd26;
            8'd150: lift_lut = 5'd26;
            8'd151: lift_lut = 5'd26;
            8'd152: lift_lut = 5'd26;
            8'd153: lift_lut = 5'd26;
            8'd154: lift_lut = 5'd25;
            8'd155: lift_lut = 5'd25;
            8'd156: lift_lut = 5'd25;
            8'd157: lift_lut = 5'd24;
            8'd158: lift_lut = 5'd24;
            8'd159: lift_lut = 5'd24;
            8'd160: lift_lut = 5'd24;
            8'd161: lift_lut = 5'd23;
            8'd162: lift_lut = 5'd23;
            8'd163: lift_lut = 5'd23;
            8'd164: lift_lut = 5'd22;
            8'd165: lift_lut = 5'd22;
            8'd166: lift_lut = 5'd22;
            8'd167: lift_lut = 5'd21;
            8'd168: lift_lut = 5'd21;
            8'd169: lift_lut = 5'd21;
            8'd170: lift_lut = 5'd21;
            8'd171: lift_lut = 5'd20;
            8'd172: lift_lut = 5'd20;
            8'd173: lift_lut = 5'd20;
            8'd174: lift_lut = 5'd19;
            8'd175: lift_lut = 5'd19;
            8'd176: lift_lut = 5'd19;
            8'd177: lift_lut = 5'd19;
            8'd178: lift_lut = 5'd18;
            8'd179: lift_lut = 5'd18;
            8'd180: lift_lut = 5'd18;
            8'd181: lift_lut = 5'd17;
            8'd182: lift_lut = 5'd17;
            8'd183: lift_lut = 5'd17;
            8'd184: lift_lut = 5'd17;
            8'd185: lift_lut = 5'd16;
            8'd186: lift_lut = 5'd16;
            8'd187: lift_lut = 5'd16;
            8'd188: lift_lut = 5'd15;
            8'd189: lift_lut = 5'd15;
            8'd190: lift_lut = 5'd15;
            8'd191: lift_lut = 5'd15;
            8'd192: lift_lut = 5'd14;
            8'd193: lift_lut = 5'd14;
            8'd194: lift_lut = 5'd14;
            8'd195: lift_lut = 5'd14;
            8'd196: lift_lut = 5'd13;
            8'd197: lift_lut = 5'd13;
            8'd198: lift_lut = 5'd13;
            8'd199: lift_lut = 5'd12;
            8'd200: lift_lut = 5'd12;
            8'd201: lift_lut = 5'd12;
            8'd202: lift_lut = 5'd12;
            8'd203: lift_lut = 5'd11;
            8'd204: lift_lut = 5'd11;
            8'd205: lift_lut = 5'd11;
            8'd206: lift_lut = 5'd10;
            8'd207: lift_lut = 5'd10;
            8'd208: lift_lut = 5'd10;
            8'd209: lift_lut = 5'd9;
            8'd210: lift_lut = 5'd9;
            8'd211: lift_lut = 5'd9;
            8'd212: lift_lut = 5'd9;
            8'd213: lift_lut = 5'd8;
            8'd214: lift_lut = 5'd8;
            8'd215: lift_lut = 5'd8;
            8'd216: lift_lut = 5'd7;
            8'd217: lift_lut = 5'd7;
            8'd218: lift_lut = 5'd7;
            8'd219: lift_lut = 5'd7;
            8'd220: lift_lut = 5'd6;
            8'd221: lift_lut = 5'd6;
            8'd222: lift_lut = 5'd6;
            8'd223: lift_lut = 5'd5;
            8'd224: lift_lut = 5'd5;
            8'd225: lift_lut = 5'd5;
            8'd226: lift_lut = 5'd5;
            8'd227: lift_lut = 5'd4;
            8'd228: lift_lut = 5'd4;
            8'd229: lift_lut = 5'd4;
            8'd230: lift_lut = 5'd3;
            8'd231: lift_lut = 5'd3;
            8'd232: lift_lut = 5'd3;
            8'd233: lift_lut = 5'd3;
            8'd234: lift_lut = 5'd2;
            8'd235: lift_lut = 5'd2;
            8'd236: lift_lut = 5'd2;
            8'd237: lift_lut = 5'd1;
            8'd238: lift_lut = 5'd1;
            8'd239: lift_lut = 5'd1;
            8'd240: lift_lut = 5'd1;
            8'd241: lift_lut = 5'd0;
            8'd242: lift_lut = 5'd0;
            8'd243: lift_lut = 5'd0;
            8'd244: lift_lut = 5'd0;
            8'd245: lift_lut = 5'd0;
            8'd246: lift_lut = 5'd0;
            8'd247: lift_lut = 5'd0;
            8'd248: lift_lut = 5'd0;
            8'd249: lift_lut = 5'd0;
            8'd250: lift_lut = 5'd0;
            8'd251: lift_lut = 5'd0;
            8'd252: lift_lut = 5'd0;
            8'd253: lift_lut = 5'd0;
            8'd254: lift_lut = 5'd0;
            8'd255: lift_lut = 5'd0;
            default: lift_lut = 5'd0;
        endcase end
    endfunction
    // All comparisons have 13-bit unsigned width: chroma*32 and max*odd.
    function [2:0] saturation_level;
        input [7:0] chroma;
        input [7:0] maximum;
        reg [12:0] c32, m1, m3, m5, m7, m9, m11;
        begin
            c32 = {chroma,5'b0};
            m1 = {5'b0,maximum};
            m3 = (m1 << 1) + m1;
            m5 = (m1 << 2) + m1;
            m7 = (m1 << 3) - m1;
            m9 = (m1 << 3) + m1;
            m11 = (m1 << 3) + (m1 << 1) + m1;
            if(maximum == 0 || c32 < m1) saturation_level = 0;
            else if(c32 < m3) saturation_level = 1;
            else if(c32 < m5) saturation_level = 2;
            else if(c32 < m7) saturation_level = 3;
            else if(c32 < m9) saturation_level = 4;
            else if(c32 < m11) saturation_level = 5;
            else saturation_level = 6;
        end
    endfunction
    function signed [11:0] saturate_delta;
        input signed [9:0] delta;
        input [2:0] level;
        input warm;
        reg signed [16:0] extended, bonus, total;
        begin
            extended = {{7{delta[9]}},delta};
            case(level)
                0: bonus = 0;
                1: bonus = extended << 3;
                2: bonus = extended << 4;
                3: bonus = (extended << 4) + (extended << 3);
                default: bonus = extended << 5;
            endcase
            if(warm && level >= 3) bonus = extended << 4;
            total = (extended << 6) + bonus + 17'sd32;
            saturate_delta = total >>> 6;
        end
    endfunction
    // Coarse warm color region, not a semantic face/skin detector. Bounds keep
    // saturated red, orange and yellow out of the protected region.
    function warm_protect;
        input [7:0] r, g, b;
        reg [8:0] rg, gb, rb;
        begin
            rg = {1'b0,r} - {1'b0,g};
            gb = {1'b0,g} - {1'b0,b};
            rb = {1'b0,r} - {1'b0,b};
            warm_protect = r > g && g > b && r >= 64 && g >= 40 && b >= 24
                           && rg >= 8 && rg <= 80 && gb >= 4 && gb <= 64 && rb <= 112;
        end
    endfunction
    function [4:0] colorful_lift;
        input [4:0] base;
        input [2:0] level;
        reg [6:0] expanded;
        begin
            expanded = {2'b0,base};
            case(level)
                3: colorful_lift = (expanded + 7'd2) >> 2;
                4: colorful_lift = (expanded + 7'd1) >> 1;
                5: colorful_lift = ((expanded << 1) + expanded + 7'd2) >> 2;
                6: colorful_lift = base;
                default: colorful_lift = 0;
            endcase
        end
    endfunction
    function [5:0] white_lut;
        input [7:0] maximum;
        input [2:0] level;
        begin
            if(level >= 4 || maximum <= 204) white_lut = 0;
            else if(maximum >= 225) begin
                case(level)
                    2: white_lut = 10;
                    3: white_lut = 3;
                    default: white_lut = 16;
                endcase
            end else begin case(maximum)
            8'd205: begin case(level) 3'd2: white_lut = 6'd0; 3'd3: white_lut = 6'd0; default: white_lut = 6'd1; endcase end
            8'd206: begin case(level) 3'd2: white_lut = 6'd1; 3'd3: white_lut = 6'd0; default: white_lut = 6'd2; endcase end
            8'd207: begin case(level) 3'd2: white_lut = 6'd1; 3'd3: white_lut = 6'd1; default: white_lut = 6'd2; endcase end
            8'd208: begin case(level) 3'd2: white_lut = 6'd2; 3'd3: white_lut = 6'd1; default: white_lut = 6'd3; endcase end
            8'd209: begin case(level) 3'd2: white_lut = 6'd2; 3'd3: white_lut = 6'd1; default: white_lut = 6'd4; endcase end
            8'd210: begin case(level) 3'd2: white_lut = 6'd3; 3'd3: white_lut = 6'd1; default: white_lut = 6'd5; endcase end
            8'd211: begin case(level) 3'd2: white_lut = 6'd3; 3'd3: white_lut = 6'd1; default: white_lut = 6'd5; endcase end
            8'd212: begin case(level) 3'd2: white_lut = 6'd4; 3'd3: white_lut = 6'd1; default: white_lut = 6'd6; endcase end
            8'd213: begin case(level) 3'd2: white_lut = 6'd4; 3'd3: white_lut = 6'd2; default: white_lut = 6'd7; endcase end
            8'd214: begin case(level) 3'd2: white_lut = 6'd5; 3'd3: white_lut = 6'd2; default: white_lut = 6'd8; endcase end
            8'd215: begin case(level) 3'd2: white_lut = 6'd5; 3'd3: white_lut = 6'd2; default: white_lut = 6'd9; endcase end
            8'd216: begin case(level) 3'd2: white_lut = 6'd6; 3'd3: white_lut = 6'd2; default: white_lut = 6'd9; endcase end
            8'd217: begin case(level) 3'd2: white_lut = 6'd6; 3'd3: white_lut = 6'd2; default: white_lut = 6'd10; endcase end
            8'd218: begin case(level) 3'd2: white_lut = 6'd7; 3'd3: white_lut = 6'd2; default: white_lut = 6'd11; endcase end
            8'd219: begin case(level) 3'd2: white_lut = 6'd7; 3'd3: white_lut = 6'd3; default: white_lut = 6'd12; endcase end
            8'd220: begin case(level) 3'd2: white_lut = 6'd8; 3'd3: white_lut = 6'd3; default: white_lut = 6'd13; endcase end
            8'd221: begin case(level) 3'd2: white_lut = 6'd8; 3'd3: white_lut = 6'd3; default: white_lut = 6'd13; endcase end
            8'd222: begin case(level) 3'd2: white_lut = 6'd9; 3'd3: white_lut = 6'd3; default: white_lut = 6'd14; endcase end
            8'd223: begin case(level) 3'd2: white_lut = 6'd9; 3'd3: white_lut = 6'd3; default: white_lut = 6'd15; endcase end
            8'd224: begin case(level) 3'd2: white_lut = 6'd10; 3'd3: white_lut = 6'd3; default: white_lut = 6'd16; endcase end
                default: white_lut = 0;
            endcase end
        end
    endfunction
    function [7:0] clamp_byte;
        input signed [12:0] value;
        begin
            if(value < 0) clamp_byte = 0;
            else if(value > 255) clamp_byte = 255;
            else clamp_byte = value[7:0];
        end
    endfunction
    function [7:0] color_sum;
        input signed [12:0] common;
        input signed [11:0] delta;
        input signed [9:0] detail;
        reg [7:0] pre_detail;
        reg signed [12:0] sum;
        begin
            sum = common + $signed({delta[11],delta});
            pre_detail = clamp_byte(sum);
            sum = $signed({5'b0,pre_detail}) + $signed({{3{detail[9]}},detail});
            color_sum = clamp_byte(sum);
        end
    endfunction
    function [7:0] palette_blend;
        input [7:0] value;
        begin case(value)
            8'd2: palette_blend = 8'd1;
            8'd6: palette_blend = 8'd5;
            8'd10: palette_blend = 8'd9;
            8'd14: palette_blend = 8'd13;
            8'd18: palette_blend = 8'd17;
            8'd22: palette_blend = 8'd21;
            8'd26: palette_blend = 8'd25;
            8'd30: palette_blend = 8'd29;
            8'd34: palette_blend = 8'd33;
            8'd35: palette_blend = 8'd36;
            8'd38: palette_blend = 8'd37;
            8'd39: palette_blend = 8'd40;
            8'd42: palette_blend = 8'd41;
            8'd43: palette_blend = 8'd44;
            8'd46: palette_blend = 8'd45;
            8'd47: palette_blend = 8'd48;
            8'd51: palette_blend = 8'd52;
            8'd55: palette_blend = 8'd56;
            8'd59: palette_blend = 8'd60;
            8'd63: palette_blend = 8'd64;
            8'd67: palette_blend = 8'd68;
            8'd71: palette_blend = 8'd72;
            8'd75: palette_blend = 8'd76;
            8'd79: palette_blend = 8'd80;
            8'd83: palette_blend = 8'd84;
            8'd87: palette_blend = 8'd86;
            8'd91: palette_blend = 8'd90;
            8'd95: palette_blend = 8'd94;
            8'd99: palette_blend = 8'd98;
            8'd103: palette_blend = 8'd102;
            8'd107: palette_blend = 8'd106;
            8'd111: palette_blend = 8'd110;
            8'd115: palette_blend = 8'd114;
            8'd119: palette_blend = 8'd118;
            8'd120: palette_blend = 8'd121;
            8'd123: palette_blend = 8'd122;
            8'd124: palette_blend = 8'd125;
            8'd127: palette_blend = 8'd126;
            8'd128: palette_blend = 8'd129;
            8'd131: palette_blend = 8'd130;
            8'd132: palette_blend = 8'd133;
            8'd136: palette_blend = 8'd137;
            8'd140: palette_blend = 8'd141;
            8'd144: palette_blend = 8'd145;
            8'd148: palette_blend = 8'd149;
            8'd152: palette_blend = 8'd153;
            8'd156: palette_blend = 8'd157;
            8'd160: palette_blend = 8'd161;
            8'd164: palette_blend = 8'd165;
            8'd168: palette_blend = 8'd169;
            8'd172: palette_blend = 8'd171;
            8'd176: palette_blend = 8'd175;
            8'd180: palette_blend = 8'd179;
            8'd184: palette_blend = 8'd183;
            8'd188: palette_blend = 8'd187;
            8'd192: palette_blend = 8'd191;
            8'd196: palette_blend = 8'd195;
            8'd200: palette_blend = 8'd199;
            8'd204: palette_blend = 8'd203;
            8'd205: palette_blend = 8'd206;
            8'd208: palette_blend = 8'd207;
            8'd209: palette_blend = 8'd210;
            8'd212: palette_blend = 8'd211;
            8'd213: palette_blend = 8'd214;
            8'd216: palette_blend = 8'd215;
            8'd217: palette_blend = 8'd218;
            8'd221: palette_blend = 8'd222;
            8'd225: palette_blend = 8'd226;
            8'd229: palette_blend = 8'd230;
            8'd233: palette_blend = 8'd234;
            8'd237: palette_blend = 8'd238;
            8'd241: palette_blend = 8'd242;
            8'd245: palette_blend = 8'd246;
            8'd249: palette_blend = 8'd250;
            8'd253: palette_blend = 8'd254;
            default: palette_blend = value;
        endcase end
    endfunction
    function [13:0] white_product;
        input [7:0] delta;
        input [5:0] factor;
        reg [13:0] d, a, b, c;
        begin
            d = {6'b0,delta};
            a = (factor[0] ? d : 14'd0) + (factor[1] ? (d << 1) : 14'd0);
            b = (factor[2] ? (d << 2) : 14'd0) + (factor[3] ? (d << 3) : 14'd0);
            c = (factor[4] ? (d << 4) : 14'd0) + (factor[5] ? (d << 5) : 14'd0);
            white_product = a + b + c;
        end
    endfunction
    function [15:0] edge_product;
        input [7:0] value;
        input [7:0] strength;
        reg [15:0] v, a, b, c, d;
        begin
            v = {8'b0,value};
            a = (strength[0] ? v : 16'd0) + (strength[1] ? (v << 1) : 16'd0);
            b = (strength[2] ? (v << 2) : 16'd0) + (strength[3] ? (v << 3) : 16'd0);
            c = (strength[4] ? (v << 4) : 16'd0) + (strength[5] ? (v << 5) : 16'd0);
            d = (strength[6] ? (v << 6) : 16'd0) + (strength[7] ? (v << 7) : 16'd0);
            edge_product = (a + b) + (c + d);
        end
    endfunction

    wire [7:0] red = I_rgb[23:16], green = I_rgb[15:8], blue = I_rgb[7:0];
    wire [7:0] maximum = red > green ? (red > blue ? red : blue) : (green > blue ? green : blue);
    wire [7:0] minimum = red < green ? (red < blue ? red : blue) : (green < blue ? green : blue);
    wire [15:0] r16 = {8'b0,red}, g16 = {8'b0,green}, b16 = {8'b0,blue};
    wire [15:0] y_sum = (r16 << 6) + (r16 << 3) + (r16 << 2) + r16
                          + (g16 << 7) + (g16 << 4) + (g16 << 2) + (g16 << 1)
                          + (b16 << 5) - (b16 << 1) - b16 + 16'd128;
    wire signed [8:0] bounded_detail = I_detail > 9'sd15 ? 9'sd15 :
                                      I_detail < -9'sd15 ? -9'sd15 : I_detail;
    // Production flat rendition: quantize luminance only. Applying one signed
    // offset to R/G/B preserves their differences (except unavoidable clipping)
    // and avoids the coloured patches caused by independent channel palettes.
    // Bounded neutral lift0..7 retains both black/white, without a full tone,
    // saturation or highlight reconstruction. The legacy style remains below.
    function [7:0] flat_palette;
        input [7:0] value;
        reg [3:0] hi,lo,q;
        begin
            hi=value[7:4];lo=value[3:0];q=hi;
            // Exact nearest multiple of17; no multiplier, divider or ROM.
            if(hi>=9 && {1'b0,lo}<({1'b0,hi}-5'd8)) q=hi-1'b1;
            else if(hi<=6 && {1'b0,lo}>=({1'b0,hi}+5'd9)) q=hi+1'b1;
            flat_palette={q,q};
        end
    endfunction
    function [7:0] flat_shift;
        input [7:0] channel;
        input signed [8:0] offset;
        reg signed [9:0] shifted;
        begin
            shifted=$signed({2'b00,channel})+offset;
            flat_shift=shifted<0 ? 8'd0:(shifted>255 ? 8'd255:shifted[7:0]);
        end
    endfunction
    generate if(FLAT_STYLE) begin: flat_colour
        reg [23:0] colour[0:7];
        reg [7:0] luma0;
        reg signed [8:0] offset1;
        integer k;
        wire [7:0] rg_max=colour[0][23:16]>=colour[0][15:8] ?
            colour[0][23:16]:colour[0][15:8];
        wire [7:0] max_channel=rg_max>=colour[0][7:0] ? rg_max:colour[0][7:0];
        wire [3:0] neutral_lift=max_channel[7] ? ~max_channel[7:4]:max_channel[7:4];
        wire [7:0] lifted_luma=luma0+neutral_lift;
        wire [7:0] quantized_luma=flat_palette(lifted_luma);
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                for(k=0;k<8;k=k+1) colour[k]<=0;
                luma0<=0;offset1<=0;O_rgb<=0;
            end else begin
                colour[0]<=I_rgb;luma0<=y_sum[15:8];
                colour[1]<=colour[0];
                offset1<=$signed({1'b0,quantized_luma})-$signed({1'b0,luma0});
                colour[2]<={flat_shift(colour[1][23:16],offset1),
                             flat_shift(colour[1][15:8],offset1),flat_shift(colour[1][7:0],offset1)};
                for(k=3;k<8;k=k+1) colour[k]<=colour[k-1];
                O_rgb<=colour[7];
            end
        end
    end else begin: legacy_colour
    reg [23:0] rgb0, rgb1;
    reg [7:0] y0, y1, max0, max1, tone1, tone2;
    reg [7:0] chroma0;
    reg [2:0] level1;
    reg warm0, warm1;
    reg signed [8:0] detail0, detail1;
    reg signed [9:0] detail2;
    reg signed [11:0] dr2, dg2, db2;
    reg [4:0] lift2;
    reg [5:0] white2;
    reg [23:0] rgb3, rgb4, rgb5, rgb6, rgb7;
    reg [5:0] white3;
    reg [13:0] wr4, wg4, wb4;
    reg [15:0] er7, eg7, eb7;
    reg [7:0] edge0, edge1, edge2, edge3, edge4, edge5, edge6;
    wire signed [12:0] common2 = $signed({5'b0,tone2}) + $signed({8'b0,lift2});
    wire [8:0] wrr = ({1'b0,wr4} + 15'd16) >> 5;
    wire [8:0] wgr = ({1'b0,wg4} + 15'd16) >> 5;
    wire [8:0] wbr = ({1'b0,wb4} + 15'd16) >> 5;
    wire [8:0] err = ({1'b0,er7} + 17'd128) >> 8;
    wire [8:0] egr = ({1'b0,eg7} + 17'd128) >> 8;
    wire [8:0] ebr = ({1'b0,eb7} + 17'd128) >> 8;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            rgb0<=0; rgb1<=0; y0<=0; y1<=0; max0<=0; max1<=0; tone1<=0; tone2<=0;
            chroma0<=0; level1<=0; warm0<=0; warm1<=0; detail0<=0; detail1<=0; detail2<=0;
            dr2<=0; dg2<=0; db2<=0; lift2<=0; white2<=0; white3<=0;
            rgb3<=0; rgb4<=0; rgb5<=0; rgb6<=0; rgb7<=0;
            wr4<=0; wg4<=0; wb4<=0; er7<=0; eg7<=0; eb7<=0;
            edge0<=0; edge1<=0; edge2<=0; edge3<=0; edge4<=0; edge5<=0; edge6<=0;
            O_rgb<=0;
        end else begin
            // 0: RGB, Rec.601 approximation, shared incoming detail/edge.
            rgb0 <= I_rgb; y0 <= y_sum[15:8]; max0 <= maximum; chroma0 <= maximum-minimum;
            warm0 <= warm_protect(red,green,blue);
            detail0 <= bounded_detail;
            edge0 <= I_edge_strength > 77 ? 8'd77 : I_edge_strength;
            // 1: tone and saturation index.
            rgb1 <= rgb0; y1 <= y0; max1 <= max0; tone1 <= tone_lut(y0);
            level1 <= saturation_level(chroma0,max0);
            warm1 <= warm0;
            detail1 <= detail0; edge1 <= edge0;
            // 2: component chroma scaling, detail and highlight coefficients.
            dr2 <= saturate_delta($signed({1'b0,rgb1[23:16]})-$signed({1'b0,y1}),level1,warm1);
            dg2 <= saturate_delta($signed({1'b0,rgb1[15:8]})-$signed({1'b0,y1}),level1,warm1);
            db2 <= saturate_delta($signed({1'b0,rgb1[7:0]})-$signed({1'b0,y1}),level1,warm1);
            tone2 <= tone1;
            // Flat cartoon areas use the edge-preserving low-pass directly;
            // adding high-frequency detail back would undo the color blocks.
            detail2 <= FLAT_STYLE ? 10'sd0 :
                $signed(detail1) + (($signed(detail1) + 10'sd8) >>> 4);
            lift2 <= colorful_lift(lift_lut(max1),level1);
            white2 <= white_lut(max1,level1); edge2 <= edge1;
            // 3: clamp style, bounded detail addition, clamp before whitening.
            rgb3 <= {color_sum(common2,dr2,detail2),color_sum(common2,dg2,detail2),color_sum(common2,db2,detail2)};
            white3 <= white2; edge3 <= edge2;
            // 4: Q5 products for neutral highlight.
            wr4 <= white_product(8'd255-rgb3[23:16],white3);
            wg4 <= white_product(8'd255-rgb3[15:8],white3);
            wb4 <= white_product(8'd255-rgb3[7:0],white3);
            rgb4 <= rgb3; edge4 <= edge3;
            // 5: white blend has mathematically bounded output <=255.
            rgb5 <= {clamp_byte($signed({5'b0,rgb4[23:16]})+$signed({4'b0,wrr})),
                     clamp_byte($signed({5'b0,rgb4[15:8]})+$signed({4'b0,wgr})),
                     clamp_byte($signed({5'b0,rgb4[7:0]})+$signed({4'b0,wbr}))};
            edge5 <= edge4;
            // 6: sixteen evenly spaced levels/channel, endpoints 0 and255.
            // Repeating the upper nibble is a wiring-only quantizer.
            rgb6 <= FLAT_STYLE ?
                {rgb5[23:20],rgb5[23:20],rgb5[15:12],rgb5[15:12],rgb5[7:4],rgb5[7:4]} :
                {palette_blend(rgb5[23:16]),palette_blend(rgb5[15:8]),palette_blend(rgb5[7:0])};
            edge6 <= edge5;
            // 7: contour opacity products, separate from color mapping.
            er7 <= edge_product(rgb6[23:16],edge6);
            eg7 <= edge_product(rgb6[15:8],edge6);
            eb7 <= edge_product(rgb6[7:0],edge6); rgb7 <= rgb6;
            // 8: saturated subtract, rounded opacity /256.
            O_rgb <= {clamp_byte($signed({5'b0,rgb7[23:16]})-$signed({4'b0,err})),
                      clamp_byte($signed({5'b0,rgb7[15:8]})-$signed({4'b0,egr})),
                      clamp_byte($signed({5'b0,rgb7[7:0]})-$signed({4'b0,ebr}))};
        end
    end
    end endgenerate
endmodule
`endif

