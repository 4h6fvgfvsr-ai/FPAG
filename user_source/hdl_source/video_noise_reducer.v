`ifndef VIDEO_NOISE_REDUCER_V
`define VIDEO_NOISE_REDUCER_V

// Mild, edge-preserving spatial noise reduction before image algorithms/OSD.
// The current pixel keeps 5/8 of the weight; each similar raw neighbor adds
// 1/8. Neighbors across an edge are replaced by the current pixel, so no
// spatial offset or frame history is required. All video signals use the
// same three-register pipeline, including RGB during inactive video.
module video_noise_reducer #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [7:0] THRESHOLD = 8'd24,
    parameter integer ENABLE = 1,
    parameter integer STRONG_FILTER = 0,
    parameter integer TONE_ENABLE = 0
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_video_vsync,
    input wire I_video_hsync,
    input wire I_video_de,
    input wire I_video_user,
    input wire I_video_last,
    input wire [23:0] I_rgb,
    output reg O_video_vsync,
    output reg O_video_hsync,
    output reg O_video_de,
    output reg O_video_user,
    output reg O_video_last,
    output reg [23:0] O_rgb
);
    reg [11:0] S_x, S_y;
    wire [11:0] S_pixel_x = I_video_user ? 12'd0 : S_x;
    wire [11:0] S_pixel_y = I_video_user ? 12'd0 : S_y;
    wire S_sample = I_video_de && (S_pixel_x < IMG_WIDTH) &&
                                      (S_pixel_y < IMG_HEIGHT);

    // Read-before-write gives the raw pixel at (x,y-1). Keeping this memory
    // synchronous allows embedded RAM inference. It is intentionally not
    // reset: first-row validity masks all data from before the current SOF.
    reg [23:0] S_line [0:IMG_WIDTH-1];
    reg [23:0] S_above_1;
    reg [23:0] S_raw_left;
    reg [23:0] S_center_1, S_left_1, S_diagonal_1;
    reg S_sample_1, S_above_valid_1, S_diagonal_valid_1;
    reg [4:0] S_control_1, S_control_2;
    reg [23:0] S_center_2, S_left_2, S_above_2, S_diagonal_2;
    reg S_sample_2;
    reg [23:0] S_filtered_3;
    reg [4:0] S_control_3;
    reg S_sample_3;

    function F_similar;
        input [23:0] center;
        input [23:0] neighbor;
        reg [7:0] dr, dg, db;
        begin
            dr = center[23:16] >= neighbor[23:16]
               ? center[23:16] - neighbor[23:16]
               : neighbor[23:16] - center[23:16];
            dg = center[15:8] >= neighbor[15:8]
               ? center[15:8] - neighbor[15:8]
               : neighbor[15:8] - center[15:8];
            db = center[7:0] >= neighbor[7:0]
               ? center[7:0] - neighbor[7:0]
               : neighbor[7:0] - center[7:0];
            // One decision for all RGB channels avoids mixing hues at edges.
            F_similar = (dr <= THRESHOLD) && (dg <= THRESHOLD) &&
                        (db <= THRESHOLD);
        end
    endfunction

    function [7:0] F_average;
        input [7:0] center, left, above, diagonal;
        reg [10:0] sum;
        begin
            // Maximum including rounding is 8*255+4 = 2044 (11 bits).
            if(STRONG_FILTER) begin
                sum = {3'b000,center}+{3'b000,left}+
                      {3'b000,above}+{3'b000,diagonal}+11'd2;
                F_average = sum[9:2];
            end else begin
                sum = {1'b0, center, 2'b00} + {3'b000, center} +
                      {3'b000, left} + {3'b000, above} +
                      {3'b000, diagonal} + 11'd4;
                F_average = sum[10:3];
            end
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end else if(I_video_user) begin
            S_x <= I_video_de && !I_video_last ? 12'd1 : 12'd0;
            S_y <= I_video_de && I_video_last ? 12'd1 : 12'd0;
        end else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk) begin
        if(S_sample) begin
            S_above_1 <= S_line[S_pixel_x];
            S_line[S_pixel_x] <= I_rgb;
        end
    end

    // Stage 1: capture the center and left neighbors alongside the RAM read.
    // The prior registered read is (x-1,y-1); no read occurs on DE pauses.
    // At x=0/y=0 the corresponding absent neighbors become the center.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_raw_left <= 24'd0;
            S_center_1 <= 24'd0;
            S_left_1 <= 24'd0;
            S_diagonal_1 <= 24'd0;
            S_sample_1 <= 1'b0;
            S_above_valid_1 <= 1'b0;
            S_diagonal_valid_1 <= 1'b0;
            S_control_1 <= 5'd0;
        end else begin
            S_control_1 <= {I_video_vsync, I_video_hsync, I_video_de,
                            I_video_user, I_video_last};
            S_center_1 <= I_rgb;
            S_sample_1 <= S_sample;
            S_above_valid_1 <= S_sample && (S_pixel_y != 0);
            S_diagonal_valid_1 <= S_sample && (S_pixel_x != 0) &&
                                                     (S_pixel_y != 0);
            if(S_sample) begin
                S_raw_left <= I_rgb;
                S_left_1 <= S_pixel_x == 0 ? I_rgb : S_raw_left;
                S_diagonal_1 <= S_pixel_x == 0 || S_pixel_y == 0
                             ? I_rgb : S_above_1;
            end else begin
                S_left_1 <= I_rgb;
                S_diagonal_1 <= I_rgb;
            end
        end
    end

    // Stage 2: reject edge-crossing neighbors before the short adder path.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_center_2 <= 24'd0;
            S_left_2 <= 24'd0;
            S_above_2 <= 24'd0;
            S_diagonal_2 <= 24'd0;
            S_sample_2 <= 1'b0;
            S_control_2 <= 5'd0;
        end else begin
            S_control_2 <= S_control_1;
            S_center_2 <= S_center_1;
            S_sample_2 <= S_sample_1;
            S_left_2 <= ENABLE && S_sample_1 &&
                        F_similar(S_center_1, S_left_1)
                      ? S_left_1 : S_center_1;
            S_above_2 <= ENABLE && S_above_valid_1 &&
                         F_similar(S_center_1, S_above_1)
                       ? S_above_1 : S_center_1;
            S_diagonal_2 <= ENABLE && S_diagonal_valid_1 &&
                            F_similar(S_center_1, S_diagonal_1)
                          ? S_diagonal_1 : S_center_1;
        end
    end

    // Stage 3: rounded RGB output; blanking and disabled processing pass the
    // exact RGB through with the same latency as every video control signal.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_control_3 <= 5'd0;
            S_filtered_3 <= 24'd0;
            S_sample_3 <= 1'b0;
        end else begin
            S_control_3 <= S_control_2;
            S_sample_3 <= S_sample_2;
            S_filtered_3 <= ENABLE && S_sample_2 ? {
                F_average(S_center_2[23:16], S_left_2[23:16],
                          S_above_2[23:16], S_diagonal_2[23:16]),
                F_average(S_center_2[15:8], S_left_2[15:8],
                          S_above_2[15:8], S_diagonal_2[15:8]),
                F_average(S_center_2[7:0], S_left_2[7:0],
                          S_above_2[7:0], S_diagonal_2[7:0])}
                : S_center_2;
        end
    end

    // Common RGB gain derived from the largest channel: approximate gamma
    // 0.65 with a maximum 2x shadow lift. Equal scaling preserves chromatic
    // ratios; highlights and neutral gray remain neutral. The coefficient
    // table is generated from the documented curve, never from live frames.
    // Tone processing adds two clocks to ALL tags and RGB, without changing
    // the DDR read enable, active pixels or frame cadence.
    function [9:0] F_tone_gain;
        input [7:0] value;
        begin
            case(value)
                                8'd0: F_tone_gain=10'd512;
                8'd1: F_tone_gain=10'd512;
                8'd2: F_tone_gain=10'd512;
                8'd3: F_tone_gain=10'd512;
                8'd4: F_tone_gain=10'd512;
                8'd5: F_tone_gain=10'd512;
                8'd6: F_tone_gain=10'd512;
                8'd7: F_tone_gain=10'd512;
                8'd8: F_tone_gain=10'd512;
                8'd9: F_tone_gain=10'd512;
                8'd10: F_tone_gain=10'd512;
                8'd11: F_tone_gain=10'd512;
                8'd12: F_tone_gain=10'd512;
                8'd13: F_tone_gain=10'd512;
                8'd14: F_tone_gain=10'd512;
                8'd15: F_tone_gain=10'd512;
                8'd16: F_tone_gain=10'd512;
                8'd17: F_tone_gain=10'd512;
                8'd18: F_tone_gain=10'd512;
                8'd19: F_tone_gain=10'd512;
                8'd20: F_tone_gain=10'd512;
                8'd21: F_tone_gain=10'd512;
                8'd22: F_tone_gain=10'd512;
                8'd23: F_tone_gain=10'd512;
                8'd24: F_tone_gain=10'd512;
                8'd25: F_tone_gain=10'd512;
                8'd26: F_tone_gain=10'd512;
                8'd27: F_tone_gain=10'd512;
                8'd28: F_tone_gain=10'd512;
                8'd29: F_tone_gain=10'd512;
                8'd30: F_tone_gain=10'd512;
                8'd31: F_tone_gain=10'd512;
                8'd32: F_tone_gain=10'd512;
                8'd33: F_tone_gain=10'd512;
                8'd34: F_tone_gain=10'd512;
                8'd35: F_tone_gain=10'd512;
                8'd36: F_tone_gain=10'd508;
                8'd37: F_tone_gain=10'd503;
                8'd38: F_tone_gain=10'd498;
                8'd39: F_tone_gain=10'd494;
                8'd40: F_tone_gain=10'd490;
                8'd41: F_tone_gain=10'd485;
                8'd42: F_tone_gain=10'd481;
                8'd43: F_tone_gain=10'd477;
                8'd44: F_tone_gain=10'd474;
                8'd45: F_tone_gain=10'd470;
                8'd46: F_tone_gain=10'd466;
                8'd47: F_tone_gain=10'd463;
                8'd48: F_tone_gain=10'd459;
                8'd49: F_tone_gain=10'd456;
                8'd50: F_tone_gain=10'd453;
                8'd51: F_tone_gain=10'd450;
                8'd52: F_tone_gain=10'd447;
                8'd53: F_tone_gain=10'd444;
                8'd54: F_tone_gain=10'd441;
                8'd55: F_tone_gain=10'd438;
                8'd56: F_tone_gain=10'd435;
                8'd57: F_tone_gain=10'd432;
                8'd58: F_tone_gain=10'd430;
                8'd59: F_tone_gain=10'd427;
                8'd60: F_tone_gain=10'd425;
                8'd61: F_tone_gain=10'd422;
                8'd62: F_tone_gain=10'd420;
                8'd63: F_tone_gain=10'd418;
                8'd64: F_tone_gain=10'd415;
                8'd65: F_tone_gain=10'd413;
                8'd66: F_tone_gain=10'd411;
                8'd67: F_tone_gain=10'd409;
                8'd68: F_tone_gain=10'd407;
                8'd69: F_tone_gain=10'd405;
                8'd70: F_tone_gain=10'd402;
                8'd71: F_tone_gain=10'd400;
                8'd72: F_tone_gain=10'd399;
                8'd73: F_tone_gain=10'd397;
                8'd74: F_tone_gain=10'd395;
                8'd75: F_tone_gain=10'd393;
                8'd76: F_tone_gain=10'd391;
                8'd77: F_tone_gain=10'd389;
                8'd78: F_tone_gain=10'd388;
                8'd79: F_tone_gain=10'd386;
                8'd80: F_tone_gain=10'd384;
                8'd81: F_tone_gain=10'd382;
                8'd82: F_tone_gain=10'd381;
                8'd83: F_tone_gain=10'd379;
                8'd84: F_tone_gain=10'd378;
                8'd85: F_tone_gain=10'd376;
                8'd86: F_tone_gain=10'd375;
                8'd87: F_tone_gain=10'd373;
                8'd88: F_tone_gain=10'd372;
                8'd89: F_tone_gain=10'd370;
                8'd90: F_tone_gain=10'd369;
                8'd91: F_tone_gain=10'd367;
                8'd92: F_tone_gain=10'd366;
                8'd93: F_tone_gain=10'd364;
                8'd94: F_tone_gain=10'd363;
                8'd95: F_tone_gain=10'd362;
                8'd96: F_tone_gain=10'd360;
                8'd97: F_tone_gain=10'd359;
                8'd98: F_tone_gain=10'd358;
                8'd99: F_tone_gain=10'd356;
                8'd100: F_tone_gain=10'd355;
                8'd101: F_tone_gain=10'd354;
                8'd102: F_tone_gain=10'd353;
                8'd103: F_tone_gain=10'd352;
                8'd104: F_tone_gain=10'd350;
                8'd105: F_tone_gain=10'd349;
                8'd106: F_tone_gain=10'd348;
                8'd107: F_tone_gain=10'd347;
                8'd108: F_tone_gain=10'd346;
                8'd109: F_tone_gain=10'd345;
                8'd110: F_tone_gain=10'd344;
                8'd111: F_tone_gain=10'd343;
                8'd112: F_tone_gain=10'd341;
                8'd113: F_tone_gain=10'd340;
                8'd114: F_tone_gain=10'd339;
                8'd115: F_tone_gain=10'd338;
                8'd116: F_tone_gain=10'd337;
                8'd117: F_tone_gain=10'd336;
                8'd118: F_tone_gain=10'd335;
                8'd119: F_tone_gain=10'd334;
                8'd120: F_tone_gain=10'd333;
                8'd121: F_tone_gain=10'd332;
                8'd122: F_tone_gain=10'd331;
                8'd123: F_tone_gain=10'd330;
                8'd124: F_tone_gain=10'd329;
                8'd125: F_tone_gain=10'd329;
                8'd126: F_tone_gain=10'd328;
                8'd127: F_tone_gain=10'd327;
                8'd128: F_tone_gain=10'd326;
                8'd129: F_tone_gain=10'd325;
                8'd130: F_tone_gain=10'd324;
                8'd131: F_tone_gain=10'd323;
                8'd132: F_tone_gain=10'd322;
                8'd133: F_tone_gain=10'd321;
                8'd134: F_tone_gain=10'd321;
                8'd135: F_tone_gain=10'd320;
                8'd136: F_tone_gain=10'd319;
                8'd137: F_tone_gain=10'd318;
                8'd138: F_tone_gain=10'd317;
                8'd139: F_tone_gain=10'd317;
                8'd140: F_tone_gain=10'd316;
                8'd141: F_tone_gain=10'd315;
                8'd142: F_tone_gain=10'd314;
                8'd143: F_tone_gain=10'd313;
                8'd144: F_tone_gain=10'd313;
                8'd145: F_tone_gain=10'd312;
                8'd146: F_tone_gain=10'd311;
                8'd147: F_tone_gain=10'd310;
                8'd148: F_tone_gain=10'd310;
                8'd149: F_tone_gain=10'd309;
                8'd150: F_tone_gain=10'd308;
                8'd151: F_tone_gain=10'd308;
                8'd152: F_tone_gain=10'd307;
                8'd153: F_tone_gain=10'd306;
                8'd154: F_tone_gain=10'd305;
                8'd155: F_tone_gain=10'd305;
                8'd156: F_tone_gain=10'd304;
                8'd157: F_tone_gain=10'd303;
                8'd158: F_tone_gain=10'd303;
                8'd159: F_tone_gain=10'd302;
                8'd160: F_tone_gain=10'd301;
                8'd161: F_tone_gain=10'd301;
                8'd162: F_tone_gain=10'd300;
                8'd163: F_tone_gain=10'd299;
                8'd164: F_tone_gain=10'd299;
                8'd165: F_tone_gain=10'd298;
                8'd166: F_tone_gain=10'd298;
                8'd167: F_tone_gain=10'd297;
                8'd168: F_tone_gain=10'd296;
                8'd169: F_tone_gain=10'd296;
                8'd170: F_tone_gain=10'd295;
                8'd171: F_tone_gain=10'd294;
                8'd172: F_tone_gain=10'd294;
                8'd173: F_tone_gain=10'd293;
                8'd174: F_tone_gain=10'd293;
                8'd175: F_tone_gain=10'd292;
                8'd176: F_tone_gain=10'd291;
                8'd177: F_tone_gain=10'd291;
                8'd178: F_tone_gain=10'd290;
                8'd179: F_tone_gain=10'd290;
                8'd180: F_tone_gain=10'd289;
                8'd181: F_tone_gain=10'd289;
                8'd182: F_tone_gain=10'd288;
                8'd183: F_tone_gain=10'd288;
                8'd184: F_tone_gain=10'd287;
                8'd185: F_tone_gain=10'd286;
                8'd186: F_tone_gain=10'd286;
                8'd187: F_tone_gain=10'd285;
                8'd188: F_tone_gain=10'd285;
                8'd189: F_tone_gain=10'd284;
                8'd190: F_tone_gain=10'd284;
                8'd191: F_tone_gain=10'd283;
                8'd192: F_tone_gain=10'd283;
                8'd193: F_tone_gain=10'd282;
                8'd194: F_tone_gain=10'd282;
                8'd195: F_tone_gain=10'd281;
                8'd196: F_tone_gain=10'd281;
                8'd197: F_tone_gain=10'd280;
                8'd198: F_tone_gain=10'd280;
                8'd199: F_tone_gain=10'd279;
                8'd200: F_tone_gain=10'd279;
                8'd201: F_tone_gain=10'd278;
                8'd202: F_tone_gain=10'd278;
                8'd203: F_tone_gain=10'd277;
                8'd204: F_tone_gain=10'd277;
                8'd205: F_tone_gain=10'd276;
                8'd206: F_tone_gain=10'd276;
                8'd207: F_tone_gain=10'd275;
                8'd208: F_tone_gain=10'd275;
                8'd209: F_tone_gain=10'd274;
                8'd210: F_tone_gain=10'd274;
                8'd211: F_tone_gain=10'd274;
                8'd212: F_tone_gain=10'd273;
                8'd213: F_tone_gain=10'd273;
                8'd214: F_tone_gain=10'd272;
                8'd215: F_tone_gain=10'd272;
                8'd216: F_tone_gain=10'd271;
                8'd217: F_tone_gain=10'd271;
                8'd218: F_tone_gain=10'd270;
                8'd219: F_tone_gain=10'd270;
                8'd220: F_tone_gain=10'd270;
                8'd221: F_tone_gain=10'd269;
                8'd222: F_tone_gain=10'd269;
                8'd223: F_tone_gain=10'd268;
                8'd224: F_tone_gain=10'd268;
                8'd225: F_tone_gain=10'd267;
                8'd226: F_tone_gain=10'd267;
                8'd227: F_tone_gain=10'd267;
                8'd228: F_tone_gain=10'd266;
                8'd229: F_tone_gain=10'd266;
                8'd230: F_tone_gain=10'd265;
                8'd231: F_tone_gain=10'd265;
                8'd232: F_tone_gain=10'd265;
                8'd233: F_tone_gain=10'd264;
                8'd234: F_tone_gain=10'd264;
                8'd235: F_tone_gain=10'd263;
                8'd236: F_tone_gain=10'd263;
                8'd237: F_tone_gain=10'd263;
                8'd238: F_tone_gain=10'd262;
                8'd239: F_tone_gain=10'd262;
                8'd240: F_tone_gain=10'd261;
                8'd241: F_tone_gain=10'd261;
                8'd242: F_tone_gain=10'd261;
                8'd243: F_tone_gain=10'd260;
                8'd244: F_tone_gain=10'd260;
                8'd245: F_tone_gain=10'd260;
                8'd246: F_tone_gain=10'd259;
                8'd247: F_tone_gain=10'd259;
                8'd248: F_tone_gain=10'd259;
                8'd249: F_tone_gain=10'd258;
                8'd250: F_tone_gain=10'd258;
                8'd251: F_tone_gain=10'd257;
                8'd252: F_tone_gain=10'd257;
                8'd253: F_tone_gain=10'd257;
                8'd254: F_tone_gain=10'd256;
                8'd255: F_tone_gain=10'd256;
                default: F_tone_gain=10'd256;
            endcase
        end
    endfunction
    function [7:0] F_clip_product;
        input [17:0] product;
        reg [18:0] rounded;
        begin
            rounded={1'b0,product}+19'd128;
            F_clip_product=rounded[18:8]>255 ? 8'd255 : rounded[15:8];
        end
    endfunction
    generate if(TONE_ENABLE) begin : tone_pipeline
        reg [23:0] rgb_4;
        reg [4:0] control_4;
        reg sample_4;
        reg [9:0] gain_4;
        wire [7:0] rg_max=S_filtered_3[23:16]>S_filtered_3[15:8]
                           ? S_filtered_3[23:16] : S_filtered_3[15:8];
        wire [7:0] max_channel=rg_max>S_filtered_3[7:0] ? rg_max : S_filtered_3[7:0];
        wire [17:0] product_r=rgb_4[23:16]*gain_4;
        wire [17:0] product_g=rgb_4[15:8]*gain_4;
        wire [17:0] product_b=rgb_4[7:0]*gain_4;
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin rgb_4<=0;control_4<=0;sample_4<=0;gain_4<=256;end
            else begin
                rgb_4<=S_filtered_3;control_4<=S_control_3;sample_4<=S_sample_3;
                gain_4<=F_tone_gain(max_channel);
            end
        end
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                {O_video_vsync,O_video_hsync,O_video_de,O_video_user,O_video_last}<=0;
                O_rgb<=0;
            end else begin
                {O_video_vsync,O_video_hsync,O_video_de,O_video_user,O_video_last}<=control_4;
                O_rgb<=ENABLE && sample_4 ? {
                    F_clip_product(product_r),F_clip_product(product_g),
                    F_clip_product(product_b)} : rgb_4;
            end
        end
    end else begin : legacy_pipeline
        always @(*) begin
            {O_video_vsync,O_video_hsync,O_video_de,O_video_user,O_video_last}=S_control_3;
            O_rgb=S_filtered_3;
        end
    end endgenerate
endmodule

`endif
