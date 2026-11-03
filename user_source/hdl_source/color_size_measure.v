`ifndef COLOR_SIZE_MEASURE_V
`define COLOR_SIZE_MEASURE_V
// Sparse inverse lens/plane mapping. No video pixel is resampled. The ROM is
// generated from the actual camera pictures; Q10.6 mm, signed. One seven-cycle
// shift/add interpolator is reused for both coordinates. No DSP is required.
module color_size_point_map(
    input wire I_clk,I_rst_n,I_start,I_abort,
    input wire [12:0] I_x_half,I_y_half,
    output reg O_done,
    output reg signed [15:0] O_x,O_y
);
    `include "color_size_map_rom.vh"
    reg [31:0] map_rom[0:511];
    integer r;
    initial begin
        for(r=0;r<512;r=r+1) map_rom[r]=F_calibration_vertex(r);
    end
    reg [8:0] address,base;
    reg [31:0] data,vertex[0:3];
    always @(posedge I_clk) data<=map_rom[address];
    reg [3:0] state;
    reg [1:0] corner,stage;
    reg axis;
    reg [6:0] fx,fy,coefficient;
    reg [2:0] count;
    reg signed [15:0] top,bottom,a;
    reg signed [23:0] accumulator,shifted_delta;
    reg signed [15:0] operand_a,operand_b;
    wire signed [16:0] delta=$signed({operand_b[15],operand_b})-$signed({operand_a[15],operand_a});
    wire signed [23:0] product_next=accumulator+(coefficient[0] ? shifted_delta : 24'sd0);
    wire signed [23:0] rounded=(product_next+24'sd64) >>> 7;
    wire signed [23:0] result_wide=$signed(a)+rounded;
    wire [15:0] row=I_y_half[12:7];
    wire [15:0] start_address=(row<<4)+(row<<2)+row+I_x_half[12:7];
    function signed [15:0] F_coordinate;
        input [31:0] word;
        input select_y;
        begin F_coordinate=select_y ? word[15:0] : word[31:16];end
    endfunction
    always @* begin
        case(stage)
            0:begin operand_a=F_coordinate(vertex[0],axis);operand_b=F_coordinate(vertex[1],axis);end
            1:begin operand_a=F_coordinate(vertex[2],axis);operand_b=F_coordinate(vertex[3],axis);end
            default:begin operand_a=top;operand_b=bottom;end
        endcase
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_done<=0;O_x<=0;O_y<=0;state<=0;address<=0;base<=0;
            corner<=0;stage<=0;axis<=0;fx<=0;fy<=0;coefficient<=0;
            count<=0;top<=0;bottom<=0;a<=0;accumulator<=0;shifted_delta<=0;
            for(r=0;r<4;r=r+1)vertex[r]<=0;
        end else if(I_abort) begin
            state<=0;O_done<=0;
        end else begin
            O_done<=0;
            case(state)
                0:if(I_start)begin
                    base<=start_address[8:0];address<=start_address[8:0];
                    fx<=I_x_half[6:0];fy<=I_y_half[6:0];corner<=0;axis<=0;stage<=0;state<=1;
                end
                1:state<=2; // synchronous ROM latency
                2:begin
                    vertex[corner]<=data;
                    case(corner)
                        0:begin address<=base+1'b1;corner<=1;state<=1;end
                        1:begin address<=base+9'd21;corner<=2;state<=1;end
                        2:begin address<=base+9'd22;corner<=3;state<=1;end
                        3:state<=3;
                    endcase
                end
                3:begin
                    a<=operand_a;shifted_delta<={{7{delta[16]}},delta};
                    accumulator<=0;coefficient<=stage==2 ? fy : fx;count<=0;state<=4;
                end
                4:begin
                    accumulator<=product_next;shifted_delta<=shifted_delta<<<1;
                    coefficient<=coefficient>>1;
                    if(count==6)begin
                        if(stage==0)begin top<=result_wide[15:0];stage<=1;state<=3;end
                        else if(stage==1)begin bottom<=result_wide[15:0];stage<=2;state<=3;end
                        else if(!axis)begin O_x<=result_wide[15:0];axis<=1;stage<=0;state<=3;end
                        else begin O_y<=result_wide[15:0];O_done<=1;state<=0;end
                    end else count<=count+1'b1;
                end
                default:state<=0;
            endcase
        end
    end
endmodule

// Frame-rate metrology, independent of the smoothed/held drawing rectangle.
// The four raw strong-core bounding edges are mapped into the reference plane.
// Average opposing edge lengths, then report the longer/shorter in 0.1 mm.
// These are image-axis bounding-box dimensions; arbitrary rotated shapes need
// an oriented-contour algorithm and are deliberately not claimed here.
module color_size_measure(
    input wire I_clk,I_rst_n,I_enable,I_source_fresh,I_sof,
    input wire [2:0] I_confirmed,I_measured,
    input wire I_result,I_match,
    input wire [1:0] I_color,
    input wire [11:0] I_left,I_right,I_top,I_bottom,
    output reg [2:0] O_valid,
    output reg [47:0] O_length_x10,O_width_x10
);
    localparam [3:0] IDLE=0,MAP_START=1,MAP_WAIT=2,EDGE_START=3,
        MULTIPLY=4,MULTIPLY_Y=5,SQRT_START=6,SQRT_RUN=7,FINISH=8;
    reg [3:0] state;
    reg [2:0] pending,work_valid,snapshot_valid;
    reg [11:0] queued_left[0:2],queued_right[0:2],queued_top[0:2],queued_bottom[0:2];
    reg [47:0] work_length,work_width;
    reg snapshot_pending;
    reg [1:0] color,point,edge_index;
    reg [11:0] left,right,top,bottom;
    reg [12:0] map_x_half,map_y_half;
    reg map_start,job_valid;
    wire map_done;
    wire signed [15:0] map_x,map_y;
    reg signed [15:0] px[0:3],py[0:3];
    reg signed [15:0] edge_from_x,edge_from_y,edge_to_x,edge_to_y;
    wire signed [16:0] dx=$signed({edge_to_x[15],edge_to_x})-$signed({edge_from_x[15],edge_from_x});
    wire signed [16:0] dy=$signed({edge_to_y[15],edge_to_y})-$signed({edge_from_y[15],edge_from_y});
    reg [15:0] abs_x,abs_y,multiplier;
    reg [31:0] multiplicand,product,x_squared;
    wire [31:0] product_next=product+(multiplier[0] ? multiplicand : 32'd0);
    reg [4:0] count;
    reg [33:0] radicand;
    reg [18:0] remainder;
    reg [16:0] root,edge_length[0:3];
    wire [20:0] remainder_shifted={remainder,radicand[33:32]};
    wire [20:0] trial={2'b00,root,2'b01};
    wire [21:0] trial_delta={1'b0,remainder_shifted}-{1'b0,trial};
    wire subtract_ok=!trial_delta[21];
    wire [20:0] remainder_next=subtract_ok ? trial_delta[20:0] : remainder_shifted;
    wire [16:0] root_next={root[15:0],subtract_ok};
    wire [18:0] horizontal={2'd0,edge_length[0]}+{2'd0,edge_length[2]};
    wire [18:0] vertical={2'd0,edge_length[1]}+{2'd0,edge_length[3]};
    // (opposing Q6 edges / 2) * 10, rounded to nearest 0.1 mm.
    wire [21:0] horizontal_x10=((({3'd0,horizontal}<<2)+horizontal)+22'd32)>>6;
    wire [21:0] vertical_x10=((({3'd0,vertical}<<2)+vertical)+22'd32)>>6;
    integer j;
    color_size_point_map u_point_map(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(map_start),
        .I_abort(I_sof || !I_enable || !I_source_fresh),
        .I_x_half(map_x_half),.I_y_half(map_y_half),.O_done(map_done),.O_x(map_x),.O_y(map_y));
    always @* begin
        case(edge_index)
            0:begin edge_from_x=px[0];edge_to_x=px[1];edge_from_y=py[0];edge_to_y=py[1];end
            1:begin edge_from_x=px[1];edge_to_x=px[2];edge_from_y=py[1];edge_to_y=py[2];end
            2:begin edge_from_x=px[2];edge_to_x=px[3];edge_from_y=py[2];edge_to_y=py[3];end
            default:begin edge_from_x=px[3];edge_to_x=px[0];edge_from_y=py[3];edge_to_y=py[0];end
        endcase
    end
    // Raw results are produced during vertical blanking. Snapshot before
    // clearing the work set; publish one clock after SOF, after tracker NBA.
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            snapshot_pending<=0;snapshot_valid<=0;
            O_valid<=0;O_length_x10<=0;O_width_x10<=0;
        end else begin
            snapshot_pending<=I_sof;
            if(I_sof)begin
                snapshot_valid<=work_valid;
            end
            if(!I_enable || !I_source_fresh) O_valid<=0;
            else if(snapshot_pending)begin
                O_valid<=snapshot_valid & I_confirmed & I_measured;
                O_length_x10<=work_length;O_width_x10<=work_width;
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            state<=IDLE;pending<=0;work_valid<=0;work_length<=0;work_width<=0;
            color<=0;point<=0;edge_index<=0;left<=0;right<=0;top<=0;bottom<=0;
            map_x_half<=0;map_y_half<=0;map_start<=0;job_valid<=0;
            abs_x<=0;abs_y<=0;multiplier<=0;multiplicand<=0;product<=0;x_squared<=0;
            count<=0;radicand<=0;remainder<=0;root<=0;
            for(j=0;j<3;j=j+1)begin queued_left[j]<=0;queued_right[j]<=0;queued_top[j]<=0;queued_bottom[j]<=0;end
            for(j=0;j<4;j=j+1)begin px[j]<=0;py[j]<=0;edge_length[j]<=0;end
        end else if(!I_enable || !I_source_fresh || I_sof)begin
            state<=IDLE;pending<=0;work_valid<=0;map_start<=0;
        end else begin
            map_start<=0;
            if(I_result && I_color>=1 && I_color<=3)begin
                pending[I_color-1'b1]<=I_match;
                work_valid[I_color-1'b1]<=0;
                queued_left[I_color-1'b1]<=I_left;queued_right[I_color-1'b1]<=I_right;
                queued_top[I_color-1'b1]<=I_top;queued_bottom[I_color-1'b1]<=I_bottom;
            end
            case(state)
                IDLE:if(pending!=0)begin
                    if(pending[0])begin color<=0;left<=queued_left[0];right<=queued_right[0];top<=queued_top[0];bottom<=queued_bottom[0];pending[0]<=0;end
                    else if(pending[1])begin color<=1;left<=queued_left[1];right<=queued_right[1];top<=queued_top[1];bottom<=queued_bottom[1];pending[1]<=0;end
                    else begin color<=2;left<=queued_left[2];right<=queued_right[2];top<=queued_top[2];bottom<=queued_bottom[2];pending[2]<=0;end
                    point<=0;job_valid<=1;state<=MAP_START;
                end
                MAP_START:begin
                    if(left==0 || top==0 || right>=1280 || bottom>=720 || right<=left || bottom<=top)begin
                        job_valid<=0;state<=FINISH;
                    end else begin
                        // Inclusive pixel bounds -> geometric pixel edges (+/-0.5 px).
                        map_x_half<=point==0 || point==3 ? {left,1'b0}-1'b1 : {right,1'b0}+1'b1;
                        map_y_half<=point==0 || point==1 ? {top,1'b0}-1'b1 : {bottom,1'b0}+1'b1;
                        map_start<=1;state<=MAP_WAIT;
                    end
                end
                MAP_WAIT:if(map_done)begin
                    px[point]<=map_x;py[point]<=map_y;
                    if(map_x < -16'sd1216 || map_x > 16'sd9728 ||
                       map_y < -16'sd1216 || map_y > 16'sd6080)job_valid<=0;
                    if(point==3)begin edge_index<=0;state<=EDGE_START;end
                    else begin point<=point+1'b1;state<=MAP_START;end
                end
                EDGE_START:begin
                    abs_x<=dx[16] ? -dx : dx;abs_y<=dy[16] ? -dy : dy;
                    multiplier<=dx[16] ? -dx : dx;multiplicand<=dx[16] ? -dx : dx;
                    product<=0;count<=0;state<=MULTIPLY;
                end
                MULTIPLY:begin
                    product<=product_next;multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
                    if(count==15)begin x_squared<=product_next;product<=0;multiplier<=abs_y;multiplicand<=abs_y;count<=0;state<=MULTIPLY_Y;end
                    else count<=count+1'b1;
                end
                MULTIPLY_Y:begin
                    product<=product_next;multiplicand<=multiplicand<<1;multiplier<=multiplier>>1;
                    if(count==15)begin radicand<={2'd0,x_squared}+{2'd0,product_next};state<=SQRT_START;end
                    else count<=count+1'b1;
                end
                SQRT_START:begin count<=0;root<=0;remainder<=0;state<=SQRT_RUN;end
                SQRT_RUN:begin
                    root<=root_next;remainder<=remainder_next[18:0];radicand<=radicand<<2;
                    if(count==16)begin
                        edge_length[edge_index]<=root_next;
                        if(edge_index==3)state<=FINISH;
                        else begin edge_index<=edge_index+1'b1;state<=EDGE_START;end
                    end else count<=count+1'b1;
                end
                FINISH:begin
                    if(job_valid && horizontal_x10<=9999 && vertical_x10<=9999 && horizontal_x10!=0 && vertical_x10!=0)begin
                        work_valid[color]<=1;
                        work_length[color*16 +:16]<=horizontal_x10>=vertical_x10 ? horizontal_x10[15:0] : vertical_x10[15:0];
                        work_width[color*16 +:16]<=horizontal_x10>=vertical_x10 ? vertical_x10[15:0] : horizontal_x10[15:0];
                    end
                    state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
`endif
