/*
creat time: 2020/12/3 15:50

engineer:   xiong guo qiang

description:
对按键操作进行消抖操作，每次按下产生一个时钟周期的高脉冲

*/


module key_remove_shakes( 
	input wire      I_clk,    
	input wire      I_rst_n,
	
	input wire      I_key_in,        
	
	output reg      O_key_trig_out   
);

	///同步到处理时钟域，打三拍
	reg       S_key_in_sync_1d;      
	reg       S_key_in_sync_2d;      
	reg       S_key_in_sync_3d;      
	reg	      S_key_in_sync_4d;
	wire      S_key_in_n_edge;       
			  
	reg       S_delay_en;            
	reg       S_delay_en_1d;         
	wire      S_delay_en_n_edge;     
	reg[16:0] S_delay_cnt;           
	
	
	always @ (posedge I_clk) begin
		S_key_in_sync_1d <= I_key_in;
		S_key_in_sync_2d <= S_key_in_sync_1d;
		S_key_in_sync_3d <= S_key_in_sync_2d;
		S_key_in_sync_4d <= S_key_in_sync_3d;
	end
	

	assign S_key_in_n_edge = ~S_key_in_sync_3d & S_key_in_sync_4d;
	
	
	always @ (posedge I_clk or negedge I_rst_n) begin
		if(!I_rst_n)
			S_delay_en <= 1'b0;
		else
			if(S_key_in_n_edge)
				S_delay_en <= 1'b1;
			else if(S_delay_cnt == 'd100000)
				S_delay_en <= 1'b0;
			else
				S_delay_en <= S_delay_en;
	end
	
	
	always @ (posedge I_clk or negedge I_rst_n) begin
		if(!I_rst_n)
			S_delay_cnt <= 'd0;
		else
			if(S_delay_en)
				S_delay_cnt <= S_delay_cnt + 1'b1;
			else
				S_delay_cnt <= 'd0;
	end
	
	
	always @ (posedge I_clk) begin
		S_delay_en_1d <= S_delay_en;
	end
	
	
	assign S_delay_en_n_edge = ~S_delay_en & S_delay_en_1d;
	
	
	always @ (posedge I_clk) begin
		if(S_delay_en_n_edge && !S_key_in_sync_4d)
			O_key_trig_out <= 1'b1;
		else
			O_key_trig_out <= 1'b0;
	end
	

endmodule


// Active-low push-button synchronizer and debounce filter.
// Kept in this source file because key_remove_shakes.v is part of the
// original project file set and is therefore preserved when TD regenerates
// the synthesis run project.
module key_press_debounce #(
	parameter integer STABLE_CYCLES = 750000,
	parameter integer COUNTER_WIDTH = 20
)(
	input  wire I_clk,
	input  wire I_rst_n,
	input  wire I_key_n,
	output reg  O_press_pulse
);

	reg S_key_meta;
	reg S_key_sync;
	reg S_key_stable;
	reg [COUNTER_WIDTH-1:0] S_stable_count;

	always @(posedge I_clk or negedge I_rst_n) begin
		if(!I_rst_n) begin
			S_key_meta     <= 1'b1;
			S_key_sync     <= 1'b1;
			S_key_stable   <= 1'b1;
			S_stable_count <= {COUNTER_WIDTH{1'b0}};
			O_press_pulse  <= 1'b0;
		end
		else begin
			S_key_meta    <= I_key_n;
			S_key_sync    <= S_key_meta;
			O_press_pulse <= 1'b0;

			if(S_key_sync == S_key_stable) begin
				S_stable_count <= {COUNTER_WIDTH{1'b0}};
			end
			else if(S_stable_count == STABLE_CYCLES - 1) begin
				S_stable_count <= {COUNTER_WIDTH{1'b0}};
				S_key_stable   <= S_key_sync;
				if(!S_key_sync)
					O_press_pulse <= 1'b1;
			end
			else begin
				S_stable_count <= S_stable_count + {{(COUNTER_WIDTH-1){1'b0}}, 1'b1};
			end
		end
	end

endmodule
