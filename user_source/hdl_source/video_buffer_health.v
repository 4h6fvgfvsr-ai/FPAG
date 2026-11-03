// Passive sticky video-buffer fault observer. It never gates pixels or requests.
// Fault bits (HDMI clock domain):
// 0 camera input overflow; 1 DDR->video FIFO overflow;
// 2 display FIFO underflow; 3 DDR command FIFO overflow.
// Each event means requested write while full / requested read while empty.
// Buffer resets suppress expected reset activity but DO NOT clear old faults.
// Underflow is counted only when I_display_monitor_enable (e.g. valid latency)
// qualifies a running, initialized video stream. Only global reset clears faults.
// Three local reset-release flops protect staggered/stopped clocks. Sticky
// levels cross to HDMI through two flops; events cannot be lost as short pulses.
`ifndef VIDEO_BUFFER_HEALTH_V
`define VIDEO_BUFFER_HEALTH_V

module video_buffer_health (
    input wire I_capture_clk,
    input wire I_ddr_clk,
    input wire I_display_clk,
    input wire I_rst_n,
    input wire I_capture_write,
    input wire I_capture_full,
    input wire I_capture_reset,
    input wire I_ddr_video_write,
    input wire I_ddr_video_full,
    input wire I_ddr_video_reset,
    input wire I_ddr_command_write,
    input wire I_ddr_command_full,
    input wire I_display_read,
    input wire I_display_empty,
    input wire I_display_monitor_enable,
    output wire [3:0] O_faults
);
    (* syn_preserve=1, async_reg="true" *) reg [2:0] capture_release,ddr_release,display_release;
    reg capture_overflow,video_overflow,command_overflow,display_underflow;
    (* syn_preserve=1, async_reg="true" *) reg [1:0] capture_sync,video_sync,command_sync;
    always @(posedge I_capture_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin capture_release<=0;capture_overflow<=0;end
        else begin
            capture_release<={capture_release[1:0],1'b1};
            if(capture_release[2] && I_capture_write && I_capture_full && !I_capture_reset)
                capture_overflow<=1;
        end
    end
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin ddr_release<=0;video_overflow<=0;command_overflow<=0;end
        else begin
            ddr_release<={ddr_release[1:0],1'b1};
            if(ddr_release[2]) begin
                if(I_ddr_video_write && I_ddr_video_full && !I_ddr_video_reset)
                    video_overflow<=1;
                if(I_ddr_command_write && I_ddr_command_full)
                    command_overflow<=1;
            end
        end
    end
    always @(posedge I_display_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            display_release<=0;display_underflow<=0;
            capture_sync<=0;video_sync<=0;command_sync<=0;
        end else begin
            display_release<={display_release[1:0],1'b1};
            if(display_release[2]) begin
                capture_sync<={capture_sync[0],capture_overflow};
                video_sync<={video_sync[0],video_overflow};
                command_sync<={command_sync[0],command_overflow};
                if(I_display_monitor_enable && I_display_read && I_display_empty)
                    display_underflow<=1;
            end else begin
                capture_sync<=0;video_sync<=0;command_sync<=0;
            end
        end
    end
    assign O_faults={command_sync[1],display_underflow,video_sync[1],capture_sync[1]};
endmodule
`endif

