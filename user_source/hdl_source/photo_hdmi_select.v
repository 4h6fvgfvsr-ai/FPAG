`ifndef PHOTO_HDMI_SELECT_V
`define PHOTO_HDMI_SELECT_V
// Replay bypasses algorithms/OSD. Delay raster controls by the two stages in
// video_out's unpacker; select paths and snapshot image-valid in vertical blank.
module photo_hdmi_select #(parameter integer RAW_CONTROL_DELAY=2)(
    input wire I_clk,I_rst_n,I_view_active,I_view_valid,
    input wire I_raw_vsync,I_raw_de,input wire [23:0] I_raw_rgb,
    input wire I_live_vsync,I_live_de,input wire [23:0] I_live_rgb,
    output wire O_vsync,O_de,output wire [23:0] O_rgb,
    output wire O_replay_ready
);
    reg [1:0] raw_vsync_delay,raw_de_delay;
    reg raw_vsync_previous,selected_view,valid_frame;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            raw_vsync_delay<=3;raw_de_delay<=0;raw_vsync_previous<=1;
            selected_view<=0;valid_frame<=0;
        end else begin
            raw_vsync_delay<={raw_vsync_delay[0],I_raw_vsync};
            raw_de_delay<={raw_de_delay[0],I_raw_de};
            raw_vsync_previous<=I_raw_vsync;
            if(raw_vsync_previous && !I_raw_vsync && !I_live_de)begin
                selected_view<=I_view_active;valid_frame<=I_view_valid;
            end
        end
    end
    wire replay_vsync=RAW_CONTROL_DELAY==0 ? I_raw_vsync:raw_vsync_delay[1];
    wire replay_de=RAW_CONTROL_DELAY==0 ? I_raw_de:raw_de_delay[1];
    assign O_vsync=selected_view ? replay_vsync:I_live_vsync;
    assign O_de=selected_view ? replay_de:I_live_de;
    assign O_rgb=selected_view ? (valid_frame ? I_raw_rgb:24'd0):I_live_rgb;
    // Replay writes page0 in place. Wait until a vertical boundary has
    // actually hidden the previous image before accepting its replacement.
    assign O_replay_ready=selected_view && !valid_frame;
endmodule
`endif
