module nexysVideo (
    input [0:0] sys_clk,
    input [0:0] sys_rst,
    
    // I2C Config Interface
    inout [0:0] adau1761_cclk,
    inout [0:0] adau1761_cout,

    // Audio stuff
    input [0:0] ac_adc_sdata,
    output [0:0] ac_mclk,
    output [0:0] ac_dac_sdata,
    output [0:0] ac_bclk,
    output [0:0] ac_lrclk
);

codec_init
#()
codec_init_inst
(
    .clk(sys_clk),
    .rst(rst_n),
    .sda(adau1761_cout),
    .scl(adau1761_cclk)
);

// Busses for holding audio data
wire [23:0] in_audioL;
wire [23:0] in_audioR;
wire [23:0] out_audioL;
wire [23:0] out_audioR;


i2s_ctrl
#()
i2s_ctrl_inst
(
    .CLK_I(sys_clk), // Sys clock
    .RST_I(sys_rst), // Sys reset
    .EN_TX_I(1'b1), // Transmit enable (push data into chip)
    .EN_RX_I(1'b1), // Receive enable (push data out of chip)
    .FS_I(4'b0101), // Sampling rate selection
    .MM_I(1'b0), // Master mode select
    .D_L_I(in_audioL), // left channel input data
    .D_R_I(in_audioR), // right channel input data
    .D_L_O(out_audioL), // left channel output data
    .D_R_O(out_audioR), // right channel output data
    .BCLK_O(ac_bclk), // serial clock
    .LRCLK_O(ac_lrclk), // channel clock
    .SDATA_O(ac_dac_sdata), // output serial data
    .SDATA_I(ac_adc_sdata) // input serial data
);

endmodule
