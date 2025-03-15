module nexysVideo (
    input [0:0] sys_clk,
    
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

i2s_ctrl
#()
i2s_ctrl_inst
(
    .CLK_I(clk_12),
    .RST_I(rst_n),
    .EN_TX_I(1'b1),
    .EN_RX_I(1'b0),
    .FS_I(4'b0101), // div rate of 4, clock rate of 12.288 should result in 48 khz sample rate
    .MM_I(1'b0),
    .D_L_I(synth_sound_o),
    .D_R_I(synth_sound_o),  // change to whatever wave 
    .D_L_O(),
    .D_R_O(),
    .BCLK_O(ac_bclk),
    .LRCLK_O(ac_lrclk),
    .SDATA_O(ac_dac_sdata),
    .SDATA_I(1'b0)
);

endmodule
