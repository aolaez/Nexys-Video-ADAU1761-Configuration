# nexys-video-audio-passthrough

## Purpose
The purpose of this repo is to show how to implement audio passthrough from the line-in and line-out jacks on the Nexys Video board from Digilent using the onboard ADAU1761 audio codec. TO assist, I'm using old digilent IP from [this](https://digilent.com/reference/learn/programmable-logic/tutorials/nexys-video-looper-demo/start?redirect=1) example project written in VHDL and updating it to SystemVerilog because nobody likes VHDL.

I recently needed to get audio working on this codec and really struggled to find resources that weren't either for microcontrollers or softcores, the previously linked example project was the only pure HDL implementation I could find after many hours of searching so hopefully this helps anyone trying to interface with the ADAU1761 on an FPGA. This project uses Vivado IP and a tcl script to run through the Vivado design flow.

## How to generate bitstream
If you're not interested in the process of writing the modules for interfacing with the codec and simply want to generate the audio passthrough, navigate to synth->vivado_nexysVideo->build and run
`vivado -mode batch -source ../vivado.tcl`

## Project Overview and Resources
In order to interface with the codec properly we will need modules for initializing the codec, which we'll use I2C for as the board is configured for that by default, and I2S in order to stream audio data. Relevant pins for these protocols can be found from the datasheet below.
[ADAU1761 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adau1761.pdf)
[I2C](https://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/) and [I2S](https://www.allaboutcircuits.com/technical-articles/introduction-to-the-i2s-interface/) protocols explained

## First Steps
Create a new project in Vivado, selecting the Nexys Video as the board used, the part number for this specific project is `xc7a200tlsbg484-2L` but. From reading the datasheet linked above, we see that there are a few pins we'll need to interact with so you should uncomment those from the constraint file provided by Vivado. The relevant lines are
```
Clock Signal
set_property -dict { PACKAGE_PIN R4    IOSTANDARD LVCMOS33 } [get_ports sys_clk]; #IO_L13P_T2_MRCC_34 Sch=sysclk

## Audio Codec
set_property -dict { PACKAGE_PIN T4    IOSTANDARD LVCMOS33 } [get_ports { ac_adc_sdata }]; #IO_L13N_T2_MRCC_34 Sch=ac_adc_sdata
set_property -dict { PACKAGE_PIN T5    IOSTANDARD LVCMOS33 } [get_ports { ac_bclk }]; #IO_L14P_T2_SRCC_34 Sch=ac_bclk
set_property -dict { PACKAGE_PIN W6    IOSTANDARD LVCMOS33 } [get_ports { ac_dac_sdata }]; #IO_L15P_T2_DQS_34 Sch=ac_dac_sdata
set_property -dict { PACKAGE_PIN U5    IOSTANDARD LVCMOS33 } [get_ports { ac_lrclk }]; #IO_L14N_T2_SRCC_34 Sch=ac_lrclk
set_property -dict { PACKAGE_PIN U6    IOSTANDARD LVCMOS33 } [get_ports { ac_mclk }]; #IO_L16P_T2_34 Sch=ac_mclk

## I2C
set_property -dict { PACKAGE_PIN W5    IOSTANDARD LVCMOS33 } [get_ports { adau1761_cclk }]; #IO_L15N_T2_DQS_34 Sch=scl
set_property -dict { PACKAGE_PIN V5    IOSTANDARD LVCMOS33 } [get_ports { adau1761_cout }]; #IO_L16N_T2_34 Sch=sda
```

Each of these properties should be uncommented and can be given any name you choose, mine are as seen in the get_ports {} sections. We will also create a top level module within vivado_nexysVideo directory called nexysVideo.sv. The top module skeleton is below

```
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

endmodule
```
