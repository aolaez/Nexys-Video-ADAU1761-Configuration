# Configuring the ADAU1761 Audio Codec on the Nexys Video Board

## Purpose
The purpose of this repo is to show how to implement audio passthrough from the line-in and line-out jacks on the Nexys Video board from Digilent using the onboard ADAU1761 audio codec. TO assist, I'm using old digilent IP from [this](https://digilent.com/reference/learn/programmable-logic/tutorials/nexys-video-looper-demo/start?redirect=1) example project written in VHDL and updating it to SystemVerilog because nobody likes VHDL.

I recently needed to get audio working on this codec and really struggled to find resources that weren't either for microcontrollers or softcores, the previously linked example project was the only pure HDL implementation I could find after many hours of searching so hopefully this helps anyone trying to interface with the ADAU1761 on an FPGA. This project uses Vivado IP and a tcl script to run through the Vivado design flow.

## How to generate bitstream
If you're not interested in the process of writing the modules for interfacing with the codec and simply want to generate the audio passthrough, navigate to `synth->vivado_nexysVideo->build` and run
`vivado -mode batch -source ../vivado.tcl`

## Project Overview and Resources
In order to interface with the codec properly we will need modules for initializing the codec, which we'll use I2C for as the board is configured for that by default, and I2S in order to stream audio data. Relevant pins for these protocols can be found from the datasheet below.
[ADAU1761 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adau1761.pdf)
[I2C](https://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/) and [I2S](https://www.allaboutcircuits.com/technical-articles/introduction-to-the-i2s-interface/) protocols explained

## First Steps
Create a new project in Vivado, selecting the Nexys Video as the board used, the part number for this specific project is `xc7a200tlsbg484-2L`. From reading the datasheet linked above, we see that there are a few pins we'll need to interact with so you should uncomment those from the constraint file provided by Vivado. We'll also use a button on the board as our reset signal. The relevant lines are
```
## Clock Signal
set_property -dict { PACKAGE_PIN R4    IOSTANDARD LVCMOS33 } [get_ports sys_clk]; #IO_L13P_T2_MRCC_34 Sch=sysclk

## Buttons
set_property -dict { PACKAGE_PIN B22 IOSTANDARD LVCMOS12 } [get_ports { sys_rst }]; #IO_L20N_T3_16 Sch=btnc

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
    input [0:0] sys_rst.
    
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

With the project now set up, it's time to write the I2C and I2S protocol modules, starting with I2S.

## I2S Module
Going off of the I2S module in the reference folder, we can begin by declaring all the ports for the I2S module along with signals to be used internally.

### Define module

```
module i2s_ctrl#(
    // Supports 24/20/18/16 bits wide
    parameter DATA_WIDTH = 16
)
(
    input logic [0:0] CLK_I, // 100 MHz sys clock
    input logic [0:0] RST_I, // sys reset
    input logic [0:0] EN_TX_I, // Transmit enable (pushes sound data into chip)
    input logic [0:0] EN_RX_I, // Receive enable (pushes sound data out of chip)
    input logic [3:0] FS_I, // Sampling rate selector/Frame sync selection
    input logic [0:0] MM_I, // Master mode select
    input logic [DATA_WIDTH-1:0] D_L_I, // Left channel data in
    input logic [DATA_WIDTH-1:0] D_R_I, // Right channel data out
    output logic [DATA_WIDTH-1:0] D_L_O, // Left channel data out
    output logic [DATA_WIDTH-1:0] D_R_O, // Right channel data out
    output logic [0:0] BCLK_O, // Serial bit clock
    output logic [0:0] LRCLK_O, // Left/Right channel clock
    output logic [0:0] SDATA_O, // Serial data out
    input logic [0:0] SDATA_I // Serial data in
);

//////////////////////////////////////////////////////////////////////////////////////////
//                                  Signal Declaration                                  //
//////////////////////////////////////////////////////////////////////////////////////////

// Clock Divider Counter
logic [4:0] Cnt_Bclk;

// Counter for L/R channel select clock divider
logic [4:0] Cnt_Lrclk;

// Rising and Falling edges of serial clock
logic [0:0] BCLK_Fall, BCLK_Rise, BCLK_Fall_int, BCLK_Rise_int;

// Internal synchronous BCLK and LRCLK signals
logic [0:0] BCLK_int, LRCLK, LRCLK_int;

// Misc signals
logic [31:0] Data_Out_int, Data_In_int;
logic [DATA_WIDTH-1:0] D_L_O_int, D_R_O_int;

// Div rate for BCLK and LRCLK
logic [4:0] DIV_RATE = 4;
```

Each port and it's purpose has been stated above. The I2S protocol makes use of many different clock signals for channel selection and data transmission so the next step is to define clock dividers to generate these clocks. The clock division rates also depend on the sampling rate selected by the FS_I input port.

### Clock Division
```
// Sampling frequency and data width decoder
always_ff @(posedge CLK_I) begin
    case (FS_I)
        4'h0: DIV_RATE <= 24;
        4'h1: DIV_RATE <= 16;
        4'h2: DIV_RATE <= 12;
        4'h3: DIV_RATE <= 8;
        4'h4: DIV_RATE <= 6;
        4'h5: DIV_RATE <= 4;
        4'h6: DIV_RATE <= 2;
        default: DIV_RATE <= 4;
    endcase    
end

//////////////////////////////////////////////////////////////////////////////////////////
//                              Serial Clock Generator                                  //
//////////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge CLK_I) begin
    if (RST_I) begin
        Cnt_Bclk <= 0;
        BCLK_int <= 0;
    end else if (Cnt_Bclk == ((DIV_RATE/2) - 1)) begin
        Cnt_Bclk <= 0;
        BCLK_int <= ~BCLK_int;
    end else begin
        Cnt_Bclk <= Cnt_Bclk + 1;
    end
end

// Set rising and falling edges when in slave mode, or edges w.r.t Master mode bit, and sck output
always_comb begin
    BCLK_Fall_int = ((Cnt_Bclk == ((DIV_RATE/2)-1)) & (BCLK_int) & ((EN_RX_I) || (EN_TX_I)));
    BCLK_Rise_int = ((Cnt_Bclk == ((DIV_RATE/2)-1)) & (~BCLK_int) & ((EN_RX_I) || (EN_TX_I)));
    BCLK_Fall = BCLK_Fall_int;
    BCLK_Rise = BCLK_Rise_int;
    BCLK_O = (EN_RX_I || EN_TX_I) ? BCLK_int : 1'b1;
end

//////////////////////////////////////////////////////////////////////////////////////////
//                              Left/Right Clock Generator                              //
//////////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge CLK_I) begin
    if (RST_I) begin
        Cnt_Lrclk <= 0;
        LRCLK <= 0; // Left channel active by default
    end else if (BCLK_Fall) begin
        if (Cnt_Lrclk == 31) begin // half of 64 bit frame
            Cnt_Lrclk <= 0;
            LRCLK <= ~LRCLK;
        end else begin
            Cnt_Lrclk <= Cnt_Lrclk + 1;
        end
    end
end

// L/R Clock Output
always_comb begin
    LRCLK_O = (EN_RX_I || EN_TX_I) ? LRCLK : 0;
    LRCLK_int = LRCLK;
end
```

With all the clock signals created, the next step is writing the RTL needed for the actual exchange of data which will be done with PISO (parallel in, serial out) and SIPO (serial in, parallel out) shift registers.

### PISO/SIPO
```
//////////////////////////////////////////////////////////////////////////////////////////
//                                  PISO Protocol                                       //
//////////////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge CLK_I) begin
    if (RST_I) begin
        Data_Out_int[31] <= 0;
        Data_Out_int[30 : 31-DATA_WIDTH] <= D_L_I; // Takes left channel data by default
        Data_Out_int[30-DATA_WIDTH : 0] <= '0;
    end else if ((Cnt_Lrclk == 0) && BCLK_Rise) begin // parallel in
        if (LRCLK_int) begin
            Data_Out_int[31] <= 0;
            Data_Out_int[30 : 31-DATA_WIDTH] <= D_R_I; // Takes right channel data
            Data_Out_int[30-DATA_WIDTH : 0] <= '0;
        end else begin
            Data_Out_int[31] <= 0;
            Data_Out_int[30 : 31-DATA_WIDTH] <= D_L_I; // Takes left channel data 
            Data_Out_int[30-DATA_WIDTH : 0] <= '0;
        end
    end else if (BCLK_Fall) begin // serial out
        Data_Out_int <= {Data_Out_int[30:0], 1'b0};
    end
end

// Serial data output
always_comb begin
    SDATA_O = (EN_TX_I) ? Data_Out_int[31] : 0;
end


//////////////////////////////////////////////////////////////////////////////////////////
//                                  SIPO Protocol                                       //
//////////////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge CLK_I) begin
    if (RST_I) begin
        Data_In_int <= '0;
        D_L_O_int <= '0;
        D_R_O_int <= '0;
    end else if ((Cnt_Lrclk == 0) && BCLK_Fall) begin // Load out parallel data
        if (LRCLK_int) begin
            D_L_O_int <= Data_In_int[31 : 32-DATA_WIDTH];
            Data_In_int <= '0;
        end else begin
            D_R_O_int <= Data_In_int[31 : 32-DATA_WIDTH];
            Data_In_int <= '0;
        end
    end else if (BCLK_Rise) begin
        Data_In_int <= {Data_In_int[30:0], SDATA_I};
    end
end
```
The entire I2S protocol has now been implemented and all that's left is connect the output ports to our input signals.

```
always_comb begin
    D_L_O = D_L_O_int;
    D_R_O = D_R_O_int;
end
```
With this module now complete, we can instantiate it in our top level and connect it to some of the pins on our audio codec which we looked at earlier.

```
module nexysVideo (
    input [0:0] sys_clk,
    input [0:0] sys_rst.
    
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
```

With our top level updated and I2S complete, we can move to configuring the ADAU1761 using I2C/two wire interface.

## I2C





