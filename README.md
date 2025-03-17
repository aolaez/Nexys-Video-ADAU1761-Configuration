# Configuring the ADAU1761 Audio Codec on the Nexys Video Board

## Purpose
The purpose of this repo is to show how to implement audio passthrough from the line-in and line-out jacks on the Nexys Video board from Digilent using the onboard ADAU1761 audio codec. To assist, I'm using old digilent IP from [this](https://digilent.com/reference/learn/programmable-logic/tutorials/nexys-video-looper-demo/start?redirect=1) example project written in VHDL and updating it to SystemVerilog to make it compatible with more modern tools like DPI-C, to be able to use all the added features that come with SystemVerilog, and because nobody likes VHDL.

I recently needed to get audio working on this codec and really struggled to find resources that weren't either for microcontrollers or softcores, the previously linked example project was the only pure HDL implementation I could find after many hours of searching so hopefully this helps anyone trying to interface with the ADAU1761 on an FPGA. This project uses Vivado IP and a tcl script to run through the Vivado design flow.

## How to generate bitstream
If you're not interested in the process of writing the modules for interfacing with the codec and simply want to generate the audio passthrough, navigate to `synth->vivado_nexysVideo->build` and run
`vivado -mode batch -source ../vivado.tcl`

## Project Overview and Resources
In order to interface with the codec properly we will need modules for initializing the codec, which we'll use I2C for as the board is configured for that by default, and I2S in order to stream audio data. Relevant pins for these protocols can be found from the ADAU1761 datasheet [here](https://www.analog.com/media/en/technical-documentation/data-sheets/adau1761.pdf). If you're unfamiliar with the [I2C](https://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/) and [I2S](https://www.allaboutcircuits.com/technical-articles/introduction-to-the-i2s-interface/) protocols you can use these articles for reference.

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
Going off of the TWICtrl module within the reference folder, we can begin rewriting this in SystemVerilog by defining all ports and internal signals.
```
module TWICtrl #(
    parameter CLOCKFREQ = 50
) 
(
    input logic [0:0] MSG_I, // New message
    input logic [0:0] STB_I, // strobe
    input logic [7:0] A_I, // address input bus
    input logic [7:0] D_I, // data input bus
    output logic [7:0] D_O, // data output bus
    output logic [0:0] DONE_O, // done status signal
    output logic [0:0] ERR_O, // error status
    input logic [0:0] CLK,
    input logic [0:0] SRST,
    // TWI Bus signals
    inout logic [0:0] SDA, // TWI SDA
    inout logic [0:0] SCL // TWI SCL
);

//////////////////////////////////////////////////////////////////////////////////////////
//                                  Signal Declaration                                  //
//////////////////////////////////////////////////////////////////////////////////////////
    
    localparam int FSCL = 400_000; // SCL Clock Freq in Hz
    localparam int TIMEOUT = 10; // TWI timeout for slave wait period in ms
    localparam int TSCL_CYCLES = int'($ceil(real'(CLOCKFREQ * 1_000_000) / FSCL));
    localparam int TIMEOUT_CYCLES = int'($ceil(real'(CLOCKFREQ * TIMEOUT * 1_000)));

    typedef enum logic [3:0] { stIdle, stStart, stRead, stWrite, stError, stStop,
        stSAck, stMAck, stMNAckStop, stMNAckStart, stStopError } state_type;
    typedef enum logic [1:0] { busUnknown, busBusy, busFree } busState_type;
    typedef enum logic [0:0] { errArb, errNAck } error_type;

    state_type state, nstate;
    busState_type busState = busUnknown;
    error_type errTypeR, errType;

    logic [0:0] rSda, rScl = 1'b1;
    logic [0:0] latchData, latchAddr, iDone, iErr, iSda, iScl, shiftBit, dataBitOut, rwBit, addrNData;
    logic [0:0] dSda, ddSda, dScl, ddScl;
    logic [0:0] slaveWait, arbLost;
    logic [0:0] fStart, fStop;
    logic [0:0] int_Rst = 1'b0;
    logic [1:0] subState = 2'b0;
    logic [2:0] bitCount = 3'd7;
    logic [7:0] dataByte, loadByte, currAddr; // Shift register and parallel load
    logic [31 : 0] sclCnt = TSCL_CYCLES;
    logic [31 : 0] busFreeCnt = TSCL_CYCLES;
    logic [31 : 0] timeOutCnt = TIMEOUT_CYCLES;
```

We then need to define lots of sequential logic to handle clock division rates, detecting the status of data busses, detecting whether we are in slave or master mode, and much more.
```
//////////////////////////////////////////////////////////////////////////////////////////
//                                 Bus State Detection                                  //
//////////////////////////////////////////////////////////////////////////////////////////

// Sync FF's
always_ff @(posedge CLK) begin
    dSda <= SDA;
    ddSda <= dSda;
    dScl <= SCL;

    fStart <= (dScl && ~dSda && ddSda); // if SCL high while SDA falling, start condition
    fStop <= (dScl && dSda && ~ddSda); // if SCL high while SDA rising, stop condition
end

// TWI State
always_ff @(posedge CLK) begin
    if (int_Rst) begin
        busState <= busUnknown;
    end else if (fStart) begin // set bus to busy if start condition detected
        busState <= busBusy;
    end else if (busFreeCnt == 0) begin
        busState <= busFree;
    end
end

// TBUF_CNT
always_ff @(posedge CLK) begin
    if ((~dScl) || (~dSda) || (int_Rst)) begin
        busFreeCnt <= TSCL_CYCLES;
    end else if (dScl && dSda) begin
        busFreeCnt <= busFreeCnt - 1; // Count down 1 SCL period on free bus
    end
end

// Slave devices can insert wait states by keeping SCL low
// If SDA line doesn't correspond to transmitted data while SCL line is at high
// level the master lost an arbitration to another master
always_comb begin
    slaveWait = (~dScl && rScl) ? 1'b1 : 1'b0;
    arbLost = (dScl && ~dSda && rSda) ? 1'b1 : 1'b0;
end

// Internal reset signal, RST_PROC
always_ff @(posedge CLK) begin
    if ((state == stIdle) && ~SRST) begin
        int_Rst <= 1'b0;
    end else if (SRST) begin
        int_Rst <= 1'b1;
    end
end

// Serial clock period counter, SCL_CNT
always_ff @(posedge CLK) begin
    if ((sclCnt == 0) || (state == stIdle)) begin
        sclCnt <= TSCL_CYCLES/4;
    end else if (~slaveWait) begin // clock synchronization with other masters
        sclCnt <= sclCnt - 1;
    end
end

// TIMEOUT_CNT
always_ff @(posedge CLK) begin
    if ((timeOutCnt == 0) || ~slaveWait) begin
        timeOutCnt <= TIMEOUT_CYCLES;
    end else if (slaveWait) begin // count timeout on wait period inserted by slave
        timeOutCnt <= timeOutCnt - 1;
    end
end

//////////////////////////////////////////////////////////////////////////////////////////
//                             Data byte Shift Register                                 //
//////////////////////////////////////////////////////////////////////////////////////////
// Stores bytes to be written or the byte read depending on transfer direction

// DATABYTE_SHREG
always_ff @(posedge CLK) begin
    if ((latchData || latchAddr) && (sclCnt == 0)) begin
        dataByte <= loadByte; // latch address/data
        bitCount <= 3'd7;
        if (latchData) begin
            addrNData <= 1'b0;
        end else begin
            addrNData <= 1'b1;
        end
    end else if (shiftBit && (sclCnt == 0)) begin
        dataByte <= {dataByte[6:0], dSda};
        bitCount <= bitCount - 1;
    end
end

always_comb begin
    loadByte = (latchAddr) ? A_I : D_I;
    dataBitOut = dataByte[7];
    D_O = dataByte;
end

//////////////////////////////////////////////////////////////////////////////////////////
//                             Current Address Register                                 //
//////////////////////////////////////////////////////////////////////////////////////////
// Stores TWI slave address

always_ff @(posedge CLK) begin
    if (latchAddr) begin
        currAddr <= A_I; // Latch address/data
    end
end

always_comb begin
    rwBit = currAddr[0];
end
```

With the groundwork laid we now need a state machine to handle communication with the audio codec using all the previously defined signals.

```
//////////////////////////////////////////////////////////////////////////////////////////
//                                     Substate Counter                                 //
//////////////////////////////////////////////////////////////////////////////////////////
// Divides each state into 4 for compliance with setup and hold times of TWI bus

always_ff @(posedge CLK) begin
    if (state == stIdle) begin
        subState <= 2'b0;
    end else if (sclCnt == 0) begin
        subState <= subState + 1;
    end
end

always_ff @(posedge CLK) begin
    state <= nstate;
    rSda <= iSda;
    rScl <= iScl;
    DONE_O <= iDone;
    ERR_O <= iErr;
    errTypeR <= errType;
end

always_comb begin
    iSda = rSda; 
    iScl = rScl;
    iDone = 0;
    iErr = 0;
    errType = errTypeR;
    shiftBit = 0;
    latchAddr = 0;
    latchData = 0;

    if (state == stStart) begin
        case (subState)
            2'd0: begin
                iSda = 1; // keep SCL
            end
            2'd1: begin
                iSda = 1;
                iScl = 1;
            end
            2'd2: begin
                iSda = 0;
                iScl = 1;
            end
            2'd3: begin
                iSda = 0;
                iScl = 0;
            end
        endcase
    end

    if ((state == stStop) || (state == stStopError)) begin
        case (subState)
            2'd0: begin
                iSda = 0; // keep SCL
            end
            2'd1: begin
                iSda = 0;
                iScl = 1;
            end
            2'd2: begin
                iSda = 1;
                iScl = 1;
            end
            default: ;
        endcase
    end

    if ((state == stRead) || (state == stSAck)) begin
        case (subState)
            2'd0: begin
                iSda = 1; // Z on SDA, keep SCL
            end
            2'd1: begin
                iScl = 1; // Keep SDA
            end
            2'd2: begin
                iScl = 1; // Keep SDA
            end
            2'd3: begin
                iScl = 0; // keep SDA
            end
        endcase
    end

    if (state == stWrite) begin
        case (subState)
            2'd0: begin
                iSda = dataBitOut; // Keep SCL
            end
            2'd1: begin
                iScl = 1; // Keep SDA
            end
            2'd2: begin
                iScl = 1; // Keep SDA
            end
            2'd3: begin
                iScl = 0; // keep SDA
            end
        endcase
    end

    if (state == stMAck) begin
        case (subState)
            2'd0: begin
                iSda = 0; // acknowledge by writing zero
            end
            2'd1: begin
                iScl = 1; // Keep SDA
            end
            2'd2: begin
                iScl = 1; // Keep SDA
            end
            2'd3: begin
                iScl = 0; // keep SDA
            end
        endcase
    end


    if ((state == stMNAckStop) || (state == stMNAckStart)) begin
        case (subState)
            2'd0: begin
                iSda = 1; // not acknowledge by writing 1
            end
            2'd1: begin
                iScl = 1; // Keep SDA
            end
            2'd2: begin
                iScl = 1; // Keep SDA
            end
            2'd3: begin
                iScl = 0; // keep SDA
            end
        endcase
    end

    if ((state == stSAck) && (sclCnt == 0) && (subState == 2'b01)) begin
        if (dSda) begin
            iDone = 1;
            iErr = 1; // not acknowledged
            errType = errNAck;
        end else if (~addrNData) begin
            iDone = 1; // only done when data is sent after address
        end
    end

    if ((state == stRead) && (subState == 2'b01) && (sclCnt == 0) && (bitCount == 0)) begin
        iDone = 1; // read finished
    end

    if ((state == stWrite) && arbLost) begin
        iDone = 1; // write done
        iErr = 1; // lost arbitration
        errType = errArb;
    end

    if (((state == stWrite) && (sclCnt == 0) && (subState == 2'b11)) || (((state == stSAck )|| (state == stRead)) && (subState == 2'b1))) begin
        shiftBit = 1;
    end

    if (state == stStart) begin
        latchAddr = 1;
    end

    if ((state == stSAck) && (subState == 2'b11)) begin // get data byte for next write
        latchData = 1;
    end
end


always_comb begin
    nstate = state; // default to staying in current state

    case (state)
        stIdle: begin
            if (STB_I && (busState == busFree) && ~SRST) begin
                nstate = stStart;
            end
        end

        stStart: begin
            if ((subState == 2'b11) && (sclCnt == 0)) begin
                nstate = stWrite;
            end
        end

        stWrite: begin
            if (arbLost) begin
                nstate = stIdle;
            end else if ((subState == 2'b11) && (sclCnt == 0) && (bitCount == 0)) begin
                nstate = stSAck;
            end
        end

        stSAck: begin
            if ((subState == 2'b11) && (sclCnt == 0)) begin
                if (int_Rst || dataByte[0]) begin
                    nstate = stStop;
                end else begin
                    if (addrNData) begin
                        if (rwBit) begin
                            nstate = stRead;
                        end else begin
                            nstate = stWrite;
                        end
                    end else if (STB_I) begin
                        if (MSG_I || (currAddr != A_I)) begin
                            nstate = stStart;
                        end else begin
                            if (rwBit) begin
                                nstate = stRead;
                            end else begin
                                nstate = stWrite;
                            end
                        end
                    end else begin
                        nstate = stStop;
                    end
                end
            end
        end

        stStop: begin
            if ((subState == 2'b10) && (sclCnt == 0)) begin
                nstate = stIdle;
            end
        end

        stRead: begin
            if ((subState == 2'b11) && (sclCnt == 0) && (bitCount == 7)) begin // bitcount will underflow
                if (~int_Rst && STB_I) begin
                    if (MSG_I || (currAddr != A_I)) begin
                        nstate = stMNAckStart;
                    end else begin
                        nstate = stMAck;
                    end
                end else begin 
                    nstate = stMNAckStop;
                end
            end
        end

        stMAck: begin
            if ((subState == 2'b11) && (sclCnt == 0)) begin
                nstate = stRead;
            end
        end

        stMNAckStart: begin
            if (arbLost) begin
                nstate = stIdle; // arbitration lost, back off, received data already so no error
            end else if ((subState == 2'b11) && (sclCnt == 0)) begin
                nstate = stStart;
            end
        end

        stMNAckStop: begin
            if (arbLost) begin
                nstate = stIdle; // same as above
            end else if ((subState == 2'b11) && (sclCnt == 0)) begin
                nstate = stStop;
            end
        end

        default: nstate = stIdle;
    endcase
end
```
Finally, all that's left is to assign our ourputs SDA and SCL which will correspond to 2 pinds on the ADAU1761 which we saw in the constraint file earlier.
```
// Open-drain outputs for bi-directional SDA and SCL
always_comb begin
    SDA = (rSda) ? 1'bz : 1'b0;
    SCL = (rScl) ? 1'bz : 1'b0;
end
```

Now we will make use of the codec initialization module from the references folder which acts as a wrapper for our TWI module, and doesn't need to be altered as its in Verilog, not VHDL. We will now instantiate this module in our top level which should now look like this:

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
```

This top level is now ready to be used for audio passthrough as configured above, but can be customized to fit your needs. For example, if you want to generate sound using RTL like you would for a drum machine or synthesizer, you can achieve that by passing different configuration options to the I2S module. A tcl script is provided in the `synth` directory to program audio passthrough automatically with Vivado.

