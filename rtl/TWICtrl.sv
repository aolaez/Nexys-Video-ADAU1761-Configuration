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

// Open-drain outputs for bi-directional SDA and SCL
assign SDA = (rSda) ? 1'bz : 1'b0;
assign SCL = (rScl) ? 1'bz : 1'b0;



endmodule
