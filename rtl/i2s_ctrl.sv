module i2s_ctrl#(
    // Supports 24/20/18/16 bits wide
    parameter DATA_WIDTH = 16
)
(
    input logic [0:0] CLK_I,
    input logic [0:0] RST_I,
    input logic [0:0] EN_TX_I,
    input logic [0:0] EN_RX_I,
    input logic [3:0] FS_I,
    input logic [0:0] MM_I,
    input logic [DATA_WIDTH-1:0] D_L_I,
    input logic [DATA_WIDTH-1:0] D_R_I,
    output logic [DATA_WIDTH-1:0] D_L_O,
    output logic [DATA_WIDTH-1:0] D_R_O,
    output logic [0:0] BCLK_O,
    output logic [0:0] LRCLK_O,
    output logic [0:0] SDATA_O,
    input logic [0:0] SDATA_I
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

//////////////////////////////////////////////////////////////////////////////////////////
//                                  I2S Implementation                                  //
//////////////////////////////////////////////////////////////////////////////////////////

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

always_comb begin
    D_L_O = D_L_O_int;
    D_R_O = D_R_O_int;
end

endmodule
