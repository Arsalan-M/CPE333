`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: A. Mughal
//           
// Design Name: 
// Module Name: OTTER_mem_parity_check 
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module OTTER_mem_parity_check(MEM_CLK,MEM_IN,PAR_IN,MEM_OUT);
    
    input MEM_CLK;
    input [31:0] MEM_IN;

    //input [1:0] MEM_BYTE_EN1;
    //input [1:0] MEM_BYTE_EN2;
    input [5:0] PAR_IN;
    output logic [31:0] MEM_OUT;
    
    logic hammBit1;
    logic hammBit2;
    logic hammBit3;
    logic hammBit4;
    logic hammBit5;
    logic hammBit6;
    logic MEM_SIZE;
    logic [31:0] mem_out;
    logic [37:0] HAMM_CODE;
    logic [5:0] PAR_CODE;
    logic [5:0] PAR_DIFF;
   

    assign hammBit1 = MEM_IN[0] ^ MEM_IN[1] ^ MEM_IN[3] ^ MEM_IN[4] ^ MEM_IN[6] ^ MEM_IN[8] ^ MEM_IN[10] ^ MEM_IN[11] ^ MEM_IN[13] ^ MEM_IN[15] ^ MEM_IN[17] ^ MEM_IN[19] ^ MEM_IN[21] ^ MEM_IN[23] ^ MEM_IN[25] ^ MEM_IN[26] ^ MEM_IN[28] ^ MEM_IN[30];
    assign hammBit2 = MEM_IN[0] ^ MEM_IN[2] ^ MEM_IN[3] ^ MEM_IN[5] ^ MEM_IN[6] ^ MEM_IN[9] ^ MEM_IN[10] ^ MEM_IN[12] ^ MEM_IN[13] ^ MEM_IN[16] ^ MEM_IN[17] ^ MEM_IN[20] ^ MEM_IN[21] ^ MEM_IN[24] ^ MEM_IN[25] ^ MEM_IN[27] ^ MEM_IN[28] ^ MEM_IN[31];
    assign hammBit3 = MEM_IN[1] ^ MEM_IN[2] ^ MEM_IN[3] ^ MEM_IN[7] ^ MEM_IN[8] ^ MEM_IN[9] ^ MEM_IN[10] ^ MEM_IN[14] ^ MEM_IN[15] ^ MEM_IN[16] ^ MEM_IN[17] ^ MEM_IN[22] ^ MEM_IN[23] ^ MEM_IN[24] ^ MEM_IN[25] ^ MEM_IN[29] ^ MEM_IN[30] ^ MEM_IN[31];
    assign hammBit4 = MEM_IN[4] ^ MEM_IN[5] ^ MEM_IN[6] ^ MEM_IN[7] ^ MEM_IN[8] ^ MEM_IN[9] ^ MEM_IN[10] ^ MEM_IN[18] ^ MEM_IN[19] ^ MEM_IN[20] ^ MEM_IN[21] ^ MEM_IN[22] ^ MEM_IN[23] ^ MEM_IN[24] ^ MEM_IN[25];
    assign hammBit5 = MEM_IN[11] ^ MEM_IN[12] ^ MEM_IN[13] ^ MEM_IN[14] ^ MEM_IN[15] ^ MEM_IN[16] ^ MEM_IN[17] ^ MEM_IN[18] ^ MEM_IN[19] ^ MEM_IN[20] ^ MEM_IN[21] ^ MEM_IN[22] ^ MEM_IN[23] ^ MEM_IN[24] ^ MEM_IN[25];
    assign hammBit6 = MEM_IN[26] ^ MEM_IN[27] ^ MEM_IN[28] ^ MEM_IN[29] ^ MEM_IN[30] ^ MEM_IN[31];
    
    
    

    // generate parity bits
    always_comb
    begin
        PAR_CODE[0] = hammBit1;
        PAR_CODE[1] = hammBit2;
        PAR_CODE[2] = hammBit3;
        PAR_CODE[3] = hammBit4;
        PAR_CODE[4] = hammBit5;
        PAR_CODE[5] = hammBit6;
    end   
    
    // determine parity difference
    assign PAR_DIFF = PAR_CODE ^ PAR_IN;
    
    // create Hamming Code
    always_comb
    begin
        HAMM_CODE[0] = hammBit1;
        HAMM_CODE[1] = hammBit2;
        HAMM_CODE[2] = MEM_IN[0];
        HAMM_CODE[3] = hammBit3;
        HAMM_CODE[4] = MEM_IN[1];
        HAMM_CODE[5] = MEM_IN[2];
        HAMM_CODE[6] = MEM_IN[3];
        HAMM_CODE[7] = hammBit4;
        HAMM_CODE[8] = MEM_IN[4];
        HAMM_CODE[9] = MEM_IN[5];
        HAMM_CODE[10] = MEM_IN[6];
        HAMM_CODE[11] = MEM_IN[7];
        HAMM_CODE[12] = MEM_IN[8];
        HAMM_CODE[13] = MEM_IN[9];
        HAMM_CODE[14] = MEM_IN[10];
        HAMM_CODE[15] = hammBit5;
        HAMM_CODE[16] = MEM_IN[11];
        HAMM_CODE[17] = MEM_IN[12];
        HAMM_CODE[18] = MEM_IN[13];
        HAMM_CODE[19] = MEM_IN[14];
        HAMM_CODE[20] = MEM_IN[15];
        HAMM_CODE[21] = MEM_IN[16];
        HAMM_CODE[22] = MEM_IN[17];
        HAMM_CODE[23] = MEM_IN[18];
        HAMM_CODE[24] = MEM_IN[19];
        HAMM_CODE[25] = MEM_IN[20];
        HAMM_CODE[26] = MEM_IN[21];
        HAMM_CODE[27] = MEM_IN[22];
        HAMM_CODE[28] = MEM_IN[23];
        HAMM_CODE[29] = MEM_IN[24];
        HAMM_CODE[30] = MEM_IN[25];
        HAMM_CODE[31] = hammBit6;
        HAMM_CODE[32] = MEM_IN[26];
        HAMM_CODE[33] = MEM_IN[27];
        HAMM_CODE[34] = MEM_IN[28];
        HAMM_CODE[35] = MEM_IN[29];
        HAMM_CODE[36] = MEM_IN[30];
        HAMM_CODE[37] = MEM_IN[31];
    end 
    
    // dtermines error position
    integer n, error_pos = 0;
    always_comb
    begin
        for (n=0; n<6; n++)
            if (PAR_DIFF[n] == 1)
                error_pos += 2**n;
    end
    
    // strips parity bits and correct memory data
    always_comb
    begin
        if (PAR_CODE == PAR_IN) begin
            mem_out = MEM_IN;
        end
        else begin
            HAMM_CODE[error_pos] = ~HAMM_CODE[error_pos];
            mem_out[0] = HAMM_CODE[2];
            mem_out[1] = HAMM_CODE[4];
            mem_out[2] = HAMM_CODE[5];
            mem_out[3] = HAMM_CODE[6];
            mem_out[4] = HAMM_CODE[8];
            mem_out[5] = HAMM_CODE[9];
            mem_out[6] = HAMM_CODE[10];
            mem_out[7] = HAMM_CODE[11];
            mem_out[8] = HAMM_CODE[12];
            mem_out[9] = HAMM_CODE[13];
            mem_out[10] = HAMM_CODE[14];
            mem_out[11] = HAMM_CODE[16];
            mem_out[12] = HAMM_CODE[17];
            mem_out[13] = HAMM_CODE[18];
            mem_out[14] = HAMM_CODE[19];
            mem_out[15] = HAMM_CODE[20];
            mem_out[16] = HAMM_CODE[21];
            mem_out[17] = HAMM_CODE[22];
            mem_out[18] = HAMM_CODE[23];
            mem_out[19] = HAMM_CODE[24];
            mem_out[20] = HAMM_CODE[25];
            mem_out[21] = HAMM_CODE[26];
            mem_out[22] = HAMM_CODE[27];
            mem_out[23] = HAMM_CODE[28];
            mem_out[24] = HAMM_CODE[29];
            mem_out[25] = HAMM_CODE[30];
            mem_out[26] = HAMM_CODE[32];
            mem_out[27] = HAMM_CODE[33];
            mem_out[28] = HAMM_CODE[34];
            mem_out[29] = HAMM_CODE[35];
            mem_out[30] = HAMM_CODE[36];
            mem_out[31] = HAMM_CODE[37];
        end    
    end
    
    // output memory data
    always_ff @(posedge MEM_CLK)
    begin
         MEM_OUT <= mem_out;
    end

endmodule
