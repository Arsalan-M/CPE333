`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: A. Mughal
//           
// Design Name: 
// Module Name: OTTER_mem_parity 
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

module OTTER_mem_parity(MEM_CLK,MEM_ADDR1,MEM_ADDR2,MEM_DIN2,MEM_WRITE2,MEM_READ1,MEM_READ2,PAR_DOUT1,PAR_DOUT2);
    parameter ACTUAL_WIDTH=14;
    parameter NUM_COL = 4;
    parameter COL_WIDTH_PAR = 6;
    
    input [31:0] MEM_ADDR1;     //Instruction Memory Port
    input [31:0] MEM_ADDR2;     //Data Memory Port
    input MEM_CLK;
    input [31:0] MEM_DIN2;
    input MEM_WRITE2;
    input MEM_READ1;
    input MEM_READ2;
    output logic [5:0] PAR_DOUT1;
    output logic [5:0] PAR_DOUT2;
    
    (* rom_style="{distributed | block}" *) 
    (* ram_decomp = "power" *) logic [5:0] par_memory [0:2**ACTUAL_WIDTH-1];
    
    logic hammBit1;
    logic hammBit2;
    logic hammBit3;
    logic hammBit4;
    logic hammBit5;
    logic hammBit6;
    logic MEM_SIZE;
    logic [5:0] HAMM_CODE;
   
    wire [ACTUAL_WIDTH-1:0] parAddr1,parAddr2;
    logic memWrite2;  
    logic [5:0] parOut2;
    logic [31:0] ioIn_buffer=0;
    logic [NUM_COL-1:0] weA;
   
    assign parAddr1 =MEM_ADDR1[ACTUAL_WIDTH+1:2];
    assign parAddr2 =MEM_ADDR2[ACTUAL_WIDTH+1:2];
   
    assign weA = 4'b1 << MEM_ADDR2[1:0];   //sbend
    assign MEM_SIZE = 0;
    
    assign hammBit1 = MEM_DIN2[0] ^ MEM_DIN2[1] ^ MEM_DIN2[3] ^ MEM_DIN2[4] ^ MEM_DIN2[6] ^ MEM_DIN2[8] ^ MEM_DIN2[10] ^ MEM_DIN2[11] ^ MEM_DIN2[13] ^ MEM_DIN2[15] ^ MEM_DIN2[17] ^ MEM_DIN2[19] ^ MEM_DIN2[21] ^ MEM_DIN2[23] ^ MEM_DIN2[25] ^ MEM_DIN2[26] ^ MEM_DIN2[28] ^ MEM_DIN2[30];
    assign hammBit2 = MEM_DIN2[0] ^ MEM_DIN2[2] ^ MEM_DIN2[3] ^ MEM_DIN2[5] ^ MEM_DIN2[6] ^ MEM_DIN2[9] ^ MEM_DIN2[10] ^ MEM_DIN2[12] ^ MEM_DIN2[13] ^ MEM_DIN2[16] ^ MEM_DIN2[17] ^ MEM_DIN2[20] ^ MEM_DIN2[21] ^ MEM_DIN2[24] ^ MEM_DIN2[25] ^ MEM_DIN2[27] ^ MEM_DIN2[28] ^ MEM_DIN2[31];
    assign hammBit3 = MEM_DIN2[1] ^ MEM_DIN2[2] ^ MEM_DIN2[3] ^ MEM_DIN2[7] ^ MEM_DIN2[8] ^ MEM_DIN2[9] ^ MEM_DIN2[10] ^ MEM_DIN2[14] ^ MEM_DIN2[15] ^ MEM_DIN2[16] ^ MEM_DIN2[17] ^ MEM_DIN2[22] ^ MEM_DIN2[23] ^ MEM_DIN2[24] ^ MEM_DIN2[25] ^ MEM_DIN2[29] ^ MEM_DIN2[30] ^ MEM_DIN2[31];
    assign hammBit4 = MEM_DIN2[4] ^ MEM_DIN2[5] ^ MEM_DIN2[6] ^ MEM_DIN2[7] ^ MEM_DIN2[8] ^ MEM_DIN2[9] ^ MEM_DIN2[10] ^ MEM_DIN2[18] ^ MEM_DIN2[19] ^ MEM_DIN2[20] ^ MEM_DIN2[21] ^ MEM_DIN2[22] ^ MEM_DIN2[23] ^ MEM_DIN2[24] ^ MEM_DIN2[25];
    assign hammBit5 = MEM_DIN2[11] ^ MEM_DIN2[12] ^ MEM_DIN2[13] ^ MEM_DIN2[14] ^ MEM_DIN2[15] ^ MEM_DIN2[16] ^ MEM_DIN2[17] ^ MEM_DIN2[18] ^ MEM_DIN2[19] ^ MEM_DIN2[20] ^ MEM_DIN2[21] ^ MEM_DIN2[22] ^ MEM_DIN2[23] ^ MEM_DIN2[24] ^ MEM_DIN2[25];
    assign hammBit6 = MEM_DIN2[26] ^ MEM_DIN2[27] ^ MEM_DIN2[28] ^ MEM_DIN2[29] ^ MEM_DIN2[30] ^ MEM_DIN2[31];
    
    integer i,j;
    always_ff @(posedge MEM_CLK) begin
        //PORT 2  //Data
        if(MEM_WRITE2)
        begin
            j=0;
            for(i=0;i<NUM_COL;i=i+1) begin
                if(weA[i]) begin
                        case(MEM_SIZE)
                            0: par_memory[parAddr2][i*COL_WIDTH_PAR +: COL_WIDTH_PAR] <= HAMM_CODE[5:0]; //MEM_DIN2[(3-i)*COL_WIDTH +: COL_WIDTH];
                            1: begin 
                                    par_memory[parAddr2][i*COL_WIDTH_PAR +: COL_WIDTH_PAR] <= MEM_DIN2[j*COL_WIDTH_PAR +: COL_WIDTH_PAR];
                                    j=j+1;
                               end
                            2: par_memory[parAddr2][i*COL_WIDTH_PAR +: COL_WIDTH_PAR] <= MEM_DIN2[i*COL_WIDTH_PAR +: COL_WIDTH_PAR];
                            default:  par_memory[parAddr2][i*COL_WIDTH_PAR +: COL_WIDTH_PAR] <= HAMM_CODE[i*COL_WIDTH_PAR +: COL_WIDTH_PAR];
                        endcase
                end
            end
         end
         
                 //PORT 1  //Instructions
        if(MEM_READ1)
            PAR_DOUT1 = par_memory[parAddr1];  
    end
    
        always_ff @(negedge MEM_CLK) begin
            if(MEM_READ2)
            PAR_DOUT2 = par_memory[parAddr2]; 
    end

    // generate parity bits
    always_comb
    begin
        HAMM_CODE[0] = hammBit1;
        HAMM_CODE[1] = hammBit2;
        HAMM_CODE[2] = hammBit3;
        HAMM_CODE[3] = hammBit4;
        HAMM_CODE[4] = hammBit5;
        HAMM_CODE[5] = hammBit6;
    end    

endmodule
