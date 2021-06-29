`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/29/2020 05:15:49 PM
// Design Name: 
// Module Name: testAllSim
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


module testAllSim();
   
     logic CLK=0,BTNL,BTNC,PS2Clk,PS2Data,VGA_HS,VGA_V,Tx;
     logic [15:0] SWITCHES,LEDS;
     logic [7:0] CATHODES,VGA_RGB;
     logic [3:0] ANODES;
   
    OTTER_Wrapper wrap(.*);

    initial forever  #10  CLK =  !CLK; 
   
    
    initial begin 
        BTNC=0;
        BTNL=0;
        SWITCHES=15'd0;
      //$finish;
    end
    
    
       
  /*  initial begin
         if(ld_use_hazard)
            $display("%t -------> Stall ",$time);
        if(branch_taken)
            $display("%t -------> branch taken",$time); 
      end*/
endmodule