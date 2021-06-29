`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/19/2020 02:05:47 PM
// Design Name: 
// Module Name: ForwardLg
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


module ForwardToDecode(rin_addr, rin_data, ex_opcode, ex_addr, ex_data, mem_opcode, mem_addr, mem_data, wb_opcode, wb_addr, wb_data, 
rin_used, stall, data_out);
    
    input [6:0] ex_opcode, mem_opcode, wb_opcode;
    input [4:0] rin_addr, ex_addr, mem_addr, wb_addr;
    input [31:0] rin_data, ex_data, mem_data, wb_data;
    output logic [31:0] data_out;
    input rin_used;
    output logic stall;
    
    always_comb begin
    stall = 0;
    if (rin_used)
    begin
        if (rin_addr == ex_addr)
        begin
            if (ex_opcode == 7'b0000011) stall = 1;
            else data_out = ex_data;
        end
        else if (rin_addr == mem_addr)
        begin
            if (mem_opcode == 7'b0000011) stall = 1;
            else data_out = mem_data;
        end
        else if (rin_addr == wb_addr)
            data_out = wb_data;
        else data_out = rin_data;
    end
    else data_out = rin_data;
    end
endmodule

module ForwardToExec(rin_addr, rin_data, mem_opcode, mem_addr, mem_data, wb_opcode, wb_addr, wb_data, rin_used, stall, data_out);
    
    input [4:0] rin_addr, mem_addr, wb_addr;
    input [31:0] rin_data, mem_data, wb_data;
    input [6:0] mem_opcode, wb_opcode;
    output logic [31:0] data_out;
    input rin_used;
    output logic stall;
    
    always_comb begin
    stall = 0;
    if (rin_used)
    begin
        if (rin_addr == mem_addr)
        begin
            if (mem_opcode == 7'b0000011) stall = 1;
            else data_out = mem_data;
        end
        else if (rin_addr == wb_addr)
            data_out = wb_data;
        else data_out = rin_data;
    end
    else data_out = rin_data;
    end
endmodule
