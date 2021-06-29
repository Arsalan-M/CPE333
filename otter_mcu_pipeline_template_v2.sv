`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  A. Mughal
// 
// 
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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
//Notable differences from previous
//  Branch is always taken, Two forwarding logic locations (decode and ex),
//  JAL and JALR jump immediately in decode stage, branch comparisons is done in ex stage
//////////////////////////////////////////////////////////////////////////////////

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, branch_de_pc, branch_prev_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg,mem_data,aluAFin,aluBFin;
    
    wire [31:0] IR;
    wire memRead1, memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize, pc_sel_decode;
    logic [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel, stall_r1, stall_r2, stall_aluB, stall_aluA;
    
    wire [31:0] wb_dataOut2;
    
    logic br_lt,br_eq,br_ltu, stallDe, stallEx, resetDe, jump_readStop, branch_readStop;
    
    instr_t de_ex_inst, de_inst, ex_mem_inst, ex_inst, mem_inst, mem_wb_inst, wb_inst;
    assign opcode = IR[6:0];
    logic r1Sel, r2Sel, branch_sel, brn_cond;
    logic [31:0] wb_aluRes, if_de_pc;
    logic [31:0] de_ex_opA, de_ex_opB, de_ex_rs2;
    logic [31:0] ex_mem_rs2, ex_mem_aluRes;
    logic [31:0] r1_forwarded, r2_forwarded;
    logic [31:0] mem_wb_aluRes, mem_rs2, mem_aluRes, mem_wb_data;
    logic [2:0] de_ex_func3;
    
    wire [2:0] ex_func3;
    
    ProgCount PCCounter(.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite),
                        .PC_DIN(pc_value), .PC_COUNT(pc));
    Mult4to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, pc_sel, pc_value); //No Interrupt
    
    OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc),.MEM_ADDR2(mem_aluRes),.MEM_DIN2(mem_rs2),
                               .MEM_WRITE2(mem_inst.memWrite),.MEM_READ1(memRead1),.MEM_READ2(mem_inst.memRead2),
                               .MEM_DOUT1(IR),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(mem_inst.mem_type[1:0]),.MEM_SIGN(mem_inst.mem_type[2]));
           
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(de_inst.opcode), .CU_FUNC3(IR[14:12]),.CU_FUNC7(IR[31:25]), .CU_PCSOURCE(pc_sel_decode),
             .CU_ALU_SRCA(opA_sel),.CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(wb_sel));
    
	Mult4to1 ALUBinput (B, I_immed, S_immed, de_inst.pc, opB_sel, aluBin);
	Mult2to1 ALUAinput (A, U_immed, opA_sel, aluAin); 
	
	Mult2to1 branchMux (branch_de_pc, branch_prev_pc, branch_sel, branch_pc);
	
	OTTER_registerFile RF (de_inst.rs1_addr, de_inst.rs2_addr, wb_inst.rd_addr, rfIn, wb_inst.regWrite, A, B, CLK); // Register file
 
    Mult4to1 regWriteback (wb_inst.pc + 4,csr_reg,wb_dataOut2,wb_aluRes,wb_inst.rf_wr_sel,rfIn);
	
	 // Creates a RISC-V ALU
    OTTER_ALU ALU (ex_inst.alu_fun, aluAFin, aluBFin, aluResult); // the ALU
    
    //Forward Logic comparing values from ex, mem, and wb stage to decode stage; Raises a stall if forwarding requires a stall
    ForwardToDecode jalrForward (de_inst.rs1_addr, A, ex_inst.opcode, ex_inst.rd_addr, aluResult, 
                                mem_inst.opcode, mem_inst.rd_addr, mem_aluRes, wb_inst.opcode, wb_inst.rd_addr, rfIn,
                                de_inst.rs1_used, stall_r1, r1_forwarded);
    ForwardToDecode r2StoreForward (de_inst.rs2_addr, B, ex_inst.opcode, ex_inst.rd_addr, aluResult,
                                mem_inst.opcode, mem_inst.rd_addr, mem_aluRes, wb_inst.opcode, wb_inst.rd_addr, rfIn,
                                de_inst.rs2_used, stall_r2, r2_forwarded);
                                
    //Forward Logic comparing values from mem, and wb stage to ex stage; Raises stall a stall if forwarding requries a stall
    ForwardToExec AForward (ex_inst.rs1_addr, de_ex_opA, mem_inst.opcode, mem_inst.rd_addr, mem_aluRes,
                            wb_inst.opcode, wb_inst.rd_addr, rfIn, ex_inst.rs1_used, stall_aluA, aluAFin);
    ForwardToExec BForward (ex_inst.rs2_addr, de_ex_opB, mem_inst.opcode, mem_inst.rd_addr, mem_aluRes,
                            wb_inst.opcode, wb_inst.rd_addr, rfIn, (ex_inst.rs2_used & (ex_inst.opcode!=STORE)),
                            stall_aluB, aluBFin);
    //branch_sel is a bit that dictates what goes to the branch_pc through a 2-1 mux
    //If branch wasn't taken, branch_sel is 1 and reverts to the old pc in the next cycle. Else follow pc_sel from decoder            
    always_comb
    begin
        if (branch_sel) pc_sel = 2'b10;
        else pc_sel = pc_sel_decode;
    end
    
              
//==== Instruction Fetch ===========================================

     always_ff @(posedge CLK) begin
                if_de_pc <= pc;
     end
     
     assign next_pc = pc + 4;
     assign stallEx = stall_aluA | stall_aluB; //Brings stallEx high if any of the exe stage forwarding logic raises a stall
     assign pcWrite = ~stallDe & ~stallEx; 	//Always on unless a stall occurs
     assign memRead1 = pcWrite;
     assign resetDe = pcWrite & (jump_readStop | branch_readStop); //resetDe is the signal to clear the decode stage

    //Gives a flag (jump_readStop) if JAL or JALR is in the exe stage
    always_comb
    begin
        if (ex_inst.opcode == JAL || ex_inst.opcode == JALR) jump_readStop = 1;
        else jump_readStop = 0;
    end

    //Stalls for the unique case of two branches in a row (since the ex stage branch overrites the auto jump of the branch in the decode stage)
    always_comb
    begin
        if (de_inst.opcode == BRANCH && ex_inst.opcode == BRANCH && brn_cond == 0) stallDe = 1;
        else stallDe = stall_r1 | stall_r2;
    end
     
//==== Instruction Decode ===========================================

    always_ff @(posedge CLK)
	begin
	   if (~stallDe & ~stallEx)    //If no stall occurs, everything is given to the next stage as usual otherwise reset.
	   begin
	        de_ex_opA <= aluAin;
            de_ex_opB <= aluBin;
            de_ex_rs2 <= r2_forwarded;
            de_ex_func3 <= IR[14:12];
            de_ex_inst <= de_inst;
       end
       else
       begin
            de_ex_opA <= 0;
            de_ex_opB <= 0;
            de_ex_rs2 <= 0;
            de_ex_func3 <= 0;
            de_ex_inst <= 0;        
       end
	end
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(opcode);
    
    assign jalr_pc = I_immed + r1_forwarded;
    //assign branch_pc = pc + {{21{IR[31]}},IR[7],IR[30:25],IR[11:8] ,1'b0};   //word aligned addresses
    assign branch_de_pc = de_inst.pc + {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};   //byte aligned addresses
    assign jump_pc = de_inst.pc + {{12{IR[31]}}, IR[19:12], IR[20],IR[30:21],1'b0};
    //assign int_pc = 0;
    
    // Generate immediates
    assign S_immed = {{20{IR[31]}},IR[31:25],IR[11:7]};
    assign I_immed = {{20{IR[31]}},IR[31:20]};
    assign U_immed = {IR[31:12],{12{1'b0}}};
    
    always_comb
    begin
        if (resetDe)
            de_inst = 0;
        else begin
            de_inst.rs1_addr=IR[19:15];
            de_inst.rs2_addr=IR[24:20];
            de_inst.rd_addr=IR[11:7];
            de_inst.opcode=OPCODE;
            de_inst.pc=if_de_pc;
            de_inst.regWrite = OPCODE != BRANCH && OPCODE != STORE;
            de_inst.memRead2 = OPCODE == LOAD;
            de_inst.memWrite = OPCODE == STORE;
            de_inst.mem_type = IR[14:12]; 
            de_inst.alu_fun = alu_fun;
            de_inst.rf_wr_sel = wb_sel;
           
            de_inst.rs1_used=    de_inst.rs1_addr != 0
                                        && de_inst.opcode != LUI
                                        && de_inst.opcode != AUIPC
                                        && de_inst.opcode != JAL;
                                        
            de_inst.rs2_used=   de_inst.rs2_addr != 0
                                        && (de_inst.opcode == BRANCH
                                        || de_inst.opcode == STORE
                                        || de_inst.opcode == OP);
                                        
            de_inst.rd_used=     de_inst.rd_addr != 0
                                        && de_inst.opcode != BRANCH
                                        && de_inst.opcode != STORE;
            end
    end
	
//==== Execute ======================================================
     logic prev_brn_cond;
     
     always_ff @ (posedge CLK)
     begin
        if (~stallEx)
        begin
            ex_mem_aluRes <= aluResult;
            ex_mem_inst <= ex_inst;
            ex_mem_rs2 <= de_ex_rs2;
            prev_brn_cond <= brn_cond;
        end
        else
        begin
            ex_mem_aluRes <= 0;
            ex_mem_inst <= 0;
            ex_mem_rs2 <= 0;
        end
     end
     
     //Branch Condition Generator
    always_comb
    begin
        br_lt=0; br_eq=0; br_ltu=0;
        if($signed(aluAFin) < $signed(aluBFin)) br_lt=1;
        if(aluAFin==aluBFin) br_eq=1;
        if(aluAFin<aluBFin) br_ltu=1;
    end
    
    //Branch logic occurs within the execute stage
    always_comb
            case(ex_func3)
                        3'b000: brn_cond = br_eq;     //BEQ 
                        3'b001: brn_cond = ~br_eq;    //BNE
                        3'b100: brn_cond = br_lt;     //BLT
                        3'b101: brn_cond = ~br_lt;    //BGE
                        3'b110: brn_cond = br_ltu;    //BLTU
                        3'b111: brn_cond = ~br_ltu;   //BGEU
                        default: brn_cond =0;
            endcase

    always_comb
    begin
        branch_sel = 0;
        branch_readStop = 0;
        if (ex_inst.opcode == BRANCH)
            if (brn_cond) branch_readStop = 1;  //brn_cond being 1 is when the branch was supposed to jump and 0 if it wasn't
            else branch_sel = 1;                //If branch was supposed to be taken, reset decode stage else set pc to branch_prev_pc
        if (mem_inst.opcode == BRANCH && prev_brn_cond == 0)    //This clears the instruction from the branch in the decode stage 
            branch_readStop = 1;                                //in the event when the branch was not taken
    end
     
     assign ex_inst = de_ex_inst;
     assign ex_func3 = de_ex_func3;
     assign branch_prev_pc = ex_inst.pc + 8;

//==== Memory ======================================================
    
    assign IOBUS_ADDR = mem_wb_aluRes;
    assign IOBUS_OUT = mem_rs2;
    
    always_ff @ (posedge CLK)
    begin
        mem_wb_inst <= mem_inst;
        mem_wb_aluRes <= mem_aluRes;
        mem_wb_data <= mem_data;
    end
    
    assign mem_inst = ex_mem_inst;
    assign mem_rs2 = ex_mem_rs2;
    assign mem_aluRes = ex_mem_aluRes;
 
 //mem size might be wrong
     
//==== Write Back ==================================================
    
    assign wb_inst = mem_wb_inst;
    assign wb_aluRes = mem_wb_aluRes;
    assign wb_dataOut2 = mem_wb_data;
       //change into multi cycle
            
endmodule
