`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.04.2025 16:34:30
// Design Name: 
// Module Name: riscv_multicycle
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

module riscv_multicycle 
#(
    parameter DMemInitFile  = "dmem.mem",       // data memory initialization file
    parameter IMemInitFile  = "imem.mem",       // instruction memory initialization file
    parameter XLEN          = 32
)   
(
    input  logic             clk_i,       // system clock
    input  logic             rstn_i,      // system reset
    
    
    input  logic  [XLEN-1:0] addr_i,      // memory adddres input for reading
    output logic  [XLEN-1:0] data_o,      // memory data output for reading
    output logic             update_model,
    output logic             update_o,    // retire signal
    output logic  [XLEN-1:0] Instr_o,     // retired instruction
    output logic  [     4:0] reg_addr_o,  // retired register address
    output logic  [XLEN-1:0] reg_data_o,  // retired register data
    output logic  [XLEN-1:0] mem_addr_o,  // retired memory address
    output logic  [XLEN-1:0] mem_data_o,  // retired memory data
    output logic             mem_wrt_o,    // retired memory write enable signal
    
    output logic  [XLEN-1:0] pc_f,        // retired program counter (fetch)
    output logic  [XLEN-1:0] pc_d,        // retired program counter (decode)
    output logic  [XLEN-1:0] pc_e,        // retired program counter (execute)
    output logic  [XLEN-1:0] pc_m,        // retired program counter (memory) 
    output logic  [XLEN-1:0] pc_wb,       // retired program counter  (write back)
    
    output logic             StallF,
    output logic             StallD,      // Stall outputs for tb
    
    output logic             FlushD,
    output logic             FlushE      // Flush outputs for tb
    
);
  
logic pcsrce1,MemWrite1, ALUSrc1 ,RegWrite1, Jump1, Branch1, resultsrce_01, regwrite_m1, regwrite_w1;
logic [3:0] ALUControl1;
logic [2:0] ImmSrc1;
logic [1:0] ResultSrc1, TargetSrc1, ForwardAE1, ForwardBE1;
logic StallF1, StallD1, FlushD1, FlushE1;
logic [31:0] Instr1;
logic [19:15] rs1d1, rs1_e1;
logic [24:20] rs2d1, rs2_e1;
logic [11:7]  rd_e1, rd_m1, rd_w1;
logic [31:0]  Instr_raw;

logic [4:0] reg_addr_o1;

assign reg_addr_o = (Instr_o[6:0] == 7'b1100011) ? 5'h0 : reg_addr_o1; // send zero for branch, Instr_o = InstrW
                                                              // reg_addr_o1 = a3 (from regfile)

always_comb begin
    update_o = 1;

    if (Instr_raw == 32'h0) begin
        update_o = 0;
    end
 
 end
 
 
 always_comb begin
    update_model = 1;

    if (Instr_o == 32'h0) begin                             // Instr_o = InstrW (instruction at writeback stage)
        update_model = 0;
    end
 
 end


assign StallF = StallF1;
assign StallD = StallD1;

assign FlushD = FlushD1;
assign FlushE = FlushE1;


Pipeline_Datapath 
Datapath 
(   
    .Jump(Jump1),
    .Branch(Branch1),
    .MemWrite(MemWrite1),
    .ALUSrc(ALUSrc1),
    .RegWrite(RegWrite1),
    
    .clk(clk_i),
    .rstn_i(rstn_i),            
    
    .TargetSrc(TargetSrc1),
    .ALUControl(ALUControl1),
    .ImmSrc(ImmSrc1),
    .ResultSrc(ResultSrc1),
    .Instr(Instr1),            // Control Unit Output
    
    .ForwardAE(ForwardAE1),
    .ForwardBE(ForwardBE1),
    
    .StallF(StallF1),
    .StallD(StallD1),
    
    .FlushD(FlushD1),
    .FlushE(FlushE1),
    
    .rs1d(rs1d1),
    .rs1_e(rs1_e1),
    .rs2d(rs2d1),
    .rs2_e(rs2_e1),
    .rd_e(rd_e1),
    .rd_m(rd_m1),
    .rd_w(rd_w1),
    
    .pcsrce(pcsrce1), 
    .resultsrce_0(resultsrce_01),
    .regwrite_m(regwrite_m1),
    .regwrite_w(regwrite_w1),
    
    .addr_i(addr_i),                 //input for data mem port (additional)
    
    .data_o(data_o),        
    .Instr_o(Instr_o),   
    
    .reg_addr_o(reg_addr_o1),        //conditional output for tb
    
    .reg_data_o(reg_data_o),
    .mem_addr_o(mem_addr_o),         //testbecnh outputs
    .mem_data_o(mem_data_o),
    .mem_wrt_o(mem_wrt_o),
    
    
    .pc_f(pc_f),  
    .pc_d(pc_d),  
    .pc_e(pc_e),                     // table components for pipe.log
    .pc_m(pc_m),  
    .pc_wb(pc_wb),
    
    .Instr_raw(Instr_raw)            // output from imem        
 ); 
 
 

Control_Unit 
ctrl 
(
    .op(Instr1[6:0]),
  //.rstn_i(rstn_i),               
    .funct3(Instr1[14:12]),
    .funct7(Instr1[30]),
    .Jump(Jump1),
    .Branch(Branch1),
    .MemWrite(MemWrite1),
    .ALUSrc(ALUSrc1),
    .RegWrite(RegWrite1),
    .ResultSrc(ResultSrc1),
    .ImmSrc(ImmSrc1),
    .ALUControl(ALUControl1),
    .TargetSrc(TargetSrc1)
);
 
 
 Hazard_Unit 
 unit 
 (
    .rs1d(rs1d1),
  //.rstn_i(rstn_i),
    .rs1e(rs1_e1),
    .rs2d(rs2d1),
    .rs2e(rs2_e1),
    .rde(rd_e1),
    .rdm(rd_m1),
    .rdw(rd_w1),
    .resultsrce0(resultsrce_01),
    .regwritem(regwrite_m1),
    .regwritew(regwrite_w1),
    .pcsrce(pcsrce1),
    .StallF(StallF1),
    .StallD(StallD1),
    .FlushD(FlushD1),
    .FlushE(FlushE1),
    .ForwardAE(ForwardAE1),
    .ForwardBE(ForwardBE1)
); 
 
endmodule

