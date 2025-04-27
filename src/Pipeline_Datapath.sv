`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.04.2025 22:51:48
// Design Name: 
// Module Name: Pipeline_Datapath
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

module Pipeline_Datapath
(
    input  logic         Jump,Branch,MemWrite, ALUSrc ,RegWrite, clk, rstn_i,
    input  logic [3:0]   ALUControl, 
    input  logic [1:0]   ResultSrc, TargetSrc, ForwardAE, ForwardBE,
    input  logic [2:0]   ImmSrc,
    
    input  logic [31:0]  addr_i, //port for reading from outside
    
    input  logic         StallF, StallD, FlushD,FlushE ,
    
    output logic [31:0]  Instr, //control unit input
    
    output logic [19:15] rs1d,rs1_e, 
    output logic [24:20] rs2d,rs2_e,
    output logic [11:7]  rd_e, rd_m, rd_w, 
    output logic         pcsrce, resultsrce_0, regwrite_m,regwrite_w,
    
    output logic [31:0]  Instr_o,
    output logic [31:0]  data_o,
    output logic [4:0]   reg_addr_o,
    output logic [31:0]  reg_data_o,          //testbench outputs
    output logic [31:0]  mem_addr_o,
    output logic [31:0]  mem_data_o,
    output logic         mem_wrt_o,
    
    
    
     output logic  [31:0] pc_f,        // retired program counter (fetch)      
     output logic  [31:0] pc_d,        // retired program counter (decode)     
     output logic  [31:0] pc_e,        // retired program counter (execute)    
     output logic  [31:0] pc_m,        // retired program counter (memory)     
     output logic  [31:0] pc_wb        // retired program counter  (write back)

);
              
logic  jumpe, branche; 
logic  flag; 
  
/*              
always_ff @(negedge rstn_i)
begin
    jumpe   <= 1'b0;
    branche <= 1'b0;           //initial condition
end             
*/      

              
              
logic [31:0]instr,RD1,RD2, ImmExtD, PCTargetE, four, PCNext, ALUResult, ReadData, Target;             
logic [31:0] pcnext, result,target;
logic [19:15] a1;
logic [24:20] a2;
logic [11:7] a3;
logic ZeroE, PCSrcE;
 
logic  memwritee, alusrce, regwritee;  
logic  [3:0] alucontrole;
logic  [1:0] resultsrce;
logic  [31:0] rd,pcf,pcplus4f,pcd,pce,immexte,pcplus4e,rd1e,rd2e;   
logic  [31:0] InstrD, ResultW,PCF,PCD, PCPlus4F,PCPlus4D,PCPlus4W,ImmExtE, ALUResultM;
logic  [11:7] RdD,RdE,RdM,RdW;
logic  [19:15] rs1e;
logic  [24:20] rs2e;
logic  [11:7] rde; 


assign four =32'h4;
assign a1 = InstrD[19:15];
assign a2 = InstrD[24:20];
//assign a3 = InstrD[11:7];
assign InstrD = rd; 
assign PCD = pcf;
assign PCPlus4D = pcplus4f;
 
 
logic [31:0] srcae, srcbe, writedatae, aluresultw;
logic [31:0] WriteDataE, SrcAE,SrcBE, ALUResultW;

logic [3:0] ALUControlE; 
logic [31:0] WriteDataM, PCPlus4M;
logic RegWriteM, MemWriteM;
logic [14:12] Funct3M;
logic RegWriteW;
  
Instruction_Memory d0 (.A(PCF) , .RD(instr));  
 
Register_File d1(.A1(a1) , .A2(a2), .A3(RdW), .clk(clk), .WE3(RegWriteW), .WD3(ResultW), .RD1(RD1), .RD2(RD2), .reset(rstn_i) ); 
 
Extend d2 (.Instr(InstrD[31:7]), .ImmSrc(ImmSrc), .ImmExt(ImmExtD)); 
     
PC d3 (.PCNext(PCNext), .CLK(clk), .PC(PCF), .StallF(StallF), .reset(rstn_i));
 
ALU d4 (.A(SrcAE), .B(SrcBE), .F(ALUResult), .Zero(ZeroE),.op(ALUControlE));
 
Data_Memory d5 (.A(ALUResultM), .WD(WriteDataM), .CLK(clk), .WE(MemWriteM), .RD(ReadData), .funct3(Funct3M), .addr_i(addr_i), .data_o(data_o)); 

//---------------------------------------------
always_ff @(posedge clk or negedge rstn_i) begin
    
    if (!rstn_i) begin
        rd<=32'b0;
        pcplus4f<=32'b0;     //reset for pipe
        pcf <= 32'b0;               
    end
    
    else if(StallD) begin
    end
    
    else if (FlushD) begin
        rd<=32'b0;
        pcplus4f<=32'b0;     
        pcf <= 32'b0;                   //pipe1
    end
    
    else begin
        rd<=instr;
        pcplus4f<=PCPlus4F;
        pcf <= PCF;
    end
    
end   
//--------------------------------------------  
logic [1:0] targetsrce;   
logic RegWriteE, MemWriteE,JumpE,BranchE,ALUSrcE;
logic [1:0] ResultSrcE, TargetSrcE ;
logic [31:0] RD1E,RD2E,PCE,PCPlus4E;
logic [19:15] Rs1E;
logic [24:20] Rs2E;
logic [14:12] funct3e;
logic [14:12] Funct3E;

assign RegWriteE = regwritee;
assign MemWriteE = memwritee;
assign JumpE = jumpe;
assign BranchE = branche;
assign ALUSrcE = alusrce;
assign ResultSrcE = resultsrce;
assign ALUControlE = alucontrole;
assign RD1E =rd1e;
assign RD2E =rd2e;
assign PCE = pce;
assign ImmExtE = immexte;
assign PCPlus4E =pcplus4e;
assign Rs1E = rs1e;
assign Rs2E = rs2e;
assign RdE = rde;
assign TargetSrcE = targetsrce;
assign Funct3E = funct3e;
//-----------------------------------------------   

always_ff @(posedge clk or negedge rstn_i) begin

    if(!rstn_i) begin
        jumpe   <= 1'b0;
        branche <= 1'b0;           //initial condition for PCSrcE
    end
    
    
    else if(FlushE) begin
        pce<= 32'b0;
        rs1e<=5'b0;
        rs2e<=5'b0;
        rde<=5'b0;
        immexte<=32'b0;
        regwritee<=1'b0;
        resultsrce<=2'b0;
        memwritee<=1'b0;
        jumpe<=1'b0;
        branche<=1'b0;
        alucontrole<=4'b0;
        alusrce<=1'b0;
        pcplus4e<=32'b0;
        rd1e<=32'b0;
        rd2e<=32'b0;
        targetsrce<=2'b0;
        funct3e<=3'b0;
    end
    
    else begin
        pce<= PCD;
        rs1e<=InstrD[19:15];
        rs2e<=InstrD[24:20];
        rde<=InstrD[11:7];
        immexte<=ImmExtD;                   //pipe2
        regwritee<=RegWrite;
        resultsrce<=ResultSrc;
        memwritee<=MemWrite;
        jumpe<= Jump;
        branche<= Branch;
        alucontrole<=ALUControl;
        alusrce<=ALUSrc;
        pcplus4e<=PCPlus4D;
        rd1e<=RD1;
        rd2e<=RD2;
        targetsrce <= TargetSrc;
        funct3e<=InstrD[14:12];
    end
    
end   
   
 
logic regwritem ,memwritem;
logic [1:0] resultsrcm;
logic [1:0] ResultSrcM;
logic [31:0] aluresultm, writedatam,pcplus4m,pctargetm;
logic [11:7] rdm;
logic [14:12] funct3m;
logic [31:0] PCTargetM;
logic [31:0] pcm;
logic [31:0] PCM;
 
assign RegWriteM = regwritem;
assign ResultSrcM = resultsrcm;
assign MemWriteM = memwritem;
assign ALUResultM = aluresultm;
assign WriteDataM = writedatam;
assign RdM = rdm;
assign PCPlus4M = pcplus4m;
assign Funct3M = funct3m;
assign PCTargetM = pctargetm;
assign PCM = pcm;


//--------------------------------------------------------------- 
always_ff @(posedge clk or negedge rstn_i) begin
    
    if(!rstn_i) begin
    end
    
    else begin
        regwritem <= RegWriteE;                          //pipe3
        resultsrcm <=ResultSrcE;
        memwritem <= MemWriteE;
        aluresultm <= ALUResult;
        writedatam <= WriteDataE;
        rdm <=RdE;
        pcplus4m <=PCPlus4E;
        funct3m <= Funct3E;
        pctargetm <= PCTargetE;
        pcm <= PCE;
    end    
end
 
 //--------------------------------------------
 
 logic regwritew;
 logic [1:0] resultsrcw;
 logic [31:0] readdataw,pcplus4w,pctargetw;
 logic [11:7] rdw;
 logic [1:0] ResultSrcW;
 logic [31:0] ReadDataW,PCTargetW;
 logic [31:0] pcw;
 logic [31:0] PCW;
 
 
 
 assign RegWriteW = regwritew;
 assign ResultSrcW = resultsrcw;
 assign ReadDataW = readdataw;
 assign PCPlus4W = pcplus4w;
 assign PCTargetW = pctargetw;
 assign RdW = rdw;
 assign ALUResultW = aluresultw;
 assign PCW = pcw;
 
 //-------------------------------------------------------------
 always_ff @(posedge clk or negedge rstn_i) begin
 
    if(!rstn_i) begin
    end
 
    else begin
        regwritew <= RegWriteM;
        resultsrcw <= ResultSrcM;
        readdataw <= ReadData;
        pcplus4w <= PCPlus4M;          //pipe4
        pctargetw <= PCTargetM;  
        rdw<= RdM;  
        aluresultw <=ALUResultM; 
        pcw <=  PCM;               
    end
    
 end
 
 //-------------------------------------------------------------------------------

  
core_adder adder0 (.a_i(Target), .b_i(ImmExtE), .sum(PCTargetE));
core_adder adder1 (.a_i(PCF), .b_i(four), .sum(PCPlus4F));        
  
  
assign flag =   (Funct3E[12]) ? ~ZeroE  : ZeroE;
        
assign PCNext = pcnext;
assign ResultW = result;        
assign Target = target;
assign PCSrcE = (flag & BranchE) | JumpE ;


 
assign WriteDataE = writedatae;
assign SrcAE = srcae;
assign SrcBE = srcbe;
        
always_comb begin 

    /*
    case (Funct3E[12])
    1'b0: flag = ZeroE;
    1'b1: flag =~ZeroE;
    endcase
    */
    Instr = InstrD;      
    rs1d = InstrD[19:15];
    rs2d = InstrD[24:20];
    rs1_e = Rs1E;
    rs2_e = Rs2E;
    rd_e = RdE;
    pcsrce= PCSrcE;
    resultsrce_0 = ResultSrcE[0];
    rd_m = RdM;
    regwrite_m = RegWriteM;
    rd_w = RdW;
    regwrite_w = RegWriteW;
    //
    //pcnext = PCNext;
    //srcb = SrcB;
    //result = Result;
    
    
    
    case(ForwardAE)
        2'b00: srcae = RD1E;
        2'b01: srcae = ResultW;
        2'b10: srcae = ALUResultM;
    endcase
    
    case(ForwardBE)
        2'b00: writedatae = RD2E;
        2'b01: writedatae = ResultW;
        2'b10: writedatae = ALUResultM;
    endcase
    
    case(ALUSrcE)
        1'b0: srcbe = WriteDataE;
        1'b1: srcbe = ImmExtE;
    endcase
    
    
    case(PCSrcE) 
        1'b0 : pcnext = PCPlus4F;
        1'b1 : pcnext = PCTargetE; 
    endcase
                  
    
    case(ResultSrcW)
        2'b00: result = ALUResultW;           
        2'b01: result = ReadDataW;
        2'b10: result = PCPlus4W;    
        2'b11: result = PCTargetW;                  
    endcase         
    
    case(TargetSrcE)
        2'b00: target = 32'h0;
        2'b01: target = PCE;
        2'b10: target =SrcAE;
    endcase

end



//control outputs for testbench
//-----------------------------
assign reg_addr_o = RdW;
assign reg_data_o = ResultW;
assign mem_addr_o = ALUResultM;
assign mem_data_o = WriteDataM;
assign mem_wrt_o  = MemWriteM;
assign Instr_o    = instr;



assign pc_f  = PCF ;
assign pc_d  = PCD ;
assign pc_e  = PCE ;
assign pc_m  = PCM ;
assign pc_wb = PCW;

//-------------------------------     

 
endmodule

