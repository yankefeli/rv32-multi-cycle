`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.04.2025 16:25:21
// Design Name: 
// Module Name: Hazard_Unit
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


module Hazard_Unit
( 
    input  logic  [19:15] rs1d,rs1e,
    input  logic  [24:20] rs2d,rs2e,
    input  logic  [11:7]  rde, rdm, rdw,
    input  logic          resultsrce0,regwritem,regwritew, pcsrce,
  //input  logic          rstn_i,
    output logic          StallF, StallD, FlushD, FlushE,
    output logic  [1:0]   ForwardAE, ForwardBE
);
    
logic lwstall;  

/*
always_ff @(negedge rstn_i) begin
    StallF     <= 1'b0;
    StallD     <= 1'b0;
    FlushD     <= 1'b1;        //initial ?
    FlushE     <= 1'b1;
    ForwardAE  <= 2'b00;
    ForwardBE  <= 2'b00;
end
*/
 
always_comb begin
    
    if (((rs1e == rdm) &&  regwritem) && (rs1e != 0)) ForwardAE = 2'b10;
       
    else if (((rs1e == rdw) && regwritew) && (rs1e != 0)) ForwardAE = 2'b01;
    
    else ForwardAE = 00; 
        
    lwstall = resultsrce0 && ((rs1d ==rde) || (rs2d==rde));  
    StallF = lwstall;
    StallD = lwstall;
    FlushD = pcsrce;
    FlushE = lwstall | pcsrce;
end
    

always_comb begin
        
    if (((rs2e == rdm) &&  regwritem) && (rs2e != 0)) ForwardBE = 2'b10;
       
    else if (((rs2e == rdw) && regwritew) && (rs2e != 0)) ForwardBE = 2'b01;
    
    else ForwardBE = 00; 
end

endmodule
