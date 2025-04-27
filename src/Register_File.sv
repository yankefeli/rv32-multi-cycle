`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.04.2025 22:28:46
// Design Name: 
// Module Name: Register_File
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

module Register_File
(
    input  logic [4:0]  A1, A2, A3,       // Read and Write Addresses
    input  logic        clk,              // Clock
    input  logic        WE3,              // Write Enable
    input  logic        reset,            // Active-low Reset
    input  logic [31:0] WD3,               // Write Data
    output logic [31:0] RD1, RD2           // Read Data Outputs
);

  logic [31:0] file [31:0];                // Register File Memory
  integer i;

  // Synchronous write: Write on negedge clk or reset
  always_ff @(negedge clk or negedge reset) begin
    if (!reset) begin
      for (i = 0; i <= 31; i = i + 1)
        file[i] <= 32'd0;
    end
    else begin
      if (WE3 && (A3 != 5'd0))
        file[A3] <= WD3;
    end
  end

  // Combinational read: Support write forwarding
  always_comb begin
    // x0 register is hardwired to zero
    file[5'd0] = 32'd0;

    // Read Port 1
    if (WE3 && (A1 == A3) && (A1 != 5'd0))
      RD1 = WD3;
    else
      RD1 = file[A1];

    // Read Port 2
    if (WE3 && (A2 == A3) && (A2 != 5'd0))
      RD2 = WD3;
    else
      RD2 = file[A2];
  end

endmodule
