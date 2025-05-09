`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 25.04.2025 17:05:32
// Design Name: 
// Module Name: tb
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
module tb();

  logic [31:0] addr;
  logic [31:0] data;
  logic [31:0] instr;
  logic [4:0]  reg_addr;
  logic [31:0] reg_data;
  logic [31:0] mem_addr;
  logic [31:0] mem_data;
  logic        mem_wrt;
  
  logic        update;
  logic        update_model;

  logic        clk;
  logic        rstn;

  logic [31:0] pc_f;
  logic [31:0] pc_d;
  logic [31:0] pc_e;
  logic [31:0] pc_m;
  logic [31:0] pc_wb;

  logic        StallF;
  logic        StallD;
  logic        FlushD;
  logic        FlushE;

  riscv_multicycle
  i_core_model
  (
      .clk_i(clk),
      .rstn_i(rstn),
      .addr_i(addr),

      .update_o(update),
      .update_model(update_model),

      .data_o(data),
      .Instr_o(instr),

      .reg_addr_o(reg_addr),
      .reg_data_o(reg_data),
      .mem_addr_o(mem_addr),
      .mem_data_o(mem_data),
      .mem_wrt_o(mem_wrt),

      .pc_f(pc_f),
      .pc_d(pc_d),
      .pc_e(pc_e),
      .pc_m(pc_m),
      .pc_wb(pc_wb),

      .StallF(StallF),
      .StallD(StallD),

      .FlushD(FlushD),
      .FlushE(FlushE)
  );

 integer file_pointer;
integer cycle_counter;
integer final_cycles;

bit update_fell;
bit stall_write;

initial begin
  file_pointer = $fopen("pipe.log", "w");
  $fwrite(file_pointer, "Cycle      F           D           E           M           WB\n");

  cycle_counter = 1;
  final_cycles = 0;
  update_fell = 0;
  stall_write = 0;

  #2

  forever begin
    #2;

    if (update) begin
      $fwrite(file_pointer, "%5d  ", cycle_counter);

      if (cycle_counter == 1)
        $fwrite(file_pointer, "0x%08h\n", pc_f);

      else if (cycle_counter == 2)
        $fwrite(file_pointer, "0x%08h  0x%08h\n", pc_f, pc_d);

      else if (cycle_counter == 3)
        $fwrite(file_pointer, "0x%08h  0x%08h  0x%08h\n", pc_f, pc_d, pc_e);

      else if (cycle_counter == 4)
        $fwrite(file_pointer, "0x%08h  0x%08h  0x%08h  0x%08h\n", pc_f, pc_d, pc_e, pc_m);

      else if (stall_write) begin
        $fwrite(file_pointer, "Stall     ");
        $fwrite(file_pointer, "  Stall     ");
        $fwrite(file_pointer, "  Flushed     ");
        $fwrite(file_pointer, "0x%08h  ", pc_m);
        $fwrite(file_pointer, "0x%08h\n", pc_wb);

        stall_write = 0;
      end

      else begin
        if (StallD || StallF)
          stall_write = 1;

        if (pc_f == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_f);

        if (pc_d == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_d);

        if (pc_e == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_e);

        if (pc_m == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_m);

        if (pc_wb == 32'h0)
          $fwrite(file_pointer, "Flushed     \n");
        else
          $fwrite(file_pointer, "0x%08h\n", pc_wb);
      end

      cycle_counter = cycle_counter + 1;
    end

    // BURAYA EKSİK OLANI EKLİYORUZ:
    if (!update && !update_fell) begin
      update_fell = 1;
      final_cycles = 4;  // <------ Eksik olan satır! Artık tamam
    end

    if (update_fell && (final_cycles > 0)) begin
      $fwrite(file_pointer, "%5d  ", cycle_counter);

      if (final_cycles == 4) begin
        $fwrite(file_pointer, "            "); // F boşluk
        if (pc_d == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_d);

        if (pc_e == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_e);

        if (pc_m == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_m);

        if (pc_wb == 32'h0)
          $fwrite(file_pointer, "Flushed     \n");
        else
          $fwrite(file_pointer, "0x%08h\n", pc_wb);
      end

      else if (final_cycles == 3) begin
        $fwrite(file_pointer, "                        "); // F, D boşluk
        if (pc_e == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_e);

        if (pc_m == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_m);

        if (pc_wb == 32'h0)
          $fwrite(file_pointer, "Flushed     \n");
        else
          $fwrite(file_pointer, "0x%08h\n", pc_wb);
      end

      else if (final_cycles == 2) begin
        $fwrite(file_pointer, "                                    "); // F, D, E boşluk
        if (pc_m == 32'h0)
          $fwrite(file_pointer, "Flushed     ");
        else
          $fwrite(file_pointer, "0x%08h  ", pc_m);

        if (pc_wb == 32'h0)
          $fwrite(file_pointer, "Flushed     \n");
        else
          $fwrite(file_pointer, "0x%08h\n", pc_wb);
      end

      else if (final_cycles == 1) begin
        $fwrite(file_pointer, "                                                "); // F, D, E, M boşluk
        if (pc_wb == 32'h0)
          $fwrite(file_pointer, "Flushed     \n");
        else
          $fwrite(file_pointer, "0x%08h\n", pc_wb);
      end

      final_cycles = final_cycles - 1;
      cycle_counter = cycle_counter + 1;
    end

  end
end


integer file_pointer_model;
integer write_counter = 0; // Yeni sayaç

initial begin

    file_pointer_model = $fopen("model.log", "w");
    #4

    forever begin
        if (update_model) begin

            if (write_counter >= 3) begin // sadece 3 cycle geçtikten sonra yaz

                if (reg_addr == 0 && mem_wrt ==0) begin  //branch
                    $fwrite(file_pointer_model, "0x%8h (0x%8h)", pc_wb, instr);
                end 
                
                else if (!mem_wrt && instr[6:0] != 7'b0000011) begin //not writing to datamem and not load
                
                    if (reg_addr > 9) begin
                        $fwrite(file_pointer_model, "0x%8h (0x%8h) x%0d 0x%8h", pc_wb, instr, reg_addr, reg_data);
                    end 
                    
                    else begin
                        $fwrite(file_pointer_model, "0x%8h (0x%8h) x%0d  0x%8h", pc_wb, instr, reg_addr, reg_data);
                    end
                    
                end
                
                else if (mem_wrt == 1) begin  //store
                    $fwrite(file_pointer_model, "0x%8h (0x%8h) mem 0x%8h 0x%8h",pc_wb, instr,  mem_addr, mem_data);
                end
                
                
                else if (instr[6:0] == 7'b0000011) begin  //load to register  (from memory)
                    $fwrite(file_pointer_model, "0x%8h (0x%8h) x%0d 0x%8h mem 0x%8h"
                    , pc_wb, instr, reg_addr, reg_data, mem_addr);    
                end

                $fwrite(file_pointer_model, "\n");

            end

            // Sayaç sadece 3'ten küçükken artacak
            if (write_counter < 3)
                write_counter = write_counter + 1;

        end

        #2;
    end
end
 



  initial forever begin
    clk = 0;
    #1;
    clk = 1;
    #1;
  end

  initial begin
    rstn = 0;
    #4;
    rstn = 1;
    #10000;

    for (logic [31:0] i = 32'h8000_0000; i < 32'h8000_0000 + 'h20; i = i + 4) begin
      addr = i;
      #4;
      $display("data @ mem[0x%8h] = %8h", addr, data);
    end

    $finish;
  end

  initial begin
    $dumpfile("dump.vcd");
    $dumpvars();
  end

endmodule
