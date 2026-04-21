// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`ifndef BINPATH
 `define BINPATH ""
`endif
//this is my ram module from CSE 225
// i think it will map to registers if we use it for the chip
// used for a dual clock port cdc fifo that is register based, because gf180 sram macro is single clock port

module ram_1r1w_sync
  #(parameter [31:0] width_p = 8
  ,parameter [31:0] depth_p = 512
  /* verilator lint_off UNUSEDPARAM */
  ,parameter string filename_p = "memory_init_file.bin")
  /* verilator lint_on UNUSEDPARAM */
  (input [0:0] cclk_i
  ,input [0:0] pclk_i
  ,input [0:0] reset_i

  ,input [0:0] wr_valid_i
  ,input [width_p-1:0] wr_data_i
  ,input [$clog2(depth_p) - 1 : 0] wr_addr_i

  ,input [0:0] rd_valid_i
  ,input [$clog2(depth_p) - 1 : 0] rd_addr_i
  ,output [width_p-1:0] rd_data_o);
  logic [width_p-1:0] mem [depth_p-1:0];
  logic [width_p-1:0] rd_data_o_w;
   initial begin
      // Display depth and width (must match these in init file)
      $display("%m: depth_p is %d, width_p is %d", depth_p, width_p);
      // wire [bar:0] foo [baz:0];
      // to get the memory contents in iverilog run this for loop during initialization:
      for (int i = 0; i < depth_p; i++) begin
        // $dumpvars(0,mem);
        ;
      end
   end

   always_ff @(posedge cclk_i) begin 
    if (wr_valid_i & ~reset_i) begin
      mem[wr_addr_i] <= wr_data_i;
    end
   end 
    always_ff@(posedge pclk_i) begin
      if(rd_valid_i) begin
      rd_data_o_w <= mem[rd_addr_i];
    end  
   end      
  assign rd_data_o = rd_data_o_w;
endmodule