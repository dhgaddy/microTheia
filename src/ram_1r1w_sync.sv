// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 Group G Contributors
`ifndef BINPATH
`define BINPATH ""
`endif
`timescale 1ns/1ps

module ram_1r1w_sync #(
    parameter width_p = 8,
    parameter depth_p = 512,
    parameter [8*128-1:0] filename_p = "",
    parameter synth_init_file_p = 1'b0
)(
    input [0:0] clk_i,
    input [0:0] reset_i,

    input [0:0] wr_valid_i,
    input [width_p-1:0] wr_data_i,
    input [$clog2(depth_p) - 1 : 0] wr_addr_i,

    input [0:0] rd_valid_i,
    input [$clog2(depth_p) - 1 : 0] rd_addr_i,
    output [width_p-1:0] rd_data_o
);

  logic [width_p-1:0] ram [depth_p-1:0];
  logic [width_p-1:0] rd_data_l;

`ifdef SYNTHESIS
  integer si;
  initial begin
    for (si = 0; si < depth_p; si = si + 1)
      ram[si] = '0;

    // Synthesis-friendly memory init path: use pre-quantized hex files.
    if (synth_init_file_p)
      $readmemh(filename_p, ram);
  end
`else
  integer fd;
  integer scan_rc;
  integer i;
  longint unsigned ival;

  initial begin
    for (i = 0; i < depth_p; i = i + 1)
      ram[i] = '0;

    if (filename_p != "") begin
      fd = $fopen(filename_p, "r");
      if (fd == 0)
        $warning("ram_1r1w_sync: could not open '%s' — RAM will be zero-initialised", filename_p);
      else begin
        for (i = 0; i < depth_p; i = i + 1) begin
          scan_rc = $fscanf(fd, "%h\n", ival);
          if (scan_rc != 1)
            i = depth_p;
          else
            ram[i] = ival[width_p-1:0];
        end
        $fclose(fd);
      end
    end
  end
`endif // SYNTHESIS / !SYNTHESIS

  // Keep read data register unreset so synthesis can map this to true block RAM.
  always_ff @(posedge clk_i) begin
`ifdef SYNTHESIS
    if (rd_valid_i) begin
      rd_data_l <= ram[rd_addr_i];
    end
`else
    // Deterministic simulation behavior while preserving BRAM inference in synthesis.
    if (reset_i) begin
      rd_data_l <= '0;
    end else if (rd_valid_i) begin
      rd_data_l <= ram[rd_addr_i];
    end
`endif
    if (wr_valid_i) begin
      ram[wr_addr_i] <= wr_data_i;
    end
  end

  assign rd_data_o = rd_data_l;
  wire _unused_reset = reset_i;

endmodule
