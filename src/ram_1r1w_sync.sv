`ifndef BINPATH
`define BINPATH ""
`endif
`timescale 1ns/1ps

module ram_1r1w_sync #(
    parameter width_p = 8,
    parameter depth_p = 512,
    parameter [8*128-1:0] filename_p = "",
    parameter synth_init_file_p = 1'b0,
    parameter init_offset_p = 0,
    parameter init_count_p = depth_p,
    parameter init_is_float_p = 1'b0,
    parameter init_scale_p = 1,
    parameter init_signed_p = 1'b1
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
  localparam longint signed MAX_INIT_SIGNED   = (64'sd1 << (width_p - 1)) - 1;
  localparam longint signed MIN_INIT_SIGNED   = -(64'sd1 << (width_p - 1));
  localparam longint signed MAX_INIT_UNSIGNED = (64'sd1 << width_p) - 1;

  integer fd;
  integer scan_rc;
  integer i;
  integer loaded;
  real fval;
  integer ival;
  longint signed qval;

  initial begin
    for (i = 0; i < depth_p; i = i + 1)
      ram[i] = '0;

    if (filename_p != "") begin
      fd = $fopen(filename_p, "r");
      if (fd != 0) begin
        for (i = 0; i < init_offset_p; i = i + 1) begin
          if (init_is_float_p)
            scan_rc = $fscanf(fd, "%f\n", fval);
          else
            scan_rc = $fscanf(fd, "%d\n", ival);
          if (scan_rc != 1)
            i = init_offset_p;
        end

        loaded = 0;
        for (i = 0; i < depth_p && loaded < init_count_p; i = i + 1) begin
          if (init_is_float_p)
            scan_rc = $fscanf(fd, "%f\n", fval);
          else
            scan_rc = $fscanf(fd, "%d\n", ival);

          if (scan_rc == 1) begin
            if (init_is_float_p)
              qval = $rtoi(fval * init_scale_p);
            else
              qval = ival;

            if (init_signed_p) begin
              if (qval > MAX_INIT_SIGNED)
                qval = MAX_INIT_SIGNED;
              else if (qval < MIN_INIT_SIGNED)
                qval = MIN_INIT_SIGNED;
            end else begin
              if (qval < 0)
                qval = 0;
              else if (qval > MAX_INIT_UNSIGNED)
                qval = MAX_INIT_UNSIGNED;
            end

            ram[i] = qval[width_p-1:0];
            loaded = loaded + 1;
          end else begin
            i = depth_p;
          end
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
