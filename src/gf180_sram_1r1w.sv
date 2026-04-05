// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 Group G Contributors
`timescale 1ns/1ps

// GF180MCU synchronous SRAM wrapper — single-read / single-write port.
//
// Tiling:
//   Width : BYTES = ceil(width_p/8) byte-lane macros per bank.
//   Depth : Smallest OCD 3.3V macro that fits depth_p in one bank (256, 512, or
//           1024); for depth_p beyond that, up to 4 banks are cascaded (supports
//           depth_p up to 4096 — largest use in this project is 2048, 2 banks).
//
// Read latency  : 1 cycle (synchronous — Q registered on CLK rising edge).
// Write latency : 0 cycles (write takes effect on CLK rising edge).
// Simultaneous read+write to the same bank: write wins (GWEN=0 overrides read).
//
// CEN protocol: CEN is held HIGH (disabled) while reset_i is asserted so the
//   GF180 model sees the required 1→0 falling edge on the first real access.
//   Each bank is only enabled (CEN=0) during its own active operation; all other
//   banks are disabled (CEN=1) every cycle — standard power-saving practice.
//
// Init: GF180 macros initialise to 0x00 at simulation start.  Weights and
//   thresholds are written into their SRAMs by the init FSM in voxel_bin_core
//   immediately after reset; the filename_p parameter is retained for interface
//   compatibility but is not used.
//
// Simulation note: GF180 macro models use `always @(CLK) clk_dly = #100ps CLK`
//   and evaluate read_flag/write_flag at `posedge clk_dly`.  cocotb NextTimeStep
//   fires at CLK+delta (<100ps), so FF outputs update before clk_dly fires and
//   CEN returns to 1, silently suppressing every access.  The `ifndef SYNTHESIS
//   block below is a cycle-accurate behavioral replacement for simulation.

module gf180_sram_1r1w #(
    parameter int             width_p    = 8,
    parameter int             depth_p    = 512,
    parameter [8*128-1:0]     filename_p = ""
)(
    input  logic                          clk_i,
    input  logic                          reset_i,

    input  logic                          wr_valid_i,
    input  logic [width_p-1:0]            wr_data_i,
    input  logic [$clog2(depth_p)-1:0]    wr_addr_i,

    input  logic                          rd_valid_i,
    input  logic [$clog2(depth_p)-1:0]    rd_addr_i,
    output logic [width_p-1:0]            rd_data_o
);

    // ------------------------------------------------------------------
    // Tiling parameters (used by both simulation and synthesis paths)
    // ------------------------------------------------------------------
    localparam int BYTES      = (width_p + 7) / 8;

    // OCD 3.3V SRAM set: 256×8, 512×8, 1024×8 (no 64 or 128 variants).
    localparam int MACRO_DEPTH = (depth_p <= 256) ? 256  :
                                  (depth_p <= 512) ? 512  : 1024;
    localparam int MACRO_ABITS = (MACRO_DEPTH == 256) ? 8 :
                                  (MACRO_DEPTH == 512) ? 9 : 10;
    localparam int NUM_BANKS  = (depth_p + MACRO_DEPTH - 1) / MACRO_DEPTH;
    localparam int ADDR_BITS  = $clog2(depth_p > 1 ? depth_p : 2);

`ifndef SYNTHESIS
    // ------------------------------------------------------------------
    // Behavioral simulation model — identical 1-cycle read latency.
    // Avoids the clk_dly timing incompatibility of the GF180 macro models.
    // ------------------------------------------------------------------
    logic [width_p-1:0] sim_mem [0:depth_p-1];
    logic [width_p-1:0] sim_rd_data_r;

    // Match GF180 macro sim behaviour: memory initialises to 0x00.
    integer sim_init_i;
    initial begin
        for (sim_init_i = 0; sim_init_i < depth_p; sim_init_i = sim_init_i + 1)
            sim_mem[sim_init_i] = '0;
    end

    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            sim_rd_data_r <= '0;
        end else begin
            if (wr_valid_i)
                sim_mem[wr_addr_i] <= wr_data_i;
            if (rd_valid_i)
                sim_rd_data_r <= sim_mem[rd_addr_i];
        end
    end

    assign rd_data_o = sim_rd_data_r;

`else
    // ------------------------------------------------------------------
    // Synthesis path — real GF180MCU SRAM macro tiles
    // ------------------------------------------------------------------

    // Address decomposition
    //   Bank index  : upper (ADDR_BITS - MACRO_ABITS) bits (0 if single bank)
    //   Within-bank : lower MACRO_ABITS bits (zero-extended if depth_p < MACRO_DEPTH)
    logic [1:0]              wr_bank, rd_bank;   // 2 bits → up to 4 banks
    logic [MACRO_ABITS-1:0]  wr_addr_low, rd_addr_low;

    generate
        if (NUM_BANKS > 1) begin : g_multi_addr
            assign wr_bank     = 2'(wr_addr_i[ADDR_BITS-1:MACRO_ABITS]);
            assign rd_bank     = 2'(rd_addr_i[ADDR_BITS-1:MACRO_ABITS]);
            assign wr_addr_low = wr_addr_i[MACRO_ABITS-1:0];
            assign rd_addr_low = rd_addr_i[MACRO_ABITS-1:0];
        end else begin : g_single_addr
            assign wr_bank     = 2'd0;
            assign rd_bank     = 2'd0;
            assign wr_addr_low = MACRO_ABITS'(wr_addr_i);
            assign rd_addr_low = MACRO_ABITS'(rd_addr_i);
        end
    endgenerate

    // Registered bank select — needed to mux Q one cycle after the read
    logic [1:0] rd_bank_r;
    always_ff @(posedge clk_i) begin
        if (reset_i)        rd_bank_r <= 2'd0;
        else if (rd_valid_i) rd_bank_r <= rd_bank;
    end

    // Q outputs from every tile: [bank][byte_lane]
    logic [7:0] q_all [NUM_BANKS][BYTES];

    // Assemble wide read-data from the registered bank, then trim to width_p
    logic [BYTES*8-1:0] rd_data_wide;
    always_comb begin
        rd_data_wide = '0;
        for (int i = 0; i < BYTES; i++)
            rd_data_wide[i*8 +: 8] = q_all[rd_bank_r][i];
    end
    assign rd_data_o = rd_data_wide[width_p-1:0];

    // Zero-pad write data to a whole number of bytes
    logic [BYTES*8-1:0] wr_data_pad;
    assign wr_data_pad = {{(BYTES*8 - width_p){1'b0}}, wr_data_i};

    // Supply nets for inout VDD/VSS macro ports (Icarus requires wires, not literals)
    wire sram_vdd = 1'b1;
    wire sram_vss = 1'b0;

    // Macro tiles: NUM_BANKS banks × BYTES byte-lane macros
    genvar b, byte_i;
    generate
        for (b = 0; b < NUM_BANKS; b++) begin : gen_bank
            // Per-bank address: write address when writing to this bank,
            // read address otherwise.
            logic [MACRO_ABITS-1:0] addr_b;
            assign addr_b = (wr_valid_i & (wr_bank == 2'(b)))
                            ? wr_addr_low : rd_addr_low;

            for (byte_i = 0; byte_i < BYTES; byte_i++) begin : gen_byte
                logic cen_w, gwen_w;

                // CEN=0 (active) when this bank has a valid operation.
                // CEN=1 during reset or when bank is idle.
                assign cen_w  = reset_i | ~(
                                    (wr_valid_i & (wr_bank == 2'(b))) |
                                    (rd_valid_i & (rd_bank == 2'(b)))
                                );
                // GWEN=0 = write; GWEN=1 = read.  Write wins if both target same bank.
                assign gwen_w = ~(wr_valid_i & (wr_bank == 2'(b)));

                if (MACRO_DEPTH == 256) begin : sel
                    gf180mcu_ocd_ip_sram__sram256x8m8wm1 u_sram (
                        .CLK (clk_i),
                        .CEN (cen_w),
                        .GWEN(gwen_w),
                        .WEN (8'h00),           // all bits writable
                        .A   (addr_b[7:0]),
                        .D   (wr_data_pad[byte_i*8 +: 8]),
                        .Q   (q_all[b][byte_i]),
                        .VDD (sram_vdd),
                        .VSS (sram_vss)
                    );
                end else if (MACRO_DEPTH == 512) begin : sel
                    gf180mcu_ocd_ip_sram__sram512x8m8wm1 u_sram (
                        .CLK (clk_i),
                        .CEN (cen_w),
                        .GWEN(gwen_w),
                        .WEN (8'h00),
                        .A   (addr_b[8:0]),
                        .D   (wr_data_pad[byte_i*8 +: 8]),
                        .Q   (q_all[b][byte_i]),
                        .VDD (sram_vdd),
                        .VSS (sram_vss)
                    );
                end else begin : sel  // MACRO_DEPTH == 1024
                    gf180mcu_ocd_ip_sram__sram1024x8m8wm1 u_sram (
                        .CLK (clk_i),
                        .CEN (cen_w),
                        .GWEN(gwen_w),
                        .WEN (8'h00),
                        .A   (addr_b[9:0]),
                        .D   (wr_data_pad[byte_i*8 +: 8]),
                        .Q   (q_all[b][byte_i]),
                        .VDD (sram_vdd),
                        .VSS (sram_vss)
                    );
                end
            end
        end
    endgenerate

`endif // SYNTHESIS

endmodule
