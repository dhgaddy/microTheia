// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

// Features emitted oldest->newest bins, row-major within each bin (y major, x minor).
//
// Counter memory is a GF180MCU SRAM (synchronous, 1-cycle read latency).
//
// Accumulate (ST_ACCUM) uses a 2-cycle read-modify-write pipeline:
//   Cycle N   : issue SRAM read for event address; block new events (event_ready=0).
//   Cycle N+1 : SRAM Q is old counter; write back (old+1, saturating); accept next event.
// Throughput: 1 event per 2 cycles (6 MHz at 12 MHz clock — sufficient for GenX320).
//
// Readout (ST_READOUT) issues sequential SRAM reads one cycle ahead; all output
// signals (readout_valid, readout_data, readout_index, readout_last) are registered
// one cycle after the corresponding read.  Readout takes FEATURE_COUNT + 1 cycles
// (one extra drain cycle to collect the final Q).

//
module voxel_binning #(
    parameter  int CLK_FREQ_HZ    = 12_000_000,
    parameter  int WINDOW_MS      = 1000,
    parameter  int GRID_SIZE      = 16,
    parameter  int NUM_BINS       = 8,
    parameter  int READOUT_BINS   = 8,
    parameter  int COUNTER_BITS   = 16,
    parameter  int CYCLES_PER_BIN = 0,
    localparam int RO_INDEX_WIDTH = READOUT_BINS*GRID_SIZE*GRID_SIZE
)(
    input  logic                              clk,
    input  logic                              rst,
    input  logic                              event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_x,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_y,
    output logic                              event_ready,
    input  logic                              readout_ready,
    output logic                              readout_start,
    output logic                              readout_valid,
    output logic [COUNTER_BITS-1:0]           readout_data,
    output logic [$clog2(RO_INDEX_WIDTH)-1:0] readout_index,
    output logic                              readout_last,
    output logic [30:0]                       vox_bin_dbg //debug bus, only 31 bits NOT 32
);

    localparam int CELLS_PER_BIN       = GRID_SIZE * GRID_SIZE;
    localparam int TOTAL_CELLS         = NUM_BINS * CELLS_PER_BIN;
    localparam int FEATURE_COUNT       = READOUT_BINS * CELLS_PER_BIN;
    localparam int BIN_BITS            = (NUM_BINS > 1) ? $clog2(NUM_BINS) : 1;
    localparam int CELL_BITS           = (CELLS_PER_BIN > 1) ? $clog2(CELLS_PER_BIN) : 1;
    localparam int BIN_COUNT_BITS      = $clog2(NUM_BINS + 1);
    localparam int MEM_ADDR_BITS       = $clog2(TOTAL_CELLS > 1 ? TOTAL_CELLS : 2);
    localparam int BIN_DURATION_MS     = WINDOW_MS / READOUT_BINS;
    localparam int CYCLES_PER_BIN_AUTO = (CLK_FREQ_HZ / 1000) * BIN_DURATION_MS;
    localparam int CYCLES_PER_BIN_USE  = (CYCLES_PER_BIN == 0) ? CYCLES_PER_BIN_AUTO : CYCLES_PER_BIN;
    localparam int CYCLES_PER_BIN_SAFE = (CYCLES_PER_BIN_USE < 1) ? 1 : CYCLES_PER_BIN_USE;
    localparam int TIMER_BITS          = (CYCLES_PER_BIN_SAFE > 1) ? $clog2(CYCLES_PER_BIN_SAFE) : 1;

    initial begin
        if (READOUT_BINS > NUM_BINS)
            $error("voxel_binning: READOUT_BINS (%0d) must be <= NUM_BINS (%0d)",
                   READOUT_BINS, NUM_BINS);
    end

    // ------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_ACCUM   = 2'd0,
        ST_WAIT_RD = 2'd1,
        ST_READOUT = 2'd2,
        ST_CLEAR   = 2'd3
    } state_t;

    state_t state;

    // ------------------------------------------------------------------
    // SRAM interface wires
    // ------------------------------------------------------------------
    logic                    sram_wr_valid;
    logic [MEM_ADDR_BITS-1:0] sram_wr_addr;
    logic [COUNTER_BITS-1:0] sram_wr_data;
    logic                    sram_rd_valid;
    logic [MEM_ADDR_BITS-1:0] sram_rd_addr;
    logic [COUNTER_BITS-1:0] sram_rd_data;

    gf180_sram_1r1w #(
        .width_p (COUNTER_BITS),
        .depth_p (TOTAL_CELLS)
    ) u_counter_mem (
        .clk_i      (clk),
        .reset_i    (rst),
        .wr_valid_i (sram_wr_valid),
        .wr_data_i  (sram_wr_data),
        .wr_addr_i  (sram_wr_addr),
        .rd_valid_i (sram_rd_valid),
        .rd_addr_i  (sram_rd_addr),
        .rd_data_o  (sram_rd_data)
    );

    // ------------------------------------------------------------------
    // Control registers
    // ------------------------------------------------------------------
    logic [TIMER_BITS-1:0]    timer_ctr;
    logic [BIN_BITS-1:0]      wr_bin_idx;
    logic [BIN_BITS-1:0]      clear_bin_idx;
    logic [BIN_BITS-1:0]      snapshot_start_bin;
    logic [BIN_COUNT_BITS-1:0] completed_bins;

    logic [BIN_BITS-1:0]  rd_bin_off;
    logic [CELL_BITS-1:0] rd_cell_idx;
    logic [CELL_BITS-1:0] clear_cell_idx;

    // RMW pipeline (ST_ACCUM)
    logic                    rmw_pending;
    logic [MEM_ADDR_BITS-1:0] rmw_addr;

    // Readout pipeline (ST_READOUT)
    logic rd_draining;   // extra drain cycle after last read issued

    // ------------------------------------------------------------------
    // Combinational address helpers
    // ------------------------------------------------------------------
    logic [BIN_BITS:0]         wr_bin_plus_1;
    logic [BIN_BITS-1:0]       next_wr_bin;
    logic [BIN_BITS:0]         start_calc;
    logic [BIN_COUNT_BITS-1:0] completed_bins_next;
    logic [BIN_BITS:0]         rd_bin_calc;
    logic [BIN_BITS-1:0]       rd_bin_idx;
    logic [CELL_BITS-1:0]      event_cell_idx;

    always_comb begin
        wr_bin_plus_1 = wr_bin_idx + 1'b1;
        next_wr_bin   = (wr_bin_plus_1 >= NUM_BINS)
                        ? BIN_BITS'(wr_bin_plus_1 - NUM_BINS)
                        : BIN_BITS'(wr_bin_plus_1);

        start_calc         = wr_bin_idx + NUM_BINS - (READOUT_BINS - 1);
        snapshot_start_bin = (start_calc >= NUM_BINS)
                             ? BIN_BITS'(start_calc - NUM_BINS)
                             : BIN_BITS'(start_calc);

        completed_bins_next = (completed_bins < NUM_BINS)
                              ? BIN_COUNT_BITS'(completed_bins + 1)
                              : completed_bins;

        rd_bin_calc = snapshot_start_bin + rd_bin_off;
        rd_bin_idx  = (rd_bin_calc >= NUM_BINS)
                      ? BIN_BITS'(rd_bin_calc - NUM_BINS)
                      : BIN_BITS'(rd_bin_calc);

        event_cell_idx = CELL_BITS'(event_y * GRID_SIZE + event_x);
    end

    // ------------------------------------------------------------------
    // SRAM control mux
    //   ST_ACCUM  : RD on event arrival (rmw read), WR on rmw writeback
    //   ST_READOUT: RD each cycle (sequential scan), no WR
    //   ST_CLEAR  : WR each cycle (zero-fill), no RD
    // ------------------------------------------------------------------
    logic [MEM_ADDR_BITS-1:0] rd_addr_current;
    logic [MEM_ADDR_BITS-1:0] cl_addr_current;

    assign rd_addr_current = MEM_ADDR_BITS'(rd_bin_idx  * CELLS_PER_BIN + rd_cell_idx);
    assign cl_addr_current = MEM_ADDR_BITS'(clear_bin_idx * CELLS_PER_BIN + clear_cell_idx);

    always_comb begin
        sram_rd_valid = 1'b0;
        sram_rd_addr  = '0;
        sram_wr_valid = 1'b0;
        sram_wr_addr  = '0;
        sram_wr_data  = '0;

        case (state)
            ST_ACCUM: begin
                // RMW read: issue when event arrives and no writeback is pending
                sram_rd_valid = event_valid && !rmw_pending;
                sram_rd_addr  = MEM_ADDR_BITS'(wr_bin_idx * CELLS_PER_BIN + event_cell_idx);
                // RMW writeback: one cycle after the read
                sram_wr_valid = rmw_pending;
                sram_wr_addr  = rmw_addr;
                sram_wr_data  = (sram_rd_data == {COUNTER_BITS{1'b1}})
                                ? sram_rd_data          // saturate
                                : sram_rd_data + 1'b1;
            end

            ST_READOUT: begin
                // Sequential read; stop issuing new reads during drain cycle
                sram_rd_valid = !rd_draining;
                sram_rd_addr  = rd_addr_current;
            end

            ST_CLEAR: begin
                sram_wr_valid = 1'b1;
                sram_wr_addr  = cl_addr_current;
                sram_wr_data  = '0;
            end

            default: ;
        endcase
    end

    // ------------------------------------------------------------------
    // event_ready: block during RMW writeback cycle
    // ------------------------------------------------------------------
    assign event_ready = (state == ST_ACCUM) && !rmw_pending;

    // ------------------------------------------------------------------
    // Readout output pipeline (1-cycle delay to align with SRAM latency)
    // ------------------------------------------------------------------
    logic                              rd_pipe_valid;
    logic [$clog2(RO_INDEX_WIDTH)-1:0] rd_pipe_index;
    logic                              rd_pipe_last;

    assign readout_valid = rd_pipe_valid;
    assign readout_data  = sram_rd_data;   // SRAM Q — 1-cycle latency from rd_valid
    assign readout_index = rd_pipe_index;
    assign readout_last  = rd_pipe_last;

    always_ff @(posedge clk) begin
        if (rst) begin
            rd_pipe_valid <= 1'b0;
            rd_pipe_index <= '0;
            rd_pipe_last  <= 1'b0;
        end else begin
            rd_pipe_valid <= sram_rd_valid && (state == ST_READOUT);
            rd_pipe_index <= $clog2(RO_INDEX_WIDTH)'(rd_bin_off * CELLS_PER_BIN + rd_cell_idx);
            rd_pipe_last  <= (rd_bin_off  == BIN_BITS'(READOUT_BINS - 1)) &&
                             (rd_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1));
        end
    end

    // ------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state          <= ST_CLEAR;
            timer_ctr      <= '0;
            wr_bin_idx     <= '0;
            clear_bin_idx  <= '0;
            completed_bins <= '0;
            rd_bin_off     <= '0;
            rd_cell_idx    <= '0;
            clear_cell_idx <= '0;
            readout_start  <= 1'b0;
            rmw_pending    <= 1'b0;
            rmw_addr       <= '0;
            rd_draining    <= 1'b0;
        end else begin
            readout_start <= 1'b0;

            case (state)
                // ----------------------------------------------------------
                ST_ACCUM: begin
                    // RMW pipeline
                    if (event_valid && !rmw_pending) begin
                        // Cycle N: SRAM read issued (see sram_rd_valid above)
                        rmw_addr    <= MEM_ADDR_BITS'(wr_bin_idx * CELLS_PER_BIN
                                                      + event_cell_idx);
                        rmw_pending <= 1'b1;
                    end
                    if (rmw_pending) begin
                        // Cycle N+1: SRAM write issued (see sram_wr_valid above)
                        rmw_pending <= 1'b0;
                    end

                    // Bin timer
                    if (timer_ctr == TIMER_BITS'(CYCLES_PER_BIN_SAFE - 1)) begin
                        timer_ctr      <= '0;
                        clear_bin_idx  <= next_wr_bin;
                        completed_bins <= completed_bins_next;

                        if (completed_bins_next >= BIN_COUNT_BITS'(READOUT_BINS)) begin
                            if (readout_ready) begin
                                state         <= ST_READOUT;
                                rd_bin_off    <= '0;
                                rd_cell_idx   <= '0;
                                rd_draining   <= 1'b0;
                                readout_start <= 1'b1;
                            end else begin
                                state <= ST_WAIT_RD;
                            end
                        end else begin
                            state          <= ST_CLEAR;
                            clear_cell_idx <= '0;
                        end
                    end else begin
                        timer_ctr <= timer_ctr + 1'b1;
                    end
                end

                // ----------------------------------------------------------
                ST_WAIT_RD: begin
                    if (readout_ready) begin
                        state         <= ST_READOUT;
                        rd_bin_off    <= '0;
                        rd_cell_idx   <= '0;
                        rd_draining   <= 1'b0;
                        readout_start <= 1'b1;
                    end
                end

                // ----------------------------------------------------------
                ST_READOUT: begin
                    if (rd_draining) begin
                        // Final SRAM read result arrives this cycle (rd_pipe_valid asserted
                        // via the registered path above); now move on.
                        rd_draining    <= 1'b0;
                        state          <= ST_CLEAR;
                        clear_cell_idx <= '0;
                    end else begin
                        // Determine whether this is the last address to issue
                        if ((rd_bin_off  == BIN_BITS'(READOUT_BINS - 1)) &&
                            (rd_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1))) begin
                            // Last read issued this cycle; drain next cycle
                            rd_draining <= 1'b1;
                        end else if (rd_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1)) begin
                            rd_cell_idx <= '0;
                            rd_bin_off  <= rd_bin_off + 1'b1;
                        end else begin
                            rd_cell_idx <= rd_cell_idx + 1'b1;
                        end
                    end
                end

                // ----------------------------------------------------------
                ST_CLEAR: begin
                    // SRAM write issued combinatorially above.
                    if (clear_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1)) begin
                        clear_cell_idx <= '0;
                        wr_bin_idx     <= clear_bin_idx;
                        state          <= ST_ACCUM;
                    end else begin
                        clear_cell_idx <= clear_cell_idx + 1'b1;
                    end
                end

                default: state <= ST_ACCUM;
            endcase
        end
    end

    //debug bus connections
    assign vox_bin_dbg[0] = event_ready;
    assign vox_bin_dbg[1] = readout_start;
    assign vox_bin_dbg[2] = readout_valid;
    assign vox_bin_dbg[3] = readout_last;
    assign vox_bin_dbg[14:4] =  readout_index;
    assign vox_bin_dbg[30:15] = readout_data;

endmodule
