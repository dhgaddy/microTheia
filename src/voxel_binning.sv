`timescale 1ns/1ps

// Temporal voxel histogram generator.
// - Accumulates events in a NUM_BINS ring of GRID_SIZE x GRID_SIZE counters.
// - Rotates bins every fixed period (clock-based).
// - Emits READOUT_BINS*GRID_SIZE*GRID_SIZE flattened features in strict order:
//   oldest->newest bins, and row-major within each bin (y major, x minor).

module voxel_binning #(
    parameter int CLK_FREQ_HZ    = 12_000_000,
    parameter int WINDOW_MS      = 1000,
    parameter int NUM_BINS       = 8,
    parameter int READOUT_BINS   = 8,
    parameter int GRID_SIZE      = 16,
    parameter int COUNTER_BITS   = 16,
    parameter int CYCLES_PER_BIN = 0
)(
    input  logic                           clk,
    input  logic                           rst,
    input  logic                           event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0]  event_x,
    input  logic [$clog2(GRID_SIZE)-1:0]  event_y,
    input  logic                           event_polarity,
    output logic                           event_ready,
    input  logic                           readout_ready,
    output logic                           readout_start,
    output logic                           readout_valid,
    output logic [COUNTER_BITS-1:0]        readout_data,
    output logic [$clog2(READOUT_BINS*GRID_SIZE*GRID_SIZE)-1:0] readout_index,
    output logic                           readout_last
);

    localparam int CELLS_PER_BIN      = GRID_SIZE * GRID_SIZE;
    localparam int TOTAL_CELLS        = NUM_BINS * CELLS_PER_BIN;
    localparam int FEATURE_COUNT      = READOUT_BINS * CELLS_PER_BIN;
    localparam int BIN_BITS           = (NUM_BINS > 1) ? $clog2(NUM_BINS) : 1;
    localparam int CELL_BITS          = (CELLS_PER_BIN > 1) ? $clog2(CELLS_PER_BIN) : 1;
    localparam int BIN_COUNT_BITS     = $clog2(NUM_BINS + 1);
    localparam int BIN_DURATION_MS    = WINDOW_MS / READOUT_BINS;
    localparam int CYCLES_PER_BIN_AUTO = (CLK_FREQ_HZ / 1000) * BIN_DURATION_MS;
    localparam int CYCLES_PER_BIN_USE = (CYCLES_PER_BIN == 0) ? CYCLES_PER_BIN_AUTO : CYCLES_PER_BIN;
    localparam int CYCLES_PER_BIN_SAFE = (CYCLES_PER_BIN_USE < 1) ? 1 : CYCLES_PER_BIN_USE;
    localparam int TIMER_BITS         = (CYCLES_PER_BIN_SAFE > 1) ? $clog2(CYCLES_PER_BIN_SAFE) : 1;

    typedef enum logic [1:0] {
        ST_ACCUM      = 2'd0,
        ST_WAIT_RD    = 2'd1,
        ST_READOUT    = 2'd2,
        ST_CLEAR      = 2'd3
    } state_t;

    state_t state;

    logic [COUNTER_BITS-1:0] mem [0:TOTAL_CELLS-1];

    logic [TIMER_BITS-1:0] timer_ctr;
    logic [BIN_BITS-1:0]   wr_bin_idx;
    logic [BIN_BITS-1:0]   clear_bin_idx;
    logic [BIN_BITS-1:0]   snapshot_start_bin;
    logic [BIN_COUNT_BITS-1:0] completed_bins;

    logic [BIN_BITS-1:0] rd_bin_off;
    logic [CELL_BITS-1:0] rd_cell_idx;
    logic [CELL_BITS-1:0] clear_cell_idx;

    logic [BIN_BITS:0] wr_bin_plus_1;
    logic [BIN_BITS-1:0] next_wr_bin;
    logic [BIN_BITS:0] start_calc;
    logic [BIN_COUNT_BITS-1:0] completed_bins_next;

    logic [BIN_BITS:0] rd_bin_calc;
    logic [BIN_BITS-1:0] rd_bin_idx;
    logic [CELL_BITS+$clog2(GRID_SIZE)-1:0] event_cell_math;
    logic [CELL_BITS-1:0] event_cell_idx;

    integer idx;
    initial begin
        if (READOUT_BINS > NUM_BINS)
            $error("voxel_binning: READOUT_BINS (%0d) must be <= NUM_BINS (%0d)", READOUT_BINS, NUM_BINS);
        for (idx = 0; idx < TOTAL_CELLS; idx = idx + 1)
            mem[idx] = '0;
    end

    assign event_cell_math = (event_y * GRID_SIZE) + event_x;
    assign event_cell_idx  = event_cell_math[CELL_BITS-1:0];

    always_comb begin
        wr_bin_plus_1 = wr_bin_idx + 1'b1;
        if (wr_bin_plus_1 >= NUM_BINS)
            next_wr_bin = wr_bin_plus_1 - NUM_BINS;
        else
            next_wr_bin = wr_bin_plus_1[BIN_BITS-1:0];

        start_calc = wr_bin_idx + NUM_BINS - (READOUT_BINS - 1);
        if (start_calc >= NUM_BINS)
            snapshot_start_bin = start_calc - NUM_BINS;
        else
            snapshot_start_bin = start_calc[BIN_BITS-1:0];

        if (completed_bins < NUM_BINS)
            completed_bins_next = completed_bins + 1'b1;
        else
            completed_bins_next = completed_bins;

        rd_bin_calc = snapshot_start_bin + rd_bin_off;
        if (rd_bin_calc >= NUM_BINS)
            rd_bin_idx = rd_bin_calc - NUM_BINS;
        else
            rd_bin_idx = rd_bin_calc[BIN_BITS-1:0];
    end

    assign event_ready   = (state == ST_ACCUM);
    assign readout_valid = (state == ST_READOUT);
    assign readout_index = (rd_bin_off * CELLS_PER_BIN) + rd_cell_idx;
    assign readout_last  = (rd_bin_off == READOUT_BINS-1) && (rd_cell_idx == CELLS_PER_BIN-1);
    assign readout_data  = mem[(rd_bin_idx * CELLS_PER_BIN) + rd_cell_idx];

    always_ff @(posedge clk) begin
        if (rst) begin
            state             <= ST_CLEAR;
            timer_ctr         <= '0;
            wr_bin_idx        <= '0;
            clear_bin_idx     <= '0;
            completed_bins    <= '0;
            rd_bin_off        <= '0;
            rd_cell_idx       <= '0;
            clear_cell_idx    <= '0;
            readout_start     <= 1'b0;
        end else begin
            readout_start <= 1'b0;

            case (state)
                ST_ACCUM: begin
                    if (event_valid) begin
                        if (mem[(wr_bin_idx * CELLS_PER_BIN) + event_cell_idx] != {COUNTER_BITS{1'b1}})
                            mem[(wr_bin_idx * CELLS_PER_BIN) + event_cell_idx]
                                <= mem[(wr_bin_idx * CELLS_PER_BIN) + event_cell_idx] + 1'b1;
                    end

                    if (timer_ctr == CYCLES_PER_BIN_SAFE - 1) begin
                        timer_ctr      <= '0;
                        clear_bin_idx  <= next_wr_bin;
                        completed_bins <= completed_bins_next;

                        if (completed_bins_next >= READOUT_BINS) begin
                            if (readout_ready) begin
                                state         <= ST_READOUT;
                                rd_bin_off    <= '0;
                                rd_cell_idx   <= '0;
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

                ST_WAIT_RD: begin
                    if (readout_ready) begin
                        state         <= ST_READOUT;
                        rd_bin_off    <= '0;
                        rd_cell_idx   <= '0;
                        readout_start <= 1'b1;
                    end
                end

                ST_READOUT: begin
                    if (readout_last) begin
                        state          <= ST_CLEAR;
                        clear_cell_idx <= '0;
                    end else if (rd_cell_idx == CELLS_PER_BIN - 1) begin
                        rd_cell_idx <= '0;
                        rd_bin_off  <= rd_bin_off + 1'b1;
                    end else begin
                        rd_cell_idx <= rd_cell_idx + 1'b1;
                    end
                end

                ST_CLEAR: begin
                    mem[(clear_bin_idx * CELLS_PER_BIN) + clear_cell_idx] <= '0;
                    if (clear_cell_idx == CELLS_PER_BIN - 1) begin
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

    // Polarity is carried for interface completeness; current voxel feature is count-only.
    wire _unused_polarity = event_polarity;

endmodule
