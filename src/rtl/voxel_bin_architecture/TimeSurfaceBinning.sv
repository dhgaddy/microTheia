`timescale 1ns/1ps

// Accumulates DVS events into a ring buffer of NUM_BINS 2D histograms.
// Rotates bins on a timer; flattens READOUT_BINS bins for classification.
// Parallel reads (PARALLEL_READS values/cycle) feed the systolic array.

module TimeSurfaceBinning #(
    parameter CLK_FREQ_HZ    = 12_000_000,
    parameter WINDOW_MS      = 400,
    parameter NUM_BINS       = 4,
    parameter READOUT_BINS   = 4,
    parameter GRID_SIZE      = 16,
    parameter COUNTER_BITS   = 6,
    parameter PARALLEL_READS = 4,
    parameter CYCLES_PER_BIN = 0  // 0 = auto-compute from WINDOW_MS/NUM_BINS
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        event_valid,
    input  logic signed [4:0] event_x,
    input  logic signed [4:0] event_y,
    input  logic        event_polarity,
    output logic        readout_start,
    output logic [PARALLEL_READS*COUNTER_BITS-1:0] readout_data,
    output logic        readout_valid
);

    localparam integer BIN_DURATION_MS     = WINDOW_MS / NUM_BINS;
    localparam integer CYCLES_PER_BIN_AUTO = (CLK_FREQ_HZ / 1000) * BIN_DURATION_MS;
    localparam integer CYCLES_PER_BIN_USE  = (CYCLES_PER_BIN == 0) ? CYCLES_PER_BIN_AUTO : CYCLES_PER_BIN;
    localparam integer TIMER_BITS          = $clog2(CYCLES_PER_BIN_USE + 1);
    localparam integer CELLS_PER_BIN       = GRID_SIZE * GRID_SIZE;
    localparam integer TOTAL_CELLS         = NUM_BINS * CELLS_PER_BIN;
    localparam integer CELL_ADDR_BITS      = $clog2(TOTAL_CELLS);
    localparam integer BIN_IDX_BITS        = $clog2(NUM_BINS);
    localparam integer GRID_ADDR_BITS      = $clog2(CELLS_PER_BIN);
    localparam integer PARALLEL_BITS       = $clog2(PARALLEL_READS);
    localparam integer CYCLES_PER_BIN_READ = (CELLS_PER_BIN + PARALLEL_READS - 1) / PARALLEL_READS;

    logic [TIMER_BITS-1:0]   bin_timer;
    logic [BIN_IDX_BITS-1:0] current_bin_idx;

    logic [COUNTER_BITS-1:0] mem [0:TOTAL_CELLS-1];

    logic [BIN_IDX_BITS-1:0] event_bin_idx;
    logic [3:0]              mapped_x, mapped_y;
    logic [GRID_ADDR_BITS-1:0] event_cell_addr;
    logic [CELL_ADDR_BITS-1:0] event_full_addr;

    assign mapped_x        = event_x + 5'd8;
    assign mapped_y        = event_y + 5'd8;
    assign event_cell_addr = {mapped_y[3:0], mapped_x[3:0]};

    typedef enum logic [1:0] {
        S_IDLE,
        S_READOUT,
        S_CLEAR
    } state_t;

    state_t state;

    logic [BIN_IDX_BITS-1:0]                   readout_bin_ptr;
    logic [GRID_ADDR_BITS-PARALLEL_BITS:0]     readout_cell_ctr;
    logic [GRID_ADDR_BITS-1:0]                 clear_cell_ctr;
    logic                                      trigger_readout;

    always_ff @(posedge clk) begin
        if (rst) begin
            bin_timer       <= '0;
            current_bin_idx <= '0;
            trigger_readout <= 1'b0;
            state           <= S_CLEAR;
            clear_cell_ctr  <= '0;
        end else begin
            trigger_readout <= 1'b0;

            if (state == S_IDLE || state == S_READOUT) begin
                if (bin_timer >= CYCLES_PER_BIN_USE - 1) begin
                    bin_timer <= '0;

                    if (current_bin_idx == NUM_BINS - 1)
                        current_bin_idx <= '0;
                    else
                        current_bin_idx <= current_bin_idx + 1'b1;

                    trigger_readout <= 1'b1;
                    state           <= S_CLEAR;
                    clear_cell_ctr  <= '0;
                end else begin
                    bin_timer <= bin_timer + 1'b1;
                end
            end else if (state == S_CLEAR) begin
                if (clear_cell_ctr == CELLS_PER_BIN - 1)
                    state <= S_IDLE;
                clear_cell_ctr <= clear_cell_ctr + 1'b1;
            end
        end
    end

    logic ram_wen;
    logic [CELL_ADDR_BITS-1:0] ram_addr;
    logic [COUNTER_BITS-1:0]   ram_wdata;
    logic [COUNTER_BITS-1:0]   ram_rdata;

    logic readout_busy;
    logic [BIN_IDX_BITS-1:0]   rd_bin_offset;
    logic [GRID_ADDR_BITS-1:0] rd_cell_ctr;

    always_ff @(posedge clk) begin
        if (rst) begin
            readout_busy   <= 1'b0;
            readout_start  <= 1'b0;
            rd_bin_offset  <= '0;
            rd_cell_ctr    <= '0;
        end else begin
            readout_start <= 1'b0;

            if (trigger_readout) begin
                readout_busy  <= 1'b1;
                readout_start <= 1'b1;
                rd_bin_offset <= '0;
                rd_cell_ctr   <= '0;
            end else if (readout_busy) begin
                if (rd_cell_ctr >= CYCLES_PER_BIN_READ - 1) begin
                    rd_cell_ctr <= '0;
                    if (rd_bin_offset == READOUT_BINS - 1)
                        readout_busy <= 1'b0;
                    else
                        rd_bin_offset <= rd_bin_offset + 1'b1;
                end else begin
                    rd_cell_ctr <= rd_cell_ctr + 1'b1;
                end
            end
        end
    end

    // Chronological readout: current_bin_idx first (oldest, being cleared),
    // then wrapping through the ring buffer to the newest filled bin.
    // The clear and readout happen concurrently; readout sees pre-clear values.
    logic [BIN_IDX_BITS:0]   calc_bin_idx;
    logic [BIN_IDX_BITS-1:0] actual_rd_bin_idx;

    always_comb begin
        calc_bin_idx = current_bin_idx + rd_bin_offset;
        if (calc_bin_idx >= NUM_BINS)
            actual_rd_bin_idx = calc_bin_idx - NUM_BINS;
        else
            actual_rd_bin_idx = calc_bin_idx;
    end

    logic [CELL_ADDR_BITS-1:0] port_a_addr;
    logic [COUNTER_BITS-1:0]   port_a_rdata;
    logic [COUNTER_BITS-1:0]   port_a_wdata;
    logic                      port_a_wen;

    logic                      event_valid_pipe;
    logic [CELL_ADDR_BITS-1:0] evt_addr_pipe;

    logic [CELL_ADDR_BITS-1:0] clear_addr;
    logic [CELL_ADDR_BITS-1:0] evt_addr;

    assign clear_addr = {current_bin_idx, clear_cell_ctr};
    assign evt_addr   = {current_bin_idx, event_cell_addr};

    always_ff @(posedge clk) begin
        port_a_rdata     <= mem[port_a_addr];
        event_valid_pipe <= event_valid;
        evt_addr_pipe    <= evt_addr;

        if (port_a_wen)
            mem[port_a_addr] <= port_a_wdata;
    end

    logic [CELL_ADDR_BITS-1:0] rd_addr_parallel [0:PARALLEL_READS-1];
    logic [COUNTER_BITS-1:0]   ram_rdata_parallel [0:PARALLEL_READS-1];

    genvar p;
    generate
        for (p = 0; p < PARALLEL_READS; p = p + 1) begin : gen_parallel_reads
            logic [GRID_ADDR_BITS-1:0] cell_offset;
            assign cell_offset         = (rd_cell_ctr * PARALLEL_READS) + p;
            assign rd_addr_parallel[p] = {actual_rd_bin_idx, cell_offset[GRID_ADDR_BITS-1:0]};

            always_ff @(posedge clk) begin
                if (readout_busy && cell_offset < CELLS_PER_BIN)
                    ram_rdata_parallel[p] <= mem[rd_addr_parallel[p]];
                else
                    ram_rdata_parallel[p] <= '0;
            end
        end
    endgenerate

    always_comb begin
        for (int i = 0; i < PARALLEL_READS; i = i + 1)
            readout_data[i*COUNTER_BITS +: COUNTER_BITS] = ram_rdata_parallel[i];
    end

    always_comb begin
        port_a_addr  = '0;
        port_a_wdata = '0;
        port_a_wen   = 1'b0;

        if (state == S_CLEAR) begin
            port_a_addr  = clear_addr;
            port_a_wdata = '0;
            port_a_wen   = 1'b1;
        end else if (event_valid_pipe) begin
            port_a_addr  = evt_addr_pipe;
            port_a_wdata = (port_a_rdata != {COUNTER_BITS{1'b1}}) ? port_a_rdata + 1'b1 : port_a_rdata;
            port_a_wen   = 1'b1;
        end else if (event_valid) begin
            port_a_addr = evt_addr;
            port_a_wen  = 1'b0;
        end
    end

    logic readout_valid_d;
    logic [GRID_ADDR_BITS-1:0] cell_offset_max;

    always_comb
        cell_offset_max = (rd_cell_ctr * PARALLEL_READS) + (PARALLEL_READS - 1);

    always_ff @(posedge clk)
        readout_valid_d <= (readout_busy && cell_offset_max < CELLS_PER_BIN);

    assign readout_valid = readout_valid_d;

endmodule
