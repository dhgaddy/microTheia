`timescale 1ns/1ps

// Gradient mapping / time-surface encoder.
// Stores last-event timestamps per grid cell and produces an
// exponentially decayed value on read:
//   S(x,y,t) = MAX_VALUE * 2^(-(t - t_last) / 2^DECAY_SHIFT)
//
// This module encapsulates both the timestamp memory and the
// exponential decay pipeline, so the core datapath can simply
// treat it as the "gradient_mapping" stage.

module gradient_mapping #(
    parameter GRID_SIZE   = 16,
    parameter ADDR_BITS   = 8,
    parameter TS_BITS     = 16,
    parameter VALUE_BITS  = 8,
    parameter MAX_VALUE   = 255,
    parameter DECAY_SHIFT = 6
)(
    input  logic                         clk,
    input  logic                         rst,
    input  logic [TS_BITS-1:0]           t_now,
    input  logic                         event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0] event_x,
    input  logic [$clog2(GRID_SIZE)-1:0] event_y,
    input  logic [TS_BITS-1:0]           event_ts,
    input  logic                         read_enable,
    input  logic [ADDR_BITS-1:0]         read_addr,
    output logic [VALUE_BITS-1:0]        read_value,
    output logic [TS_BITS-1:0]           read_ts_raw
);

    localparam NUM_CELLS = GRID_SIZE * GRID_SIZE;

    // Timestamp storage and per-cell valid flag.
    logic [TS_BITS-1:0] mem        [0:NUM_CELLS-1];
    logic               cell_valid [0:NUM_CELLS-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < NUM_CELLS; init_i = init_i + 1) begin
            mem[init_i]        = '0;
            cell_valid[init_i] = 1'b0;
        end
    end

    // Write address: row-major mapping of (x,y) into GRID_SIZE x GRID_SIZE.
    wire [ADDR_BITS-1:0] write_addr =
        ADDR_BITS'(event_y) * ADDR_BITS'(GRID_SIZE) + ADDR_BITS'(event_x);

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int wi = 0; wi < NUM_CELLS; wi = wi + 1)
                cell_valid[wi] <= 1'b0;
        end else if (event_valid) begin
            mem[write_addr]        <= event_ts;
            cell_valid[write_addr] <= 1'b1;
        end
    end

    // BRAM read stage: registered timestamp and valid flag.
    logic [TS_BITS-1:0] ts_r;
    logic               valid_r;

    always_ff @(posedge clk) begin
        ts_r    <= mem[read_addr];
        valid_r <= cell_valid[read_addr];
    end

    assign read_ts_raw = ts_r;

    // Stage 1: compute Δt and decay_steps with wrap-safe subtraction.
    logic [TS_BITS-1:0] delta_t_r1;
    logic [TS_BITS-1:0] decay_steps_r1;
    logic               cell_valid_r1;

    always_ff @(posedge clk) begin
        if (rst) begin
            delta_t_r1     <= '0;
            decay_steps_r1 <= '0;
            cell_valid_r1  <= 1'b0;
        end else if (read_enable) begin
            delta_t_r1     <= t_now - ts_r;
            decay_steps_r1 <= (t_now - ts_r) >> DECAY_SHIFT;
            cell_valid_r1  <= valid_r;
        end
    end

    // Stage 2: apply exponential decay via right-shift of MAX_VALUE.
    always_ff @(posedge clk) begin
        if (rst) begin
            read_value <= '0;
        end else begin
            if (!cell_valid_r1)
                read_value <= '0;
            else if (decay_steps_r1 >= TS_BITS'(VALUE_BITS))
                read_value <= '0;
            else
                read_value <= VALUE_BITS'(MAX_VALUE) >> decay_steps_r1[3:0];
        end
    end

endmodule

