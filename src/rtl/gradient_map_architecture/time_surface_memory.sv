`timescale 1ns/1ps

// Time-surface memory: stores last-event timestamp per grid cell.
// Lazy decay on read: value = MAX >> (Δt >> DECAY_SHIFT).
// Per-cell valid flag prevents phantom values on unwritten cells.
// Dual-port: port A writes (events), port B reads (scan). 2-cycle read latency.

module time_surface_memory #(
    parameter GRID_SIZE   = 16,
    parameter ADDR_BITS   = 8,
    parameter TS_BITS     = 16,
    parameter VALUE_BITS  = 8,
    parameter MAX_VALUE   = 255,
    parameter DECAY_SHIFT = 6
)(
    input  logic                    clk,
    input  logic                    rst,
    input  logic [TS_BITS-1:0]      t_now,
    input  logic                    event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0] event_x,
    input  logic [$clog2(GRID_SIZE)-1:0] event_y,
    input  logic [TS_BITS-1:0]      event_ts,
    input  logic                    read_enable,
    input  logic [ADDR_BITS-1:0]    read_addr,
    output logic [VALUE_BITS-1:0]   read_value,
    output logic [TS_BITS-1:0]      read_ts_raw,
    output logic                    read_cell_valid
);

    localparam NUM_CELLS = GRID_SIZE * GRID_SIZE;

    logic [TS_BITS-1:0] mem [0:NUM_CELLS-1];
    logic               cell_valid [0:NUM_CELLS-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < NUM_CELLS; init_i = init_i + 1) begin
            mem[init_i]        = '0;
            cell_valid[init_i] = 1'b0;
        end
    end

    wire [ADDR_BITS-1:0] write_addr = ADDR_BITS'(event_y) * ADDR_BITS'(GRID_SIZE) +
                                       ADDR_BITS'(event_x);

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int wi = 0; wi < NUM_CELLS; wi = wi + 1)
                cell_valid[wi] <= 1'b0;
        end else if (event_valid) begin
            mem[write_addr]        <= event_ts;
            cell_valid[write_addr] <= 1'b1;
        end
    end

    logic [TS_BITS-1:0] bram_dout;
    logic               bram_valid;

    always_ff @(posedge clk) begin
        bram_dout  <= mem[read_addr];
        bram_valid <= cell_valid[read_addr];
    end

    assign read_ts_raw    = bram_dout;
    assign read_cell_valid = bram_valid;

    // Decay stage: value = MAX_VALUE >> (Δt >> DECAY_SHIFT)
    logic [TS_BITS-1:0] delta_t;
    logic [7:0]         decay_steps;

    always_ff @(posedge clk) begin
        if (rst) begin
            read_value <= '0;
        end else if (read_enable) begin
            if (!bram_valid) begin
                read_value <= '0;
            end else begin
                delta_t    = t_now - bram_dout;
                decay_steps = TS_BITS'(delta_t >> DECAY_SHIFT);
                if (decay_steps >= 8)
                    read_value <= 8'd0;
                else
                    read_value <= VALUE_BITS'(MAX_VALUE >> decay_steps);
            end
        end
    end

endmodule
