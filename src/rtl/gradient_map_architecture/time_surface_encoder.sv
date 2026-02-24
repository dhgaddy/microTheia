`timescale 1ns/1ps

// Exponential-decay time surface: S(x,y,t) = 255 * 2^(-Δt / 2^DECAY_SHIFT)
// Wraps time_surface_memory; replaces its linear decay with exponential shift.
// Two-stage pipeline preserves the same 2-cycle read latency.

module time_surface_encoder #(
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
    input  logic                              event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_x,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_y,
    input  logic [TS_BITS-1:0]                event_ts,
    input  logic                    read_enable,
    input  logic [ADDR_BITS-1:0]    read_addr,
    output logic [VALUE_BITS-1:0]   read_value,
    output logic [TS_BITS-1:0]      read_ts_raw
);

    logic [TS_BITS-1:0]   raw_ts;
    logic [VALUE_BITS-1:0] ts_mem_value;
    logic                  ts_mem_cell_valid;

    time_surface_memory #(
        .GRID_SIZE  (GRID_SIZE),
        .ADDR_BITS  (ADDR_BITS),
        .TS_BITS    (TS_BITS),
        .VALUE_BITS (VALUE_BITS),
        .MAX_VALUE  (MAX_VALUE),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_ts_mem (
        .clk            (clk),
        .rst            (rst),
        .t_now          (t_now),
        .event_valid    (event_valid),
        .event_x        (event_x),
        .event_y        (event_y),
        .event_ts       (event_ts),
        .read_enable    (read_enable),
        .read_addr      (read_addr),
        .read_value     (ts_mem_value),
        .read_ts_raw    (raw_ts),
        .read_cell_valid(ts_mem_cell_valid)
    );

    assign read_ts_raw = raw_ts;

    // Stage 1: compute Δt and decay_steps (16-bit wrap-around safe)
    logic [TS_BITS-1:0] delta_t_r1;
    logic [TS_BITS-1:0] decay_steps_r1;
    logic               cell_valid_r1;

    always_ff @(posedge clk) begin
        if (rst) begin
            delta_t_r1     <= '0;
            decay_steps_r1 <= '0;
            cell_valid_r1  <= 1'b0;
        end else if (read_enable) begin
            delta_t_r1     <= t_now - raw_ts;
            decay_steps_r1 <= (t_now - raw_ts) >> DECAY_SHIFT;
            cell_valid_r1  <= ts_mem_cell_valid;
        end
    end

    // Stage 2: apply exponential decay via right-shift
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
