`timescale 1ns/1ps

// Scans all NUM_CELLS time-surface cells in row-major order and captures
// their decayed values into a flat vector. 2-cycle BRAM+decay read latency.

module flatten_buffer #(
    parameter GRID_SIZE  = 16,
    parameter VALUE_BITS = 8,
    parameter NUM_CELLS  = 256,
    parameter PIPE_DEPTH = 2
)(
    input  logic                          clk,
    input  logic                          rst,
    input  logic                          start,
    output logic [$clog2(NUM_CELLS)-1:0]  ts_addr,
    output logic                          ts_en,
    input  logic [VALUE_BITS-1:0]         ts_val,
    output logic                          flat_valid,
    output logic [NUM_CELLS*VALUE_BITS-1:0] flat_data
);

    logic [VALUE_BITS-1:0] flat_data_int [0:NUM_CELLS-1];

    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_FILL = 2'd1,
        S_SCAN = 2'd2,
        S_DONE = 2'd3
    } state_t;

    state_t state;

    localparam CNT_BITS = $clog2(NUM_CELLS + PIPE_DEPTH + 2);

    logic [CNT_BITS-1:0] issue_cnt;
    logic [CNT_BITS-1:0] capture_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            issue_cnt   <= '0;
            capture_cnt <= '0;
            ts_addr     <= '0;
            ts_en       <= 1'b0;
            flat_valid  <= 1'b0;
        end else begin
            flat_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        issue_cnt   <= CNT_BITS'(1);
                        capture_cnt <= '0;
                        ts_addr     <= '0;
                        ts_en       <= 1'b1;
                        state       <= S_FILL;
                    end
                end

                S_FILL: begin
                    if (issue_cnt < CNT_BITS'(NUM_CELLS)) begin
                        ts_addr   <= $clog2(NUM_CELLS)'(issue_cnt);
                        ts_en     <= 1'b1;
                        issue_cnt <= issue_cnt + 1'b1;
                    end else begin
                        ts_en <= 1'b0;
                    end

                    capture_cnt <= capture_cnt + 1'b1;
                    if (capture_cnt == CNT_BITS'(PIPE_DEPTH - 1)) begin
                        state <= S_SCAN;
                        capture_cnt <= '0;
                    end
                end

                S_SCAN: begin
                    flat_data_int[capture_cnt[$clog2(NUM_CELLS)-1:0]] <= ts_val;
                    capture_cnt <= capture_cnt + 1'b1;

                    if (issue_cnt < CNT_BITS'(NUM_CELLS)) begin
                        ts_addr   <= $clog2(NUM_CELLS)'(issue_cnt);
                        ts_en     <= 1'b1;
                        issue_cnt <= issue_cnt + 1'b1;
                    end else begin
                        ts_en   <= 1'b0;
                        ts_addr <= '0;
                    end

                    if (capture_cnt == CNT_BITS'(NUM_CELLS - 1))
                        state <= S_DONE;
                end

                S_DONE: begin
                    flat_valid  <= 1'b1;
                    ts_en       <= 1'b0;
                    issue_cnt   <= '0;
                    capture_cnt <= '0;
                    state       <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    genvar i;
    generate
        for (i = 0; i < NUM_CELLS; i = i + 1) begin : gen_pack
            assign flat_data[i*VALUE_BITS +: VALUE_BITS] = flat_data_int[i];
        end
    endgenerate

endmodule
