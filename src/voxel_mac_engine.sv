// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 Group G Contributors
`timescale 1ns/1ps

module voxel_mac_engine #(
    parameter int FEATURE_COUNT = 2048,
    parameter int COUNTER_BITS  = 16,
    parameter int WEIGHT_BITS   = 8,
    parameter int NUM_CLASSES   = 4,
    parameter int SCORE_BITS    = COUNTER_BITS + WEIGHT_BITS + $clog2(FEATURE_COUNT) + 1
)(
    input  logic clk,
    input  logic rst,

    input  logic start,
    output logic busy,

    output logic                               rd_en,
    output logic [$clog2(FEATURE_COUNT)-1:0]   rd_addr,

    input  logic [COUNTER_BITS-1:0]            feature_data,
    input  logic [NUM_CLASSES*WEIGHT_BITS-1:0] weight_data_flat,

    output logic [NUM_CLASSES*SCORE_BITS-1:0]  scores_flat,
    output logic                               scores_valid
);

    localparam int ADDR_BITS   = $clog2(FEATURE_COUNT);
    localparam int STREAM_BITS = $clog2(FEATURE_COUNT + 1);

    typedef enum logic [1:0] {
        ST_IDLE    = 2'd0,
        ST_STREAM  = 2'd1,
        ST_PUBLISH = 2'd2
    } state_t;

    state_t                 state;
    logic [STREAM_BITS-1:0] stream_idx;
    logic                   mac_valid; // data from last cycle's read is ready to accumulate
    logic                   mac_last;  // this is the final accumulation

    logic [SCORE_BITS-1:0]  score_acc [0:NUM_CLASSES-1];

    // rd_addr held at 0 when idle to avoid X propagation in simulation.
    assign rd_en   = (state == ST_STREAM) && (stream_idx < FEATURE_COUNT);
    assign rd_addr = rd_en ? stream_idx[ADDR_BITS-1:0] : '0;
    assign busy    = (state != ST_IDLE);

    always_comb begin
        for (int g = 0; g < NUM_CLASSES; g++)
            scores_flat[g*SCORE_BITS +: SCORE_BITS] = score_acc[g];
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            stream_idx   <= '0;
            mac_valid    <= 1'b0;
            mac_last     <= 1'b0;
            scores_valid <= 1'b0;
            for (int g = 0; g < NUM_CLASSES; g++)
                score_acc[g] <= '0;
        end else begin
            scores_valid <= 1'b0;

            // mac_valid is delayed one cycle to match synchronous RAM read latency.
            // The first rd_en fires at T+1 (state=ST_STREAM, idx=0), so the first
            // valid SRAM data arrives at T+2 — mac_valid=1 at T+2 is correct.
            mac_valid <= (state == ST_STREAM) && (stream_idx < FEATURE_COUNT);
            mac_last  <= (state == ST_STREAM) && (stream_idx == STREAM_BITS'(FEATURE_COUNT - 1));

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        stream_idx <= '0;
                        for (int g = 0; g < NUM_CLASSES; g++)
                            score_acc[g] <= '0;
                        state <= ST_STREAM;
                    end
                end

                ST_STREAM: begin
                    if (stream_idx < FEATURE_COUNT)
                        stream_idx <= stream_idx + 1'b1;

                    if (mac_valid) begin
                        for (int g = 0; g < NUM_CLASSES; g++) begin
                            // Keep operand widths tight: 16-bit × 8-bit = 24-bit product,
                            // zero-extended to SCORE_BITS before accumulation.
                            logic [COUNTER_BITS+WEIGHT_BITS-1:0] product;
                            product = feature_data *
                                      weight_data_flat[g*WEIGHT_BITS +: WEIGHT_BITS];
                            score_acc[g] <= score_acc[g] + SCORE_BITS'(product);
                        end
                    end

                    if (mac_last)
                        state <= ST_PUBLISH;
                end

                ST_PUBLISH: begin
                    scores_valid <= 1'b1;
                    state        <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
