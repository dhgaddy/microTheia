// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 Group G Contributors
`timescale 1ns/1ps

module voxel_gesture_classifier #(
    parameter int NUM_CLASSES = 4,
    parameter int SCORE_BITS  = 32
)(
    input  logic                              clk,
    input  logic                              rst,
    input  logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat,
    input  logic                              scores_valid,

    output logic                  thresh_rd_valid,
    output logic [2:0]            thresh_rd_addr,
    input  logic [SCORE_BITS-1:0] thresh_data,

    output logic [1:0] class_gesture,
    output logic       class_valid,
    output logic       class_pass,
    
    output logic [1:0] gesture,
    output logic       gesture_valid,
    output logic       gesture_confidence
);

    localparam int ADDR_BITS = $clog2(NUM_CLASSES);

    generate
        if (NUM_CLASSES != 4) begin : gen_invalid
            initial $error("gesture_classifier requires NUM_CLASSES=4 (got %0d)", NUM_CLASSES);
        end
    endgenerate

    logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat_r;
    logic                              scores_valid_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            scores_flat_r  <= '0;
            scores_valid_r <= 1'b0;
        end else begin
            scores_flat_r  <= scores_flat;
            scores_valid_r <= scores_valid;
        end
    end

    logic [SCORE_BITS-1:0] s0, s1, s2, s3;
    assign s0 = scores_flat_r[0*SCORE_BITS +: SCORE_BITS];
    assign s1 = scores_flat_r[1*SCORE_BITS +: SCORE_BITS];
    assign s2 = scores_flat_r[2*SCORE_BITS +: SCORE_BITS];
    assign s3 = scores_flat_r[3*SCORE_BITS +: SCORE_BITS];

    logic [SCORE_BITS-1:0] pair0_max_c, pair0_min_c;
    logic [SCORE_BITS-1:0] pair1_max_c, pair1_min_c;
    logic [1:0]            pair0_cls_c, pair1_cls_c;

    always_comb begin
        if (s0 >= s1) begin
            pair0_max_c = s0; pair0_min_c = s1; pair0_cls_c = 2'd0;
        end else begin
            pair0_max_c = s1; pair0_min_c = s0; pair0_cls_c = 2'd1;
        end
        if (s2 >= s3) begin
            pair1_max_c = s2; pair1_min_c = s3; pair1_cls_c = 2'd2;
        end else begin
            pair1_max_c = s3; pair1_min_c = s2; pair1_cls_c = 2'd3;
        end
    end

    logic                  pair_valid_r;
    logic [SCORE_BITS-1:0] pair0_max_r, pair0_min_r;
    logic [SCORE_BITS-1:0] pair1_max_r, pair1_min_r;
    logic [1:0]            pair0_cls_r, pair1_cls_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            pair_valid_r <= 1'b0;
            pair0_max_r  <= '0; pair0_min_r <= '0;
            pair1_max_r  <= '0; pair1_min_r <= '0;
            pair0_cls_r  <= '0; pair1_cls_r <= '0;
        end else begin
            pair_valid_r <= scores_valid_r;
            if (scores_valid_r) begin
                pair0_max_r <= pair0_max_c; pair0_min_r <= pair0_min_c;
                pair1_max_r <= pair1_max_c; pair1_min_r <= pair1_min_c;
                pair0_cls_r <= pair0_cls_c; pair1_cls_r <= pair1_cls_c;
            end
        end
    end

    logic [SCORE_BITS-1:0] max_score_b, second_a_b, second_b_b, second_score_b;
    logic [1:0]            max_class_b;
    logic [SCORE_BITS-1:0] diff_b;

    assign max_score_b    = (pair0_max_r >= pair1_max_r) ? pair0_max_r : pair1_max_r;
    assign max_class_b    = (pair0_max_r >= pair1_max_r) ? pair0_cls_r : pair1_cls_r;
    assign second_a_b     = (pair0_max_r >= pair1_max_r) ? pair1_max_r : pair0_max_r;
    assign second_b_b     = (pair0_max_r >= pair1_max_r) ? pair0_min_r : pair1_min_r;
    assign second_score_b = (second_a_b >= second_b_b) ? second_a_b : second_b_b;
    assign diff_b         = max_score_b - second_score_b;

    // Stage-2 registers declared before the continuous assigns that reference them.
    logic                  decision_valid_r;
    logic [1:0]            max_class_r;
    logic [SCORE_BITS-1:0] max_score_r;
    logic [SCORE_BITS-1:0] diff_r;

    // Staggered reads: pair_valid_r issues class_thresh read, decision_valid_r issues
    // diff_thresh read one cycle later — single read port is never double-booked.
    assign thresh_rd_valid = pair_valid_r | decision_valid_r;
    assign thresh_rd_addr  = pair_valid_r
                             ? {1'b0, max_class_b[ADDR_BITS-1:0]}
                             : {1'b1, max_class_r[ADDR_BITS-1:0]};

    always_ff @(posedge clk) begin
        if (rst) begin
            decision_valid_r <= 1'b0;
            max_class_r      <= '0;
            max_score_r      <= '0;
            diff_r           <= '0;
        end else begin
            decision_valid_r <= pair_valid_r;
            if (pair_valid_r) begin
                max_class_r <= max_class_b;
                max_score_r <= max_score_b;
                diff_r      <= diff_b;
            end
        end
    end

    logic                  decision_valid_r2;
    logic [1:0]            max_class_r2;
    logic [SCORE_BITS-1:0] max_score_r2;
    logic [SCORE_BITS-1:0] diff_r2;
    logic [SCORE_BITS-1:0] class_thresh_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            decision_valid_r2 <= 1'b0;
            max_class_r2      <= '0;
            max_score_r2      <= '0;
            diff_r2           <= '0;
            class_thresh_r    <= '0;
        end else begin
            decision_valid_r2 <= decision_valid_r;
            if (decision_valid_r) begin
                max_class_r2   <= max_class_r;
                max_score_r2   <= max_score_r;
                diff_r2        <= diff_r;
                class_thresh_r <= thresh_data;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            class_gesture      <= 2'd0;
            class_valid        <= 1'b0;
            class_pass         <= 1'b0;
            gesture            <= 2'd0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= 1'b0;
        end else begin
            class_valid        <= 1'b0;
            class_pass         <= 1'b0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= 1'b0;

            if (decision_valid_r2) begin
                class_gesture <= max_class_r2;
                class_valid   <= 1'b1;

                if (max_score_r2 > class_thresh_r) begin
                    class_pass         <= 1'b1;
                    gesture            <= max_class_r2;
                    gesture_valid      <= 1'b1;
                    // diff_thresh arrives this cycle (1 cycle after decision_valid_r).
                    gesture_confidence <= (diff_r2 > thresh_data);
                end
            end
        end
    end

endmodule
