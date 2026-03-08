// Gesture classifier over 4 class scores:
//   0: Down, 1: Left, 2: Right, 3: Up
// class_pass = (max_score - second_score) > PASS_MARGIN
// gesture_valid requires PERSISTENCE_COUNT consecutive passing windows with same class.

module gesture_classifier #(
    parameter int NUM_CLASSES        = 4,
    parameter int SCORE_BITS         = 32,
    parameter int PASS_MARGIN        = 64,
    parameter int PERSISTENCE_COUNT  = 2,
    parameter int CONF_BITS          = 8,
    parameter int CONF_SHIFT         = 4
)(
    input  logic                               clk,
    input  logic                               rst,
    input  logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat,
    input  logic                               scores_valid,
    output logic [1:0]                         class_gesture,
    output logic                               class_valid,
    output logic                               class_pass,
    output logic [1:0]                         gesture,
    output logic                               gesture_valid,
    output logic [CONF_BITS-1:0]               gesture_confidence,
    output logic [2:0]                         debug_state
);

    localparam int PERSIST_BITS = (PERSISTENCE_COUNT > 1) ? $clog2(PERSISTENCE_COUNT + 1) : 1;

    logic signed [SCORE_BITS-1:0] s0, s1, s2, s3;
    logic signed [SCORE_BITS-1:0] pair0_max_c, pair0_min_c;
    logic signed [SCORE_BITS-1:0] pair1_max_c, pair1_min_c;
    logic [1:0]                   pair0_cls_c, pair1_cls_c;

    logic                         pair_valid_r;
    logic signed [SCORE_BITS-1:0] pair0_max_r, pair0_min_r;
    logic signed [SCORE_BITS-1:0] pair1_max_r, pair1_min_r;
    logic [1:0]                   pair0_cls_r, pair1_cls_r;

    logic signed [SCORE_BITS-1:0] max_score_b;
    logic signed [SCORE_BITS-1:0] second_score_b;
    logic signed [SCORE_BITS-1:0] second_a_b, second_b_b;
    logic [1:0]                   max_class_b;
    logic signed [SCORE_BITS:0]   margin_b;
    logic                         pass_b;

    logic [1:0] last_pass_class;
    logic [PERSIST_BITS-1:0] pass_streak;
    logic [PERSIST_BITS-1:0] next_streak_c;

    // Stage-0 capture of incoming score vector.
    logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat_r;
    logic scores_valid_r;

    // Stage-1 registered classifier decision.
    logic                               decision_valid_r;
    logic [1:0]                         max_class_r;
    logic                               pass_r;
    logic signed [SCORE_BITS:0]         margin_r;

    logic                               conf_sat_c;
    logic [CONF_BITS-1:0]               conf_quant_c;

    generate
        if (NUM_CLASSES != 4) begin : gen_invalid_num_classes
            initial $error("gesture_classifier currently supports NUM_CLASSES=4 only (got %0d)", NUM_CLASSES);
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (rst) begin
            scores_flat_r  <= '0;
            scores_valid_r <= 1'b0;
        end else begin
            scores_flat_r  <= scores_flat;
            scores_valid_r <= scores_valid;
        end
    end

    always_comb begin
        s0 = $signed(scores_flat_r[0*SCORE_BITS +: SCORE_BITS]);
        s1 = $signed(scores_flat_r[1*SCORE_BITS +: SCORE_BITS]);
        s2 = $signed(scores_flat_r[2*SCORE_BITS +: SCORE_BITS]);
        s3 = $signed(scores_flat_r[3*SCORE_BITS +: SCORE_BITS]);
    end

    always_comb begin
        if (s0 >= s1) begin
            pair0_max_c = s0;
            pair0_min_c = s1;
            pair0_cls_c = 2'd0;
        end else begin
            pair0_max_c = s1;
            pair0_min_c = s0;
            pair0_cls_c = 2'd1;
        end

        if (s2 >= s3) begin
            pair1_max_c = s2;
            pair1_min_c = s3;
            pair1_cls_c = 2'd2;
        end else begin
            pair1_max_c = s3;
            pair1_min_c = s2;
            pair1_cls_c = 2'd3;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            pair_valid_r <= 1'b0;
            pair0_max_r  <= '0;
            pair0_min_r  <= '0;
            pair1_max_r  <= '0;
            pair1_min_r  <= '0;
            pair0_cls_r  <= 2'd0;
            pair1_cls_r  <= 2'd0;
        end else begin
            pair_valid_r <= scores_valid_r;
            if (scores_valid_r) begin
                pair0_max_r <= pair0_max_c;
                pair0_min_r <= pair0_min_c;
                pair1_max_r <= pair1_max_c;
                pair1_min_r <= pair1_min_c;
                pair0_cls_r <= pair0_cls_c;
                pair1_cls_r <= pair1_cls_c;
            end
        end
    end

    always_comb begin
        if (pair0_max_r >= pair1_max_r) begin
            max_score_b = pair0_max_r;
            max_class_b = pair0_cls_r;
            second_a_b  = pair1_max_r;
            second_b_b  = pair0_min_r;
        end else begin
            max_score_b = pair1_max_r;
            max_class_b = pair1_cls_r;
            second_a_b  = pair0_max_r;
            second_b_b  = pair1_min_r;
        end

        second_score_b = (second_a_b >= second_b_b) ? second_a_b : second_b_b;
        margin_b = $signed({max_score_b[SCORE_BITS-1], max_score_b}) -
                   $signed({second_score_b[SCORE_BITS-1], second_score_b});
        pass_b = (margin_b > PASS_MARGIN);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            decision_valid_r <= 1'b0;
            max_class_r      <= 2'd0;
            pass_r           <= 1'b0;
            margin_r         <= '0;
        end else begin
            decision_valid_r <= pair_valid_r;
            if (pair_valid_r) begin
                max_class_r <= max_class_b;
                pass_r      <= pass_b;
                margin_r    <= margin_b;
            end
        end
    end

    always_comb begin
        conf_sat_c   = 1'b0;
        conf_quant_c = '0;
        if (margin_r > 0) begin
            conf_quant_c = margin_r[CONF_SHIFT +: CONF_BITS];
            if (SCORE_BITS >= (CONF_SHIFT + CONF_BITS))
                conf_sat_c = |margin_r[SCORE_BITS:CONF_SHIFT+CONF_BITS];
        end
    end

    always_comb begin
        if (max_class_r == last_pass_class) begin
            if (pass_streak < PERSISTENCE_COUNT)
                next_streak_c = pass_streak + 1'b1;
            else
                next_streak_c = pass_streak;
        end else begin
            next_streak_c = {{(PERSIST_BITS-1){1'b0}}, 1'b1};
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            class_gesture      <= 2'd0;
            class_valid        <= 1'b0;
            class_pass         <= 1'b0;
            gesture            <= 2'd0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= '0;
            last_pass_class    <= 2'd0;
            pass_streak        <= '0;
            debug_state        <= 3'd0;
        end else begin
            class_valid   <= 1'b0;
            class_pass    <= 1'b0;
            gesture_valid <= 1'b0;
            debug_state   <= 3'd0;

            if (decision_valid_r) begin
                class_gesture <= max_class_r;
                class_valid   <= 1'b1;
                class_pass    <= pass_r;

                if (pass_r) begin
                    last_pass_class <= max_class_r;
                    pass_streak     <= next_streak_c;

                    if (next_streak_c >= PERSISTENCE_COUNT) begin
                        gesture       <= max_class_r;
                        gesture_valid <= 1'b1;
                        debug_state   <= 3'd2;
                    end else begin
                        debug_state <= 3'd1;
                    end

                    if (conf_sat_c)
                        gesture_confidence <= {CONF_BITS{1'b1}};
                    else if (margin_r <= 0)
                        gesture_confidence <= '0;
                    else
                        gesture_confidence <= conf_quant_c;
                end else begin
                    pass_streak <= '0;
                    debug_state <= 3'd0;
                end
            end
        end
    end

endmodule
