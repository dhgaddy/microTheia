`timescale 1ns/1ps

// Parallel-feed systolic array: 4 weight ROMs in parallel, one cell per cycle
// Total latency: NUM_CELLS + 1 drain cycle

module systolic_array #(
    parameter NUM_CLASSES = 4,
    parameter NUM_CELLS   = 1024,
    parameter VALUE_BITS  = 8,
    parameter WEIGHT_BITS = 8,
    parameter ACC_BITS    = 24
)(
    input  logic                                    clk,
    input  logic                                    rst,
    input  logic                                    start,
    input  logic [VALUE_BITS-1:0]                   feature_in,
    output logic [$clog2(NUM_CELLS)-1:0]            w_addr,
    input  logic [NUM_CLASSES*WEIGHT_BITS-1:0]      w_data_flat,
    output logic                                    result_valid,
    output logic [1:0]                              best_class,
    output logic [NUM_CLASSES*ACC_BITS-1:0]         scores_flat
);

    localparam CNT_BITS = $clog2(NUM_CELLS + 4);

    typedef enum logic [1:0] {
        S_IDLE    = 2'd0,
        S_RUNNING = 2'd1,
        S_DRAIN   = 2'd2,
        S_ARGMAX  = 2'd3
    } state_t;

    state_t state;

    logic [CNT_BITS-1:0] cell_cnt;

    logic signed [ACC_BITS-1:0] acc   [0:NUM_CLASSES-1];
    logic signed [ACC_BITS-1:0] acc_r [0:NUM_CLASSES-1];

    logic [VALUE_BITS-1:0] feat_pipe_r;
    logic                  pipe_valid_r;

    wire signed [WEIGHT_BITS-1:0] w [0:NUM_CLASSES-1];
    generate
        genvar gk;
        for (gk = 0; gk < NUM_CLASSES; gk = gk + 1) begin : gen_w_unpack
            assign w[gk] = $signed(w_data_flat[(gk+1)*WEIGHT_BITS-1 : gk*WEIGHT_BITS]);
        end
    endgenerate

    logic signed [ACC_BITS-1:0] comb_max_score;
    logic [1:0]                 comb_best_class;

    always_comb begin
        comb_max_score  = acc[0];
        comb_best_class = 2'd0;
        for (int ci = 1; ci < NUM_CLASSES; ci = ci + 1) begin
            if ($signed(acc[ci]) > $signed(comb_max_score)) begin
                comb_max_score  = acc[ci];
                comb_best_class = 2'(ci);
            end
        end
    end

    integer k;

    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            cell_cnt     <= '0;
            w_addr       <= '0;
            feat_pipe_r  <= '0;
            pipe_valid_r <= 1'b0;
            result_valid <= 1'b0;
            best_class   <= '0;
            for (k = 0; k < NUM_CLASSES; k = k + 1) begin
                acc  [k] <= '0;
                acc_r[k] <= '0;
            end
        end else begin
            result_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    pipe_valid_r <= 1'b0;
                    if (start) begin
                        for (k = 0; k < NUM_CLASSES; k = k + 1)
                            acc[k] <= '0;
                        cell_cnt    <= '0;
                        w_addr      <= '0;
                        feat_pipe_r <= feature_in;
                        state       <= S_RUNNING;
                    end
                end

                S_RUNNING: begin
                    if (pipe_valid_r) begin
                        for (k = 0; k < NUM_CLASSES; k = k + 1)
                            acc[k] <= acc[k] +
                                ACC_BITS'($signed({1'b0, feat_pipe_r}) * $signed(w[k]));
                    end

                    pipe_valid_r <= 1'b1;
                    feat_pipe_r  <= feature_in;
                    cell_cnt     <= cell_cnt + 1'b1;

                    if (cell_cnt < CNT_BITS'(NUM_CELLS - 1))
                        w_addr <= w_addr + 1'b1;

                    if (cell_cnt == CNT_BITS'(NUM_CELLS - 1))
                        state <= S_DRAIN;
                end

                S_DRAIN: begin
                    for (k = 0; k < NUM_CLASSES; k = k + 1)
                        acc[k] <= acc[k] +
                            ACC_BITS'($signed({1'b0, feat_pipe_r}) * $signed(w[k]));
                    pipe_valid_r <= 1'b0;
                    state        <= S_ARGMAX;
                end

                S_ARGMAX: begin
                    for (k = 0; k < NUM_CLASSES; k = k + 1)
                        acc_r[k] <= acc[k];
                    best_class   <= comb_best_class;
                    result_valid <= 1'b1;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    generate
        genvar gs;
        for (gs = 0; gs < NUM_CLASSES; gs = gs + 1) begin : gen_scores
            assign scores_flat[(gs+1)*ACC_BITS-1 : gs*ACC_BITS] = acc_r[gs];
        end
    endgenerate

endmodule
