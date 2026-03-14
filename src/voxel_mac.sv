module voxel_mac #(
    parameter int FEAT_BITS   = 6,   // COUNTER_BITS
    parameter int WEIGHT_BITS = 8,   // weight width
    parameter int ACC_BITS    = 32   // accumulator width
)(
    input  logic                   clk,
    input  logic                   rst,

    // Pulse to clear accumulator for a new window
    input  logic                   start,

    // Feature stream
    input  logic                   feature_valid,
    input  logic [FEAT_BITS-1:0]   feature_value,
    input  logic                   feature_last,

    // Weight stream
    input  logic                   weight_valid,
    input  logic [WEIGHT_BITS-1:0] weight_value,

    // Output score
    output logic                   score_valid,
    output logic [ACC_BITS-1:0]    score
);

    localparam int PROD_BITS = FEAT_BITS + WEIGHT_BITS;

    logic [ACC_BITS-1:0]  acc;
    logic [PROD_BITS-1:0] prod;

    assign prod = feature_value * weight_value;

    always_ff @(posedge clk) begin
        if (rst) begin
            acc         <= '0;
            score       <= '0;
            score_valid <= 1'b0;
        end else begin
            score_valid <= 1'b0;

            // Reset accumulation at start of a new window
            if (start) begin
                acc <= '0;
            end

            // Accumulate when both streams are valid
            if (feature_valid && weight_valid) begin
                acc <= acc + {{(ACC_BITS-PROD_BITS){1'b0}}, prod};
            end

            // On the last element, publish score including this cycle's product
            if (feature_valid && weight_valid && feature_last) begin
                score       <= acc + {{(ACC_BITS-PROD_BITS){1'b0}}, prod};
                score_valid <= 1'b1;
            end
        end
    end

endmodule
