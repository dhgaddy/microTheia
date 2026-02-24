// Two-stage pipeline: (1) late - early differences, (2) absolute values

module MotionComputer #(
    parameter ACC_SUM_BITS   = 18,
    parameter ACC_COUNT_BITS = 12
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        trigger,
    input  logic signed [ACC_SUM_BITS-1:0] early_sum_x,
    input  logic signed [ACC_SUM_BITS-1:0] early_sum_y,
    input  logic [ACC_COUNT_BITS-1:0]      early_count,
    input  logic signed [ACC_SUM_BITS-1:0] late_sum_x,
    input  logic signed [ACC_SUM_BITS-1:0] late_sum_y,
    input  logic [ACC_COUNT_BITS-1:0]      late_count,
    output logic signed [ACC_SUM_BITS-1:0] delta_x,
    output logic signed [ACC_SUM_BITS-1:0] delta_y,
    output logic [ACC_SUM_BITS-1:0]        abs_delta_x,
    output logic [ACC_SUM_BITS-1:0]        abs_delta_y,
    output logic [ACC_COUNT_BITS-1:0]      total_events,
    output logic                           valid
);

    logic signed [ACC_SUM_BITS-1:0] diff_x, diff_y;
    logic [ACC_COUNT_BITS-1:0]      sum_count;
    logic                           stage1_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            diff_x       <= '0;
            diff_y       <= '0;
            sum_count    <= '0;
            stage1_valid <= 1'b0;
        end else begin
            stage1_valid <= trigger;
            if (trigger) begin
                diff_x    <= late_sum_x - early_sum_x;
                diff_y    <= late_sum_y - early_sum_y;
                sum_count <= early_count + late_count;
            end
        end
    end

    wire [ACC_SUM_BITS-1:0] abs_diff_x = (diff_x < 0) ? (-diff_x) : diff_x;
    wire [ACC_SUM_BITS-1:0] abs_diff_y = (diff_y < 0) ? (-diff_y) : diff_y;

    always_ff @(posedge clk) begin
        if (rst) begin
            delta_x      <= '0;
            delta_y      <= '0;
            abs_delta_x  <= '0;
            abs_delta_y  <= '0;
            total_events <= '0;
            valid        <= 1'b0;
        end else begin
            valid <= stage1_valid;
            if (stage1_valid) begin
                delta_x      <= diff_x;
                delta_y      <= diff_y;
                abs_delta_x  <= abs_diff_x;
                abs_delta_y  <= abs_diff_y;
                total_events <= sum_count;
            end
        end
    end

endmodule
