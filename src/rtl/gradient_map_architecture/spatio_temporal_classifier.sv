`timescale 1ns/1ps

// Spatio-temporal DVS gesture classifier
// Pipeline: frame pulse → direct BRAM-to-systolic stream → argmax → threshold gate

module spatio_temporal_classifier #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 10,
    parameter GRID_SIZE       = 16,
    parameter ADDR_BITS       = 8,
    parameter VALUE_BITS      = 8,
    parameter MOMENT_BITS     = 24,
    parameter WEIGHT_BITS     = 8,
    parameter SCORE_BITS      = 24,
    parameter NUM_CLASSES     = 4,
    parameter MIN_MASS_THRESH = 100
)(
    input  logic                    clk,
    input  logic                    rst,
    output logic [ADDR_BITS-1:0]    ts_read_addr,
    output logic                    ts_read_enable,
    input  logic [VALUE_BITS-1:0]   ts_read_value,
    output logic [1:0]              gesture_class,
    output logic                    gesture_valid,
    output logic [7:0]              gesture_confidence,
    output logic [MOMENT_BITS-1:0]  debug_m00,
    output logic [MOMENT_BITS-1:0]  debug_m10,
    output logic [MOMENT_BITS-1:0]  debug_m01,
    output logic [2:0]              debug_state
);

    localparam NUM_CELLS  = GRID_SIZE * GRID_SIZE;
    localparam PIPE_DEPTH = 2;
    localparam CNT_BITS   = $clog2(NUM_CELLS + PIPE_DEPTH + 2);

    localparam FRAME_CYCLES   = (CLK_FREQ_HZ / 1000) * FRAME_PERIOD_MS;
    localparam FRAME_CNT_BITS = $clog2(FRAME_CYCLES + 1);

    logic [FRAME_CNT_BITS-1:0] frame_counter;
    logic frame_pulse;

    always_ff @(posedge clk) begin
        if (rst) begin
            frame_counter <= '0;
            frame_pulse   <= 1'b0;
        end else begin
            if (frame_counter >= FRAME_CNT_BITS'(FRAME_CYCLES - 1)) begin
                frame_counter <= '0;
                frame_pulse   <= 1'b1;
            end else begin
                frame_counter <= frame_counter + 1'b1;
                frame_pulse   <= 1'b0;
            end
        end
    end

    // Direct BRAM-to-systolic streaming (2-cycle pipeline: BRAM + decay)
    logic                    scan_active;
    logic [ADDR_BITS-1:0]    scan_addr;
    logic [ADDR_BITS-1:0]    scan_addr_pipe [0:PIPE_DEPTH-1];
    logic [CNT_BITS-1:0]     scan_cnt;
    logic [MOMENT_BITS-1:0]  energy_acc;
    logic                    data_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            scan_active <= 1'b0;
            scan_addr   <= '0;
            scan_cnt    <= '0;
            energy_acc  <= '0;
            data_valid  <= 1'b0;
            for (integer i = 0; i < PIPE_DEPTH; i = i + 1)
                scan_addr_pipe[i] <= '0;
        end else begin
            scan_addr_pipe[0] <= scan_addr;
            for (integer i = 1; i < PIPE_DEPTH; i = i + 1)
                scan_addr_pipe[i] <= scan_addr_pipe[i-1];

            data_valid <= scan_active && (scan_cnt >= CNT_BITS'(PIPE_DEPTH));

            if (frame_pulse) begin
                scan_active <= 1'b1;
                scan_addr   <= '0;
                scan_cnt    <= '0;
                energy_acc  <= '0;
                for (integer i = 0; i < PIPE_DEPTH; i = i + 1)
                    scan_addr_pipe[i] <= '0;
            end else if (scan_active) begin
                if (data_valid)
                    energy_acc <= energy_acc + MOMENT_BITS'(ts_read_value);

                if (scan_addr < ADDR_BITS'(NUM_CELLS - 1)) begin
                    scan_addr <= scan_addr + 1'b1;
                    scan_cnt  <= scan_cnt + 1'b1;
                end else begin
                    if (scan_cnt >= CNT_BITS'(NUM_CELLS + PIPE_DEPTH - 1)) begin
                        scan_active <= 1'b0;
                        scan_cnt    <= '0;
                    end else begin
                        scan_cnt <= scan_cnt + 1'b1;
                    end
                end
            end
        end
    end

    assign ts_read_addr   = scan_addr;
    assign ts_read_enable = scan_active && (scan_addr < ADDR_BITS'(NUM_CELLS));

    logic [ADDR_BITS-1:0]               w_addr;
    logic [NUM_CLASSES*WEIGHT_BITS-1:0] w_data_flat;
    logic signed [WEIGHT_BITS-1:0]      w0, w1, w2, w3;

    weight_rom #(.CLASS_IDX(0), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom0 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w0));

    weight_rom #(.CLASS_IDX(1), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom1 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w1));

    weight_rom #(.CLASS_IDX(2), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom2 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w2));

    weight_rom #(.CLASS_IDX(3), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom3 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w3));

    assign w_data_flat = {w3, w2, w1, w0};

    logic                              sa_start;
    logic [VALUE_BITS-1:0]             sa_feature_in;
    logic                              sa_result_valid;
    logic [1:0]                        sa_best_class;
    logic [SCORE_BITS*NUM_CLASSES-1:0] sa_scores_flat;

    wire signed [SCORE_BITS-1:0] sa_score0 = $signed(sa_scores_flat[1*SCORE_BITS-1:0*SCORE_BITS]);
    wire signed [SCORE_BITS-1:0] sa_score1 = $signed(sa_scores_flat[2*SCORE_BITS-1:1*SCORE_BITS]);
    wire signed [SCORE_BITS-1:0] sa_score2 = $signed(sa_scores_flat[3*SCORE_BITS-1:2*SCORE_BITS]);
    wire signed [SCORE_BITS-1:0] sa_score3 = $signed(sa_scores_flat[4*SCORE_BITS-1:3*SCORE_BITS]);

    systolic_array #(
        .NUM_CLASSES(NUM_CLASSES),
        .NUM_CELLS  (NUM_CELLS),
        .VALUE_BITS (VALUE_BITS),
        .WEIGHT_BITS(WEIGHT_BITS),
        .ACC_BITS   (SCORE_BITS)
    ) u_systolic (
        .clk         (clk),
        .rst         (rst),
        .start       (sa_start),
        .feature_in  (sa_feature_in),
        .w_addr      (w_addr),
        .w_data_flat (w_data_flat),
        .result_valid(sa_result_valid),
        .best_class  (sa_best_class),
        .scores_flat (sa_scores_flat)
    );

    logic                    sa_feeding;
    logic [ADDR_BITS-1:0]    sa_feature_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            sa_start       <= 1'b0;
            sa_feature_in  <= '0;
            sa_feeding     <= 1'b0;
            sa_feature_cnt <= '0;
        end else begin
            sa_start <= 1'b0;

            if (!sa_feeding && data_valid && scan_active) begin
                sa_start       <= 1'b1;
                sa_feature_in  <= ts_read_value;
                sa_feeding     <= 1'b1;
                sa_feature_cnt <= ADDR_BITS'(1);
            end else if (sa_feeding && data_valid) begin
                sa_feature_in <= ts_read_value;
                if (sa_feature_cnt < ADDR_BITS'(NUM_CELLS - 1)) begin
                    sa_feature_cnt <= sa_feature_cnt + 1'b1;
                end else begin
                    sa_feeding     <= 1'b0;
                    sa_feature_cnt <= '0;
                end
            end

            if (frame_pulse) begin
                sa_feeding     <= 1'b0;
                sa_feature_cnt <= '0;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            gesture_class      <= '0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= '0;
        end else begin
            gesture_valid <= 1'b0;
            if (sa_result_valid) begin
                if (energy_acc >= MOMENT_BITS'(MIN_MASS_THRESH)) begin
                    gesture_valid      <= 1'b1;
                    gesture_class      <= sa_best_class;
                    gesture_confidence <= (energy_acc > MOMENT_BITS'(65535))
                                          ? 8'd255
                                          : energy_acc[15:8];
                end
            end
        end
    end

    assign debug_m00   = energy_acc;
    assign debug_m10   = MOMENT_BITS'(sa_score0);
    assign debug_m01   = MOMENT_BITS'(sa_score1);
    assign debug_state = {1'b0, sa_feeding, scan_active};

endmodule
