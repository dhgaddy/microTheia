`timescale 1ns/1ps

// Full voxel-bin processing core.
// Data flow:
// EVT2 word stream -> evt2_input_fifo -> EVT2Decoder -> TimeSurfaceBinning
// -> SystolicMatrixMultiply + weight_ram -> OutputRegister

module voxel_bin_core #(
    parameter CLK_FREQ_HZ        = 12_000_000,
    parameter WINDOW_MS          = 400,
    parameter GRID_SIZE          = 16,
    parameter FIFO_DEPTH         = 128,
    parameter MIN_EVENT_THRESH   = 20,
    parameter MOTION_THRESH      = 8,
    parameter PERSISTENCE_COUNT  = 2,
    parameter CYCLES_PER_BIN     = 0,
    parameter PARALLEL_READS     = 4
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture,
    output logic        gesture_valid,
    output logic [3:0]  gesture_confidence,
    output logic [7:0]  debug_event_count,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full,
    output logic        debug_temporal_phase
);

    localparam integer FIFO_PTR_BITS   = $clog2(FIFO_DEPTH);
    localparam integer COUNTER_BITS    = 6;
    localparam integer NUM_CLASSES     = 4;
    localparam integer NUM_BINS        = 4;
    localparam integer NUM_CELLS       = NUM_BINS * GRID_SIZE * GRID_SIZE;
    localparam integer WEIGHT_BITS     = 8;
    localparam integer ACC_BITS        = 24;
    localparam integer MIN_SCORE_THRESH = 30;

    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_rd_valid;

    logic [3:0]  decoded_grid_x;
    logic [3:0]  decoded_grid_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;
    logic signed [4:0] binner_x;
    logic signed [4:0] binner_y;

    logic        readout_start;
    logic [PARALLEL_READS*COUNTER_BITS-1:0] readout_data;
    logic        readout_valid;
    logic        binner_event_ready;

    logic [PARALLEL_READS*$clog2(NUM_CELLS)-1:0]       w_addr_flat;
    logic [PARALLEL_READS*NUM_CLASSES*WEIGHT_BITS-1:0] w_data_flat;
    logic [$clog2(NUM_CELLS)-1:0]       w_addr [0:PARALLEL_READS-1];
    logic [NUM_CLASSES*WEIGHT_BITS-1:0] w_data [0:PARALLEL_READS-1];
    logic                               sys_result_valid;
    logic [1:0]                         sys_best_class;
    logic [NUM_CLASSES*ACC_BITS-1:0]    sys_scores_flat;

    logic signed [ACC_BITS-1:0] sys_scores [0:NUM_CLASSES-1];
    logic signed [ACC_BITS-1:0] best_score;
    logic [ACC_BITS-1:0]        abs_best_score;
    logic                       score_above_thresh;
    logic [17:0]                pseudo_mag_x;
    logic [17:0]                pseudo_mag_y;

    assign evt_word_ready      = !fifo_full;
    // Do not consume FIFO words while voxel_binning is clearing a bin.
    // This prevents silent event loss during clear cycles.
    assign fifo_rd_en          = !fifo_empty && binner_event_ready;
    assign debug_fifo_empty    = fifo_empty;
    assign debug_fifo_full     = fifo_full;
    assign debug_temporal_phase = 1'b0;

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    always_ff @(posedge clk) begin
        if (rst)
            debug_event_count <= '0;
        else if (evt_word_valid && evt_word_ready)
            debug_event_count <= debug_event_count + 1'b1;
    end

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk    (clk),
        .rst    (rst),
        .wr_en  (evt_word_valid && evt_word_ready),
        .wr_data(evt_word),
        .rd_en  (fifo_rd_en),
        .rd_data(fifo_rd_data),
        .empty  (fifo_empty),
        .full   (fifo_full),
        .count  ()
    );

    evt2_decoder #(
        .GRID_BITS(4)
    ) u_evt2_decoder (
        .clk       (clk),
        .rst       (rst),
        .data_in   (fifo_rd_data),
        .data_valid(fifo_rd_valid),
        .data_ready(),
        .x_out     (decoded_grid_x),
        .y_out     (decoded_grid_y),
        .polarity  (decoded_polarity),
        .timestamp (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign binner_x = $signed({1'b0, decoded_grid_x}) - 5'sd8;
    assign binner_y = $signed({1'b0, decoded_grid_y}) - 5'sd8;

    voxel_binning #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .WINDOW_MS     (WINDOW_MS),
        .NUM_BINS      (NUM_BINS),
        .READOUT_BINS  (NUM_BINS),
        .GRID_SIZE     (GRID_SIZE),
        .COUNTER_BITS  (COUNTER_BITS),
        .PARALLEL_READS(PARALLEL_READS),
        .CYCLES_PER_BIN(CYCLES_PER_BIN)
    ) u_voxel_binning (
        .clk           (clk),
        .rst           (rst),
        .event_valid   (decoded_valid),
        .event_x       (binner_x),
        .event_y       (binner_y),
        .event_polarity(decoded_polarity),
        .event_ready   (binner_event_ready),
        .readout_start (readout_start),
        .readout_data  (readout_data),
        .readout_valid (readout_valid)
    );

    systolic_array #(
        .NUM_CLASSES    (NUM_CLASSES),
        .NUM_CELLS      (NUM_CELLS),
        .VALUE_BITS     (COUNTER_BITS),
        .WEIGHT_BITS    (WEIGHT_BITS),
        .ACC_BITS       (ACC_BITS),
        .PARALLEL_INPUTS(PARALLEL_READS)
    ) u_systolic_array (
        .clk          (clk),
        .rst          (rst),
        .start        (readout_start),
        .feature_in   (readout_data),
        .feature_valid(readout_valid),
        .w_addr_flat  (w_addr_flat),
        .w_data_flat  (w_data_flat),
        .result_valid (sys_result_valid),
        .best_class   (sys_best_class),
        .scores_flat  (sys_scores_flat)
    );

    generate
        genvar pa;
        for (pa = 0; pa < PARALLEL_READS; pa = pa + 1) begin : gen_unpack_addr
            assign w_addr[pa] = w_addr_flat[(pa+1)*$clog2(NUM_CELLS)-1 : pa*$clog2(NUM_CELLS)];
        end
    endgenerate

    generate
        genvar p, k;
        for (p = 0; p < PARALLEL_READS; p = p + 1) begin : gen_parallel_rams
            for (k = 0; k < NUM_CLASSES; k = k + 1) begin : gen_class_rams
                weight_ram #(
                    .CLASS_IDX  (k),
                    .NUM_CELLS  (NUM_CELLS),
                    .GRID_SIZE  (GRID_SIZE),
                    .WEIGHT_BITS(WEIGHT_BITS)
                ) u_weight_ram (
                    .clk      (clk),
                    .rst      (rst),
                    .we       (1'b0),
                    .cell_addr(w_addr[p]),
                    .din      ('0),
                    .dout     (w_data[p][(k+1)*WEIGHT_BITS-1 : k*WEIGHT_BITS])
                );
            end
        end
    endgenerate

    generate
        genvar pd;
        for (pd = 0; pd < PARALLEL_READS; pd = pd + 1) begin : gen_pack_data
            assign w_data_flat[(pd+1)*NUM_CLASSES*WEIGHT_BITS-1 : pd*NUM_CLASSES*WEIGHT_BITS] = w_data[pd];
        end
    endgenerate

    generate
        genvar gi;
        for (gi = 0; gi < NUM_CLASSES; gi = gi + 1) begin : gen_unpack_sys_scores
            assign sys_scores[gi] = sys_scores_flat[(gi+1)*ACC_BITS-1 : gi*ACC_BITS];
        end
    endgenerate

    always_comb begin
        best_score = sys_scores[sys_best_class];

        if (best_score < 0)
            abs_best_score = -best_score;
        else
            abs_best_score = best_score;

        score_above_thresh = (abs_best_score >= MIN_SCORE_THRESH);
    end

    assign pseudo_mag_x = {2'b0, abs_best_score[15:0]};
    assign pseudo_mag_y = 18'd0;

    gesture_classifier #(
        .ACC_SUM_BITS     (18),
        .PERSISTENCE_COUNT(PERSISTENCE_COUNT)
    ) u_gesture_classifier (
        .clk               (clk),
        .rst               (rst),
        .class_gesture     (sys_best_class),
        .class_valid       (sys_result_valid),
        .class_pass        (sys_result_valid && score_above_thresh),
        .abs_delta_x       (pseudo_mag_x),
        .abs_delta_y       (pseudo_mag_y),
        .gesture           (gesture),
        .gesture_valid     (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .debug_state       (debug_state)
    );

endmodule
