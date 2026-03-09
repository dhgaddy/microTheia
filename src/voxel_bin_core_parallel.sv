`timescale 1ns/1ps

// Parallel voxel-bin core:
// - Buffers EVT2 words in a FIFO
// - Decodes EVT2 words into (x,y,polarity,timestamp) events
// - Bins events into a NUM_BINS ring of GRID_SIZE*GRID_SIZE histograms
// - When a bin readout starts, capture ONE bin (GRID_SIZE*GRID_SIZE values) into A_matrix_flat
// - In parallel, request weights for each cell (for 4 classes) and build B_matrix_flat[class]
// - Run 4 MatMul engines in parallel: A * B_class
// - Reduce each output matrix into a single score (sum of all outputs)
// - Pick best class, threshold it, and feed persistence-based gesture classifier

module voxel_bin_core_parallel #(
    parameter  CLK_FREQ_HZ        = 12_000_000,
    parameter  WINDOW_MS          = 400,
    parameter  GRID_SIZE          = 16,
    parameter  NUM_BINS           = 4,
    parameter  FIFO_DEPTH         = 128,
    parameter  PERSISTENCE_COUNT  = 2,
    parameter  CYCLES_PER_BIN     = 0,
    parameter  PARALLEL_READS     = 4,     // how many bin counters stream out per cycle
    parameter  DATA_WIDTH         = 32,
    parameter  ACC_SUM_BITS       = 18,
    localparam NUM_CELLS          = NUM_BINS * GRID_SIZE * GRID_SIZE
)(
    input  logic        clk,
    input  logic        rst,

    // Input EVT2 word stream
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,

    // Final gesture outputs
    output logic [1:0]  gesture,
    output logic        gesture_valid,
    output logic [3:0]  gesture_confidence,

    // Debug
    output logic [7:0]  debug_event_count,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full,
    output logic        debug_temporal_phase
);

    localparam integer COUNTER_BITS       = 6;  // per-cell event counter width in binning
    localparam integer NUM_CLASSES        = 4;  // 4 gesture directions/classes
    localparam integer WEIGHT_BITS        = 8;  // weight width from weight RAM
    localparam integer CELLS_PER_BIN      = GRID_SIZE * GRID_SIZE; // 256 for 16x16
    localparam integer CELL_ADDR_BITS     = $clog2(NUM_CELLS);     // address over all bins
    localparam integer BIN_CELL_ADDR_BITS = $clog2(CELLS_PER_BIN); // address within a bin

    // MatMul sizing
    localparam integer MAT_N              = GRID_SIZE;
    localparam integer MAT_DATA_BITS      = WEIGHT_BITS;          // MatMul inputs are 8-bit
    localparam integer MAT_PRODUCT_BITS   = 2 * MAT_DATA_BITS;
    localparam integer MAT_ACC_BITS       = MAT_PRODUCT_BITS + $clog2(MAT_N);
    localparam integer SCORE_BITS         = MAT_ACC_BITS + $clog2(CELLS_PER_BIN);
    localparam integer MIN_SCORE_THRESH   = 30;

    // FIFO wires
    logic        fifo_empty, fifo_full;
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_rd_valid;

    // EVT2 decoder outputs
    logic [3:0]  decoded_grid_x;
    logic [3:0]  decoded_grid_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    // Convert grid coords to signed center-relative for binner
    logic signed [4:0] binner_x, binner_y;

    // Binner readout interface
    logic        readout_start;  // pulse: bin window ended, readout begins
    logic [PARALLEL_READS*COUNTER_BITS-1:0] readout_data; // packed counters
    logic        readout_valid;  // high while readout_data is valid
    logic        binner_event_ready; // binner can accept events (not clearing)

    // Classification result
    logic [1:0]  sys_best_class;
    logic [SCORE_BITS-1:0] sys_best_score;
    logic        sys_result_valid;

    // Inputs to gesture classifier
    logic [17:0] pseudo_mag_x, pseudo_mag_y;
    logic        score_above_thresh;


    // FSM
    typedef enum logic [2:0] {
        ST_IDLE,         // waiting for bin readout_start
        ST_CAPTURE,      // capture A from readout stream, request weights
        ST_WAIT_WEIGHTS, // wait until all weights have returned
        ST_MAT_START,    // pulse start to all MatMul blocks (1 cycle)
        ST_MAT_WAIT      // wait for all MatMul done, then latch result
    } core_state_t;

    core_state_t core_state;

    // Flattened matrices:
    // A is one 16x16 captured bin
    // B is one 16x16 weight matrix per class
    // Out is one 16x16 output per class
    logic [MAT_N*MAT_N*MAT_DATA_BITS-1:0] A_matrix_flat;
    logic [MAT_N*MAT_N*MAT_DATA_BITS-1:0] B_matrix_flat   [0:NUM_CLASSES-1];
    logic [MAT_N*MAT_N*MAT_ACC_BITS-1:0]  Out_matrix_flat [0:NUM_CLASSES-1];

    // MatMul status per class
    logic [NUM_CLASSES-1:0] mat_busy;
    logic [NUM_CLASSES-1:0] mat_done;

    wire all_mat_done = &mat_done; // 1 when every mat_done is 1

    // Counters tracking how much we loaded
    logic [BIN_CELL_ADDR_BITS:0] a_load_count; // number of A cells written
    logic [BIN_CELL_ADDR_BITS:0] w_load_count; // number of weight cells written

    // Weight RAM address/data plumbing
    // We read 4 lanes at once
    // Each lane returns 4 weights one cycle later
    logic [CELL_ADDR_BITS-1:0] w_addr_lane      [0:PARALLEL_READS-1]; // current request address
    logic [CELL_ADDR_BITS-1:0] w_addr_pipe      [0:PARALLEL_READS-1]; // delayed address (for returned data)
    logic                      w_lane_valid_pipe[0:PARALLEL_READS-1]; // delayed valid for returned RAM data

    logic signed [WEIGHT_BITS-1:0] w_data_lane [0:PARALLEL_READS-1][0:NUM_CLASSES-1];

    // Scoring combinational logic:
    // score[class] = sum of all 256 output elements (reduce matrix -> scalar)
    // Then pick argmax
    logic [SCORE_BITS-1:0] class_score_comb [0:NUM_CLASSES-1];
    logic [1:0]            best_class_comb;
    logic [SCORE_BITS-1:0] best_score_comb;

    // Ready/handshake wiring
    // - ready to accept new evt_word when FIFO not full
    // - only read FIFO when binner can accept events
    assign evt_word_ready       = !fifo_full;
    assign fifo_rd_en           = !fifo_empty && binner_event_ready;
    assign debug_fifo_empty     = fifo_empty;
    assign debug_fifo_full      = fifo_full;
    assign debug_temporal_phase = !binner_event_ready; // 1 when binner is NOT ready (clearing)

    // Register fifo_rd_valid so decoder sees valid aligned with data
    always_ff @(posedge clk) begin
        if (rst) fifo_rd_valid <= 1'b0;
        else     fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    // Simple debug counter: count accepted input words
    always_ff @(posedge clk) begin
        if (rst) debug_event_count <= '0;
        else if (evt_word_valid && evt_word_ready)
            debug_event_count <= debug_event_count + 1'b1;
    end

    // FIFO instance: buffers incoming EVT2 words
    input_fifo #(
        .FIFO_DEPTH(FIFO_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
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

    // EVT2 decoder: turns 32-bit words into events
    evt2_decoder #(
        .GRID_SIZE(GRID_SIZE)
    ) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_grid_x),
        .y_out      (decoded_grid_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    // Center grid coords to signed [-8..+7] space for binner
    assign binner_x = $signed({1'b0, decoded_grid_x}) - 5'sd8;
    assign binner_y = $signed({1'b0, decoded_grid_y}) - 5'sd8;

    // Voxel binning:
    // - accumulates events into ring of NUM_BINS histograms
    // - produces readout_start/readout_valid/readout_data at end of bin window
    voxel_binning #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .WINDOW_MS     (WINDOW_MS),
        .GRID_SIZE     (GRID_SIZE),
        .NUM_BINS      (NUM_BINS),
        .READOUT_BINS  (NUM_BINS),
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

    // Weight RAMs:
    // For each lane (0..PARALLEL_READS-1), instantiate one RAM per class.
    // Output comes back 1 cycle after address (sync RAM behavior).
    genvar p, c;
    generate
        for (p = 0; p < PARALLEL_READS; p = p + 1) begin : gen_weight_lanes
            for (c = 0; c < NUM_CLASSES; c = c + 1) begin : gen_weight_classes
                voxel_weight_ram #(
                    .CLASS_IDX  (c),
                    .GRID_SIZE  (GRID_SIZE),
                    .NUM_BINS   (NUM_BINS),
                    .WEIGHT_BITS(WEIGHT_BITS)
                ) u_weight_ram (
                    .clk      (clk),
                    .rst      (rst),
                    .we       (1'b0),          // ROM behavior (no writes)
                    .cell_addr(w_addr_lane[p]),// address requested this cycle
                    .din      ('0),
                    .dout     (w_data_lane[p][c]) // returned weight (next cycle)
                );
            end
        end
    endgenerate

    // 4 MatMul engines in parallel:
    // - same A matrix
    // - different B matrix per class
    // - start pulse when we enter ST_MAT_START
    generate
        for (c = 0; c < NUM_CLASSES; c = c + 1) begin : gen_matmul
            MatMul #(
                .N               (MAT_N),
                .DATA_BIT_SIZE   (MAT_DATA_BITS),
                .PRODUCT_BIT_SIZE(MAT_PRODUCT_BITS),
                .ACC_BIT_SIZE    (MAT_ACC_BITS)
            ) u_matmul (
                .clk            (clk),
                .reset          (rst),
                .start          (core_state == ST_MAT_START),
                .A_matrix_flat  (A_matrix_flat),
                .B_matrix_flat  (B_matrix_flat[c]),
                .Out_matrix_flat(Out_matrix_flat[c]),
                .busy           (mat_busy[c]),
                .done           (mat_done[c])
            );
        end
    endgenerate

    // Score calculation + argmax:
    // score[class] = sum of all 256 output cells
    // best_class = argmax(score)
    always_comb begin
        for (int cls = 0; cls < NUM_CLASSES; cls = cls + 1) begin
            class_score_comb[cls] = '0;
            for (int idx = 0; idx < CELLS_PER_BIN; idx = idx + 1)
                class_score_comb[cls] = class_score_comb[cls] +
                    Out_matrix_flat[cls][idx*MAT_ACC_BITS +: MAT_ACC_BITS];
        end

        best_class_comb = 2'd0;
        best_score_comb = class_score_comb[0];
        for (int cls2 = 1; cls2 < NUM_CLASSES; cls2 = cls2 + 1) begin
            if (class_score_comb[cls2] > best_score_comb) begin
                best_score_comb = class_score_comb[cls2];
                best_class_comb = cls2[1:0];
            end
        end
    end

    // Core FSM:
    // - capture A from readout
    // - request and capture weights into B
    // - start MatMul blocks
    // - wait done, latch best class/score
    always_ff @(posedge clk) begin
        if (rst) begin
            core_state        <= ST_IDLE;
            a_load_count      <= '0;
            w_load_count      <= '0;
            sys_result_valid  <= 1'b0;
            sys_best_class    <= '0;
            sys_best_score    <= '0;

            // clear weight address pipes
            for (int i = 0; i < PARALLEL_READS; i = i + 1) begin
                w_addr_lane[i]       <= '0;
                w_addr_pipe[i]       <= '0;
                w_lane_valid_pipe[i] <= 1'b0;
            end

            // clear A storage
            for (int a = 0; a < CELLS_PER_BIN; a = a + 1)
                A_matrix_flat[a*MAT_DATA_BITS +: MAT_DATA_BITS] <= '0;

            // clear B storage for all classes
            for (int cls = 0; cls < NUM_CLASSES; cls = cls + 1)
                for (int b = 0; b < CELLS_PER_BIN; b = b + 1)
                    B_matrix_flat[cls][b*MAT_DATA_BITS +: MAT_DATA_BITS] <= '0;
        end else begin
            integer prev_weight_writes;
            integer next_a_writes;
            integer rd_idx;

            // default: sys_result_valid is only a pulse when we finish
            sys_result_valid <= 1'b0;

            // Weight return capture (1-cycle delayed)
            // If last cycle we requested weights (w_lane_valid_pipe=1),
            // then this cycle we write those weights into B_matrix_flat.
            prev_weight_writes = 0;
            for (int lane = 0; lane < PARALLEL_READS; lane = lane + 1) begin
                if (w_lane_valid_pipe[lane]) begin
                    prev_weight_writes = prev_weight_writes + 1;
                    for (int clsw = 0; clsw < NUM_CLASSES; clsw = clsw + 1) begin
                        B_matrix_flat[clsw][w_addr_pipe[lane]*MAT_DATA_BITS +: MAT_DATA_BITS] <=
                            w_data_lane[lane][clsw];
                    end
                end
            end
            if (prev_weight_writes != 0)
                w_load_count <= w_load_count + prev_weight_writes[BIN_CELL_ADDR_BITS:0];

            // Default: no new weight returns next cycle unless we set it below
            for (int lane2 = 0; lane2 < PARALLEL_READS; lane2 = lane2 + 1)
                w_lane_valid_pipe[lane2] <= 1'b0;

            // State machine
            case (core_state)
                // Wait for a new binner window to start readout
                ST_IDLE: begin
                    if (readout_start) begin
                        a_load_count <= '0;
                        w_load_count <= '0;
                        core_state   <= ST_CAPTURE;
                    end
                end

                // Capture one full 16x16 bin into A_matrix_flat.
                // In the same loop, issue weight RAM reads for each captured index.
                ST_CAPTURE: begin
                    next_a_writes = 0;

                    if (readout_valid && (a_load_count < CELLS_PER_BIN)) begin
                        for (int lane3 = 0; lane3 < PARALLEL_READS; lane3 = lane3 + 1) begin
                            rd_idx = a_load_count + lane3;
                            if (rd_idx < CELLS_PER_BIN) begin
                                // Store bin counter into A (zero-extend 6-bit -> 8-bit)
                                A_matrix_flat[rd_idx*MAT_DATA_BITS +: MAT_DATA_BITS] <=
                                    {{(MAT_DATA_BITS-COUNTER_BITS){1'b0}},
                                     readout_data[lane3*COUNTER_BITS +: COUNTER_BITS]};

                                // Request weights for this same cell index
                                w_addr_lane[lane3]       <= rd_idx[CELL_ADDR_BITS-1:0];
                                w_addr_pipe[lane3]       <= rd_idx[CELL_ADDR_BITS-1:0];
                                w_lane_valid_pipe[lane3] <= 1'b1; // enables write-back NEXT cycle
                                next_a_writes            = next_a_writes + 1;
                            end
                        end
                    end

                    if (next_a_writes != 0)
                        a_load_count <= a_load_count + next_a_writes[BIN_CELL_ADDR_BITS:0];

                    // once A is captured, move on to waiting for weights to finish returning
                    if (a_load_count >= CELLS_PER_BIN)
                        core_state <= ST_WAIT_WEIGHTS;
                end

                // Wait until we have written ALL weight cells into B_matrix_flat
                ST_WAIT_WEIGHTS: begin
                    if (w_load_count >= CELLS_PER_BIN)
                        core_state <= ST_MAT_START;
                end

                // Pulse start to MatMul blocks for 1 cycle
                ST_MAT_START: begin
                    core_state <= ST_MAT_WAIT;
                end

                // Wait until all MatMul blocks done, then publish best class
                ST_MAT_WAIT: begin
                    if (all_mat_done) begin
                        sys_best_class   <= best_class_comb;
                        sys_best_score   <= best_score_comb;
                        sys_result_valid <= 1'b1;
                        core_state       <= ST_IDLE;
                    end
                end

                default: core_state <= ST_IDLE;
            endcase
        end
    end

    // Threshold and confidence inputs for gesture classifier
    assign score_above_thresh = (sys_best_score >= MIN_SCORE_THRESH);
    assign pseudo_mag_x       = {2'b0, sys_best_score[15:0]};
    assign pseudo_mag_y       = 18'd0;

    // Gesture classifier:
    // adds persistence: requires same class multiple times before outputting gesture_valid
    voxel_gesture_classifier #(
        .ACC_SUM_BITS     (ACC_SUM_BITS),
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

    // Prevent "unused signal" warnings (timestamp not used here)
    wire _unused_decoder_timestamp = decoded_timestamp[0];

endmodule
