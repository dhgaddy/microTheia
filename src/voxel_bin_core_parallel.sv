`timescale 1ns/1ps

// Parallel version of voxel_bin_core
//
// Data flow:
// input_fifo -> evt2_decoder -> voxel_binning -> feature_ram
// -> (tile load) -> 4x voxel_systolic_array (one per class) -> accumulate scores
// -> voxel_gesture_classifier

module voxel_bin_core_parallel #(
    parameter int              CLK_FREQ_HZ       = 12_000_000,
    parameter int              WINDOW_MS         = 400,
    parameter int              GRID_SIZE         = 8,
    parameter int              NUM_BINS          = 4,
    parameter int              READOUT_BINS      = 4,
    parameter int              COUNTER_BITS      = 4,
    parameter int              FIFO_DEPTH        = 256,
    parameter int              DATA_WIDTH        = 32,
    parameter int              REQUIRE_TIME_HIGH = 1,
    parameter int              SWAP_INPUT_BYTES  = 0,
    parameter int              SENSOR_WIDTH      = 320,
    parameter int              SENSOR_HEIGHT     = 320,
    parameter int              WEIGHT_BITS       = 8,
    parameter int              WEIGHT_SCALE      = 1024,
    parameter int              N                 = 4,
    parameter int              PASS_MARGIN       = 64,
    parameter int              PERSISTENCE_COUNT = 2,
    parameter int              CONF_BITS         = 4,
    parameter int              CONF_SHIFT        = 4,
    parameter int              NUM_CLASSES       = 4,
    parameter int              CYCLES_PER_BIN    = 0,
    parameter [8*128-1:0]      WEIGHT            = "weights/gesture_weights_down_left_right_up_8x8_4bins.txt",
    parameter [8*128-1:0]      WEIGHT_MEM_C0     = "../weights/256weights_q8_c0.mem",
    parameter [8*128-1:0]      WEIGHT_MEM_C1     = "../weights/256weights_q8_c1.mem",
    parameter [8*128-1:0]      WEIGHT_MEM_C2     = "../weights/256weights_q8_c2.mem",
    parameter [8*128-1:0]      WEIGHT_MEM_C3     = "../weights/256weights_q8_c3.mem"
)(
    input  logic                 clk,
    input  logic                 rst,
    input  logic [31:0]          evt_word,
    input  logic                 evt_word_valid,
    output logic                 evt_word_ready,
    output logic [1:0]           gesture,
    output logic                 gesture_valid,
    output logic [CONF_BITS-1:0] gesture_confidence,
    output logic [7:0]           debug_event_count,
    output logic [2:0]           debug_state,
    output logic                 debug_fifo_empty,
    output logic                 debug_fifo_full,
    output logic                 debug_temporal_phase,
    output logic                 debug_class_valid,
    output logic                 debug_class_pass,
    output logic                 debug_feature_window_ready,
    output logic                 debug_capture_active,
    output logic                 debug_score_busy
);

    // Derived sizes
    localparam int FEATURE_COUNT    = READOUT_BINS * GRID_SIZE * GRID_SIZE;
    localparam int FEATURE_BITS     = $clog2(FEATURE_COUNT);
    localparam int GRID_BITS        = $clog2(GRID_SIZE);
    localparam int WEIGHT_ADDR_BITS = $clog2(FEATURE_COUNT);
    localparam int TILES            = FEATURE_COUNT / N;
    localparam int SA_DATA_BITS     = ((COUNTER_BITS > WEIGHT_BITS) ? COUNTER_BITS : WEIGHT_BITS) + 1;
    localparam int SA_PRODUCT_BITS  = 2 * SA_DATA_BITS;
    localparam int SA_ACC_BITS      = SA_PRODUCT_BITS + $clog2(N);
    localparam int SCORE_BITS       = SA_ACC_BITS + $clog2(TILES) + 2;
    localparam int LOAD_BITS        = $clog2(N + 1);
    localparam int TILE_BITS        = (TILES > 1) ? $clog2(TILES) : 1;

    // Sanity: tile config must be valid
    generate
        if ((FEATURE_COUNT % N) != 0) begin : gen_invalid_tile_config
            initial $error("voxel_bin_core_parallel: FEATURE_COUNT (%0d) must be divisible by N (%0d)", FEATURE_COUNT, N);
        end
    endgenerate

    // Score FSM 
    typedef enum logic [2:0] {
        SC_IDLE      = 3'd0,
        SC_LOAD      = 3'd1,
        SC_SYS_START = 3'd2,
        SC_SYS_WAIT  = 3'd3,
        SC_ACCUM     = 3'd4,
        SC_PUBLISH   = 3'd5
    } score_state_t;

    score_state_t score_state;

    // FIFO -> decoder wiring (ready/valid)
    logic fifo_out_valid;
    logic fifo_out_ready;
    logic [31:0] fifo_out_data;

    // Decoder outputs
    logic [GRID_BITS-1:0] dec_x16;
    logic [GRID_BITS-1:0] dec_y16;
    logic                 dec_polarity;
    logic [33:0]          dec_timestamp;
    logic                 dec_event_valid;
    logic                 dec_data_ready;

    // Binner control
    logic                 binner_event_ready;
    logic                 binner_readout_ready;
    logic                 binner_readout_start;
    logic                 binner_readout_valid;
    logic [COUNTER_BITS-1:0] binner_readout_data;
    logic [FEATURE_BITS-1:0] binner_readout_index;
    logic                 binner_readout_last;

    // Capture window tracking
    logic capture_active;
    logic feature_window_ready;
    logic consume_feature_window;

    // Feature RAM read port (tile load)
    logic                    feature_rd_valid;
    logic [FEATURE_BITS-1:0] feature_rd_addr;
    logic [COUNTER_BITS-1:0] feature_rd_data;

    // Weight ROM read port (tile load)
    logic [WEIGHT_ADDR_BITS-1:0] weight_rd_addr;
    logic                        weight_rd_valid;
    logic [WEIGHT_BITS-1:0]      weight_rd_raw [0:NUM_CLASSES-1];

    // Tile counters + registers for a tile
    logic [LOAD_BITS-1:0] load_cycle;
    logic [TILE_BITS-1:0] tile_idx;
    logic [SA_DATA_BITS-1:0] a_row [0:N-1];
    logic [WEIGHT_BITS-1:0]  weight_tile [0:N-1][0:NUM_CLASSES-1];

    // Parallel systolic engines:
    // Each class gets its own A/B/Out and done/busy.
    // We'll build B matrices so only column 0 carries the class's weights.
    // Then output[0,0] equals the dot-product for that class.
    logic [N*N*SA_DATA_BITS-1:0] sa_a_flat;
    logic [N*N*SA_DATA_BITS-1:0] sa_b_flat_cls [0:NUM_CLASSES-1];
    logic [N*N*SA_ACC_BITS-1:0]  sa_out_flat_cls [0:NUM_CLASSES-1];
    logic [NUM_CLASSES-1:0]      sa_busy_cls;
    logic [NUM_CLASSES-1:0]      sa_done_cls;

    logic sa_start;
    wire  all_sa_done = &sa_done_cls;

    // Accumulated class scores
    logic [SCORE_BITS-1:0] score_acc [0:NUM_CLASSES-1];
    logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat;
    logic scores_valid;

    // Gesture classifier interface
    logic [1:0] class_gesture;
    logic       class_valid;
    logic       class_pass;

    // Debug outputs
    assign debug_fifo_empty            = ~fifo_out_valid;
    assign debug_fifo_full             = ~evt_word_ready;
    assign debug_temporal_phase        = ~binner_event_ready;
    assign debug_class_valid           = class_valid;
    assign debug_class_pass            = class_pass;
    assign debug_feature_window_ready  = feature_window_ready;
    assign debug_capture_active        = capture_active;
    assign debug_score_busy            = (score_state != SC_IDLE);

    // FIFO ready comes from decoder ready
    assign fifo_out_ready = dec_data_ready;

    // Binner readout is allowed only when we are idle and not holding a prior window
    assign binner_readout_ready = (!capture_active) && (score_state == SC_IDLE) && (!feature_window_ready);

    // Count accepted input words
    always_ff @(posedge clk) begin
        if (rst)
            debug_event_count <= '0;
        else if (evt_word_valid && evt_word_ready)
            debug_event_count <= debug_event_count + 1'b1;
    end

    // input_fifo (ready/valid style)
    input_fifo #(
        .FIFO_DEPTH(FIFO_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_input_fifo (
        .clk_i   (clk),
        .reset_i (rst),
        .data_i  (evt_word),
        .ready_i (fifo_out_ready),
        .valid_i (evt_word_valid),
        .ready_o (evt_word_ready),
        .valid_o (fifo_out_valid),
        .data_o  (fifo_out_data)
    );

    // EVT2 decoder
    evt2_decoder #(
        .SENSOR_WIDTH     (SENSOR_WIDTH),
        .SENSOR_HEIGHT    (SENSOR_HEIGHT),
        .GRID_SIZE        (GRID_SIZE),
        .REQUIRE_TIME_HIGH(REQUIRE_TIME_HIGH),
        .SWAP_INPUT_BYTES (SWAP_INPUT_BYTES)
    ) u_evt2_decoder (
        .clk          (clk),
        .rst          (rst),
        .data_in      (fifo_out_data),
        .data_valid   (fifo_out_valid),
        .event_ready_i(binner_event_ready),
        .data_ready   (dec_data_ready),
        .x_out        (dec_x16),
        .y_out        (dec_y16),
        .polarity     (dec_polarity),
        .timestamp    (dec_timestamp),
        .event_valid  (dec_event_valid)
    );

    // Voxel binning
    voxel_binning #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .WINDOW_MS     (WINDOW_MS),
        .GRID_SIZE     (GRID_SIZE),
        .NUM_BINS      (NUM_BINS),
        .READOUT_BINS  (READOUT_BINS),
        .COUNTER_BITS  (COUNTER_BITS),
        .CYCLES_PER_BIN(CYCLES_PER_BIN)
    ) u_voxel_binning (
        .clk           (clk),
        .rst           (rst),
        .event_valid   (dec_event_valid),
        .event_x       (dec_x16),
        .event_y       (dec_y16),
        .event_polarity(dec_polarity),
        .event_ready   (binner_event_ready),
        .readout_ready (binner_readout_ready),
        .readout_start (binner_readout_start),
        .readout_valid (binner_readout_valid),
        .readout_data  (binner_readout_data),
        .readout_index (binner_readout_index),
        .readout_last  (binner_readout_last)
    );

    // Feature RAM: store the full feature window (FEATURE_COUNT entries)
    // - binner streams (data,index) pairs
    // - we write them into RAM for later tile reads
    ram_1r1w_sync #(
        .width_p (COUNTER_BITS),
        .depth_p (FEATURE_COUNT)
    ) u_feature_ram (
        .clk_i      (clk),
        .reset_i    (rst),
        .wr_valid_i (binner_readout_valid),
        .wr_data_i  (binner_readout_data),
        .wr_addr_i  (binner_readout_index),
        .rd_valid_i (feature_rd_valid),
        .rd_addr_i  (feature_rd_addr),
        .rd_data_o  (feature_rd_data)
    );

    // Track capture_active and feature_window_ready
    always_ff @(posedge clk) begin
        if (rst) begin
            capture_active       <= 1'b0;
            feature_window_ready <= 1'b0;
        end else begin
            if (consume_feature_window)
                feature_window_ready <= 1'b0;

            if (binner_readout_start)
                capture_active <= 1'b1;

            if (binner_readout_valid && binner_readout_last) begin
                capture_active       <= 1'b0;
                feature_window_ready <= 1'b1;
            end
        end
    end

    // Tile load control:
    // During SC_LOAD, read 1 feature + 1 weight address each cycle for N cycles
    always_comb begin
        weight_rd_valid  = 1'b0;
        weight_rd_addr   = '0;
        feature_rd_valid = 1'b0;
        feature_rd_addr  = '0;

        if ((score_state == SC_LOAD) && (load_cycle < N)) begin
            weight_rd_valid  = 1'b1;
            weight_rd_addr   = (tile_idx * N) + load_cycle;
            feature_rd_valid = 1'b1;
            feature_rd_addr  = (tile_idx * N) + load_cycle;
        end
    end

    // Weight ROMs
    // - simulation uses ram_1r1w_sync reading from WEIGHT file
    // - synthesis uses $readmemh paths (parameters)
`ifdef SYNTHESIS
    logic [WEIGHT_BITS-1:0] weight_mem_c0 [0:FEATURE_COUNT-1];
    logic [WEIGHT_BITS-1:0] weight_mem_c1 [0:FEATURE_COUNT-1];
    logic [WEIGHT_BITS-1:0] weight_mem_c2 [0:FEATURE_COUNT-1];
    logic [WEIGHT_BITS-1:0] weight_mem_c3 [0:FEATURE_COUNT-1];

    initial begin
        $readmemh(WEIGHT_MEM_C0, weight_mem_c0);
        $readmemh(WEIGHT_MEM_C1, weight_mem_c1);
        $readmemh(WEIGHT_MEM_C2, weight_mem_c2);
        $readmemh(WEIGHT_MEM_C3, weight_mem_c3);
    end

    always_ff @(posedge clk) begin
        if (1'b0) begin
            weight_mem_c0[0] <= '0;
            weight_mem_c1[0] <= '0;
            weight_mem_c2[0] <= '0;
            weight_mem_c3[0] <= '0;
        end
        if (weight_rd_valid) begin
            weight_rd_raw[0] <= weight_mem_c0[weight_rd_addr];
            weight_rd_raw[1] <= weight_mem_c1[weight_rd_addr];
            weight_rd_raw[2] <= weight_mem_c2[weight_rd_addr];
            weight_rd_raw[3] <= weight_mem_c3[weight_rd_addr];
        end
    end
`else
    genvar g;
    generate
        for (g = 0; g < NUM_CLASSES; g = g + 1) begin : gen_weight_rams
            ram_1r1w_sync #(
                .width_p        (WEIGHT_BITS),
                .depth_p        (FEATURE_COUNT),
                .filename_p     (WEIGHT),
                .init_offset_p  (g * FEATURE_COUNT),
                .init_count_p   (FEATURE_COUNT),
                .init_is_float_p(1'b1),
                .init_scale_p   (WEIGHT_SCALE),
                .init_signed_p  (1'b0)
            ) u_weight_ram (
                .clk_i      (clk),
                .reset_i    (rst),
                .wr_valid_i (1'b0),
                .wr_data_i  ('0),
                .wr_addr_i  ('0),
                .rd_valid_i (weight_rd_valid),
                .rd_addr_i  (weight_rd_addr),
                .rd_data_o  (weight_rd_raw[g])
            );
        end
    endgenerate
`endif

    // Build systolic inputs:
    // - sa_a_flat: put a_row[k] into row 0, col k
    // - sa_b_flat_cls[class]:
    //      put weight_tile[k][class] into row k, col 0
    //      (everything else zero)
    // This makes output[0,0] == dot(a_row, weight_vec_for_class).
    always_comb begin
        // default clear A
        for (int r = 0; r < N; r = r + 1)
            for (int c = 0; c < N; c = c + 1)
                sa_a_flat[(r*N + c)*SA_DATA_BITS +: SA_DATA_BITS] = '0;

        // default clear all class B matrices
        for (int cls = 0; cls < NUM_CLASSES; cls = cls + 1)
            for (int r = 0; r < N; r = r + 1)
                for (int c = 0; c < N; c = c + 1)
                    sa_b_flat_cls[cls][(r*N + c)*SA_DATA_BITS +: SA_DATA_BITS] = '0;

        // fill row0 of A
        for (int k = 0; k < N; k = k + 1) begin
            sa_a_flat[(0*N + k)*SA_DATA_BITS +: SA_DATA_BITS] = a_row[k];
        end

        // fill col0 of each class's B
        for (int k = 0; k < N; k = k + 1) begin
            for (int cls = 0; cls < NUM_CLASSES; cls = cls + 1) begin
                sa_b_flat_cls[cls][(k*N + 0)*SA_DATA_BITS +: SA_DATA_BITS] =
                    {{(SA_DATA_BITS-WEIGHT_BITS){1'b0}}, weight_tile[k][cls]};
            end
        end
    end

    // Start pulse to all systolic engines
    assign sa_start = (score_state == SC_SYS_START);

    // Instantiate 4 systolic engines (one per class)
    genvar gc;
    generate
        for (gc = 0; gc < NUM_CLASSES; gc = gc + 1) begin : gen_sa_per_class
            voxel_systolic_array #(
                .N             (N),
                .DATA_BIT_SIZE (SA_DATA_BITS)
            ) u_voxel_systolic_array (
                .clk            (clk),
                .reset          (rst),
                .start          (sa_start),
                .A_matrix_flat  (sa_a_flat),
                .B_matrix_flat  (sa_b_flat_cls[gc]),
                .Out_matrix_flat(sa_out_flat_cls[gc]),
                .busy           (sa_busy_cls[gc]),
                .done           (sa_done_cls[gc])
            );
        end
    endgenerate

    // Pack scores for gesture classifier
    always_comb begin
        for (int gi = 0; gi < NUM_CLASSES; gi = gi + 1)
            scores_flat[gi*SCORE_BITS +: SCORE_BITS] = score_acc[gi];
    end

    // Score FSM
    //  - waits for all_sa_done instead of sa_done
    //  - accumulates from each class's sa_out_flat_cls[cls]
    always_ff @(posedge clk) begin
        if (rst) begin
            score_state            <= SC_IDLE;
            load_cycle             <= '0;
            tile_idx               <= '0;
            scores_valid           <= 1'b0;
            consume_feature_window <= 1'b0;

            for (int c = 0; c < NUM_CLASSES; c = c + 1)
                score_acc[c] <= '0;

            for (int k = 0; k < N; k = k + 1) begin
                a_row[k] <= '0;
                for (int c = 0; c < NUM_CLASSES; c = c + 1)
                    weight_tile[k][c] <= '0;
            end

        end else begin
            scores_valid           <= 1'b0;
            consume_feature_window <= 1'b0;

            case (score_state)
                // Wait until a full feature window has been captured into feature_ram
                SC_IDLE: begin
                    if (feature_window_ready) begin
                        consume_feature_window <= 1'b1;
                        tile_idx   <= '0;
                        load_cycle <= '0;

                        for (int c = 0; c < NUM_CLASSES; c = c + 1)
                            score_acc[c] <= '0;

                        score_state <= SC_LOAD;
                    end
                end

                // Load N features + N weights (one per cycle).
                // NOTE: reads are synchronous; data arrives the following cycle.
                SC_LOAD: begin
                    if (load_cycle > 0) begin
                        int cap_idx;
                        cap_idx = load_cycle - 1;

                        // feature -> a_row
                        a_row[cap_idx] <= {{(SA_DATA_BITS-COUNTER_BITS){1'b0}}, feature_rd_data};

                        // weights -> weight_tile[cap_idx][class]
                        for (int c = 0; c < NUM_CLASSES; c = c + 1)
                            weight_tile[cap_idx][c] <= weight_rd_raw[c];
                    end

                    if (load_cycle == N) begin
                        score_state <= SC_SYS_START;
                    end else begin
                        load_cycle <= load_cycle + 1'b1;
                    end
                end

                // Pulse start to all systolic engines
                SC_SYS_START: begin
                    score_state <= SC_SYS_WAIT;
                end

                // Wait for all 4 systolic engines to finish
                SC_SYS_WAIT: begin
                    if (all_sa_done)
                        score_state <= SC_ACCUM;
                end

                // Accumulate the dot-product outputs into score_acc[class]
                // We read output[0,0] from each class engine.
                SC_ACCUM: begin
                    for (int c = 0; c < NUM_CLASSES; c = c + 1) begin
                        score_acc[c] <= score_acc[c] + sa_out_flat_cls[c][(0*N + 0)*SA_ACC_BITS +: SA_ACC_BITS];
                    end

                    if (tile_idx == TILES - 1) begin
                        score_state <= SC_PUBLISH;
                    end else begin
                        tile_idx    <= tile_idx + 1'b1;
                        load_cycle  <= '0;
                        score_state <= SC_LOAD;
                    end
                end

                // Publish the scores vector for gesture classifier
                SC_PUBLISH: begin
                    scores_valid <= 1'b1;
                    score_state  <= SC_IDLE;
                end

                default: score_state <= SC_IDLE;
            endcase
        end
    end

    // Gesture classifier
    voxel_gesture_classifier #(
        .NUM_CLASSES       (NUM_CLASSES),
        .SCORE_BITS        (SCORE_BITS),
        .PASS_MARGIN       (PASS_MARGIN),
        .PERSISTENCE_COUNT (PERSISTENCE_COUNT),
        .CONF_BITS         (CONF_BITS),
        .CONF_SHIFT        (CONF_SHIFT)
    ) u_voxel_gesture_classifier (
        .clk               (clk),
        .rst               (rst),
        .scores_flat       (scores_flat),
        .scores_valid      (scores_valid),
        .class_gesture     (class_gesture),
        .class_valid       (class_valid),
        .class_pass        (class_pass),
        .gesture           (gesture),
        .gesture_valid     (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .debug_state       (debug_state)
    );

    // Avoid unused warning
    wire _unused_decoder_outputs = dec_timestamp[0];

endmodule
