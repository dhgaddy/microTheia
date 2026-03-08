`timescale 1ns/1ps

// Full voxel-bin processing core.
// Data flow:
// input_fifo -> evt2_decoder -> voxel_binning -> systolic_array (tiled GEMV) -> gesture_classifier

module voxel_bin_core #(
    parameter int CLK_FREQ_HZ       = 12_000_000,
    parameter int WINDOW_MS         = 1000,
    parameter int GRID_SIZE         = 16,
    parameter int NUM_BINS          = 8,
    parameter int READOUT_BINS      = 8,
    parameter int COUNTER_BITS      = 16,
    parameter int FIFO_DEPTH_LOG2   = 8,
    parameter int SENSOR_WIDTH      = 320,
    parameter int SENSOR_HEIGHT     = 320,
    parameter int WEIGHT_BITS       = 8,
    parameter int WEIGHT_SCALE      = 1024,
    parameter int N                 = 16,
    parameter int PASS_MARGIN       = 64,
    parameter int PERSISTENCE_COUNT = 2,
    parameter int CONF_BITS         = 4,
    parameter int CONF_SHIFT        = 4,
    parameter int CYCLES_PER_BIN    = 0
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture,
    output logic        gesture_valid,
    output logic [CONF_BITS-1:0] gesture_confidence,
    output logic [7:0]  debug_event_count,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full,
    output logic        debug_temporal_phase,
    output logic        debug_class_valid,
    output logic        debug_class_pass,
    output logic        debug_feature_window_ready,
    output logic        debug_capture_active,
    output logic        debug_score_busy
);

    localparam int NUM_CLASSES   = 4;
    localparam int FEATURE_COUNT = READOUT_BINS * GRID_SIZE * GRID_SIZE;
    localparam int FEATURE_BITS  = $clog2(FEATURE_COUNT);
    localparam int GRID_BITS     = $clog2(GRID_SIZE);
    localparam int WEIGHT_FILE_CLASS_STRIDE = 256;
    localparam int WEIGHT_ADDR_BITS = $clog2(FEATURE_COUNT);
    localparam int TILES         = FEATURE_COUNT / N;
    localparam int SA_DATA_BITS  = ((COUNTER_BITS > WEIGHT_BITS) ? COUNTER_BITS : WEIGHT_BITS) + 1;
    localparam int SA_PRODUCT_BITS = 2 * SA_DATA_BITS;
    localparam int SA_ACC_BITS   = SA_PRODUCT_BITS + $clog2(N);
    localparam int SCORE_BITS    = SA_ACC_BITS + $clog2(TILES) + 2;
    localparam int LOAD_BITS     = $clog2(N + 1);
    localparam int TILE_BITS     = (TILES > 1) ? $clog2(TILES) : 1;

    typedef enum logic [2:0] {
        SC_IDLE      = 3'd0,
        SC_LOAD      = 3'd1,
        SC_SYS_START = 3'd2,
        SC_SYS_WAIT  = 3'd3,
        SC_ACCUM     = 3'd4,
        SC_PUBLISH   = 3'd5
    } score_state_t;

    score_state_t score_state;

    logic fifo_out_valid;
    logic fifo_out_ready;
    logic [31:0] fifo_out_data;

    logic [GRID_BITS-1:0] dec_x16;
    logic [GRID_BITS-1:0] dec_y16;
    logic                 dec_polarity;
    logic [33:0]          dec_timestamp;
    logic                 dec_event_valid;
    logic                 dec_data_ready;

    logic                 binner_event_ready;
    logic                 binner_readout_ready;
    logic                 binner_readout_start;
    logic                 binner_readout_valid;
    logic [COUNTER_BITS-1:0] binner_readout_data;
    logic [FEATURE_BITS-1:0] binner_readout_index;
    logic                 binner_readout_last;

    logic                    capture_active;
    logic                    feature_window_ready;
    logic                    consume_feature_window;
    logic                    feature_rd_valid;
    logic [FEATURE_BITS-1:0] feature_rd_addr;
    logic [COUNTER_BITS-1:0] feature_rd_data;

    logic [WEIGHT_ADDR_BITS-1:0] weight_rd_addr;
    logic                        weight_rd_valid;
    logic [WEIGHT_BITS-1:0]      weight_rd_raw [0:NUM_CLASSES-1];

    logic [LOAD_BITS-1:0] load_cycle;
    logic [TILE_BITS-1:0] tile_idx;
    logic [SA_DATA_BITS-1:0] a_row [0:N-1];
    logic [WEIGHT_BITS-1:0]  weight_tile [0:N-1][0:NUM_CLASSES-1];

    logic [N*N*SA_DATA_BITS-1:0] sa_a_flat;
    logic [N*N*SA_DATA_BITS-1:0] sa_b_flat;
    logic [N*N*SA_ACC_BITS-1:0]  sa_out_flat;
    logic                        sa_start;
    logic                        sa_busy;
    logic                        sa_done;

    logic [SCORE_BITS-1:0] score_acc [0:NUM_CLASSES-1];
    int cap_idx;
    logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat;
    logic scores_valid;

    logic [1:0] class_gesture;
    logic       class_valid;
    logic       class_pass;

    generate
        if ((FEATURE_COUNT % N) != 0) begin : gen_invalid_tile_config
            initial $error("voxel_bin_core: FEATURE_COUNT (%0d) must be divisible by N (%0d)", FEATURE_COUNT, N);
        end
        if (FEATURE_COUNT > WEIGHT_FILE_CLASS_STRIDE) begin : gen_invalid_weight_stride
            initial $error("voxel_bin_core: FEATURE_COUNT (%0d) exceeds weight file class stride (%0d)",
                           FEATURE_COUNT, WEIGHT_FILE_CLASS_STRIDE);
        end
    endgenerate

    assign debug_fifo_empty     = ~fifo_out_valid;
    assign debug_fifo_full      = ~evt_word_ready;
    assign debug_temporal_phase = ~binner_event_ready;
    assign debug_class_valid    = class_valid;
    assign debug_class_pass     = class_pass;
    assign debug_feature_window_ready = feature_window_ready;
    assign debug_capture_active = capture_active;
    assign debug_score_busy     = (score_state != SC_IDLE);
    assign fifo_out_ready       = dec_data_ready;
    assign binner_readout_ready = (!capture_active) && (score_state == SC_IDLE) && (!feature_window_ready);

    always_ff @(posedge clk) begin
        if (rst)
            debug_event_count <= '0;
        else if (evt_word_valid && evt_word_ready)
            debug_event_count <= debug_event_count + 1'b1;
    end

    input_fifo #(
        .width_p(32),
        .depth_log2_p(FIFO_DEPTH_LOG2)
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

    evt2_decoder #(
        .GRID_BITS        (GRID_BITS),
        .SENSOR_WIDTH     (SENSOR_WIDTH),
        .SENSOR_HEIGHT    (SENSOR_HEIGHT),
        .REQUIRE_TIME_HIGH(1'b1)
    ) u_evt2_decoder (
        .clk         (clk),
        .rst         (rst),
        .data_in     (fifo_out_data),
        .data_valid  (fifo_out_valid),
        .event_ready_i(binner_event_ready),
        .data_ready  (dec_data_ready),
        .x_out       (dec_x16),
        .y_out       (dec_y16),
        .polarity    (dec_polarity),
        .timestamp   (dec_timestamp),
        .event_valid (dec_event_valid)
    );

    voxel_binning #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .WINDOW_MS     (WINDOW_MS),
        .NUM_BINS      (NUM_BINS),
        .READOUT_BINS  (READOUT_BINS),
        .GRID_SIZE     (GRID_SIZE),
        .COUNTER_BITS  (COUNTER_BITS),
        .CYCLES_PER_BIN(CYCLES_PER_BIN)
    ) u_voxel_binning (
        .clk          (clk),
        .rst          (rst),
        .event_valid  (dec_event_valid),
        .event_x      (dec_x16),
        .event_y      (dec_y16),
        .event_polarity(dec_polarity),
        .event_ready  (binner_event_ready),
        .readout_ready(binner_readout_ready),
        .readout_start(binner_readout_start),
        .readout_valid(binner_readout_valid),
        .readout_data (binner_readout_data),
        .readout_index(binner_readout_index),
        .readout_last (binner_readout_last)
    );

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

    always_comb begin
        weight_rd_valid = 1'b0;
        weight_rd_addr  = '0;
        feature_rd_valid = 1'b0;
        feature_rd_addr  = '0;
        if ((score_state == SC_LOAD) && (load_cycle < N)) begin
            weight_rd_valid = 1'b1;
            weight_rd_addr  = (tile_idx * N) + load_cycle;
            feature_rd_valid = 1'b1;
            feature_rd_addr  = (tile_idx * N) + load_cycle;
        end
    end

    // Weight ROMs.
    // Synthesis: inline arrays with literal $readmemh paths. A dummy write port (if(1'b0))
    //   is required so Yosys infers SB_RAM40_4K and propagates INIT_* attributes.
    //   Yosys does NOT evaluate $readmemh when the filename comes from a module parameter,
    //   so ram_1r1w_sync cannot be used for synthesis weight init.
    // Simulation: ram_1r1w_sync with float init from 8192weights.txt.
`ifdef SYNTHESIS
    logic [WEIGHT_BITS-1:0] weight_mem_c0 [0:FEATURE_COUNT-1];
    logic [WEIGHT_BITS-1:0] weight_mem_c1 [0:FEATURE_COUNT-1];
    logic [WEIGHT_BITS-1:0] weight_mem_c2 [0:FEATURE_COUNT-1];
    logic [WEIGHT_BITS-1:0] weight_mem_c3 [0:FEATURE_COUNT-1];

    initial begin
        $readmemh("../weights/256weights_q8_c0.mem", weight_mem_c0);
        $readmemh("../weights/256weights_q8_c1.mem", weight_mem_c1);
        $readmemh("../weights/256weights_q8_c2.mem", weight_mem_c2);
        $readmemh("../weights/256weights_q8_c3.mem", weight_mem_c3);
    end

    always_ff @(posedge clk) begin
        if (1'b0) begin  // Dummy write: required for Yosys SB_RAM40_4K inference.
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
    generate
        genvar g;
        for (g = 0; g < NUM_CLASSES; g = g + 1) begin : gen_weight_rams
            ram_1r1w_sync #(
                .width_p        (WEIGHT_BITS),
                .depth_p        (FEATURE_COUNT),
                .filename_p     ("weights/gesture_weights_down_left_right_up_8x8_4bins.txt"),
                .init_offset_p  (g * WEIGHT_FILE_CLASS_STRIDE),
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

    always_comb begin
        for (int r = 0; r < N; r = r + 1) begin
            for (int c = 0; c < N; c = c + 1) begin
                sa_a_flat[(r*N + c)*SA_DATA_BITS +: SA_DATA_BITS] = '0;
                sa_b_flat[(r*N + c)*SA_DATA_BITS +: SA_DATA_BITS] = '0;
            end
        end

        for (int k = 0; k < N; k = k + 1) begin
            sa_a_flat[(0*N + k)*SA_DATA_BITS +: SA_DATA_BITS] = a_row[k];
            for (int gc = 0; gc < NUM_CLASSES; gc = gc + 1) begin
                sa_b_flat[(k*N + gc)*SA_DATA_BITS +: SA_DATA_BITS] =
                    {{(SA_DATA_BITS-WEIGHT_BITS){1'b0}}, weight_tile[k][gc]};
            end
        end
    end

    always_comb begin
        for (int gi = 0; gi < NUM_CLASSES; gi = gi + 1)
            scores_flat[gi*SCORE_BITS +: SCORE_BITS] = score_acc[gi];
    end

    assign sa_start = (score_state == SC_SYS_START);

    systolic_array #(
        .N               (N),
        .DATA_BIT_SIZE   (SA_DATA_BITS),
        .PRODUCT_BIT_SIZE(SA_PRODUCT_BITS),
        .ACC_BIT_SIZE    (SA_ACC_BITS)
    ) u_systolic_array (
        .clk          (clk),
        .reset        (rst),
        .start        (sa_start),
        .A_matrix_flat(sa_a_flat),
        .B_matrix_flat(sa_b_flat),
        .Out_matrix_flat(sa_out_flat),
        .busy         (sa_busy),
        .done         (sa_done)
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            score_state           <= SC_IDLE;
            load_cycle            <= '0;
            tile_idx              <= '0;
            scores_valid          <= 1'b0;
            consume_feature_window<= 1'b0;
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

                SC_LOAD: begin
                    if (load_cycle > 0) begin
                        cap_idx = load_cycle - 1;
                        a_row[cap_idx] <= {{(SA_DATA_BITS-COUNTER_BITS){1'b0}}, feature_rd_data};
                        for (int c = 0; c < NUM_CLASSES; c = c + 1)
                            weight_tile[cap_idx][c] <= weight_rd_raw[c];
                    end

                    if (load_cycle == N) begin
                        score_state <= SC_SYS_START;
                    end else begin
                        load_cycle <= load_cycle + 1'b1;
                    end
                end

                SC_SYS_START: begin
                    score_state <= SC_SYS_WAIT;
                end

                SC_SYS_WAIT: begin
                    if (sa_done)
                        score_state <= SC_ACCUM;
                end

                SC_ACCUM: begin
                    for (int c = 0; c < NUM_CLASSES; c = c + 1) begin
                        score_acc[c] <= score_acc[c] + sa_out_flat[(0*N + c)*SA_ACC_BITS +: SA_ACC_BITS];
                    end

                    if (tile_idx == TILES - 1) begin
                        score_state <= SC_PUBLISH;
                    end else begin
                        tile_idx    <= tile_idx + 1'b1;
                        load_cycle  <= '0;
                        score_state <= SC_LOAD;
                    end
                end

                SC_PUBLISH: begin
                    scores_valid <= 1'b1;
                    score_state  <= SC_IDLE;
                end

                default: score_state <= SC_IDLE;
            endcase
        end
    end

    gesture_classifier #(
        .NUM_CLASSES       (NUM_CLASSES),
        .SCORE_BITS        (SCORE_BITS),
        .PASS_MARGIN       (PASS_MARGIN),
        .PERSISTENCE_COUNT (PERSISTENCE_COUNT),
        .CONF_BITS         (CONF_BITS),
        .CONF_SHIFT        (CONF_SHIFT)
    ) u_gesture_classifier (
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

    wire _unused_decoder_outputs = dec_timestamp[0];

endmodule
