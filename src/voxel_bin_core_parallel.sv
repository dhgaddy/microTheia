`timescale 1ns/1ps

// Data flow:
// input_fifo -> evt2_decoder -> voxel_binning -> feature_ram
// -> (stream feature+weight reads) -> 4x MAC -> scores_valid -> voxel_gesture_classifier

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
    parameter int              PASS_MARGIN       = 64,
    parameter int              PERSISTENCE_COUNT = 2,
    parameter int              CONF_BITS         = 4,
    parameter int              CONF_SHIFT        = 4,
    parameter int              NUM_CLASSES       = 4,
    parameter int              CYCLES_PER_BIN    = 0,

    // Weight file parameters 
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

    // Sizing for score accumulation:
    // max sum roughly: FEATURE_COUNT * (max_feature * max_weight)
    localparam int PROD_BITS  = COUNTER_BITS + WEIGHT_BITS;
    localparam int ACC_BITS   = PROD_BITS + $clog2(FEATURE_COUNT) + 2; // safety margin
    localparam int SCORE_BITS = ACC_BITS;

    // FIFO -> decoder wiring (ready/valid)
    logic        fifo_out_valid;
    logic        fifo_out_ready;
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

    // Feature RAM read port
    logic                    feature_rd_valid;
    logic [FEATURE_BITS-1:0] feature_rd_addr;
    logic [COUNTER_BITS-1:0] feature_rd_data;

    // Weight ROM read port
    logic [WEIGHT_ADDR_BITS-1:0] weight_rd_addr;
    logic                        weight_rd_valid;
    logic [WEIGHT_BITS-1:0]      weight_rd_raw [0:NUM_CLASSES-1];

    // MAC engine outputs
    logic [NUM_CLASSES-1:0] mac_score_valid;
    logic [SCORE_BITS-1:0] mac_score [0:NUM_CLASSES-1];

    // Packed scores for gesture classifier
    logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat;
    logic scores_valid;

    // Gesture classifier internal outputs
    logic [1:0] class_gesture;
    logic       class_valid;
    logic       class_pass;

    // Scoring FSM
    typedef enum logic [1:0] {
        SC_IDLE   = 2'd0,
        SC_RUN    = 2'd1,  // issue reads over all indices
        SC_WAIT   = 2'd2,  // wait for last MAC valid pulse
        SC_PUB    = 2'd3   // publish scores_valid for 1 cycle
    } score_state_t;

    score_state_t score_state;

    logic [FEATURE_BITS-1:0] run_idx;

    // Pipeline to align sync RAM outputs
    logic                  req_v_d;
    logic                  req_last_d;
    logic [COUNTER_BITS-1:0] feat_d;

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

    // Binner readout allowed only when we are idle and not holding a prior window
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

    // Feature RAM: store full feature window
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

    // Weight ROMs
    // - simulation uses ram_1r1w_sync reading from WEIGHT file
    // - synthesis uses $readmemh paths
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

    // Streaming read requests during SC_RUN:
    // Each cycle, request:
    //   feature[run_idx] from feature_ram
    //   weights[*][run_idx] from weight ROMs
    // Both are synchronous, so data appears next cycle.
    always_comb begin
        feature_rd_valid = 1'b0;
        feature_rd_addr  = '0;
        weight_rd_valid  = 1'b0;
        weight_rd_addr   = '0;

        if (score_state == SC_RUN) begin
            feature_rd_valid = 1'b1;
            feature_rd_addr  = run_idx;
            weight_rd_valid  = 1'b1;
            weight_rd_addr   = run_idx[WEIGHT_ADDR_BITS-1:0];
        end
    end

    // Pipeline the request-valid and "last" marker to align with returned RAM data
    always_ff @(posedge clk) begin
        if (rst) begin
            req_v_d    <= 1'b0;
            req_last_d <= 1'b0;
            feat_d     <= '0;
        end else begin
            req_v_d    <= (score_state == SC_RUN);                 // returned data valid next cycle
            req_last_d <= (score_state == SC_RUN) && (run_idx == FEATURE_COUNT-1);
            feat_d     <= feature_rd_data;                         // returned feature value
        end
    end

    // Instantiate 4 MACs (one per class), all driven by the same feature stream
    // and each using its class weight stream.
    // - start pulse clears accumulators at the beginning of a run
    // - feature_valid/weight_valid are aligned (req_v_d)
    // - feature_last marks the last multiply
    logic mac_start_pulse;

    always_ff @(posedge clk) begin
        if (rst) begin
            mac_start_pulse <= 1'b0;
        end else begin
            // Pulse start when we ENTER SC_RUN (one cycle)
            mac_start_pulse <= (score_state == SC_IDLE) && feature_window_ready;
        end
    end

    generate
        for (genvar cls = 0; cls < NUM_CLASSES; cls = cls + 1) begin : gen_macs
            voxel_mac #(
                .FEAT_BITS   (COUNTER_BITS),
                .WEIGHT_BITS (WEIGHT_BITS),
                .ACC_BITS    (SCORE_BITS)
            ) u_mac (
                .clk          (clk),
                .rst          (rst),
                .start        (mac_start_pulse),
                .feature_valid(req_v_d),
                .feature_value(feat_d),
                .feature_last (req_last_d),
                .weight_valid (req_v_d),
                .weight_value (weight_rd_raw[cls]),
                .score_valid  (mac_score_valid[cls]),
                .score        (mac_score[cls])
            );
        end
    endgenerate

    // All MACs should assert score_valid together on the last element
    wire all_mac_valid = &mac_score_valid;

    // Pack scores for gesture classifier
    always_comb begin
        for (int cls = 0; cls < NUM_CLASSES; cls = cls + 1)
            scores_flat[cls*SCORE_BITS +: SCORE_BITS] = mac_score[cls];
    end

    // Score FSM:
    // - Wait for feature_window_ready
    // - Stream through all FEATURE_COUNT indices (one per cycle)
    // - Wait for MAC valid pulse, then publish scores_valid
    always_ff @(posedge clk) begin
        if (rst) begin
            score_state            <= SC_IDLE;
            run_idx                <= '0;
            scores_valid           <= 1'b0;
            consume_feature_window <= 1'b0;
        end else begin
            scores_valid           <= 1'b0;
            consume_feature_window <= 1'b0;

            case (score_state)
                SC_IDLE: begin
                    if (feature_window_ready) begin
                        consume_feature_window <= 1'b1; // clear the ready flag
                        run_idx    <= '0;
                        score_state <= SC_RUN;
                    end
                end

                SC_RUN: begin
                    // Advance index each cycle while requesting reads
                    if (run_idx == FEATURE_COUNT-1)
                        score_state <= SC_WAIT;
                    else
                        run_idx <= run_idx + 1'b1;
                end

                SC_WAIT: begin
                    // Wait for the final MAC pulse (comes 1 cycle after the last read request)
                    if (all_mac_valid)
                        score_state <= SC_PUB;
                end

                SC_PUB: begin
                    // Publish scores_valid for 1 cycle (gesture classifier consumes it)
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

    wire _unused_decoder_outputs = dec_timestamp[0];

endmodule
