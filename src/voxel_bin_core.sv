// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 Group G Contributors
`timescale 1ns/1ps

// Weights and thresholds are stored in writable GF180MCU SRAMs.
// They start at zero after reset and must be loaded via the external
// weight_wr_* / thresh_wr_* ports before classification begins.
// All four weight SRAMs share the same write address; the class is
// selected by weight_wr_class_i.

module voxel_bin_core #(
    parameter int CLK_FREQ_HZ       = 12_000_000,
    parameter int WINDOW_MS         = 1000,
    parameter int GRID_SIZE         = 16,
    parameter int NUM_BINS          = 8,
    parameter int READOUT_BINS      = 8,
    parameter int COUNTER_BITS      = 16,
    parameter int FIFO_DEPTH        = 256,
    parameter int DATA_WIDTH        = 32,
    parameter int REQUIRE_TIME_HIGH = 1,
    parameter int SWAP_INPUT_BYTES  = 0,
    parameter int SENSOR_WIDTH      = 320,
    parameter int SENSOR_HEIGHT     = 320,
    parameter int WEIGHT_BITS       = 8,
    parameter int NUM_CLASSES       = 4,
    parameter int CYCLES_PER_BIN    = 0,
    // SCORE_BITS as a parameter (not localparam) so it can appear in port widths.
    // Default matches the formula used internally; callers should not override this.
    parameter int SCORE_BITS        = COUNTER_BITS + WEIGHT_BITS +
                                      $clog2(READOUT_BINS * GRID_SIZE * GRID_SIZE) + 1
)(
    input  logic       clk,
    input  logic       rst,

    // Mode control
    input  logic [1:0] active_mode_i, // 00=PROGRAM, 01=CLASSIFY, 10=DEBUG

    // Event stream in
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,

    // Gesture outputs
    output logic [1:0]  gesture,
    output logic        gesture_valid,
    output logic        gesture_confidence,

    // Weight SRAM write port — loads weights into the per-class SRAMs at runtime.
    // Do not assert weight_wr_valid_i while the MAC engine is running (mac_busy).
    input  logic                                                weight_wr_valid_i,
    input  logic [1:0]                                          weight_wr_class_i,
    input  logic [$clog2(READOUT_BINS*GRID_SIZE*GRID_SIZE)-1:0] weight_wr_addr_i,
    input  logic [WEIGHT_BITS-1:0]                              weight_wr_data_i,

    // Threshold SRAM write port — addr 0-3 = class thresholds, 4-7 = diff thresholds.
    input  logic                  thresh_wr_valid_i,
    input  logic [2:0]            thresh_wr_addr_i,
    input  logic [SCORE_BITS-1:0] thresh_wr_data_i,

    // Debug outputs
    output logic [7:0] debug_event_count,
    output logic       debug_fifo_empty,
    output logic       debug_fifo_full,
    output logic       debug_temporal_phase,
    output logic       debug_class_valid,
    output logic       debug_class_pass,
    output logic       debug_feature_window_ready,
    output logic       debug_capture_active,
    output logic       debug_score_busy
);

    // Mode constants
    typedef enum logic [1:0] {
        MODE_PROGRAM  = 2'b00,
        MODE_CLASSIFY = 2'b01,
        MODE_DEBUG    = 2'b10
    } state_t;

    // Classification constants
    localparam int FEATURE_COUNT    = READOUT_BINS * GRID_SIZE * GRID_SIZE;
    localparam int FEATURE_BITS     = $clog2(FEATURE_COUNT);
    localparam int WEIGHT_ADDR_BITS = $clog2(FEATURE_COUNT);

    // Mode-derived enable signals
    logic mode_program;
    logic mode_classify;
 
    assign mode_program  = (active_mode_i == MODE_PROGRAM);
    assign mode_classify = (active_mode_i == MODE_CLASSIFY);
 
    // Gated SRAM write valids that only pass through in PROGRAM mode
    logic weight_wr_valid_gated;
    logic thresh_wr_valid_gated;
 
    assign weight_wr_valid_gated = weight_wr_valid_i && mode_program;
    assign thresh_wr_valid_gated = thresh_wr_valid_i && mode_program;

    // Internal wires
    logic        fifo_out_valid;
    logic        fifo_out_ready;
    logic [31:0] fifo_out_data;

    logic [($clog2(GRID_SIZE))-1:0] dec_x16;
    logic [($clog2(GRID_SIZE))-1:0] dec_y16;
    logic                           dec_event_valid;
    logic                           dec_data_ready;

    logic                    binner_event_ready;
    logic                    binner_readout_ready;
    logic                    binner_readout_start;
    logic                    binner_readout_valid;
    logic [COUNTER_BITS-1:0] binner_readout_data;
    logic [FEATURE_BITS-1:0] binner_readout_index;
    logic                    binner_readout_last;

    logic capture_active;
    logic feature_window_ready;

    logic                    feature_rd_valid;
    logic [FEATURE_BITS-1:0] feature_rd_addr;
    logic [COUNTER_BITS-1:0] feature_rd_data;

    logic [WEIGHT_ADDR_BITS-1:0] weight_rd_addr;
    logic                        weight_rd_valid;
    logic [WEIGHT_BITS-1:0]      weight_rd_raw [0:NUM_CLASSES-1];

    logic                               mac_start;
    logic                               mac_busy;
    logic                               mac_rd_en;
    logic [FEATURE_BITS-1:0]            mac_rd_addr;
    logic [NUM_CLASSES*WEIGHT_BITS-1:0] mac_weight_flat;
    logic [NUM_CLASSES*SCORE_BITS-1:0]  mac_scores_flat;
    logic                               mac_scores_valid;

    logic [1:0] class_gesture;
    logic       class_valid;
    logic       class_pass;

    logic                  thresh_rd_valid;
    logic [2:0]            thresh_rd_addr;
    logic [SCORE_BITS-1:0] thresh_data;

    // mac_start fires when a feature window is ready and the engine is idle
    assign mac_start = feature_window_ready && !mac_busy;

    assign feature_rd_valid = mac_rd_en;
    assign feature_rd_addr  = mac_rd_addr;
    assign weight_rd_valid  = mac_rd_en;
    assign weight_rd_addr   = mac_rd_addr;

    always_comb begin
        for (int g = 0; g < NUM_CLASSES; g++)
            mac_weight_flat[g*WEIGHT_BITS +: WEIGHT_BITS] = weight_rd_raw[g];
    end

    assign binner_readout_ready = (!capture_active) && (!mac_busy) && (!feature_window_ready);
    assign fifo_out_ready       = dec_data_ready;

    // Debug signal assignments
    assign debug_fifo_empty           = ~fifo_out_valid;
    assign debug_fifo_full            = ~evt_word_ready;
    assign debug_temporal_phase       = ~binner_event_ready;
    assign debug_class_valid          = class_valid;
    assign debug_class_pass           = class_pass;
    assign debug_feature_window_ready = feature_window_ready;
    assign debug_capture_active       = capture_active;
    assign debug_score_busy           = mac_busy;

    always_ff @(posedge clk) begin
        if (rst)
            debug_event_count <= '0;
        else if (evt_word_valid && evt_word_ready)
            debug_event_count <= debug_event_count + 1'b1;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            capture_active       <= 1'b0;
            feature_window_ready <= 1'b0;
        end else begin
            if (mac_start)
                feature_window_ready <= 1'b0;
            if (binner_readout_start)
                capture_active <= 1'b1;
            if (binner_readout_valid && binner_readout_last) begin
                capture_active       <= 1'b0;
                feature_window_ready <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Input FIFO
    // ------------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // EVT2 decoder
    // ------------------------------------------------------------------
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
        .event_valid  (dec_event_valid)
    );

    // ------------------------------------------------------------------
    // Voxel binning
    // ------------------------------------------------------------------
    voxel_binning #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .WINDOW_MS     (WINDOW_MS),
        .GRID_SIZE     (GRID_SIZE),
        .NUM_BINS      (NUM_BINS),
        .READOUT_BINS  (READOUT_BINS),
        .COUNTER_BITS  (COUNTER_BITS),
        .CYCLES_PER_BIN(CYCLES_PER_BIN)
    ) u_voxel_binning (
        .clk          (clk),
        .rst          (rst),
        .event_valid  (dec_event_valid),
        .event_x      (dec_x16),
        .event_y      (dec_y16),
        .event_ready  (binner_event_ready),
        .readout_ready(binner_readout_ready),
        .readout_start(binner_readout_start),
        .readout_valid(binner_readout_valid),
        .readout_data (binner_readout_data),
        .readout_index(binner_readout_index),
        .readout_last (binner_readout_last)
    );

    // ------------------------------------------------------------------
    // Feature RAM (written by binner readout, read by MAC engine)
    // ------------------------------------------------------------------
    gf180_sram_1r1w #(
        .width_p(COUNTER_BITS),
        .depth_p(FEATURE_COUNT)
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

    // ------------------------------------------------------------------
    // Weight SRAMs x NUM_CLASSES (writable at runtime via weight_wr_* ports)
    // ------------------------------------------------------------------
    genvar g;
    generate
        for (g = 0; g < NUM_CLASSES; g++) begin : gen_weight_ram
            gf180_sram_1r1w #(
                .width_p(WEIGHT_BITS),
                .depth_p(FEATURE_COUNT)
            ) u_weight_ram (
                .clk_i      (clk),
                .reset_i    (rst),
                .wr_valid_i (weight_wr_valid_gated && (weight_wr_class_i == 2'(g))),
                .wr_data_i  (weight_wr_data_i),
                .wr_addr_i  (weight_wr_addr_i),
                .rd_valid_i (weight_rd_valid),
                .rd_addr_i  (weight_rd_addr),
                .rd_data_o  (weight_rd_raw[g])
            );
        end
    endgenerate

    // ------------------------------------------------------------------
    // Threshold SRAM (writable at runtime; addr 0-3 = class, 4-7 = diff)
    // ------------------------------------------------------------------
    gf180_sram_1r1w #(
        .width_p(SCORE_BITS),
        .depth_p(2 * NUM_CLASSES)
    ) u_thresh_ram (
        .clk_i      (clk),
        .reset_i    (rst),
        .wr_valid_i (thresh_wr_valid_gated),
        .wr_data_i  (thresh_wr_data_i),
        .wr_addr_i  (thresh_wr_addr_i),
        .rd_valid_i (thresh_rd_valid),
        .rd_addr_i  (thresh_rd_addr),
        .rd_data_o  (thresh_data)
    );

    // ------------------------------------------------------------------
    // MAC engine
    // ------------------------------------------------------------------
    voxel_mac_engine #(
        .FEATURE_COUNT(FEATURE_COUNT),
        .COUNTER_BITS (COUNTER_BITS),
        .WEIGHT_BITS  (WEIGHT_BITS),
        .NUM_CLASSES  (NUM_CLASSES),
        .SCORE_BITS   (SCORE_BITS)
    ) u_voxel_mac_engine (
        .clk              (clk),
        .rst              (rst),
        .start            (mac_start),
        .busy             (mac_busy),
        .rd_en            (mac_rd_en),
        .rd_addr          (mac_rd_addr),
        .feature_data     (feature_rd_data),
        .weight_data_flat (mac_weight_flat),
        .scores_flat      (mac_scores_flat),
        .scores_valid     (mac_scores_valid)
    );

    // ------------------------------------------------------------------
    // Gesture classifier
    // ------------------------------------------------------------------
    voxel_gesture_classifier #(
        .NUM_CLASSES(NUM_CLASSES),
        .SCORE_BITS (SCORE_BITS)
    ) u_voxel_gesture_classifier (
        .clk               (clk),
        .rst               (rst),
        .scores_flat       (mac_scores_flat),
        .scores_valid      (mac_scores_valid),
        .thresh_rd_valid   (thresh_rd_valid),
        .thresh_rd_addr    (thresh_rd_addr),
        .thresh_data       (thresh_data),
        .class_gesture     (class_gesture),
        .class_valid       (class_valid),
        .class_pass        (class_pass),
        .gesture           (gesture),
        .gesture_valid     (gesture_valid),
        .gesture_confidence(gesture_confidence)
    );

endmodule
