// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

//cleaned up top level packaging of entire voxel binning -> classification system
//TODO: remove deprecated ports from voxel_bin_core module that are not present here
//using the third party SPI introduces some extra latency cycles because it is strongly transaction oriented
module system_package #(
    parameter int CLK_FREQ_HZ          = 32_000_000,
    parameter int WINDOW_MS            = 1000,
    parameter int GRID_SIZE            = 16,
    parameter int NUM_BINS             = 8,
    parameter int READOUT_BINS         = 8,
    parameter int COUNTER_BITS         = 16,
    parameter int FIFO_DEPTH           = 256,
    parameter int DATA_WIDTH           = 32,
    parameter int REQUIRE_TIME_HIGH    = 1,
    parameter int SENSOR_WIDTH         = 320,
    parameter int SENSOR_HEIGHT        = 320,
    parameter int WEIGHT_BITS          = 8,
    parameter int NUM_CLASSES          = 4,
    // SCORE_BITS must match voxel_bin_core's formula: COUNTER_BITS+WEIGHT_BITS+clog2(FC)+1
    parameter int SCORE_BITS           = COUNTER_BITS + WEIGHT_BITS +
                                         $clog2(READOUT_BINS * GRID_SIZE * GRID_SIZE) + 1
)(
    input  logic clk,
    input  logic rst,
    input  logic MOSI, //master out slave in (from off chip to in chip)
    input  logic SCLK, //no CDC or DLL needed if SCLK sufficiently slower than clk. apparently chip clock must be 4x fast minimum (32 MHz chip, 8 MHz sclk should work)
    input  logic CS, // aka SS, signals a transaction is occuring or not
    output logic MISO, //master in slave out (from in chip to off chip)
    output logic [31:0] debug_bus,
    output logic spi_ready //signal that the spi frontend module has succesfully initialized and is ready to begin operation. NOTE: rst must go high to initialize spi module
);

    logic [DATA_WIDTH-1:0] evt_word;
    logic                  evt_word_valid;
    logic                  evt_word_ready;

    logic [1:0] gesture;
    logic       gesture_valid;
    logic       gesture_confidence;

    //replaced spi module and control logic with wrapper to simplify use
    spi_wrapper #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_spi_wrapper (
        .clk                (clk),
        .rst                (rst),
        .SCLK               (SCLK),
        .CS                 (CS),
        .MOSI               (MOSI),
        .MISO               (MISO),
        .evt_word           (evt_word),
        .evt_word_valid     (evt_word_valid),
        .evt_word_ready_i   (evt_word_ready),
        .gesture            (gesture),
        .gesture_valid      (gesture_valid),
        .gesture_confidence (gesture_confidence),
        .spi_ready          (spi_ready)
    );

    voxel_bin_core #( //I believe these are the only parameters we want in production, any others present in the module should be considered for removal
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .WINDOW_MS        (WINDOW_MS),
        .GRID_SIZE        (GRID_SIZE),
        .NUM_BINS         (NUM_BINS),
        .READOUT_BINS     (READOUT_BINS),
        .COUNTER_BITS     (COUNTER_BITS),
        .FIFO_DEPTH       (FIFO_DEPTH),
        .DATA_WIDTH       (DATA_WIDTH),
        .SENSOR_WIDTH     (SENSOR_WIDTH),
        .SENSOR_HEIGHT    (SENSOR_HEIGHT),
        .WEIGHT_BITS      (WEIGHT_BITS),
        .NUM_CLASSES      (NUM_CLASSES),
        .SCORE_BITS       (SCORE_BITS)
    ) u_core ( //I am excluding several ports that need to be removed from module rather than listing them here and tieing them off.
        .clk                        (clk),
        .rst                        (rst),
        .evt_word                   (evt_word),
        .evt_word_valid             (evt_word_valid),
        .evt_word_ready             (evt_word_ready),
        .gesture                    (gesture), //2 bits to support 4 classes/gestures
        .gesture_valid              (gesture_valid), //1 bit
        .gesture_confidence         (gesture_confidence), // 1 bit 
        .debug_mux                  (debug_bus),
        .force_rollover_i           (1'b0) // maintaing for ease of access during end-to-end testing
    );

endmodule