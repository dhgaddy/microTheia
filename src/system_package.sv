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
    parameter int POR_CYCLES           = 1024,
    parameter int NUM_CLASSES          = 4,
    parameter int SOFT_RESET_CYCLES    = 64,
    // SCORE_BITS must match voxel_bin_core's formula: COUNTER_BITS+WEIGHT_BITS+clog2(FC)+1
    parameter int SCORE_BITS           = COUNTER_BITS + WEIGHT_BITS +
                                         $clog2(READOUT_BINS * GRID_SIZE * GRID_SIZE) + 1
)(
    input  logic clk,
    input  logic reset,
    input  logic MOSI, //master out slave in (from off chip to in chip)
    input  logic SCLK, //no CDC or DLL needed if SCLK sufficiently slower than clk. apparently chip clock must be 4x fast minimum (32 MHz chip, 8 MHz sclk should work)
    input  logic CS, // aka SS, signals a transaction is occuring or not
    output logic MISO, //master in slave out (from in chip to off chip)
    output logic [31:0] debug_bus,
    output logic spi_ready //signal that the spi frontend module has succesfully initialized and is ready to begin operation. NOTE: reset must go high to initialize spi module
);

logic [DATA_WIDTH -1 : 0] word_in, word_out, evt_word;
logic processing_word, process_next_word, evt_valid;

//spi from: https://github.com/janschiefer/verilog_spi
//very transaction oriented, seems to be an unavoidable delay cycle in between every finished transaction
spi_module 
	#( .SPI_MASTER (1'b0),
       .SPI_WORD_LEN (DATA_WIDTH) )
	spi_slave
	( .master_clock(clk),
	.SCLK_OUT(), //not needed because slave
  	.SCLK_IN(SCLK), 
  	.SS_OUT(), //not needed because slave
  	.SS_IN(CS),
	.OUTPUT_SIGNAL(MISO),
	.processing_word(processing_word), //Status: Is a word being processed?
	.process_next_word(process_next_word), //Flag: Set to true to process the next word after the previous word has been processed.
	.data_word_send(word_out),
	.INPUT_SIGNAL(MOSI),
	.data_word_recv(word_in), //updates with each bit received, must not grab until full word packed.
	.do_reset(reset), //must go high at start to initialize
	.is_ready(spi_ready) //signals this SPI module is initialized and ready
    );

    // spi slave we are using does not have a "word valid" signal and updates the word bit by bit,
    // so we need to signal completion by watching for the edge when it finishes processing a word
    // pulsing process_next_word triggers latching and shifting of both word in and word out, so word_out must be ready when it is asserted
    logic processing_word_d;
    logic [2:0] classification_output;
    always_ff @(posedge clk) begin
        if(reset) begin
            evt_word <= '0;
            evt_valid <= 1'b0;
            process_next_word <= 1'b0;
            processing_word_d <= 1'b0;
            classification_output <= '0;
        end
        else begin    
            processing_word_d <= processing_word;
            if (processing_word_d && !processing_word) begin //if falling edge detected then a word jsut finished processing
                evt_word  <= word_in; //take in the valid, stable word
                evt_valid <= 1'b1; 
            end else begin
                evt_valid <= 1'b0;
            end

            if(!processing_word) begin //if we're not processing a word (in both directions), start processing them
                process_next_word <= 1'b1;
            end
            else begin //if a word is being processed set the trigger back low
                process_next_word <= 1'b0;
            end

            if(gesture_valid) begin //latch classification output whenever valid, cycles needed to serially shift a word should be significantly less than cycles per bin when valid signal would toggle
                classification_output <= {gesture_confidence, gesture};
            end    
        end           
    end

    assign word_out = {classification_output, {(DATA_WIDTH - 3){1'b0}}}; //trying continuous assignment so that word_out is ready before the transaction starts
    logic [1:0]  gesture;
    logic gesture_valid, gesture_confidence;

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
    .rst                        (reset),
    .evt_word                   (evt_word),
    .evt_word_valid             (evt_valid),
    .gesture                    (gesture), //2 bits to support 4 classes/gestures
    .gesture_valid              (gesture_valid), //1 bit
    .gesture_confidence         (gesture_confidence), // 1 bit 
    .debug_mux                  (debug_bus),
    .force_rollover_i           (1'b0) // maintaing for ease of access during end-to-end testing
);



endmodule