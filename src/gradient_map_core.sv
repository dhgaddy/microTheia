`timescale 1ns/1ps

// Gradient-map architecture core.
// Data path:
//   input_fifo -> evt2_decoder -> gradient_mapping -> gesture_classifier

module gradient_map_core #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000,
    parameter FIFO_DEPTH      = 128
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture_class,
    output logic        gesture_valid,
    output logic [7:0]  gesture_confidence,
    output logic        decoded_event_valid,
    output logic [23:0] debug_m00,
    output logic [23:0] debug_m10,
    output logic [23:0] debug_m01,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full
);

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    logic [15:0] global_timestamp;
    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready   = !fifo_full;
    assign fifo_rd_en       = !fifo_empty;
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(.GRID_BITS(4)) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;

    gradient_mapping #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_gradient_mapping (
        .clk        (clk),
        .rst        (rst),
        .t_now      (global_timestamp),
        .event_valid(decoded_valid),
        .event_x    (decoded_x),
        .event_y    (decoded_y),
        .event_ts   (global_timestamp),
        .read_enable(ts_read_enable),
        .read_addr  (ts_read_addr),
        .read_value (ts_read_value),
        .read_ts_raw()
    );

    gesture_classifier #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .FRAME_PERIOD_MS(FRAME_PERIOD_MS),
        .GRID_SIZE      (16),
        .ADDR_BITS      (8),
        .VALUE_BITS     (8),
        .MOMENT_BITS    (24),
        .WEIGHT_BITS    (8),
        .SCORE_BITS     (24),
        .NUM_CLASSES    (4),
        .MIN_MASS_THRESH(MIN_MASS_THRESH)
    ) u_gesture_classifier (
        .clk               (clk),
        .rst               (rst),
        .ts_read_addr      (ts_read_addr),
        .ts_read_enable    (ts_read_enable),
        .ts_read_value     (ts_read_value),
        .gesture_class     (gesture_class),
        .gesture_valid     (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .debug_m00         (debug_m00),
        .debug_m10         (debug_m10),
        .debug_m01         (debug_m01),
        .debug_state       (debug_state)
    );

endmodule

