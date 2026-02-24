`timescale 1ns/1ps

// Top-level for gradient-map gesture classifier — pre-decoded event bus input.
// An upstream MCU or deserializer has already decoded the raw EVT2 stream and
// downsampled coordinates to the 16x16 grid.  Events arrive as (x[3:0], y[3:0],
// polarity, valid) — only 10 bus pins.  Timestamps are generated internally.
// The FIFO and EVT2 decoder are bypassed entirely.
// UART TX is retained for gesture result output.
// No external reset pin; power-on reset is generated internally.
// Target: Lattice iCE40UP5K on iCEBreaker board.

module gradient_map_processed_top #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter BAUD_RATE       = 115200,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000
)(
    input  logic       clk,
    // Pre-decoded, grid-mapped event stream from upstream MCU/deserializer.
    // x and y are already in 0–15 grid coordinates (4 bits each).
    input  logic       event_valid,
    input  logic [3:0] event_x,
    input  logic [3:0] event_y,
    input  logic       event_polarity,
    output logic       event_ready,
    // UART TX for gesture result output
    output logic       uart_tx,
    output logic       led_heartbeat,
    output logic       led_activity,
    output logic       led_up,
    output logic       led_down,
    output logic       led_left,
    output logic       led_right
);

    // -------------------------------------------------------------------------
    // Power-on reset
    // -------------------------------------------------------------------------
    reg [4:0] por_cnt = 5'd0;
    wire rst_n_internal = &por_cnt;
    logic rst;
    logic [3:0] rst_sync;

    always @(posedge clk)
        if (!rst_n_internal)
            por_cnt <= por_cnt + 1'b1;

    always_ff @(posedge clk) begin
        rst_sync <= {rst_sync[2:0], ~rst_n_internal};
        rst      <= rst_sync[3];
    end

    // -------------------------------------------------------------------------
    // Global timestamp counter
    // -------------------------------------------------------------------------
    logic [15:0] global_timestamp;

    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    // -------------------------------------------------------------------------
    // Back-pressure: always ready (no FIFO in this path)
    // -------------------------------------------------------------------------
    assign event_ready = 1'b1;

    // -------------------------------------------------------------------------
    // Time-surface encoder
    // -------------------------------------------------------------------------
    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;

    time_surface_encoder #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_time_surface (
        .clk         (clk),
        .rst         (rst),
        .t_now       (global_timestamp),
        .event_valid (event_valid),
        .event_x     (event_x),
        .event_y     (event_y),
        .event_ts    (global_timestamp),
        .read_enable (ts_read_enable),
        .read_addr   (ts_read_addr),
        .read_value  (ts_read_value),
        .read_ts_raw (ts_read_raw)
    );

    // -------------------------------------------------------------------------
    // Spatio-temporal classifier
    // -------------------------------------------------------------------------
    logic [1:0]  gesture_class;
    logic        gesture_valid;
    logic [7:0]  gesture_confidence;
    logic [23:0] debug_m00, debug_m10, debug_m01;
    logic [2:0]  debug_state;

    spatio_temporal_classifier #(
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
    ) u_spatio_classifier (
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

    // -------------------------------------------------------------------------
    // UART debug output
    // -------------------------------------------------------------------------
    uart_debug #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE  (BAUD_RATE)
    ) u_uart_debug (
        .clk               (clk),
        .rst               (rst),
        .gesture_class     (gesture_class),
        .gesture_valid     (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .uart_tx           (uart_tx)
    );

    // -------------------------------------------------------------------------
    // Heartbeat LED ~1.5 Hz at 12 MHz
    // -------------------------------------------------------------------------
    logic [22:0] heartbeat_cnt;

    always_ff @(posedge clk) begin
        if (rst)
            heartbeat_cnt <= '0;
        else
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
    end

    assign led_heartbeat = heartbeat_cnt[22];

    // -------------------------------------------------------------------------
    // Activity LED: pulses on events
    // -------------------------------------------------------------------------
    logic [19:0] activity_cnt;

    always_ff @(posedge clk) begin
        if (rst)
            activity_cnt <= '0;
        else if (event_valid)
            activity_cnt <= {20{1'b1}};
        else if (activity_cnt > 0)
            activity_cnt <= activity_cnt - 1'b1;
    end

    assign led_activity = (activity_cnt > 0);

    // -------------------------------------------------------------------------
    // Gesture direction LEDs with ~500 ms pulse stretch
    // -------------------------------------------------------------------------
    logic [23:0] led_up_cnt, led_down_cnt, led_left_cnt, led_right_cnt;
    localparam LED_STRETCH = 24'd6_000_000;

    always_ff @(posedge clk) begin
        if (rst) begin
            led_up_cnt    <= '0;
            led_down_cnt  <= '0;
            led_left_cnt  <= '0;
            led_right_cnt <= '0;
        end else begin
            if (gesture_valid && gesture_class == 2'd0)
                led_up_cnt <= LED_STRETCH;
            else if (led_up_cnt > 0)
                led_up_cnt <= led_up_cnt - 1'b1;

            if (gesture_valid && gesture_class == 2'd1)
                led_down_cnt <= LED_STRETCH;
            else if (led_down_cnt > 0)
                led_down_cnt <= led_down_cnt - 1'b1;

            if (gesture_valid && gesture_class == 2'd2)
                led_left_cnt <= LED_STRETCH;
            else if (led_left_cnt > 0)
                led_left_cnt <= led_left_cnt - 1'b1;

            if (gesture_valid && gesture_class == 2'd3)
                led_right_cnt <= LED_STRETCH;
            else if (led_right_cnt > 0)
                led_right_cnt <= led_right_cnt - 1'b1;
        end
    end

    assign led_up    = (led_up_cnt > 0);
    assign led_down  = (led_down_cnt > 0);
    assign led_left  = (led_left_cnt > 0);
    assign led_right = (led_right_cnt > 0);

endmodule
