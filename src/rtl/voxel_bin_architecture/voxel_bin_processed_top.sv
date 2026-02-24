// Top-level for voxel-bin DVS gesture accelerator — pre-decoded event bus input.
// An upstream MCU or deserializer has already decoded the raw sensor stream and
// downsampled coordinates to the 16x16 grid.  Events arrive as (x[3:0], y[3:0],
// polarity, valid) — only 10 bus pins.  The FPGA maps grid coords back to the
// centre of each 20-pixel sensor bin before passing to dvs_gesture_accel.
// Timestamps are generated internally.  UART TX is retained for gesture output.
// No external reset pin; power-on reset is generated internally.
// Target: Lattice iCE40UP5K on iCEBreaker board.

`timescale 1ns/1ps

module voxel_bin_processed_top #(
    parameter CLK_FREQ          = 12_000_000,
    parameter BAUD_RATE         = 115200,
    parameter WINDOW_MS         = 400,
    parameter GRID_SIZE         = 16,
    parameter SENSOR_RES        = 320,
    parameter MIN_EVENT_THRESH  = 20,
    parameter MOTION_THRESH     = 8,
    parameter PERSISTENCE_COUNT = 1,
    parameter CYCLES_PER_BIN    = 600
)(
    input  logic        clk,
    // Pre-decoded, grid-mapped event stream from upstream MCU/deserializer.
    // x and y are already in 0–15 grid coordinates (4 bits each).
    input  logic        event_valid,
    input  logic [3:0]  event_x,
    input  logic [3:0]  event_y,
    input  logic        event_polarity,
    output logic        event_ready,
    // UART TX for gesture result output
    output logic        uart_tx,
    output logic        led_heartbeat,
    output logic        led_gesture_valid,
    output logic        led_activity,
    output logic        led_up,
    output logic        led_down,
    output logic        led_left,
    output logic        led_right
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // Power-on reset
    reg [4:0] por_cnt = 5'd0;
    wire rst_por = ~&por_cnt;
    wire rst = rst_por;

    always @(posedge clk)
        if (!(&por_cnt))
            por_cnt <= por_cnt + 1'b1;

    // Internal timestamp counter (replaces external event_ts port)
    logic [15:0] ts_counter;
    always @(posedge clk) begin
        if (rst)
            ts_counter <= '0;
        else
            ts_counter <= ts_counter + 1'b1;
    end

    logic [7:0] tx_data;
    logic tx_valid;
    logic tx_busy;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst(rst), .data(tx_data), .valid(tx_valid),
        .tx(uart_tx), .busy(tx_busy)
    );

    logic [1:0]  gesture;
    logic        gesture_valid;
    logic [3:0]  gesture_confidence;
    logic [7:0]  debug_event_count;
    logic [2:0]  debug_state;
    logic        debug_fifo_empty;
    logic        debug_fifo_full;
    logic        accel_temporal_phase;

    // Map 4-bit grid coordinates (0–15) to the centre of each 20-pixel sensor bin.
    // bin_centre = grid_coord * (SENSOR_RES/GRID_SIZE) + (SENSOR_RES/GRID_SIZE)/2
    //            = grid_coord * 20 + 10  (for SENSOR_RES=320, GRID_SIZE=16)
    // Max value: 15*20+10 = 310, fits in 9 bits.
    localparam integer BIN_SIZE     = SENSOR_RES / GRID_SIZE;   // 20
    localparam integer BIN_SIZE_HALF = BIN_SIZE / 2;             // 10
    wire [8:0] accel_event_x = 9'(event_x) * 9'(BIN_SIZE) + 9'(BIN_SIZE_HALF);
    wire [8:0] accel_event_y = 9'(event_y) * 9'(BIN_SIZE) + 9'(BIN_SIZE_HALF);

    dvs_gesture_accel #(
        .CLK_FREQ_HZ       (CLK_FREQ),
        .WINDOW_MS         (WINDOW_MS),
        .GRID_SIZE         (GRID_SIZE),
        .SENSOR_RES        (SENSOR_RES),
        .MIN_EVENT_THRESH  (MIN_EVENT_THRESH),
        .MOTION_THRESH     (MOTION_THRESH),
        .PERSISTENCE_COUNT (PERSISTENCE_COUNT),
        .CYCLES_PER_BIN    (CYCLES_PER_BIN)
    ) u_accel (
        .clk                 (clk),
        .rst                 (rst),
        .event_valid         (event_valid),
        .event_x             (accel_event_x),
        .event_y             (accel_event_y),
        .event_polarity      (event_polarity),
        .event_ts            (ts_counter),
        .event_ready         (event_ready),
        .gesture             (gesture),
        .gesture_valid       (gesture_valid),
        .gesture_confidence  (gesture_confidence),
        .debug_event_count   (debug_event_count),
        .debug_state         (debug_state),
        .debug_fifo_empty    (debug_fifo_empty),
        .debug_fifo_full     (debug_fifo_full),
        .debug_temporal_phase(accel_temporal_phase)
    );

    // Heartbeat ~3 Hz
    logic [23:0] heartbeat_cnt;
    always @(posedge clk)
        heartbeat_cnt <= rst ? 24'd0 : heartbeat_cnt + 1'b1;
    assign led_heartbeat = ~heartbeat_cnt[22];

    // Gesture valid LED (pulse stretch)
    logic [19:0] gesture_led_cnt;
    always @(posedge clk) begin
        if (rst)
            gesture_led_cnt <= '0;
        else if (gesture_valid)
            gesture_led_cnt <= {20{1'b1}};
        else if (gesture_led_cnt > 0)
            gesture_led_cnt <= gesture_led_cnt - 1'b1;
    end
    assign led_gesture_valid = ~(gesture_led_cnt > 0);

    // Direction LEDs (pulse stretch)
    logic [19:0] dir_led_cnt;
    logic [1:0]  last_gesture;
    logic        last_gesture_valid;

    always @(posedge clk) begin
        if (rst) begin
            dir_led_cnt        <= '0;
            last_gesture       <= 2'd0;
            last_gesture_valid <= 1'b0;
        end else if (gesture_valid) begin
            dir_led_cnt        <= {20{1'b1}};
            last_gesture       <= gesture;
            last_gesture_valid <= 1'b1;
        end else if (dir_led_cnt > 0) begin
            dir_led_cnt <= dir_led_cnt - 1'b1;
        end else begin
            last_gesture_valid <= 1'b0;
        end
    end

    assign led_up    = (dir_led_cnt > 0) && last_gesture_valid && (last_gesture == 2'b00);
    assign led_down  = (dir_led_cnt > 0) && last_gesture_valid && (last_gesture == 2'b01);
    assign led_left  = (dir_led_cnt > 0) && last_gesture_valid && (last_gesture == 2'b10);
    assign led_right = (dir_led_cnt > 0) && last_gesture_valid && (last_gesture == 2'b11);

    // Activity LED
    logic [17:0] activity_led_cnt;
    always @(posedge clk) begin
        if (rst)
            activity_led_cnt <= '0;
        else if (event_valid)
            activity_led_cnt <= {18{1'b1}};
        else if (activity_led_cnt > 0)
            activity_led_cnt <= activity_led_cnt - 1'b1;
    end
    assign led_activity = ~(activity_led_cnt > 0);

    // TX: send [0xA0|gesture, confidence] on detection
    typedef enum logic [2:0] {
        TX_IDLE,
        TX_GESTURE_CMD,
        TX_GESTURE_CMD_WAIT,
        TX_GESTURE_CONF
    } tx_state_t;

    tx_state_t  tx_state;
    logic [1:0] pending_gesture;
    logic [3:0] pending_confidence;

    always @(posedge clk) begin
        if (rst) begin
            tx_state           <= TX_IDLE;
            tx_data            <= 8'd0;
            tx_valid           <= 1'b0;
            pending_gesture    <= 2'd0;
            pending_confidence <= 4'd0;
        end else begin
            tx_valid <= 1'b0;

            if (gesture_valid && tx_state == TX_IDLE) begin
                pending_gesture    <= gesture;
                pending_confidence <= gesture_confidence;
                tx_state           <= TX_GESTURE_CMD;
            end

            case (tx_state)
                TX_IDLE: ;

                TX_GESTURE_CMD: begin
                    if (!tx_busy) begin
                        tx_data  <= {4'hA, 2'b00, pending_gesture};
                        tx_valid <= 1'b1;
                        tx_state <= TX_GESTURE_CMD_WAIT;
                    end
                end

                TX_GESTURE_CMD_WAIT: begin
                    if (tx_busy) tx_state <= TX_GESTURE_CONF;
                end

                TX_GESTURE_CONF: begin
                    if (!tx_busy) begin
                        tx_data  <= {pending_confidence, debug_event_count[7:4]};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
