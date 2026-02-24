// Top-level for voxel-bin DVS gesture accelerator — raw UART sensor input.
// Events arrive as 5-byte UART packets [X_HI, X_LO, Y_HI, Y_LO, POL];
// control bytes 0xFF/FE/FD/FC provide echo/status/config/reset.
// TX: [0xA0|gesture, confidence] on detection.
// No external reset pin; power-on reset and soft-reset are generated internally.
// Target: Lattice iCE40UP5K on iCEBreaker board.

`timescale 1ns/1ps

module voxel_bin_raw_top #(
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
    input  logic clk,
    input  logic uart_rx,
    output logic uart_tx,
    output logic led_heartbeat,
    output logic led_gesture_valid,
    output logic led_activity,
    output logic led_up,
    output logic led_down,
    output logic led_left,
    output logic led_right
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    reg [4:0] por_cnt = 5'd0;
    wire rst_por = ~&por_cnt;
    logic soft_rst;
    wire rst = rst_por | soft_rst;

    always @(posedge clk) begin
        if (!(&por_cnt))
            por_cnt <= por_cnt + 1'b1;
    end

    logic [7:0] rx_data;
    logic rx_valid;
    logic [7:0] tx_data;
    logic tx_valid;
    logic tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rx(uart_rx),
        .data(rx_data), .valid(rx_valid)
    );

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
    logic        event_ready;

    logic        event_valid;
    logic [8:0]  event_x, event_y;
    logic        event_polarity;
    logic [15:0] event_ts;

    logic        accel_temporal_phase;

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
        .event_x             (event_x),
        .event_y             (event_y),
        .event_polarity      (event_polarity),
        .event_ts            (event_ts),
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

    typedef enum logic [2:0] {
        PKT_X_HI, PKT_X_LO, PKT_Y_HI, PKT_Y_LO, PKT_POL
    } pkt_state_t;

    pkt_state_t pkt_state;
    logic [8:0]  pkt_x, pkt_y;
    logic [15:0] pkt_ts;

    typedef enum logic [3:0] {
        TX_IDLE,
        TX_ECHO,
        TX_STATUS,
        TX_GESTURE_CMD,
        TX_GESTURE_CMD_WAIT,
        TX_GESTURE_CONF,
        TX_CONFIG_1,
        TX_CONFIG_1_WAIT,
        TX_CONFIG_2
    } tx_state_t;

    tx_state_t tx_state;
    logic [1:0] pending_gesture;
    logic [3:0] pending_confidence;
    logic [2:0] pending_state;

    logic [15:0] ts_counter;
    always @(posedge clk) begin
        if (rst)
            ts_counter <= '0;
        else
            ts_counter <= ts_counter + 1'b1;
    end

    always @(posedge clk) begin
        if (rst) begin
            pkt_state          <= PKT_X_HI;
            pkt_x              <= 9'd0;
            pkt_y              <= 9'd0;
            pkt_ts             <= 16'd0;
            event_valid        <= 1'b0;
            event_x            <= 9'd0;
            event_y            <= 9'd0;
            event_polarity     <= 1'b0;
            event_ts           <= 16'd0;
            tx_state           <= TX_IDLE;
            tx_data            <= 8'd0;
            tx_valid           <= 1'b0;
            pending_gesture    <= 2'd0;
            pending_confidence <= 4'd0;
            pending_state      <= 3'd0;
            soft_rst           <= 1'b0;
        end else begin
            event_valid <= 1'b0;
            tx_valid    <= 1'b0;
            soft_rst    <= 1'b0;

            if (rx_valid) begin
                case (pkt_state)
                    PKT_X_HI: begin
                        case (rx_data)
                            8'hFF: if (tx_state == TX_IDLE) tx_state <= TX_ECHO;
                            8'hFE: if (tx_state == TX_IDLE) begin
                                pending_state <= debug_state;
                                tx_state <= TX_STATUS;
                            end
                            8'hFD: if (tx_state == TX_IDLE) tx_state <= TX_CONFIG_1;
                            8'hFC: soft_rst <= 1'b1;
                            default: begin
                                pkt_x[8]  <= rx_data[0];
                                pkt_state <= PKT_X_LO;
                            end
                        endcase
                    end
                    PKT_X_LO: begin pkt_x[7:0] <= rx_data; pkt_state <= PKT_Y_HI; end
                    PKT_Y_HI: begin pkt_y[8]   <= rx_data[0]; pkt_state <= PKT_Y_LO; end
                    PKT_Y_LO: begin pkt_y[7:0] <= rx_data; pkt_state <= PKT_POL; end
                    PKT_POL: begin
                        if (event_ready) begin
                            event_x        <= pkt_x;
                            event_y        <= pkt_y;
                            event_polarity <= rx_data[0];
                            event_ts       <= ts_counter;
                            event_valid    <= 1'b1;
                        end
                        pkt_state <= PKT_X_HI;
                    end
                    default: pkt_state <= PKT_X_HI;
                endcase
            end

            if (gesture_valid && tx_state == TX_IDLE) begin
                pending_gesture    <= gesture;
                pending_confidence <= gesture_confidence;
                tx_state           <= TX_GESTURE_CMD;
            end

            case (tx_state)
                TX_IDLE: ;

                TX_ECHO: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h55;
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end

                TX_STATUS: begin
                    if (!tx_busy) begin
                        tx_data  <= {4'b1011, accel_temporal_phase, debug_fifo_full, debug_fifo_empty, 1'b0};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end

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

                TX_CONFIG_1: begin
                    if (!tx_busy) begin
                        tx_data  <= MIN_EVENT_THRESH[7:0];
                        tx_valid <= 1'b1;
                        tx_state <= TX_CONFIG_1_WAIT;
                    end
                end

                TX_CONFIG_1_WAIT: begin
                    if (tx_busy) tx_state <= TX_CONFIG_2;
                end

                TX_CONFIG_2: begin
                    if (!tx_busy) begin
                        tx_data  <= MOTION_THRESH[7:0];
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
