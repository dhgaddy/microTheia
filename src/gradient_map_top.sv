`timescale 1ns/1ps

// Top-level for gradient-map gesture classifier - raw UART EVT2.0 input.
// Events arrive as a stream of 32-bit EVT2.0 words over UART
// (four bytes per word, MSB first). This top handles UART
// reception/word assembly and UART transmission of gesture results.
// All gesture processing is inside gradient_map_core, which owns:
//   input_fifo → evt2_decoder → gradient_mapping → systolic_array → gesture_classifier.
// Target: Lattice iCE40UP5K on iCEBreaker board.

module gradient_map_top #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter BAUD_RATE       = 115200,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000
)(
    input  logic clk,
    input  logic uart_rx,
    output logic uart_tx,
    output logic led_heartbeat,
    output logic led_activity,
    output logic led_up,
    output logic led_down,
    output logic led_left,
    output logic led_right
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
    // UART RX
    // -------------------------------------------------------------------------
    logic [7:0] uart_rx_data;
    logic       uart_rx_valid;

    uart_rx #(
        .CLKS_PER_BIT(CLK_FREQ_HZ / BAUD_RATE)
    ) u_uart_rx (
        .clk  (clk),
        .rst  (rst),
        .rx   (uart_rx),
        .data (uart_rx_data),
        .valid(uart_rx_valid)
    );

    // -------------------------------------------------------------------------
    // 4-byte assembler → raw EVT2 32-bit word (MSB-first)
    // -------------------------------------------------------------------------
    logic [1:0]  uart_byte_idx;
    logic [31:0] uart_shift;
    logic [31:0] uart_evt_word;
    logic        uart_evt_pending, core_evt_word_ready;

    always_ff @(posedge clk) begin
        if (rst) begin
            uart_byte_idx    <= 2'd0;
            uart_shift       <= 32'd0;
            uart_evt_word    <= 32'd0;
            uart_evt_pending <= 1'b0;
        end else begin
            if (uart_rx_valid) begin
                case (uart_byte_idx)
                    2'd0: uart_shift[31:24] <= uart_rx_data;
                    2'd1: uart_shift[23:16] <= uart_rx_data;
                    2'd2: uart_shift[15:8]  <= uart_rx_data;
                    2'd3: begin
                        if (!uart_evt_pending) begin
                            uart_shift[7:0]  <= uart_rx_data;
                            uart_evt_word    <= {uart_shift[31:8], uart_rx_data};
                            uart_evt_pending <= 1'b1;
                        end
                    end
                    default: ;
                endcase

                uart_byte_idx <= (uart_byte_idx == 2'd3) ? 2'd0 : uart_byte_idx + 1'b1;
            end

            if (uart_evt_pending && core_evt_word_ready)
                uart_evt_pending <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Core pipeline
    // -------------------------------------------------------------------------
    logic [1:0]  gesture_class;
    logic        gesture_valid;
    logic [7:0]  gesture_confidence;
    logic [23:0] debug_m00, debug_m10, debug_m01;
    logic [2:0]  debug_state;
    logic        decoded_valid;

    gradient_map_core #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .FRAME_PERIOD_MS(FRAME_PERIOD_MS),
        .DECAY_SHIFT    (DECAY_SHIFT),
        .MIN_MASS_THRESH(MIN_MASS_THRESH)
    ) u_core (
        .clk                (clk),
        .rst                (rst),
        .evt_word           (uart_evt_word),
        .evt_word_valid     (uart_evt_pending),
        .evt_word_ready     (core_evt_word_ready),
        .gesture_class      (gesture_class),
        .gesture_valid      (gesture_valid),
        .gesture_confidence (gesture_confidence),
        .decoded_event_valid(decoded_valid),
        .debug_m00          (debug_m00),
        .debug_m10          (debug_m10),
        .debug_m01          (debug_m01),
        .debug_state        (debug_state),
        .debug_fifo_empty   (),
        .debug_fifo_full    ()
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
    // Activity LED: pulses on decoded events
    // -------------------------------------------------------------------------
    logic [19:0] activity_cnt;

    always_ff @(posedge clk) begin
        if (rst)
            activity_cnt <= '0;
        else if (decoded_valid)
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

