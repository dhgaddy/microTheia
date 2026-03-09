`timescale 1ns/1ps

// UART top-level wrapper for voxel_bin_core.
// - Ingests raw EVT2.0 words over UART (4 bytes/word, MSB first)
// - Supports control bytes at packet-boundary:
//     0xFF: echo -> 0x55
//     0xFE: status -> 0xBx (bit2=fifo_full, bit1=fifo_empty, bit3=temporal_phase)
//     0xFD: config -> 2 bytes (CONFIG_BYTE0, CONFIG_BYTE1)
//     0xFB: diag -> 2 bytes ({debug_event_count}, {debug bits})
//     0xFC: soft reset
// - Sends gesture packets on valid gesture detection:
//     byte0: 0xA0 | gesture[1:0]
//     byte1: {confidence[CONF_BITS-1:0], event_count[7:4]}
//
// Gesture encoding (matches weight file and core class order):
//   0=Down, 1=Left, 2=Right, 3=Up

module voxel_bin_top #(
    parameter int CLK_FREQ_HZ          = 12_000_000,
    parameter int BAUD_RATE            = 1_000_000,
    parameter int WINDOW_MS            = 1000,
    parameter int CYCLES_PER_BIN       = 0,
    parameter int GRID_SIZE            = 16,
    parameter int NUM_BINS             = 8,
    parameter int READOUT_BINS         = 8,
    parameter int COUNTER_BITS         = 16,  // Counter width only affects feature/bin storage; SA still uses WEIGHT_BITS path for weights
    parameter int FIFO_DEPTH           = 256,
    parameter int DATA_WIDTH           = 32,
    parameter int REQUIRE_TIME_HIGH    = 1,
    parameter int SWAP_INPUT_BYTES     = 0,
    parameter int SENSOR_WIDTH         = 320,
    parameter int SENSOR_HEIGHT        = 320,
    parameter int WEIGHT_BITS          = 8,
    parameter int WEIGHT_SCALE         = 1024,
    parameter int N                    = 16,  // 16 → 16x16 systolic array
    parameter int PASS_MARGIN          = 0, // parameter int PASS_MARGIN       = 64,
    parameter int PERSISTENCE_COUNT    = 2,
    parameter int CONF_BITS            = 4,
    parameter int CONF_SHIFT           = 4,
    parameter int UART_WORD_FIFO_DEPTH = 16,
    parameter int TX_FIFO_DEPTH        = 32,
    parameter int POR_CYCLES           = 1024,
    parameter int SOFT_RESET_CYCLES    = 64
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

    localparam logic [7:0] CONFIG_BYTE0       = 8'h08; // NUM_BINS default
    localparam logic [7:0] CONFIG_BYTE1       = 8'h08; // READOUT_BINS default

    localparam int CLK_FREQ               = CLK_FREQ_HZ; // compatibility alias

    localparam int ACTIVE_CLK_FREQ        = (CLK_FREQ != 0) ? CLK_FREQ : CLK_FREQ_HZ;
    localparam int CLKS_PER_BIT           = (BAUD_RATE > 0) ? (CLK_FREQ_HZ / BAUD_RATE) : 1;
    localparam int POR_BITS               = (POR_CYCLES > 1) ? $clog2(POR_CYCLES) : 1;
    localparam int SOFT_RST_BITS          = (SOFT_RESET_CYCLES > 1) ? $clog2(SOFT_RESET_CYCLES + 1) : 1;
    localparam int HEARTBEAT_HALF_PERIOD  = (CLK_FREQ_HZ / 3);   // ~1.5 Hz blink
    localparam int HEARTBEAT_BITS         = (HEARTBEAT_HALF_PERIOD > 1) ? $clog2(HEARTBEAT_HALF_PERIOD) : 1;
    localparam int LED_HOLD_CYCLES        = (CLK_FREQ_HZ / 20);  // 50 ms pulse
    localparam int LED_HOLD_BITS          = (LED_HOLD_CYCLES > 1) ? $clog2(LED_HOLD_CYCLES + 1) : 1;

    localparam logic [7:0] CMD_ECHO       = 8'hFF;
    localparam logic [7:0] CMD_STATUS     = 8'hFE;
    localparam logic [7:0] CMD_CONFIG     = 8'hFD;
    localparam logic [7:0] CMD_DIAG       = 8'hFB;
    localparam logic [7:0] CMD_SOFT_RESET = 8'hFC;

    typedef enum logic [1:0] {
        PKT_IDLE = 2'd0,
        PKT_B1   = 2'd1,
        PKT_B2   = 2'd2,
        PKT_B3   = 2'd3
    } pkt_state_t;

    // Internal reset (visible as dut.rst in simulation).
    logic rst;
    logic rst_por;
    logic [POR_BITS-1:0] por_ctr;
    logic [SOFT_RST_BITS-1:0] soft_rst_ctr;
    logic soft_rst_cmd_pulse;

    // UART RX/TX byte interfaces.
    logic [7:0] rx_byte;
    logic       rx_byte_valid;
    logic [7:0] tx_byte;
    logic       tx_byte_valid;
    logic       tx_busy;

    // Assembler -> word FIFO -> core input.
    pkt_state_t pkt_state;
    logic [31:0] asm_word;
    logic [31:0] word_fifo_in_data;
    logic        word_fifo_in_valid;
    logic        word_fifo_in_ready;
    logic [31:0] core_evt_word;
    logic        core_evt_valid;
    logic        core_evt_ready;

    // TX response FIFO.
    logic [7:0] tx_fifo_in_data;
    logic       tx_fifo_in_valid;
    logic       tx_fifo_in_ready;
    logic [7:0] tx_fifo_out_data;
    logic       tx_fifo_out_valid;
    logic       tx_fifo_out_ready;

    // Pending response flags.
    logic cmd_echo_pending;
    logic cmd_status_pending;
    logic cmd_config_pending;
    logic cmd_diag_pending;
    logic gesture_pkt_pending;
    logic [1:0] gesture_pkt_code;
    logic [3:0] gesture_pkt_conf;
    logic [3:0] gesture_pkt_evthi;
    logic second_byte_pending;
    logic [7:0] second_byte_data;
    logic [7:0] status_byte;
    logic [7:0] diag_byte1;

    // Core outputs.
    logic [1:0]          core_gesture;
    logic                core_gesture_valid;
    logic [CONF_BITS-1:0] core_gesture_confidence;
    logic [7:0] core_debug_event_count;
    logic [2:0] core_debug_state;
    logic       core_debug_fifo_empty;
    logic       core_debug_fifo_full;
    logic       core_debug_temporal_phase;
    logic       core_debug_class_valid;
    logic       core_debug_class_pass;
    logic       core_debug_feature_window_ready;
    logic       core_debug_capture_active;
    logic       core_debug_score_busy;

    // Sticky diagnostic bits (clear on reset).
    logic diag_seen_capture;
    logic diag_seen_feature_window;
    logic diag_seen_score_busy;
    logic diag_seen_class_valid;
    logic diag_seen_class_pass;
    logic diag_seen_gesture_valid;

    // Gesture LED/output mapping.
    // Gesture class directly matches weight file order: 0=Down,1=Left,2=Right,3=Up
    logic [1:0] last_gesture;
    logic [LED_HOLD_BITS-1:0] gesture_led_ctr;
    logic [LED_HOLD_BITS-1:0] activity_led_ctr;
    logic [HEARTBEAT_BITS-1:0] heartbeat_ctr;

    always_comb begin
        status_byte      = 8'hB0;
        status_byte[3]   = core_debug_temporal_phase;
        status_byte[2]   = core_debug_fifo_full;
        status_byte[1]   = core_debug_fifo_empty;
        status_byte[0]   = 1'b0;

        // Sticky stage visibility:
        // [7]=capture_active seen
        // [6]=feature_window_ready seen
        // [5]=score_busy seen
        // [4]=class_valid seen
        // [3]=class_pass seen
        // [2]=gesture_valid seen
        // [1]=live gesture pulse
        // [0]=temporal phase (live)
        diag_byte1       = 8'h00;
        diag_byte1[7]    = diag_seen_capture;
        diag_byte1[6]    = diag_seen_feature_window;
        diag_byte1[5]    = diag_seen_score_busy;
        diag_byte1[4]    = diag_seen_class_valid;
        diag_byte1[3]    = diag_seen_class_pass;
        diag_byte1[2]    = diag_seen_gesture_valid;
        diag_byte1[1]    = core_gesture_valid;
        diag_byte1[0]    = core_debug_temporal_phase;
    end

    // Power-on reset generator.
    initial begin
        rst_por = 1'b1;
        por_ctr = '0;
        soft_rst_ctr = '0;
    end

    always_ff @(posedge clk) begin
        if (rst_por) begin
            if (por_ctr == POR_CYCLES - 1)
                rst_por <= 1'b0;
            else
                por_ctr <= por_ctr + 1'b1;
        end
    end

    // Soft reset pulse stretcher.
    always_ff @(posedge clk) begin
        if (soft_rst_cmd_pulse)
            soft_rst_ctr <= SOFT_RESET_CYCLES[SOFT_RST_BITS-1:0];
        else if (soft_rst_ctr != 0)
            soft_rst_ctr <= soft_rst_ctr - 1'b1;
    end

    assign rst = rst_por | (soft_rst_ctr != 0);

    uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE  (BAUD_RATE)
    ) u_uart_rx (
        .clk  (clk),
        .rst  (rst),
        .rx   (uart_rx),
        .data (rx_byte),
        .valid(rx_byte_valid)
    );

    uart_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE  (BAUD_RATE)
    ) u_uart_tx (
        .clk  (clk),
        .rst  (rst),
        .data (tx_byte),
        .valid(tx_byte_valid),
        .tx   (uart_tx),
        .busy (tx_busy)
    );

    // Word FIFO decouples UART receive from core evt_word ready.
    input_fifo #(
        .FIFO_DEPTH(UART_WORD_FIFO_DEPTH),
        .DATA_WIDTH(32)
    ) u_word_fifo (
        .clk_i   (clk),
        .reset_i (rst),
        .data_i  (word_fifo_in_data),
        .ready_i (core_evt_ready),
        .valid_i (word_fifo_in_valid),
        .ready_o (word_fifo_in_ready),
        .valid_o (core_evt_valid),
        .data_o  (core_evt_word)
    );

    // TX byte FIFO decouples response generation from serial TX bandwidth.
    input_fifo #(
        .FIFO_DEPTH(TX_FIFO_DEPTH),
        .DATA_WIDTH(8)
    ) u_tx_fifo (
        .clk_i   (clk),
        .reset_i (rst),
        .data_i  (tx_fifo_in_data),
        .ready_i (tx_fifo_out_ready),
        .valid_i (tx_fifo_in_valid),
        .ready_o (tx_fifo_in_ready),
        .valid_o (tx_fifo_out_valid),
        .data_o  (tx_fifo_out_data)
    );

    voxel_bin_core #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .WINDOW_MS        (WINDOW_MS),
        .GRID_SIZE        (GRID_SIZE),
        .NUM_BINS         (NUM_BINS),
        .READOUT_BINS     (READOUT_BINS),
        .COUNTER_BITS     (COUNTER_BITS),
        .FIFO_DEPTH       (FIFO_DEPTH),
        .DATA_WIDTH       (DATA_WIDTH),
        .REQUIRE_TIME_HIGH(REQUIRE_TIME_HIGH),
        .SWAP_INPUT_BYTES (SWAP_INPUT_BYTES),
        .SENSOR_WIDTH     (SENSOR_WIDTH),
        .SENSOR_HEIGHT    (SENSOR_HEIGHT),
        .WEIGHT_BITS      (WEIGHT_BITS),
        .WEIGHT_SCALE     (WEIGHT_SCALE),
        .N                (N),
        .PASS_MARGIN      (PASS_MARGIN),
        .PERSISTENCE_COUNT(PERSISTENCE_COUNT),
        .CONF_BITS        (CONF_BITS),
        .CONF_SHIFT       (CONF_SHIFT),
        .CYCLES_PER_BIN   (CYCLES_PER_BIN),
        .NUM_CLASSES      (NUM_CLASSES)
    ) u_core (
        .clk                (clk),
        .rst                (rst),
        .evt_word           (core_evt_word),
        .evt_word_valid     (core_evt_valid),
        .evt_word_ready     (core_evt_ready),
        .gesture            (core_gesture),
        .gesture_valid      (core_gesture_valid),
        .gesture_confidence (core_gesture_confidence),
        .debug_event_count  (core_debug_event_count),
        .debug_state        (core_debug_state),
        .debug_fifo_empty   (core_debug_fifo_empty),
        .debug_fifo_full    (core_debug_fifo_full),
        .debug_temporal_phase(core_debug_temporal_phase),
        .debug_class_valid  (core_debug_class_valid),
        .debug_class_pass   (core_debug_class_pass),
        .debug_feature_window_ready(core_debug_feature_window_ready),
        .debug_capture_active(core_debug_capture_active),
        .debug_score_busy   (core_debug_score_busy)
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            pkt_state           <= PKT_IDLE;
            asm_word            <= '0;
            word_fifo_in_valid  <= 1'b0;
            word_fifo_in_data   <= '0;
            tx_fifo_in_valid    <= 1'b0;
            tx_fifo_in_data     <= '0;
            tx_byte_valid       <= 1'b0;
            tx_byte             <= '0;
            tx_fifo_out_ready   <= 1'b0;
            soft_rst_cmd_pulse  <= 1'b0;
            cmd_echo_pending    <= 1'b0;
            cmd_status_pending  <= 1'b0;
            cmd_config_pending  <= 1'b0;
            cmd_diag_pending    <= 1'b0;
            gesture_pkt_pending <= 1'b0;
            gesture_pkt_code    <= '0;
            gesture_pkt_conf    <= '0;
            gesture_pkt_evthi   <= '0;
            second_byte_pending <= 1'b0;
            second_byte_data    <= '0;
            heartbeat_ctr       <= '0;
            led_heartbeat       <= 1'b0;
            gesture_led_ctr     <= '0;
            activity_led_ctr    <= '0;
            last_gesture        <= 2'd0;
            diag_seen_capture         <= 1'b0;
            diag_seen_feature_window  <= 1'b0;
            diag_seen_score_busy      <= 1'b0;
            diag_seen_class_valid     <= 1'b0;
            diag_seen_class_pass      <= 1'b0;
            diag_seen_gesture_valid   <= 1'b0;
        end else begin
            word_fifo_in_valid <= 1'b0;
            tx_fifo_in_valid   <= 1'b0;
            tx_byte_valid      <= 1'b0;
            tx_fifo_out_ready  <= 1'b0;
            soft_rst_cmd_pulse <= 1'b0;

            // Sticky stage diagnostics.
            if (core_debug_capture_active)
                diag_seen_capture <= 1'b1;
            if (core_debug_feature_window_ready)
                diag_seen_feature_window <= 1'b1;
            if (core_debug_score_busy)
                diag_seen_score_busy <= 1'b1;
            if (core_debug_class_valid)
                diag_seen_class_valid <= 1'b1;
            if (core_debug_class_pass)
                diag_seen_class_pass <= 1'b1;
            if (core_gesture_valid)
                diag_seen_gesture_valid <= 1'b1;

            // UART RX packet/command parsing.
            if (rx_byte_valid) begin
                activity_led_ctr <= LED_HOLD_CYCLES[LED_HOLD_BITS-1:0];

                case (pkt_state)
                    PKT_IDLE: begin
                        if (rx_byte == CMD_ECHO) begin
                            cmd_echo_pending <= 1'b1;
                        end else if (rx_byte == CMD_STATUS) begin
                            cmd_status_pending <= 1'b1;
                        end else if (rx_byte == CMD_CONFIG) begin
                            cmd_config_pending <= 1'b1;
                        end else if (rx_byte == CMD_DIAG) begin
                            cmd_diag_pending <= 1'b1;
                        end else if (rx_byte == CMD_SOFT_RESET) begin
                            soft_rst_cmd_pulse <= 1'b1;
                        end else begin
                            asm_word[31:24] <= rx_byte;
                            pkt_state       <= PKT_B1;
                        end
                    end

                    PKT_B1: begin
                        asm_word[23:16] <= rx_byte;
                        pkt_state       <= PKT_B2;
                    end

                    PKT_B2: begin
                        asm_word[15:8] <= rx_byte;
                        pkt_state      <= PKT_B3;
                    end

                    PKT_B3: begin
                        if (word_fifo_in_ready) begin
                            word_fifo_in_valid <= 1'b1;
                            word_fifo_in_data  <= {asm_word[31:8], rx_byte};
                        end
                        pkt_state <= PKT_IDLE;
                    end

                    default: pkt_state <= PKT_IDLE;
                endcase
            end

            // Queue gesture response packet.
            if (core_gesture_valid) begin
                gesture_pkt_pending <= 1'b1;
                gesture_pkt_code    <= core_gesture;
                gesture_pkt_conf    <= core_gesture_confidence[3:0];
                gesture_pkt_evthi   <= core_debug_event_count[7:4];
                last_gesture        <= core_gesture;
                gesture_led_ctr     <= LED_HOLD_CYCLES[LED_HOLD_BITS-1:0];
            end else if (gesture_led_ctr != 0) begin
                gesture_led_ctr <= gesture_led_ctr - 1'b1;
            end

            if (activity_led_ctr != 0)
                activity_led_ctr <= activity_led_ctr - 1'b1;

            // Response producer (one byte/cycle into TX FIFO).
            if (tx_fifo_in_ready) begin
                if (second_byte_pending) begin
                    tx_fifo_in_valid   <= 1'b1;
                    tx_fifo_in_data    <= second_byte_data;
                    second_byte_pending <= 1'b0;
                end else if (cmd_echo_pending) begin
                    tx_fifo_in_valid  <= 1'b1;
                    tx_fifo_in_data   <= 8'h55;
                    cmd_echo_pending  <= 1'b0;
                end else if (cmd_status_pending) begin
                    tx_fifo_in_valid   <= 1'b1;
                    tx_fifo_in_data    <= status_byte;
                    cmd_status_pending <= 1'b0;
                end else if (cmd_config_pending) begin
                    tx_fifo_in_valid    <= 1'b1;
                    tx_fifo_in_data     <= CONFIG_BYTE0;
                    second_byte_pending <= 1'b1;
                    second_byte_data    <= CONFIG_BYTE1;
                    cmd_config_pending  <= 1'b0;
                end else if (cmd_diag_pending) begin
                    tx_fifo_in_valid    <= 1'b1;
                    tx_fifo_in_data     <= core_debug_event_count;
                    second_byte_pending <= 1'b1;
                    second_byte_data    <= diag_byte1;
                    cmd_diag_pending    <= 1'b0;
                end else if (gesture_pkt_pending) begin
                    tx_fifo_in_valid    <= 1'b1;
                    tx_fifo_in_data     <= 8'hA0 | gesture_pkt_code;
                    second_byte_pending <= 1'b1;
                    second_byte_data    <= {gesture_pkt_conf, gesture_pkt_evthi};
                    gesture_pkt_pending <= 1'b0;
                end
            end

            // UART TX consume from TX FIFO.
            if (!tx_busy && tx_fifo_out_valid) begin
                tx_byte           <= tx_fifo_out_data;
                tx_byte_valid     <= 1'b1;
                tx_fifo_out_ready <= 1'b1;
            end

            // Heartbeat LED.
            if (heartbeat_ctr == HEARTBEAT_HALF_PERIOD - 1) begin
                heartbeat_ctr <= '0;
                led_heartbeat <= ~led_heartbeat;
            end else begin
                heartbeat_ctr <= heartbeat_ctr + 1'b1;
            end
        end
    end

    assign led_activity      = (activity_led_ctr != 0);
    assign led_gesture_valid = (gesture_led_ctr != 0);
    assign led_down          = (gesture_led_ctr != 0) && (last_gesture == 2'd0);
    assign led_left          = (gesture_led_ctr != 0) && (last_gesture == 2'd1);
    assign led_right         = (gesture_led_ctr != 0) && (last_gesture == 2'd2);
    assign led_up            = (gesture_led_ctr != 0) && (last_gesture == 2'd3);

endmodule
