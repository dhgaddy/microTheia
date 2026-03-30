// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 Group G Contributors
`timescale 1ns/1ps

module voxel_bin_top #(
    parameter int CLK_FREQ_HZ          = 12_000_000,
    parameter int BAUD_RATE            = 1_000_000,
    parameter int WINDOW_MS            = 1000,
    parameter int CYCLES_PER_BIN       = 0,
    parameter int GRID_SIZE            = 16,
    parameter int NUM_BINS             = 8,
    parameter int READOUT_BINS         = 8,
    parameter int COUNTER_BITS         = 16,
    parameter int FIFO_DEPTH           = 256,
    parameter int DATA_WIDTH           = 32,
    parameter int REQUIRE_TIME_HIGH    = 1,
    parameter int SWAP_INPUT_BYTES     = 0,
    parameter int SENSOR_WIDTH         = 320,
    parameter int SENSOR_HEIGHT        = 320,
    parameter int WEIGHT_BITS          = 8,
    parameter int UART_WORD_FIFO_DEPTH = 16,
    parameter int TX_FIFO_DEPTH        = 32,
    parameter int POR_CYCLES           = 1024,
    parameter int NUM_CLASSES          = 4,
    parameter int SOFT_RESET_CYCLES    = 64,
    // SCORE_BITS must match voxel_bin_core's formula: COUNTER_BITS+WEIGHT_BITS+clog2(FC)+1
    parameter int SCORE_BITS           = COUNTER_BITS + WEIGHT_BITS +
                                         $clog2(READOUT_BINS * GRID_SIZE * GRID_SIZE) + 1
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

    localparam logic [7:0] CONFIG_BYTE0       = 8'(NUM_BINS);
    localparam logic [7:0] CONFIG_BYTE1       = 8'(READOUT_BINS);

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
    localparam logic [7:0] CMD_LOAD_WEIGHT = 8'hFA;
    localparam logic [7:0] CMD_LOAD_THRESH = 8'hF9;

    typedef enum logic [3:0] {
        PKT_IDLE = 4'd0,
        PKT_B1   = 4'd1,
        PKT_B2   = 4'd2,
        PKT_B3   = 4'd3,
        WLD_B1   = 4'd4,
        WLD_B2   = 4'd5,
        WLD_B3   = 4'd6,
        TLD_B1   = 4'd7,
        TLD_B2   = 4'd8,
        TLD_B3   = 4'd9,
        TLD_B4   = 4'd10,
        TLD_B5   = 4'd11,
        TLD_B6   = 4'd12
    } pkt_state_t;

    // Internal reset (visible as dut.rst in simulation).
    logic rst;
    logic rst_por;
    logic [POR_BITS-1:0] por_ctr;
    logic [SOFT_RST_BITS-1:0] soft_rst_ctr;
    logic soft_rst_cmd_pulse;

    logic [7:0] rx_byte;
    logic       rx_byte_valid;
    logic [7:0] tx_byte;
    logic       tx_byte_valid;
    logic       tx_busy;

    pkt_state_t pkt_state;
    logic [31:0] asm_word;
    logic [31:0] word_fifo_in_data;
    logic        word_fifo_in_valid;
    logic        word_fifo_in_ready;
    logic [31:0] core_evt_word;
    logic        core_evt_valid;
    logic        core_evt_ready;

    logic [7:0] tx_fifo_in_data;
    logic       tx_fifo_in_valid;
    logic       tx_fifo_in_ready;
    logic [7:0] tx_fifo_out_data;
    logic       tx_fifo_out_valid;
    logic       tx_fifo_out_ready;

    logic       cmd_echo_pending;
    logic       cmd_status_pending;
    logic       cmd_config_pending;
    logic       cmd_diag_pending;
    logic       gesture_pkt_pending;
    logic [1:0] gesture_pkt_code;
    logic       gesture_pkt_conf;
    logic [3:0] gesture_pkt_evthi;
    logic       second_byte_pending;
    logic [7:0] second_byte_data;
    logic [7:0] status_byte;
    logic [7:0] diag_byte1;

    logic [1:0] core_gesture;
    logic       core_gesture_valid;
    logic       core_gesture_confidence;
    logic [7:0] core_debug_event_count;
    logic       core_debug_fifo_empty;
    logic       core_debug_fifo_full;
    logic       core_debug_temporal_phase;
    logic       core_debug_class_valid;
    logic       core_debug_class_pass;
    logic       core_debug_feature_window_ready;
    logic       core_debug_capture_active;
    logic       core_debug_score_busy;

    // Weight/threshold write interface (driven by UART load commands)
    localparam int FEATURE_COUNT     = READOUT_BINS * GRID_SIZE * GRID_SIZE;
    localparam int WEIGHT_ADDR_BITS  = $clog2(FEATURE_COUNT);
    logic                        weight_wr_valid;
    logic [1:0]                  weight_wr_class;
    logic [WEIGHT_ADDR_BITS-1:0] weight_wr_addr;
    logic [WEIGHT_BITS-1:0]      weight_wr_data;
    logic                        thresh_wr_valid;
    logic [2:0]                  thresh_wr_addr;
    logic [SCORE_BITS-1:0]       thresh_wr_data;

    // Staging registers for multi-byte UART load commands
    logic [13:0]                 wld_addr_staging;
    logic [31:0]                 tld_data_staging;

    // Pending word register — holds an assembled EVT2 word when the word FIFO
    // was full at the time the 4th UART byte arrived.  Retried every cycle.
    logic                        word_pending;
    logic [31:0]                 word_pending_data;

    logic diag_seen_capture;
    logic diag_seen_feature_window;
    logic diag_seen_score_busy;
    logic diag_seen_class_valid;
    logic diag_seen_class_pass;
    logic diag_seen_gesture_valid;

    // Gesture class order: 0=Down, 1=Left, 2=Right, 3=Up
    logic [1:0]                last_gesture;
    logic [LED_HOLD_BITS-1:0]  gesture_led_ctr;
    logic [LED_HOLD_BITS-1:0]  activity_led_ctr;
    logic [HEARTBEAT_BITS-1:0] heartbeat_ctr;

    always_comb begin
        status_byte      = 8'hB0;
        status_byte[3]   = core_debug_temporal_phase;
        status_byte[2]   = core_debug_fifo_full;
        status_byte[1]   = core_debug_fifo_empty;
        status_byte[0]   = 1'b0;

        // diag_byte1 bit layout:
        // [7]=capture_active seen, [6]=feature_window_ready seen, [5]=score_busy seen
        // [4]=class_valid seen, [3]=class_pass seen, [2]=gesture_valid seen
        // [1]=live gesture pulse, [0]=temporal phase (live)
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
        .NUM_CLASSES      (NUM_CLASSES),
        .CYCLES_PER_BIN   (CYCLES_PER_BIN),
        .SCORE_BITS       (SCORE_BITS)
    ) u_core (
        .clk                        (clk),
        .rst                        (rst),
        .evt_word                   (core_evt_word),
        .evt_word_valid             (core_evt_valid),
        .evt_word_ready             (core_evt_ready),
        .gesture                    (core_gesture),
        .gesture_valid              (core_gesture_valid),
        .gesture_confidence         (core_gesture_confidence),
        .weight_wr_valid_i          (weight_wr_valid),
        .weight_wr_class_i          (weight_wr_class),
        .weight_wr_addr_i           (weight_wr_addr),
        .weight_wr_data_i           (weight_wr_data),
        .thresh_wr_valid_i          (thresh_wr_valid),
        .thresh_wr_addr_i           (thresh_wr_addr),
        .thresh_wr_data_i           (thresh_wr_data),
        .debug_event_count          (core_debug_event_count),
        .debug_fifo_empty           (core_debug_fifo_empty),
        .debug_fifo_full            (core_debug_fifo_full),
        .debug_temporal_phase       (core_debug_temporal_phase),
        .debug_class_valid          (core_debug_class_valid),
        .debug_class_pass           (core_debug_class_pass),
        .debug_feature_window_ready (core_debug_feature_window_ready),
        .debug_capture_active       (core_debug_capture_active),
        .debug_score_busy           (core_debug_score_busy)
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
            gesture_pkt_conf    <= 1'b0;
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
            weight_wr_valid     <= 1'b0;
            weight_wr_class     <= '0;
            weight_wr_addr      <= '0;
            weight_wr_data      <= '0;
            thresh_wr_valid     <= 1'b0;
            thresh_wr_addr      <= '0;
            thresh_wr_data      <= '0;
            wld_addr_staging    <= '0;
            tld_data_staging    <= '0;
            word_pending        <= 1'b0;
            word_pending_data   <= '0;
        end else begin
            word_fifo_in_valid <= 1'b0;
            tx_fifo_in_valid   <= 1'b0;
            tx_byte_valid      <= 1'b0;
            tx_fifo_out_ready  <= 1'b0;
            soft_rst_cmd_pulse <= 1'b0;
            weight_wr_valid    <= 1'b0;
            thresh_wr_valid    <= 1'b0;

            // Retry pending word push when FIFO has space (runs every cycle).
            if (word_pending && word_fifo_in_ready) begin
                word_fifo_in_valid <= 1'b1;
                word_fifo_in_data  <= word_pending_data;
                word_pending       <= 1'b0;
            end

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
                        end else if (rx_byte == CMD_LOAD_WEIGHT) begin
                            pkt_state <= WLD_B1;
                        end else if (rx_byte == CMD_LOAD_THRESH) begin
                            pkt_state <= TLD_B1;
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
                        if (word_fifo_in_ready && !word_pending) begin
                            word_fifo_in_valid <= 1'b1;
                            word_fifo_in_data  <= {asm_word[31:8], rx_byte};
                        end else if (!word_pending) begin
                            word_pending      <= 1'b1;
                            word_pending_data <= {asm_word[31:8], rx_byte};
                        end
                        // If word_pending is already set (prior word still
                        // waiting), this word is dropped — unavoidable without
                        // UART-level flow control.
                        pkt_state <= PKT_IDLE;
                    end

                    // ----- Weight load: 3 payload bytes after CMD_LOAD_WEIGHT -----
                    WLD_B1: begin
                        weight_wr_class        <= rx_byte[7:6];
                        wld_addr_staging[13:8] <= rx_byte[5:0];
                        pkt_state              <= WLD_B2;
                    end

                    WLD_B2: begin
                        wld_addr_staging[7:0] <= rx_byte;
                        pkt_state             <= WLD_B3;
                    end

                    WLD_B3: begin
                        if (!core_debug_score_busy) begin
                            weight_wr_valid <= 1'b1;
                            weight_wr_addr  <= wld_addr_staging[WEIGHT_ADDR_BITS-1:0];
                            weight_wr_data  <= rx_byte;
                        end
                        pkt_state <= PKT_IDLE;
                    end

                    // ----- Threshold load: 6 payload bytes after CMD_LOAD_THRESH -----
                    TLD_B1: begin
                        thresh_wr_addr <= rx_byte[2:0];
                        pkt_state      <= TLD_B2;
                    end

                    TLD_B2: begin
                        tld_data_staging[7:0] <= rx_byte;
                        pkt_state             <= TLD_B3;
                    end

                    TLD_B3: begin
                        tld_data_staging[15:8] <= rx_byte;
                        pkt_state              <= TLD_B4;
                    end

                    TLD_B4: begin
                        tld_data_staging[23:16] <= rx_byte;
                        pkt_state               <= TLD_B5;
                    end

                    TLD_B5: begin
                        tld_data_staging[31:24] <= rx_byte;
                        pkt_state               <= TLD_B6;
                    end

                    TLD_B6: begin
                        thresh_wr_valid <= 1'b1;
                        thresh_wr_data  <= {rx_byte, tld_data_staging[31:0]};
                        pkt_state       <= PKT_IDLE;
                    end

                    default: pkt_state <= PKT_IDLE;
                endcase
            end

            if (core_gesture_valid) begin
                gesture_pkt_pending <= 1'b1;
                gesture_pkt_code    <= core_gesture;
                gesture_pkt_conf    <= core_gesture_confidence;
                gesture_pkt_evthi   <= core_debug_event_count[7:4];
                last_gesture        <= core_gesture;
                gesture_led_ctr     <= LED_HOLD_CYCLES[LED_HOLD_BITS-1:0];
            end else if (gesture_led_ctr != 0) begin
                gesture_led_ctr <= gesture_led_ctr - 1'b1;
            end

            if (activity_led_ctr != 0)
                activity_led_ctr <= activity_led_ctr - 1'b1;

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
                    second_byte_data    <= {3'b0, gesture_pkt_conf, gesture_pkt_evthi};
                    gesture_pkt_pending <= 1'b0;
                end
            end

            if (!tx_busy && tx_fifo_out_valid) begin
                tx_byte           <= tx_fifo_out_data;
                tx_byte_valid     <= 1'b1;
                tx_fifo_out_ready <= 1'b1;
            end

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
