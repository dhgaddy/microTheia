// SPDX-FileCopyrightText: © 2025 XXX Authors
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module chip_core #(
    parameter NUM_INPUT_PADS,
    parameter NUM_BIDIR_PADS,
    parameter NUM_ANALOG_PADS
    )(
    `ifdef USE_POWER_PINS
    inout  wire VDD,
    inout  wire VSS,
    `endif
    
    input  wire clk,       // clock
    input  wire rst_n,     // reset (active low)
    
    input  wire [NUM_INPUT_PADS-1:0] input_in,   // Input value
    output wire [NUM_INPUT_PADS-1:0] input_pu,   // Pull-up
    output wire [NUM_INPUT_PADS-1:0] input_pd,   // Pull-down

    input  wire [NUM_BIDIR_PADS-1:0] bidir_in,   // Input value
    output wire [NUM_BIDIR_PADS-1:0] bidir_out,  // Output value
    output wire [NUM_BIDIR_PADS-1:0] bidir_oe,   // Output enable
    output wire [NUM_BIDIR_PADS-1:0] bidir_cs,   // Input type (0=CMOS Buffer, 1=Schmitt Trigger)
    output wire [NUM_BIDIR_PADS-1:0] bidir_sl,   // Slew rate (0=fast, 1=slow)
    output wire [NUM_BIDIR_PADS-1:0] bidir_ie,   // Input enable
    output wire [NUM_BIDIR_PADS-1:0] bidir_pu,   // Pull-up
    output wire [NUM_BIDIR_PADS-1:0] bidir_pd,   // Pull-down

    inout  wire [NUM_ANALOG_PADS-1:0] analog  // Analog
);

    // See here for usage: https://gf180mcu-pdk.readthedocs.io/en/latest/IPs/IO/gf180mcu_fd_io/digital.html

    // =========================================================================
    // Pin Mapping (from pinout table)
    // =========================================================================
    //
    // Pin 1 = Reset, Pin 2 = Clock → handled by top-level clk / rst_n
    //
    // input_in index  | Pin | Assignment
    // ----------------+-----+-------------------
    //        0        |  3  | SPI_CTRL_SCLK
    //        1        |  4  | SPI_CTRL_MOSI
    //        2        |  5  | SPI_CTRL_CS
    //        3        |  6  | valid_in (CPI)
    //        4        |  7  | data_in[0] (CPI)
    //        5        |  8  | data_in[1] (CPI)
    //        6        |  9  | data_in[2] (CPI)
    //        7        | 10  | data_in[3] (CPI)
    //        8        | 11  | data_in[4] (CPI)
    //        9        | 12  | data_in[5] (CPI)
    //       10        | 13  | data_in[6] (CPI)
    //       11        | 14  | data_in[7] (CPI)
    //
    // bidir index | Pin | Dir | Assignment
    // ------------+-----+-----+-------------------
    //      0      | 15  | O   | SPI_FLSH_SCLK
    //      1      | 16  | O   | SPI_FLASH_MOSI
    //      2      | 17  | O   | SPI_FLASH_CS
    //      3      | 18  | I   | SPI_FLASH_MISO
    //      4      | 19  | O   | SPI_CTRL_MISO
    //      5      | 20  | O   | ready_o (CPI)
    //    6..37    |21-52| O   | debug_bus[0:31]
    //     38      | 53  | O   | heartbeat
    //     39      | 54  | O   | boot_signal

    // =========================================================================
    // Input pad aliases
    // =========================================================================
    wire        spi_ctrl_sclk  = input_in[0];
    wire        spi_ctrl_mosi  = input_in[1];
    wire        spi_ctrl_cs_n  = input_in[2];
    wire        evt_valid_in   = input_in[3];
    wire [7:0]  evt_data_in    = input_in[11:4];

    // Disable pull-up and pull-down for input pads
    assign input_pu = '0;
    assign input_pd = '0;

    // =========================================================================
    // Bidir pad direction control
    // =========================================================================
    // All bidir pads are outputs except bidir[3] (SPI_FLASH_MISO = input).
    reg [NUM_BIDIR_PADS-1:0] bidir_oe_r;
    always @(*) begin
        bidir_oe_r      = '1;   // default: all outputs
        bidir_oe_r[3]   = 1'b0; // SPI_FLASH_MISO is an input
    end

    assign bidir_oe = bidir_oe_r;
    assign bidir_cs = '0;
    assign bidir_sl = '0;
    assign bidir_ie = ~bidir_oe;
    assign bidir_pu = '0;
    assign bidir_pd = '0;

    // Bidir input alias
    wire spi_flash_miso = bidir_in[3];

    // =========================================================================
    // Inter-module wires
    // =========================================================================

    // Flash FSM → voxel_bin_core: weight write port
    wire        weight_wr_valid;
    wire [2:0]  weight_wr_class;
    wire [10:0] weight_wr_addr;
    wire [7:0]  weight_wr_data;

    // Flash FSM → voxel_bin_core: threshold write port
    wire        thresh_wr_valid;
    wire [3:0]  thresh_wr_addr;
    wire [35:0] thresh_wr_data;

    // Flash FSM control / status
    wire        flash_core_rst;
    wire        boot_done;
    wire        boot_fail;
    wire [2:0]  main_state_dbg;
    wire [2:0]  load_state_dbg;
    wire [7:0]  id_mfr;
    wire [7:0]  id_type;
    wire [7:0]  id_capacity;

    // Flash SPI signals
    wire        spi_flash_cs_n;
    wire        spi_flash_sck;
    wire        spi_flash_mosi_o;

    // Voxel core outputs
    wire [1:0]  gesture;
    wire        gesture_valid;
    wire [35:0] gesture_confidence;

    // Voxel core debug outputs
    wire [15:0] debug_event_count;
    wire        debug_fifo_empty;
    wire        debug_fifo_full;
    wire [1:0]  debug_temporal_phase;
    wire        debug_class_valid;
    wire        debug_class_pass;
    wire        debug_feature_window_ready;
    wire        debug_capture_active;
    wire        debug_score_busy;
    wire [31:0] debug_mux;

    // =========================================================================
    // SPI Control Slave — register interface
    // =========================================================================
    wire [1:0]  active_mode;
    wire        boot_req;
    wire        reload_req;
    wire        debug_req;
    wire [4:0]  debug_select;
    wire        spi_ctrl_miso_o;

    // Combined reset: chip reset OR flash-FSM core reset
    wire core_rst = !rst_n | flash_core_rst;

    // ---- SPI shift register (clocked on spi_ctrl_sclk) ----
    reg [15:0] spi_shift;
    reg [3:0]  spi_bit_cnt;
    reg        spi_done;

    always @(posedge spi_ctrl_sclk or posedge spi_ctrl_cs_n) begin
        if (spi_ctrl_cs_n) begin
            spi_bit_cnt <= 4'd0;
            spi_done    <= 1'b0;
        end else begin
            spi_shift   <= {spi_shift[14:0], spi_ctrl_mosi};
            spi_bit_cnt <= spi_bit_cnt + 4'd1;
            spi_done    <= (spi_bit_cnt == 4'd15);
        end
    end

    wire [7:0] spi_addr  = spi_shift[15:8];
    wire [7:0] spi_wdata = spi_shift[7:0];

    // ---- SPI read-back mux ----
    reg [7:0] spi_rdata;
    always @(*) begin
        case (spi_addr)
            8'h00:   spi_rdata = {6'd0, active_mode};
            8'h02:   spi_rdata = {3'd0, debug_select};
            8'h10:   spi_rdata = debug_mux[7:0];
            8'h11:   spi_rdata = debug_mux[15:8];
            8'h12:   spi_rdata = debug_mux[23:16];
            8'h13:   spi_rdata = debug_mux[31:24];
            8'h20:   spi_rdata = {6'd0, gesture};
            8'h21:   spi_rdata = gesture_confidence[7:0];
            8'h22:   spi_rdata = gesture_confidence[15:8];
            8'h23:   spi_rdata = gesture_confidence[23:16];
            8'h24:   spi_rdata = gesture_confidence[31:24];
            8'h25:   spi_rdata = {4'd0, gesture_confidence[35:32]};
            8'h30:   spi_rdata = id_mfr;
            8'h31:   spi_rdata = id_type;
            8'h32:   spi_rdata = id_capacity;
            8'h33:   spi_rdata = {5'd0, main_state_dbg};
            8'h34:   spi_rdata = {5'd0, load_state_dbg};
            default: spi_rdata = 8'h00;
        endcase
    end

    assign spi_ctrl_miso_o = spi_rdata[7 - spi_bit_cnt[2:0]];

    // ---- CDC synchronizer: SPI done → clk domain ----
    reg spi_done_d1, spi_done_d2;
    reg [7:0] spi_addr_lat, spi_wdata_lat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_done_d1   <= 1'b0;
            spi_done_d2   <= 1'b0;
            spi_addr_lat  <= 8'd0;
            spi_wdata_lat <= 8'd0;
        end else begin
            spi_done_d1 <= spi_done;
            spi_done_d2 <= spi_done_d1;
            if (spi_done && !spi_done_d1) begin
                spi_addr_lat  <= spi_addr;
                spi_wdata_lat <= spi_wdata;
            end
        end
    end

    wire spi_wr_pulse = spi_done_d1 && !spi_done_d2;

    // ---- Control registers ----
    reg [1:0]  mode_reg;
    reg        boot_req_reg;
    reg        reload_req_reg;
    reg        debug_req_reg;
    reg [4:0]  debug_select_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_reg         <= 2'b00;
            boot_req_reg     <= 1'b0;
            reload_req_reg   <= 1'b0;
            debug_req_reg    <= 1'b0;
            debug_select_reg <= 5'd0;
        end else begin
            // Self-clearing pulses
            boot_req_reg   <= 1'b0;
            reload_req_reg <= 1'b0;
            debug_req_reg  <= 1'b0;

            if (spi_wr_pulse) begin
                case (spi_addr_lat)
                    8'h00: mode_reg         <= spi_wdata_lat[1:0];
                    8'h01: begin
                        boot_req_reg   <= spi_wdata_lat[0];
                        reload_req_reg <= spi_wdata_lat[1];
                        debug_req_reg  <= spi_wdata_lat[2];
                    end
                    8'h02: debug_select_reg <= spi_wdata_lat[4:0];
                    default: ;
                endcase
            end
        end
    end

    assign active_mode  = mode_reg;
    assign boot_req     = boot_req_reg;
    assign reload_req   = reload_req_reg;
    assign debug_req    = debug_req_reg;
    assign debug_select = debug_select_reg;

    // =========================================================================
    // Event word assembly from CPI byte interface
    // =========================================================================
    // CPI provides 8-bit data + valid. Assemble into 32-bit evt_word.
    localparam DATA_WIDTH = 32;

    reg [DATA_WIDTH-1:0] evt_word_reg;
    reg [1:0]            evt_byte_phase;
    reg                  evt_word_valid_reg;
    wire                 evt_word_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            evt_word_reg       <= '0;
            evt_byte_phase     <= 2'b00;
            evt_word_valid_reg <= 1'b0;
        end else begin
            if (evt_word_valid_reg && evt_word_ready)
                evt_word_valid_reg <= 1'b0;

            if (evt_valid_in && !evt_word_valid_reg) begin
                case (evt_byte_phase)
                    2'd0: begin
                        evt_word_reg[31:24] <= evt_data_in;
                        evt_byte_phase      <= 2'd1;
                    end
                    2'd1: begin
                        evt_word_reg[23:16] <= evt_data_in;
                        evt_byte_phase      <= 2'd2;
                    end
                    2'd2: begin
                        evt_word_reg[15:8]  <= evt_data_in;
                        evt_byte_phase      <= 2'd3;
                    end
                    2'd3: begin
                        evt_word_reg[7:0]   <= evt_data_in;
                        evt_byte_phase      <= 2'd0;
                        evt_word_valid_reg  <= 1'b1;
                    end
                endcase
            end
        end
    end

    wire cpi_ready = !evt_word_valid_reg;

    // =========================================================================
    // Heartbeat (~1 Hz toggle, sign-of-life)
    // =========================================================================
    localparam HEARTBEAT_DIV = 24'd12_000_000;
    reg [23:0] hb_cnt;
    reg        heartbeat_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt      <= '0;
            heartbeat_r <= 1'b0;
        end else begin
            if (hb_cnt == HEARTBEAT_DIV - 1) begin
                hb_cnt      <= '0;
                heartbeat_r <= ~heartbeat_r;
            end else begin
                hb_cnt <= hb_cnt + 1;
            end
        end
    end

    // =========================================================================
    // Voxel Bin Core
    // =========================================================================
    voxel_bin_core #(
        .CLK_FREQ_HZ               (12_000_000),
        .WINDOW_MS                  (1000),
        .GRID_SIZE                  (16),
        .NUM_BINS                   (8),
        .READOUT_BINS               (8),
        .COUNTER_BITS               (16),
        .FIFO_DEPTH                 (256),
        .DATA_WIDTH                 (DATA_WIDTH),
        .REQUIRE_TIME_HIGH          (1),
        .SWAP_INPUT_BYTES           (0),
        .MAP_SWAP_XY                (0),
        .MAP_FLIP_X                 (0),
        .MAP_FLIP_Y                 (0),
        .SENSOR_WIDTH               (320),
        .SENSOR_HEIGHT              (320),
        .WEIGHT_BITS                (8),
        .NUM_CLASSES                (4),
        .CYCLES_PER_BIN             (0),
        .SCORE_BITS                 (36)
    ) u_voxel_bin_core (
        .clk                        (clk),
        .rst                        (core_rst),

        .active_mode_i              (active_mode),

        .evt_word                   (evt_word_reg),
        .evt_word_valid             (evt_word_valid_reg),
        .evt_word_ready             (evt_word_ready),

        .gesture                    (gesture),
        .gesture_valid              (gesture_valid),
        .gesture_confidence         (gesture_confidence),

        .weight_wr_valid_i          (weight_wr_valid),
        .weight_wr_class_i          (weight_wr_class),
        .weight_wr_addr_i           (weight_wr_addr),
        .weight_wr_data_i           (weight_wr_data),

        .thresh_wr_valid_i          (thresh_wr_valid),
        .thresh_wr_addr_i           (thresh_wr_addr),
        .thresh_wr_data_i           (thresh_wr_data),

        .debug_event_count          (debug_event_count),
        .debug_fifo_empty           (debug_fifo_empty),
        .debug_fifo_full            (debug_fifo_full),
        .debug_temporal_phase       (debug_temporal_phase),
        .debug_class_valid          (debug_class_valid),
        .debug_class_pass           (debug_class_pass),
        .debug_feature_window_ready (debug_feature_window_ready),
        .debug_capture_active       (debug_capture_active),
        .debug_score_busy           (debug_score_busy),

        .debug_mux                  (debug_mux),
        .debug_select               (debug_select)
    );

    // =========================================================================
    // Flash FSM
    // =========================================================================
    chip_flash_fsm #(
        .PWR_WAIT_CYCLES    (10_000),
        .RST_WAIT_CYCLES    (1_000),
        .SPI_DIV            (4),
        .USE_4BYTE_ADDR     (0),
        .FLASH_WEIGHT_BASE  (32'h0000_0000),
        .FLASH_THRESH_BASE  (32'h0001_0000),

        .NUM_CLASSES        (4),
        .GRID_SIZE          (16),
        .READOUT_BINS       (8),
        .WEIGHT_BITS        (8),
        .SCORE_BITS         (36),

        .THRESH_BIG_ENDIAN  (0)
    ) u_chip_flash_fsm (
        .clk                (clk),
        .rst_n              (rst_n),

        .boot_req_i         (boot_req),
        .reload_req_i       (reload_req),
        .debug_req_i        (debug_req),

        .spi_miso_i         (spi_flash_miso),
        .spi_cs_n_o         (spi_flash_cs_n),
        .spi_sck_o          (spi_flash_sck),
        .spi_mosi_o         (spi_flash_mosi_o),

        .weight_wr_valid_o  (weight_wr_valid),
        .weight_wr_class_o  (weight_wr_class),
        .weight_wr_addr_o   (weight_wr_addr),
        .weight_wr_data_o   (weight_wr_data),

        .thresh_wr_valid_o  (thresh_wr_valid),
        .thresh_wr_addr_o   (thresh_wr_addr),
        .thresh_wr_data_o   (thresh_wr_data),

        .core_rst_o         (flash_core_rst),

        .boot_done_o        (boot_done),
        .boot_fail_o        (boot_fail),
        .main_state_dbg_o   (main_state_dbg),
        .load_state_dbg_o   (load_state_dbg),
        .id_mfr_o           (id_mfr),
        .id_type_o          (id_type),
        .id_capacity_o      (id_capacity)
    );

    // =========================================================================
    // Bidir Output Assignments
    // =========================================================================
    reg [NUM_BIDIR_PADS-1:0] bidir_out_r;

    always @(*) begin
        bidir_out_r = '0;

        bidir_out_r[0]    = spi_flash_sck;       // Pin 15: SPI_FLSH_SCLK
        bidir_out_r[1]    = spi_flash_mosi_o;    // Pin 16: SPI_FLASH_MOSI
        bidir_out_r[2]    = spi_flash_cs_n;      // Pin 17: SPI_FLASH_CS
        bidir_out_r[3]    = 1'b0;                // Pin 18: SPI_FLASH_MISO (input)
        bidir_out_r[4]    = spi_ctrl_miso_o;     // Pin 19: SPI_CTRL_MISO
        bidir_out_r[5]    = cpi_ready;           // Pin 20: ready_o (CPI)
        bidir_out_r[37:6] = debug_mux;           // Pin 21-52: debug_bus[0:31]
        bidir_out_r[38]   = heartbeat_r;         // Pin 53: heartbeat
        bidir_out_r[39]   = boot_done;           // Pin 54: boot_signal
    end

    assign bidir_out = bidir_out_r;

    // =========================================================================
    // Unused signals WAITING FOR DALTON SPI
    // =========================================================================
    logic _unused;
    assign _unused = &{bidir_in, boot_fail, gesture_valid, gesture_confidence,
                       debug_event_count, debug_fifo_empty, debug_fifo_full,
                       debug_temporal_phase, debug_class_valid, debug_class_pass,
                       debug_feature_window_ready, debug_capture_active,
                       debug_score_busy, load_state_dbg, id_mfr, id_type,
                       id_capacity, analog};

endmodule

`default_nettype wire