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
    
    // Disable pull-up and pull-down for input
    assign input_pu = '0;
    assign input_pd = '0;

    // Set the bidir as output
    assign bidir_oe = '1;
    assign bidir_cs = '0;
    assign bidir_sl = '0;
    assign bidir_ie = ~bidir_oe;
    assign bidir_pu = '0;
    assign bidir_pd = '0;
    
    logic _unused;
    assign _unused = &bidir_in;

    logic [NUM_BIDIR_PADS-1:0] count;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            count <= '0;
        end else begin
            if (&input_in) begin
                count <= count + 1;
            end
        end
    end

    logic [7:0] sram_0_out;

    gf180mcu_fd_ip_sram__sram512x8m8wm1 sram_0 (
        `ifdef USE_POWER_PINS
        .VDD  (VDD),
        .VSS  (VSS),
        `endif

        .CLK  (clk),
        .CEN  (1'b1),
        .GWEN (1'b0),
        .WEN  (8'b0),
        .A    ('0),
        .D    ('0),
        .Q    (sram_0_out)
    );

    logic [7:0] sram_1_out;

    gf180mcu_fd_ip_sram__sram512x8m8wm1 sram_1 (
        `ifdef USE_POWER_PINS
        .VDD  (VDD),
        .VSS  (VSS),
        `endif

        .CLK  (clk),
        .CEN  (1'b1),
        .GWEN (1'b0),
        .WEN  (8'b0),
        .A    ('0),
        .D    ('0),
        .Q    (sram_1_out)
    );

    assign bidir_out = count ^ {24'd0, sram_0_out, sram_1_out};


    // voxel_bin_core #(
    //     .CLK_FREQ_HZ               (),
    //     .WINDOW_MS                 (),
    //     .GRID_SIZE                 (),
    //     .NUM_BINS                  (),
    //     .READOUT_BINS              (),
    //     .COUNTER_BITS              (),
    //     .FIFO_DEPTH                (),
    //     .DATA_WIDTH                (),
    //     .REQUIRE_TIME_HIGH         (),
    //     .SWAP_INPUT_BYTES          (),
    //     .MAP_SWAP_XY               (),
    //     .MAP_FLIP_X                (),
    //     .MAP_FLIP_Y                (),
    //     .SENSOR_WIDTH              (),
    //     .SENSOR_HEIGHT             (),
    //     .WEIGHT_BITS               (),
    //     .NUM_CLASSES               (),
    //     .CYCLES_PER_BIN            (),
    //     .SCORE_BITS                ()
    // ) u_voxel_bin_core (
    //     .clk                       (clk),
    //     .rst                       (!rst_n),

    //     // Mode control
    //     .active_mode_i             (), // 00=BOOT, 01=PROGRAM, 10=CLASSIFY, 11=DEBUG

    //     // Event stream in
    //     .evt_word                  (),
    //     .evt_word_valid            (),
    //     .evt_word_ready            (),

    //     // Gesture 
    //     .gesture                   (),
    //     .gesture_valid             (),
    //     .gesture_confidence        (),

    //     // Weight SRAM write port — loads weights into the per-class SRAMs at runtime.
    //     // Do not assert weight_wr_valid_i while the MAC engine is running (mac_busy).
    //     .weight_wr_valid_i         (),
    //     .weight_wr_class_i         (),
    //     .weight_wr_addr_i          (),
    //     .weight_wr_data_i          (),

    //     // Threshold SRAM write port — addr 0-3 = class thresholds, 4-7 = diff thresholds.
    //     .thresh_wr_valid_i         (),
    //     .thresh_wr_addr_i          (),
    //     .thresh_wr_data_i          (),

    //     // Debug outputs
    //     .debug_event_count         (),
    //     .debug_fifo_empty          (),
    //     .debug_fifo_full           (),
    //     .debug_temporal_phase      (),
    //     .debug_class_valid         (),
    //     .debug_class_pass          (),
    //     .debug_feature_window_ready(),
    //     .debug_capture_active      (),
    //     .debug_score_busy          (),

    //     //debug mux output
    //     .debug_mux                 (),

    //     //debug mux input
    //     .debug_select              ()
    // );



    // chip_flash_fsm #(
    //     .PWR_WAIT_CYCLES  () 
    //     .RST_WAIT_CYCLES  ()
    //     .SPI_DIV          ()
    //     .USE_4BYTE_ADDR   () 
    //     .FLASH_WEIGHT_BASE()
    //     .FLASH_THRESH_BASE()

    //     .NUM_CLASSES      ()
    //     .GRID_SIZE        () 
    //     .READOUT_BINS     ()
    //     .WEIGHT_BITS      ()
    //     .SCORE_BITS       ()

    //     .THRESH_BIG_ENDIAN()
    // ) (
    //     .clk              (),
    //     .rst_n            (),

    //     .boot_req_i       (),
    //     .reload_req_i     (),
    //     .debug_req_i      (),

    //     .spi_miso_i       (),
    //     .spi_cs_n_o       (),
    //     .spi_sck_o        (),
    //     .spi_mosi_o       (),

    //     .weight_wr_valid_o(),
    //     .weight_wr_class_o(),
    //     .weight_wr_addr_o (),
    //     .weight_wr_data_o (),

    //     .thresh_wr_valid_o(),
    //     .thresh_wr_addr_o (),
    //     .thresh_wr_data_o (),

    //     .core_rst_o       (),

    //     .boot_done_o      (),
    //     .boot_fail_o      (),
    //     .main_state_dbg_o (),
    //     .load_state_dbg_o (),
    //     .id_mfr_o         (),
    //     .id_type_o        (),
    //     .id_capacity_o    ()
    // );


endmodule

`default_nettype wire
