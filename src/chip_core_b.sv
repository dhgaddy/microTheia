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
    
    //the way these work is for each bus, the indices of each wire all correspond to 1 pin and its configuration settings
    // example for the input bus: input_in[0] is the data in, input_pu[0] is the pull up configuration and input_pd[0] is the pull down config all for input-only pin 0
    // bidirectional pins have more configuration settings
    input  wire [NUM_INPUT_PADS-1:0] input_in,   // Input value: tie these to the input ports
    output wire [NUM_INPUT_PADS-1:0] input_pu,   // Pull-up
    output wire [NUM_INPUT_PADS-1:0] input_pd,   // Pull-down

    input  wire [NUM_BIDIR_PADS-1:0] bidir_in,   // Input value: we're not using
    output wire [NUM_BIDIR_PADS-1:0] bidir_out,  // Output value: tie the output ports to here
    output wire [NUM_BIDIR_PADS-1:0] bidir_oe,   // Output enable
    output wire [NUM_BIDIR_PADS-1:0] bidir_cs,   // Input type (0=CMOS Buffer, 1=Schmitt Trigger)
    output wire [NUM_BIDIR_PADS-1:0] bidir_sl,   // Slew rate (0=fast, 1=slow)
    output wire [NUM_BIDIR_PADS-1:0] bidir_ie,   // Input enable
    output wire [NUM_BIDIR_PADS-1:0] bidir_pu,   // Pull-up
    output wire [NUM_BIDIR_PADS-1:0] bidir_pd,   // Pull-down

    inout  wire [NUM_ANALOG_PADS-1:0] analog  // Analog: we're not using, but interested in if we could use for MIPI? super high speed streaming protocol that requires an analog pin
);

    // See here for usage: https://gf180mcu-pdk.readthedocs.io/en/latest/IPs/IO/gf180mcu_fd_io/digital.html
    //using pinout chart from design spec
    //supporting alternate input path, but currently no support for alternate output arrangement

    // pull up and pull down config
    assign input_pu = '0;
    assign input_pd = 12'b111000000011; //pulling down unused input pins

    // Set the bidir outputs
    // bidirect pins 2 - 5 are curently reserved
    assign bidir_oe = '0;
    // debug + heartbeat + spi_ready
    assign bidir_oe[37:6] = '1;
    assign bidir_oe[38]   = 1'b1;
    assign bidir_oe[39]   = 1'b1;
     // pins 0 and 1 are assigned after system_package module along with alternate mux pin logic
    assign bidir_out = '0;
   

    assign bidir_cs = '0; //not relevant since all of our bidirects are output
    assign bidir_sl = '0; //slew rate. fast is 0, using as default for all. maybe could set debug pins to slow slew rate?
    assign bidir_ie = '0; //input disabled for all bidirectional pins
    assign bidir_pu = '0; //pull ups diasbled for all output
    assign bidir_pd = '0; //pull downs disabled for all output
    
    logic _unused;
    assign _unused = &bidir_in;
    //end pin config
    //begin our system
    logic MOSI_wire, MISO_wire, CS_wire, SCLK_wire, alt_select, spi_ready;
    logic [31:0] debug_bus;

    system_package #(
    .CLK_FREQ_HZ(32_000_000),
    .WINDOW_MS(1000),
    .GRID_SIZE(16),
    .NUM_BINS(8),
    .READOUT_BINS(8),
    .COUNTER_BITS(16),
    .FIFO_DEPTH(256),
    .DATA_WIDTH(32),
    .REQUIRE_TIME_HIGH(1),
    .SENSOR_WIDTH(320),
    .SENSOR_HEIGH(320),
    .WEIGHT_BITS(8),
    .POR_CYCLES(1024),
    .NUM_CLASSES(4),
    .SOFT_RESET_CYCLES(64),
    // SCORE_BITS must match voxel_bin_core's formula: COUNTER_BITS+WEIGHT_BITS+clog2(FC)+1
    .SCORE_BITS(COUNTER_BITS + WEIGHT_BITS + $clog2(READOUT_BINS * GRID_SIZE * GRID_SIZE) + 1)
    ) system_package_a (
    .clk(clk),
    .reset(!rst_n), //active low here and then no downstream module need to be adjusted
    .MOSI(MOSI_wire), //master out slave in (from off chip to in chip)
    .SCLK(SCLK_wire), //no CDC or DLL needed if SCLK sufficiently slower than clk. apparently chip clock must be 4x fast minimum (32 MHz chip, 8 MHz sclk should work)
    .CS(CS_wire), // aka SS, signals a transaction is occuring or not
    .MISO(MISO_wire), //master in slave out (from in chip to off chip)
    .debug_bus(debug_bus), //bus from debug mux, pages selectable via commands over spi
    .spi_ready(spi_ready) //signal that the spi frontend module has succesfully initialized and is ready to begin operation. NOTE: reset must go high to initialize spi module
);
    //assigning spi ready and debug bus to pins according to pinout chart, currently do not support alternating these to a different pinout
    assign bidir_out[39] = spi_ready;
    assign bidir_out [37:6] = debug_bus;
    
    // sync ALT_INPUT_MODE pin, then detect rising edge
    logic input_in_8_sync_0;
    logic input_in_8_sync_1;
    logic input_in_8_prev;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_in_8_sync_0 <= 1'b0;
            input_in_8_sync_1 <= 1'b0;
            input_in_8_prev   <= 1'b0;
        end else begin
            input_in_8_sync_0 <= input_in[8];
            input_in_8_sync_1 <= input_in_8_sync_0;
            input_in_8_prev   <= input_in_8_sync_1;
        end
    end
    wire alt_mode_trigger;
    assign alt_mode_trigger = input_in_8_sync_1 & ~input_in_8_prev;

    //alt_select flips on rising edge from ALT_INPUT_MODE pin
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            alt_select <= 1'b0;
        end
        else if(alt_mode_trigger) begin
            alt_select <= ~alt_select;
        end       
    end 
    // muxing input pins and spi interface, using alt_select signal
    always_comb begin
        if(alt_select) begin
            MOSI_wire = input_in[3];
            SCLK_wire = input_in[2];
            CS_wire =input_in[4];
           
        end
        else begin
            MOSI_wire = input_in[6];
            SCLK_wire = input_in[5];
            CS_wire =input_in[7];
           
        end
    end          
    //muxing MISO to output pins, and disabling output for inactive ports
    assign bidir_out[0] =  !alt_select ? MISO_wire : 1'b0;
    assign bidir_out[1] = alt_select ? MISO_wire : 1'b0;
    assign bidir_oe[0] = !alt_select;  // active default MISO
    assign bidir_oe[1] = alt_select;  // active alternative MISO

    //heartbeat signal, approx 1 sec on and 1 sec off
    logic [24:0] counter;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            counter <= '0;
        else
            counter <= counter + 1'b1;
    end
    assign bidir_out[38] = counter[24]; //bidirect pin 38 is heartbeat signal.

endmodule

`default_nettype wire