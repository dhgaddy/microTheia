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
    //using pinout chart from design spec
    //supporting alternate input path, but currently no support for alternate output arrangement

    // pull up and pull down config
    assign input_pu = '0;
    assign input_pd = 12'b111000000011; //pulling down unused input pins

    // Set the bidir outputs
    // bidirect pins 2 - 5 are curently reserved
    // NOTE: do NOT use a blanket `assign bidir_oe = '0` here; doing so creates
    // multiple drivers on bits that are also driven by bit-select assigns below,
    // which produces X in simulation. Only the truly unused reserved bits get '0.
    assign bidir_oe[5:2] = '0; // reserved pins, output disabled
    // debug + heartbeat + spi_ready always-on; MISO pins (38/39) are gated by
    // alt_select below so the inactive MISO can be driven by an off-chip
    // master if desired.
    assign bidir_oe[37:6] = '1;
    assign bidir_oe[0]    = 1'b1;   // heartbeat — always observable
    assign bidir_oe[1]    = 1'b1;   // spi_ready — always observable
    assign bidir_out[5:2] = '0; // reserved pins driven to 0


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

    soc #(
        .CLK_FREQ_HZ(64_000_000),
        .WINDOW_MS(1000),
        .GRID_SIZE(16),
        .NUM_BINS(16),
        .READOUT_BINS(16),
        .COUNTER_BITS(16),
        .FIFO_DEPTH(256),
        .DATA_WIDTH(32),
        .REQUIRE_TIME_HIGH(1),
        .SENSOR_WIDTH(320),
        .SENSOR_HEIGHT(320),
        .WEIGHT_BITS(8),
        .NUM_CLASSES(4)
    ) u_soc (
        `ifdef USE_POWER_PINS
        .VDD(VDD),
        .VSS(VSS),
        `endif
        .clk(clk),
        .rst(!rst_n), //active low here and then no downstream module need to be adjusted
        .MOSI(MOSI_wire), //master out slave in (from off chip to in chip)
        .SCLK(SCLK_wire), //no CDC or DLL needed if SCLK sufficiently slower than clk. system default is 64 MHz chip clock with 32 MHz SCLK (2x ratio)
        .CS(CS_wire), // aka SS, signals a transaction is occuring or not
        .MISO(MISO_wire), //master in slave out (from in chip to off chip)
        .debug_bus(debug_bus), //bus from debug mux, pages selectable via commands over spi
        .spi_ready(spi_ready) //signal that the spi frontend module has succesfully initialized and is ready to begin operation. NOTE: reset must go high to initialize spi module
    );
    //assigning spi ready and debug bus to pins according to pinout chart, currently do not support alternating these to a different pinout
    assign bidir_out[1] = spi_ready;
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
    assign MOSI_wire = alt_select ? input_in[3] : input_in[6];
    assign SCLK_wire = alt_select ? input_in[2] : input_in[5];
    assign CS_wire   = alt_select ? input_in[4] : input_in[7];

    //muxing MISO to output pins, and disabling output for inactive ports
    assign bidir_out[38] =  !alt_select ? MISO_wire : 1'b0;
    assign bidir_out[39] = alt_select ? MISO_wire : 1'b0;
    assign bidir_oe[38] = !alt_select; // default MISO pin active when alt_select=0
    assign bidir_oe[39] = alt_select;  // alternative MISO pin active when alt_select=1

    //heartbeat signal, approx 1 sec on and 1 sec off
    logic [24:0] counter;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            counter <= '0;
        else
            counter <= counter + 1'b1;
    end
    assign bidir_out[0] = counter[24]; //bidirect pin 0 is heartbeat signal.

endmodule

`default_nettype wire
