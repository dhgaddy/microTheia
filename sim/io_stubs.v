// Behavioral stubs for GF180MCU IO pad cells.
// Used for RTL simulation so the PDK is not required.
// All power-pin ports are ignored (USE_POWER_PINS not defined in RTL sim).

`timescale 1ns/1ps

// Schmitt-trigger input pad (clk uses this)
module gf180mcu_fd_io__in_s (
    input  wire PAD,
    output wire Y,
    input  wire PU,
    input  wire PD
);
    assign Y = PAD;
endmodule

// Standard input pad
module gf180mcu_fd_io__in_c (
    input  wire PAD,
    output wire Y,
    input  wire PU,
    input  wire PD
);
    assign Y = PAD;
endmodule

// Bidirectional 24 mA pad
// When OE=1 the core drives PAD; Y always follows PAD.
module gf180mcu_fd_io__bi_24t (
    input  wire A,
    input  wire OE,
    output wire Y,
    inout  wire PAD,
    input  wire CS,
    input  wire SL,
    input  wire IE,
    input  wire PU,
    input  wire PD
);
    assign PAD = OE ? A : 1'bz;
    assign Y   = PAD;
endmodule

// Power / ground pads — no signal ports needed for RTL sim
module gf180mcu_ws_io__dvdd;
endmodule

module gf180mcu_ws_io__dvss;
endmodule

// Analog signal pad — tie off as high-Z
module gf180mcu_fd_io__asig_5p0 (
    inout wire ASIG5V
);
endmodule

// SRAM stub — 512x8 1R1W
// Provides just enough behavior to elaborate; functional testing uses soc_tb.
module gf180mcu_fd_ip_sram__sram512x8m8wm1 #(
    parameter   ADDR_WIDTH = 9,
    parameter   DATA_WIDTH = 8
)(
    input  wire               CLK,
    input  wire               CEN,
    input  wire               GWEN,
    input  wire [DATA_WIDTH-1:0] WEN,
    input  wire [ADDR_WIDTH-1:0] A,
    input  wire [DATA_WIDTH-1:0] D,
    output reg  [DATA_WIDTH-1:0] Q
);
    reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
    integer i;
    initial begin
        for (i = 0; i < (1<<ADDR_WIDTH); i = i + 1)
            mem[i] = {DATA_WIDTH{1'b0}};
    end
    always @(posedge CLK) begin
        if (!CEN) begin
            if (!GWEN)
                mem[A] <= D & ~WEN;
            Q <= mem[A];
        end
    end
endmodule
