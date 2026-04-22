// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

// EVT2 word fields: [31:28] type, [27:22] ts_lsb, [21:11] x, [10:0] y
// Packet types: 0x0 CD_OFF, 0x1 CD_ON, 0x8 TIME_HIGH

module evt2_decoder #(
    parameter int SENSOR_WIDTH      = 320,
    parameter int SENSOR_HEIGHT     = 320,
    parameter int GRID_SIZE         = 16,
    parameter bit REQUIRE_TIME_HIGH = 1'b1,
    parameter bit SWAP_INPUT_BYTES  = 1'b0,
    parameter bit MAP_SWAP_XY       = 1'b0,
    parameter bit MAP_FLIP_X        = 1'b0,
    parameter bit MAP_FLIP_Y        = 1'b0
)(
    input  logic                         clk,
    input  logic                         rst,
    input  logic [31:0]                  data_in,
    input  logic                         data_valid,
    input  logic                         event_ready_i,
    output logic                         data_ready,
    output logic [$clog2(GRID_SIZE)-1:0] x_out,
    output logic [$clog2(GRID_SIZE)-1:0] y_out,
    output logic                         event_valid,
    output logic [33:0]                  ts_out,       // full 34-bit timestamp of last CD event
    output logic [11:0]                  decoder_dbg,  //debug bus
    output logic [31:0]                  decoder_output_dbg
);

    localparam int GRID_BITS = $clog2(GRID_SIZE);

    localparam logic [3:0] EVT_CD_OFF    = 4'h0;
    localparam logic [3:0] EVT_CD_ON     = 4'h1;
    localparam logic [3:0] EVT_TIME_HIGH = 4'h8;
    localparam int SENSOR_W_M1           = SENSOR_WIDTH  - 1;
    localparam int SENSOR_H_M1           = SENSOR_HEIGHT - 1;
    localparam int X_BIN_DIV             = (SENSOR_WIDTH  / GRID_SIZE);
    localparam int Y_BIN_DIV             = (SENSOR_HEIGHT / GRID_SIZE);
    // Reciprocal-multiply constants: floor(v/D) = (v * M) >> 12 for v < SENSOR_DIM.
    // M = floor(2^12 / D) + 1 gives exact results; clamp to GRID_SIZE-1 handles edge.
    localparam int DIV_K                 = 12;

    localparam int X_M                   = (1 << DIV_K) / X_BIN_DIV + 1;
    localparam int Y_M                   = (1 << DIV_K) / Y_BIN_DIV + 1;

    wire [31:0] evt_word = SWAP_INPUT_BYTES
                         ? {data_in[7:0], data_in[15:8], data_in[23:16], data_in[31:24]}
                         : data_in;

    wire [3:0]  pkt_type = evt_word[31:28];
    wire [10:0] x_raw    = evt_word[21:11];
    wire [10:0] y_raw    = evt_word[10:0];
    wire        is_cd    = (pkt_type == EVT_CD_OFF) || (pkt_type == EVT_CD_ON);

    logic        have_time_high;
    logic [27:0] time_high_reg;

    logic [10:0]          x_clamped;
    logic [10:0]          y_clamped;
    logic [10:0]          x_oriented;
    logic [10:0]          y_oriented;
    logic [10:0]          x_swapped_raw;
    logic [10:0]          y_swapped_raw;
    logic [GRID_BITS-1:0] x_grid;
    logic [GRID_BITS-1:0] y_grid;
    logic [10+DIV_K:0]    x_prod_c, y_prod_c;
    logic [GRID_BITS:0]   x_grid_raw, y_grid_raw;

    always_comb begin
        if (x_raw >= SENSOR_WIDTH)
            x_clamped = SENSOR_W_M1[10:0];
        else
            x_clamped = x_raw;

        if (y_raw >= SENSOR_HEIGHT)
            y_clamped = SENSOR_H_M1[10:0];
        else
            y_clamped = y_raw;

        // Optional coordinate remap to align sensor orientation with trained model.
        x_swapped_raw = MAP_SWAP_XY ? y_clamped : x_clamped;
        y_swapped_raw = MAP_SWAP_XY ? x_clamped : y_clamped;

        // Re-clamp after optional swap in case SENSOR_WIDTH != SENSOR_HEIGHT.
        if (x_swapped_raw >= SENSOR_WIDTH)
            x_oriented = SENSOR_W_M1[10:0];
        else
            x_oriented = x_swapped_raw;

        if (y_swapped_raw >= SENSOR_HEIGHT)
            y_oriented = SENSOR_H_M1[10:0];
        else
            y_oriented = y_swapped_raw;

        if (MAP_FLIP_X)
            x_oriented = SENSOR_W_M1[10:0] - x_oriented;
        if (MAP_FLIP_Y)
            y_oriented = SENSOR_H_M1[10:0] - y_oriented;

        // Reciprocal-multiply: floor(v/D) = (v*M) >> DIV_K, exact for v < SENSOR_DIM.
        x_prod_c   = x_oriented * X_M;
        y_prod_c   = y_oriented * Y_M;

        x_grid_raw = x_prod_c >> DIV_K;
        y_grid_raw = y_prod_c >> DIV_K;

        // Only way to work for 8x8_4bins
        // x_grid_raw = x_clamped / X_BIN_DIV;
        // y_grid_raw = y_clamped / Y_BIN_DIV;

        x_grid = (x_grid_raw >= GRID_SIZE) ? GRID_SIZE-1 : x_grid_raw;
        y_grid = (y_grid_raw >= GRID_SIZE) ? GRID_SIZE-1 : y_grid_raw;
    end

    // Backpressure only for CD events that generate downstream samples.
    assign data_ready = (!is_cd) || event_ready_i;

    always_ff @(posedge clk) begin
        if (rst) begin
            have_time_high <= 1'b0;
            time_high_reg  <= '0;
            x_out          <= '0;
            y_out          <= '0;
            ts_out         <= '0;
            event_valid    <= 1'b0;
        end else begin
            event_valid <= 1'b0;

            if (data_valid && data_ready) begin
                case (pkt_type)
                    EVT_TIME_HIGH: begin
                        have_time_high <= 1'b1;
                        time_high_reg  <= evt_word[27:0];
                    end

                    EVT_CD_OFF,
                    EVT_CD_ON: begin
                        if (!REQUIRE_TIME_HIGH || have_time_high) begin
                            x_out       <= x_grid;
                            y_out       <= y_grid;
                            ts_out      <= {time_high_reg, evt_word[27:22]};
                            event_valid <= 1'b1;
                        end
                    end

                    default: begin
                        event_valid <= 1'b0;
                    end
                endcase
            end
        end
    end

    assign decoder_dbg        = {event_ready_i, data_valid, y_out, x_out, event_valid, data_ready};
    assign decoder_output_dbg = ts_out[31:0];
endmodule
