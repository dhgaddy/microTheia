`timescale 1ns/1ps

// UART Receiver - 8N1, synchronous active-high reset

module uart_rx #(
    parameter CLKS_PER_BIT = 104  // 12MHz / 115200
)(
    input  logic clk,
    input  logic rst,
    input  logic rx,
    output logic [7:0] data,
    output logic valid
);

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    logic [1:0] state;
    logic [7:0] clk_cnt;
    logic [2:0] bit_idx;
    logic [7:0] rx_data;
    logic rx_sync, rx_d;

    // Double-flop synchronizer for metastability
    always @(posedge clk) begin
        if (rst) begin
            rx_sync <= 1'b1;
            rx_d <= 1'b1;
        end else begin
            rx_sync <= rx;
            rx_d <= rx_sync;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            clk_cnt <= 8'd0;
            bit_idx <= 3'd0;
            rx_data <= 8'd0;
            data <= 8'd0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (state)
                IDLE: begin
                    clk_cnt <= 8'd0;
                    bit_idx <= 3'd0;
                    if (rx_d == 1'b0)
                        state <= START;
                end

                START: begin
                    if (clk_cnt == (CLKS_PER_BIT - 1) / 2) begin
                        if (rx_d == 1'b0) begin
                            clk_cnt <= 8'd0;
                            state <= DATA;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 8'd0;
                        rx_data[bit_idx] <= rx_d;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 8'd0;
                        state <= IDLE;
                        if (rx_d == 1'b1) begin
                            data <= rx_data;
                            valid <= 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
