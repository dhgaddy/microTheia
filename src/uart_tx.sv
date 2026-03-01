`timescale 1ns/1ps

// UART Transmitter - 8N1, synchronous active-high reset

module uart_tx #(
    parameter CLKS_PER_BIT = 104  // 12MHz / 115200
)(
    input  logic clk,
    input  logic rst,
    input  logic [7:0] data,
    input  logic valid,
    output logic tx,
    output logic busy
);

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    logic [1:0] state;
    logic [7:0] clk_cnt;
    logic [2:0] bit_idx;
    logic [7:0] tx_data;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            clk_cnt <= 8'd0;
            bit_idx <= 3'd0;
            tx_data <= 8'd0;
            tx <= 1'b1;
            busy <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1'b1;
                    clk_cnt <= 8'd0;
                    bit_idx <= 3'd0;
                    busy <= 1'b0;
                    if (valid) begin
                        tx_data <= data;
                        busy <= 1'b1;
                        state <= START;
                    end
                end

                START: begin
                    tx <= 1'b0;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 8'd0;
                        state <= DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                DATA: begin
                    tx <= tx_data[bit_idx];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 8'd0;
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
                    tx <= 1'b1;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 8'd0;
                        state <= IDLE;
                        busy <= 1'b0;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
