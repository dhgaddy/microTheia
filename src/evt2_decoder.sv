`timescale 1ns/1ps

// EVT2 decoder for voxel-bin raw path.
// Input: EVT2 32-bit words; Output: 16x16 grid coordinates + polarity + timestamp.
// Spatial compression matches gradient-map behavior (320x320 -> 16x16).

module evt2_decoder #(
    parameter GRID_BITS = 4
)(
    input  logic                    clk,
    input  logic                    rst,
    input  logic [31:0]             data_in,
    input  logic                    data_valid,
    output logic                    data_ready,
    output logic [GRID_BITS-1:0]    x_out,
    output logic [GRID_BITS-1:0]    y_out,
    output logic                    polarity,
    output logic [15:0]             timestamp,
    output logic                    event_valid
);

    localparam [3:0] EVT_CD_OFF    = 4'h0;
    localparam [3:0] EVT_CD_ON     = 4'h1;
    localparam [3:0] EVT_TIME_HIGH = 4'h8;

    wire [3:0]  pkt_type          = data_in[31:28];
    wire [5:0]  ts_lsb            = data_in[27:22];
    wire [10:0] x_raw             = data_in[21:11];
    wire [10:0] y_raw             = data_in[10:0];
    wire [27:0] time_high_payload = data_in[27:0];

    logic [27:0] time_high_reg;

    wire [4:0] x_grid_raw5 = x_raw[8:4];
    wire [4:0] y_grid_raw5 = y_raw[8:4];
    wire [GRID_BITS-1:0] x_grid = (x_grid_raw5 > 5'd15) ? 4'd15 : x_grid_raw5[GRID_BITS-1:0];
    wire [GRID_BITS-1:0] y_grid = (y_grid_raw5 > 5'd15) ? 4'd15 : y_grid_raw5[GRID_BITS-1:0];

    wire [15:0] full_ts = {time_high_reg[9:0], ts_lsb};

    assign data_ready = data_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            time_high_reg <= '0;
            x_out         <= '0;
            y_out         <= '0;
            polarity      <= '0;
            timestamp     <= '0;
            event_valid   <= 1'b0;
        end else begin
            event_valid <= 1'b0;

            if (data_valid) begin
                case (pkt_type)
                    EVT_TIME_HIGH: time_high_reg <= time_high_payload;

                    EVT_CD_OFF: begin
                        x_out       <= x_grid;
                        y_out       <= y_grid;
                        polarity    <= 1'b0;
                        timestamp   <= full_ts;
                        event_valid <= 1'b1;
                    end

                    EVT_CD_ON: begin
                        x_out       <= x_grid;
                        y_out       <= y_grid;
                        polarity    <= 1'b1;
                        timestamp   <= full_ts;
                        event_valid <= 1'b1;
                    end

                    default: event_valid <= 1'b0;
                endcase
            end
        end
    end

endmodule
