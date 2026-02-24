`timescale 1ns/1ps

// Single-class weight ROM. Instantiate four (one per gesture class) for parallel inference.
// Weights: positive zone = +clamp(dist*6, 10, 120), negative zone = -clamp(dist*4, 10, 80).

module weight_rom #(
    parameter CLASS_IDX   = 0,
    parameter NUM_CLASSES = 4,
    parameter NUM_CELLS   = 256,
    parameter GRID_SIZE   = 16,
    parameter WEIGHT_BITS = 8
)(
    input  logic                          clk,
    input  logic                          rst,
    input  logic [$clog2(NUM_CELLS)-1:0]  cell_addr,
    output logic signed [WEIGHT_BITS-1:0] dout
);

    logic signed [WEIGHT_BITS-1:0] rom [0:NUM_CELLS-1];

    integer cx, cy, addr, centre, dist_from_edge;
    integer raw_val;

    initial begin
        centre = GRID_SIZE / 2;

        for (cy = 0; cy < GRID_SIZE; cy = cy + 1) begin
            for (cx = 0; cx < GRID_SIZE; cx = cx + 1) begin
                addr = cy * GRID_SIZE + cx;

                case (CLASS_IDX)
                    0: begin // UP
                        if (cy < centre) begin
                            dist_from_edge = centre - cy;
                            raw_val = dist_from_edge * 6;
                        end else begin
                            dist_from_edge = cy - centre + 1;
                            raw_val = -(dist_from_edge * 4);
                        end
                    end
                    1: begin // DOWN
                        if (cy >= centre) begin
                            dist_from_edge = cy - centre + 1;
                            raw_val = dist_from_edge * 6;
                        end else begin
                            dist_from_edge = centre - cy;
                            raw_val = -(dist_from_edge * 4);
                        end
                    end
                    2: begin // LEFT
                        if (cx < centre) begin
                            dist_from_edge = centre - cx;
                            raw_val = dist_from_edge * 6;
                        end else begin
                            dist_from_edge = cx - centre + 1;
                            raw_val = -(dist_from_edge * 4);
                        end
                    end
                    3: begin // RIGHT
                        if (cx >= centre) begin
                            dist_from_edge = cx - centre + 1;
                            raw_val = dist_from_edge * 6;
                        end else begin
                            dist_from_edge = centre - cx;
                            raw_val = -(dist_from_edge * 4);
                        end
                    end
                    default: raw_val = 0;
                endcase

                if      (raw_val >  127) rom[addr] = 8'sd127;
                else if (raw_val < -128) rom[addr] = -8'sd128;
                else                    rom[addr] = WEIGHT_BITS'(raw_val);
            end
        end
    end

    always_ff @(posedge clk) begin
        dout <= rom[cell_addr];
    end

endmodule
