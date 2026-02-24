`timescale 1ns/1ps

// Weight ROM for one gesture class. Address = bin * 256 + spatial_addr.
// Weight = temp_phase * spat_phase * wdist, where:
//   temp_phase = +1 for newer bins (2,3), -1 for older bins (0,1)
//   spat_phase = +1 for destination half, -1 for source half
//   wdist = distance from grid centre boundary (1..8)
// Score is maximised when events are in the destination half in newer bins
// and in the source half in older bins.

module WeightROM #(
    parameter CLASS_IDX   = 0,
    parameter NUM_CELLS   = 1024,
    parameter GRID_SIZE   = 16,
    parameter WEIGHT_BITS = 8
)(
    input  logic                          clk,
    input  logic [$clog2(NUM_CELLS)-1:0]  cell_addr,
    output logic signed [WEIGHT_BITS-1:0] dout
);

    logic signed [WEIGHT_BITS-1:0] rom [0:NUM_CELLS-1];

    integer i, cx, cy;
    integer cells_per_bin;
    integer spatial_addr;
    integer half;
    integer raw_val;
    integer bin_idx;
    integer temp_phase;
    integer wdist;

    initial begin
        cells_per_bin = GRID_SIZE * GRID_SIZE;
        half          = GRID_SIZE / 2;

        for (i = 0; i < NUM_CELLS; i = i + 1) begin
            spatial_addr = i % cells_per_bin;
            bin_idx      = i / cells_per_bin;
            cy           = spatial_addr / GRID_SIZE;
            cx           = spatial_addr % GRID_SIZE;

            if (bin_idx >= 2)
                temp_phase = 1;
            else
                temp_phase = -1;

            raw_val = 0;

            case (CLASS_IDX)
                0: begin // UP: destination = top (cy < half)
                    if (cy < half) begin
                        wdist = half - cy;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist = cy - half + 1;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                1: begin // DOWN: destination = bottom (cy >= half)
                    if (cy >= half) begin
                        wdist = cy - half + 1;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist = half - cy;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                2: begin // LEFT: destination = left (cx < half)
                    if (cx < half) begin
                        wdist = half - cx;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist = cx - half + 1;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                3: begin // RIGHT: destination = right (cx >= half)
                    if (cx >= half) begin
                        wdist = cx - half + 1;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist = half - cx;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                default: raw_val = 0;
            endcase

            if (raw_val > 127)
                rom[i] = 8'sd127;
            else if (raw_val < -128)
                rom[i] = -8'sd128;
            else
                rom[i] = WEIGHT_BITS'(raw_val);
        end
    end

    always_ff @(posedge clk) begin
        dout <= rom[cell_addr];
    end

endmodule
