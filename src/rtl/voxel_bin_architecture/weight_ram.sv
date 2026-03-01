`timescale 1ns/1ps

// Weight RAM for one gesture class in the voxel-bin architecture.
// Address = bin * (GRID_SIZE*GRID_SIZE) + spatial_addr.
// Implemented as a synchronous single-port RAM so it maps to iCE40
// block RAMs. Weights are pre-initialised but can be updated through
// the write port if required.

module weight_ram #(
    parameter CLASS_IDX   = 0,
    parameter NUM_CELLS   = 1024,
    parameter GRID_SIZE   = 16,
    parameter WEIGHT_BITS = 8
)(
    input  logic                          clk,
    input  logic                          rst,
    // Single synchronous read/write port
    input  logic                          we,
    input  logic [$clog2(NUM_CELLS)-1:0]  cell_addr,
    input  logic signed [WEIGHT_BITS-1:0] din,
    output logic signed [WEIGHT_BITS-1:0] dout
);

    // Synchronous single-port RAM (infers block RAM on iCE40)
    (* ram_style = "block" *)
    logic signed [WEIGHT_BITS-1:0] ram [0:NUM_CELLS-1];

    integer i, cx, cy;
    integer cells_per_bin;
    integer spatial_addr;
    integer half;
    integer raw_val;
    integer bin_idx;
    integer temp_phase;
    integer wdist;

    // Pre-initialise weights at configuration time.
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
                        wdist   = half - cy;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist   = cy - half + 1;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                1: begin // DOWN: destination = bottom (cy >= half)
                    if (cy >= half) begin
                        wdist   = cy - half + 1;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist   = half - cy;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                2: begin // LEFT: destination = left (cx < half)
                    if (cx < half) begin
                        wdist   = half - cx;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist   = cx - half + 1;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                3: begin // RIGHT: destination = right (cx >= half)
                    if (cx >= half) begin
                        wdist   = cx - half + 1;
                        raw_val = temp_phase * wdist;
                    end else begin
                        wdist   = half - cx;
                        raw_val = -(temp_phase * wdist);
                    end
                end
                default: raw_val = 0;
            endcase

            if (raw_val > 127)
                ram[i] = 8'sd127;
            else if (raw_val < -128)
                ram[i] = -8'sd128;
            else
                ram[i] = WEIGHT_BITS'(raw_val);
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            dout <= '0;
        end else begin
            if (we)
                ram[cell_addr] <= din;
            dout <= ram[cell_addr];
        end
    end

endmodule

