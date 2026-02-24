// Maps 320x320 sensor coords to 16x16 grid: grid = (sensor * 13) >> 8 ≈ sensor / 20
// Output is signed position relative to grid center (cell 8 → position 0).

module SpatialCompressor #(
    parameter SENSOR_RES = 320,
    parameter GRID_SIZE  = 16,
    parameter GRID_BITS  = 4
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        in_valid,
    input  logic [8:0]  in_x,
    input  logic [8:0]  in_y,
    input  logic        in_polarity,
    output logic        in_ready,
    output logic signed [GRID_BITS:0] out_x,
    output logic signed [GRID_BITS:0] out_y,
    output logic        out_polarity,
    output logic        out_valid
);

    localparam integer MULT_FACTOR  = 13;
    localparam integer SHIFT_AMOUNT = 8;
    localparam integer CENTER       = GRID_SIZE / 2;

    wire [12:0] mult_x = in_x * MULT_FACTOR[4:0];
    wire [12:0] mult_y = in_y * MULT_FACTOR[4:0];

    wire [GRID_BITS-1:0] grid_x_raw = mult_x[11:SHIFT_AMOUNT];
    wire [GRID_BITS-1:0] grid_y_raw = mult_y[11:SHIFT_AMOUNT];

    wire [GRID_BITS-1:0] grid_x = (mult_x[12:SHIFT_AMOUNT] >= GRID_SIZE) ?
                                   (GRID_SIZE - 1) : grid_x_raw;
    wire [GRID_BITS-1:0] grid_y = (mult_y[12:SHIFT_AMOUNT] >= GRID_SIZE) ?
                                   (GRID_SIZE - 1) : grid_y_raw;

    wire signed [GRID_BITS:0] pos_x = $signed({1'b0, grid_x}) - $signed(CENTER);
    wire signed [GRID_BITS:0] pos_y = $signed({1'b0, grid_y}) - $signed(CENTER);

    always_ff @(posedge clk) begin
        if (rst) begin
            out_x        <= '0;
            out_y        <= '0;
            out_polarity <= '0;
            out_valid    <= '0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                out_x        <= pos_x;
                out_y        <= pos_y;
                out_polarity <= in_polarity;
            end
        end
    end

    assign in_ready = in_valid;

endmodule
