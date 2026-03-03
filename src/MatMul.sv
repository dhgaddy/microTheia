module MatMul #(
    parameter int N = 8,
    parameter int DATA_BIT_SIZE = 16,
    localparam int PRODUCT_BIT_SIZE = 2 * DATA_BIT_SIZE,
    localparam int ACC_BIT_SIZE = PRODUCT_BIT_SIZE + $clog2(N)
)(
    input  logic clk,
    input  logic reset,
    input  logic start,

    // A inputs (one per row)
    input  logic [DATA_BIT_SIZE-1:0] A_cell_1,
    input  logic [DATA_BIT_SIZE-1:0] A_cell_2,
    input  logic [DATA_BIT_SIZE-1:0] A_cell_3,
    input  logic [DATA_BIT_SIZE-1:0] A_cell_4,
    input  logic [DATA_BIT_SIZE-1:0] A_cell_5,
    input  logic [DATA_BIT_SIZE-1:0] A_cell_6,
    input  logic [DATA_BIT_SIZE-1:0] A_cell_7,
    input  logic [DATA_BIT_SIZE-1:0] A_cell_8,

    // B inputs (one per column)
    input  logic [DATA_BIT_SIZE-1:0] B_cell_1,
    input  logic [DATA_BIT_SIZE-1:0] B_cell_2,
    input  logic [DATA_BIT_SIZE-1:0] B_cell_3,
    input  logic [DATA_BIT_SIZE-1:0] B_cell_4,
    input  logic [DATA_BIT_SIZE-1:0] B_cell_5,
    input  logic [DATA_BIT_SIZE-1:0] B_cell_6,
    input  logic [DATA_BIT_SIZE-1:0] B_cell_7,
    input  logic [DATA_BIT_SIZE-1:0] B_cell_8,

    // 8x8 = 64 outputs
    output logic [ACC_BIT_SIZE-1:0] out1,  out2,  out3,  out4,  out5,  out6,  out7,  out8,
    output logic [ACC_BIT_SIZE-1:0] out9,  out10, out11, out12, out13, out14, out15, out16,
    output logic [ACC_BIT_SIZE-1:0] out17, out18, out19, out20, out21, out22, out23, out24,
    output logic [ACC_BIT_SIZE-1:0] out25, out26, out27, out28, out29, out30, out31, out32,
    output logic [ACC_BIT_SIZE-1:0] out33, out34, out35, out36, out37, out38, out39, out40,
    output logic [ACC_BIT_SIZE-1:0] out41, out42, out43, out44, out45, out46, out47, out48,
    output logic [ACC_BIT_SIZE-1:0] out49, out50, out51, out52, out53, out54, out55, out56,
    output logic [ACC_BIT_SIZE-1:0] out57, out58, out59, out60, out61, out62, out63, out64
);

    // Internal pipes for A and B as they move through the array
    logic [DATA_BIT_SIZE-1:0] a_pipe [0:N-1][0:N-1];
    logic [DATA_BIT_SIZE-1:0] b_pipe [0:N-1][0:N-1];

    // Accumulators for each PE
    logic [ACC_BIT_SIZE-1:0] acc [0:N-1][0:N-1];

    // Convenient input vectors
    logic [DATA_BIT_SIZE-1:0] a_in [0:N-1];
    logic [DATA_BIT_SIZE-1:0] b_in [0:N-1];

    integer r, c;

    // Map scalar ports into arrays
    always_comb begin
        a_in[0] = A_cell_1;
        a_in[1] = A_cell_2;
        a_in[2] = A_cell_3;
        a_in[3] = A_cell_4;
        a_in[4] = A_cell_5;
        a_in[5] = A_cell_6;
        a_in[6] = A_cell_7;
        a_in[7] = A_cell_8;

        b_in[0] = B_cell_1;
        b_in[1] = B_cell_2;
        b_in[2] = B_cell_3;
        b_in[3] = B_cell_4;
        b_in[4] = B_cell_5;
        b_in[5] = B_cell_6;
        b_in[6] = B_cell_7;
        b_in[7] = B_cell_8;
    end

    // Systolic array behavior
    always_ff @(posedge clk) begin
        if (reset || start) begin
            for (r = 0; r < N; r = r + 1) begin
                for (c = 0; c < N; c = c + 1) begin
                    a_pipe[r][c] <= '0;
                    b_pipe[r][c] <= '0;
                    acc[r][c]    <= '0;
                end
            end
        end else begin
            for (r = 0; r < N; r = r + 1) begin
                for (c = 0; c < N; c = c + 1) begin
                    logic [DATA_BIT_SIZE-1:0] a_val;
                    logic [DATA_BIT_SIZE-1:0] b_val;
                    logic [PRODUCT_BIT_SIZE-1:0] product;

                    // A enters from the left, then moves right
                    if (c == 0)
                        a_val = a_in[r];
                    else
                        a_val = a_pipe[r][c-1];

                    // B enters from the top, then moves down
                    if (r == 0)
                        b_val = b_in[c];
                    else
                        b_val = b_pipe[r-1][c];

                    // Register propagated values
                    a_pipe[r][c] <= a_val;
                    b_pipe[r][c] <= b_val;

                    // Multiply-accumulate
                    product = a_val * b_val;
                    acc[r][c] <= acc[r][c] + {{(ACC_BIT_SIZE-PRODUCT_BIT_SIZE){1'b0}}, product};
                end
            end
        end
    end

    // Flatten 2D accumulator matrix to your 64 outputs
    always_comb begin
        out1  = acc[0][0]; out2  = acc[0][1]; out3  = acc[0][2]; out4  = acc[0][3];
        out5  = acc[0][4]; out6  = acc[0][5]; out7  = acc[0][6]; out8  = acc[0][7];

        out9  = acc[1][0]; out10 = acc[1][1]; out11 = acc[1][2]; out12 = acc[1][3];
        out13 = acc[1][4]; out14 = acc[1][5]; out15 = acc[1][6]; out16 = acc[1][7];

        out17 = acc[2][0]; out18 = acc[2][1]; out19 = acc[2][2]; out20 = acc[2][3];
        out21 = acc[2][4]; out22 = acc[2][5]; out23 = acc[2][6]; out24 = acc[2][7];

        out25 = acc[3][0]; out26 = acc[3][1]; out27 = acc[3][2]; out28 = acc[3][3];
        out29 = acc[3][4]; out30 = acc[3][5]; out31 = acc[3][6]; out32 = acc[3][7];

        out33 = acc[4][0]; out34 = acc[4][1]; out35 = acc[4][2]; out36 = acc[4][3];
        out37 = acc[4][4]; out38 = acc[4][5]; out39 = acc[4][6]; out40 = acc[4][7];

        out41 = acc[5][0]; out42 = acc[5][1]; out43 = acc[5][2]; out44 = acc[5][3];
        out45 = acc[5][4]; out46 = acc[5][5]; out47 = acc[5][6]; out48 = acc[5][7];

        out49 = acc[6][0]; out50 = acc[6][1]; out51 = acc[6][2]; out52 = acc[6][3];
        out53 = acc[6][4]; out54 = acc[6][5]; out55 = acc[6][6]; out56 = acc[6][7];

        out57 = acc[7][0]; out58 = acc[7][1]; out59 = acc[7][2]; out60 = acc[7][3];
        out61 = acc[7][4]; out62 = acc[7][5]; out63 = acc[7][6]; out64 = acc[7][7];
    end

endmodule