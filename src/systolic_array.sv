module systolic_array #(
    parameter int N = 16,
    parameter int DATA_BIT_SIZE = 16,
    parameter int PRODUCT_BIT_SIZE = 2 * DATA_BIT_SIZE,
    parameter int ACC_BIT_SIZE = PRODUCT_BIT_SIZE + $clog2(N),
    parameter int WAVE_CYCLES = 2 * N - 1,
    parameter int TOTAL_CYCLES = 3 * N - 1,
    parameter int T_BITS = $clog2(TOTAL_CYCLES + 1)
)(
    input  logic clk,
    input  logic reset,
    input  logic start,

    // Input matrices (flattened)
    input  logic [N*N*DATA_BIT_SIZE-1:0] A_matrix_flat,
    input  logic [N*N*DATA_BIT_SIZE-1:0] B_matrix_flat,
    // Output matrix (flattened)
    output logic [N*N*ACC_BIT_SIZE-1:0]  Out_matrix_flat,

    // Status outputs
    output logic busy,
    output logic done
);

    logic [DATA_BIT_SIZE-1:0] A_matrix [0:N-1][0:N-1];
    logic [DATA_BIT_SIZE-1:0] B_matrix [0:N-1][0:N-1];

    // Internal pipes for A and B
    logic [DATA_BIT_SIZE-1:0] a_pipe [0:N-1][0:N-1];
    logic [DATA_BIT_SIZE-1:0] b_pipe [0:N-1][0:N-1];

    // Accumulators
    logic [ACC_BIT_SIZE-1:0] acc [0:N-1][0:N-1];

    // Current diagonal inputs for this cycle
    logic [DATA_BIT_SIZE-1:0] a_in [0:N-1];
    logic [DATA_BIT_SIZE-1:0] b_in [0:N-1];

    // PE combinatorial inputs
    logic [DATA_BIT_SIZE-1:0]    pe_a    [0:N-1][0:N-1];
    logic [DATA_BIT_SIZE-1:0]    pe_b    [0:N-1][0:N-1];
    logic [PRODUCT_BIT_SIZE-1:0] pe_prod [0:N-1][0:N-1];
    logic [PRODUCT_BIT_SIZE-1:0] pe_prod_r [0:N-1][0:N-1];

    // Sequencing
    logic [T_BITS-1:0] t;
    logic running;

    always_comb begin
        for (int i = 0; i < N; i++) begin
            for (int j = 0; j < N; j++) begin
                A_matrix[i][j] = A_matrix_flat[(i*N + j) * DATA_BIT_SIZE +: DATA_BIT_SIZE];
                B_matrix[i][j] = B_matrix_flat[(i*N + j) * DATA_BIT_SIZE +: DATA_BIT_SIZE];
            end
        end
    end

    // Generate the current diagonal slice from t
    always_comb begin
        // Default zero padding
        for (int r = 0; r < N; r = r + 1)
            a_in[r] = '0;

        for (int c = 0; c < N; c = c + 1)
            b_in[c] = '0;

        // Only inject real data during the active wave phase
        if (running && (t < WAVE_CYCLES)) begin
            for (int r = 0; r < N; r = r + 1) begin
                int k;
                k = int'(t) - r;
                if ((k >= 0) && (k < N))
                    a_in[r] = A_matrix[r][k];
            end

            for (int c = 0; c < N; c = c + 1) begin
                int k;
                k = int'(t) - c;
                if ((k >= 0) && (k < N))
                    b_in[c] = B_matrix[k][c];
            end
        end
    end

    // Combinatorial PE input mux: left-edge gets a_in, others get left-neighbour pipe
    always_comb begin
        for (int r = 0; r < N; r = r + 1) begin
            for (int c = 0; c < N; c = c + 1) begin
                pe_a[r][c]    = (c == 0) ? a_in[r]       : a_pipe[r][c-1];
                pe_b[r][c]    = (r == 0) ? b_in[c]       : b_pipe[r-1][c];
                pe_prod[r][c] = pe_a[r][c] * pe_b[r][c];
            end
        end
    end

    // Main control + systolic MAC update
    always_ff @(posedge clk) begin
        if (reset) begin
            running <= 1'b0;
            busy    <= 1'b0;
            done    <= 1'b0;
            t       <= '0;

            for (int r = 0; r < N; r = r + 1) begin
                for (int c = 0; c < N; c = c + 1) begin
                    a_pipe[r][c] <= '0;
                    b_pipe[r][c] <= '0;
                    acc[r][c]    <= '0;
                    pe_prod_r[r][c] <= '0;
                end
            end
        end else begin
            done <= 1'b0;

            // Start a fresh multiply
            if (start && !running) begin
                running <= 1'b1;
                busy    <= 1'b1;
                t       <= '0;

                for (int r = 0; r < N; r = r + 1) begin
                    for (int c = 0; c < N; c = c + 1) begin
                        a_pipe[r][c] <= '0;
                        b_pipe[r][c] <= '0;
                        acc[r][c]    <= '0;
                        pe_prod_r[r][c] <= '0;
                    end
                end
            end
            else if (running) begin
                // Pipeline multiplier output before accumulation to reduce critical path.
                for (int r = 0; r < N; r = r + 1) begin
                    for (int c = 0; c < N; c = c + 1) begin
                        a_pipe[r][c] <= pe_a[r][c];
                        b_pipe[r][c] <= pe_b[r][c];
                        pe_prod_r[r][c] <= pe_prod[r][c];
                        acc[r][c] <= acc[r][c] +
                            {{(ACC_BIT_SIZE - PRODUCT_BIT_SIZE){1'b0}}, pe_prod_r[r][c]};
                    end
                end

                // Advance time step
                if (t == TOTAL_CYCLES - 1) begin
                    running <= 1'b0;
                    busy    <= 1'b0;
                    done    <= 1'b1;
                end else begin
                    t <= t + 1'b1;
                end
            end
        end
    end

    // Drive flattened output directly from accumulators
    always_comb begin
        for (int i = 0; i < N; i++) begin
            for (int j = 0; j < N; j++) begin
                Out_matrix_flat[(i*N + j)*ACC_BIT_SIZE +: ACC_BIT_SIZE] = acc[i][j];
            end
        end
    end

endmodule
