module MatMul #(
    parameter  int N              = 16,
    parameter  int DATA_BIT_SIZE  = 16,
    localparam int PRODUCT_BIT_SIZE = 2 * DATA_BIT_SIZE,
    localparam int ACC_BIT_SIZE     = PRODUCT_BIT_SIZE + $clog2(N),
    localparam int WAVE_CYCLES      = 2 * N - 1,
    localparam int TOTAL_CYCLES     = 3 * N - 2,
    localparam int T_BITS           = $clog2(TOTAL_CYCLES + 1)
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
    logic [ACC_BIT_SIZE-1:0]  Out_matrix [0:N-1][0:N-1];

    // Internal pipes for A and B
    logic [DATA_BIT_SIZE-1:0] a_pipe [0:N-1][0:N-1];
    logic [DATA_BIT_SIZE-1:0] b_pipe [0:N-1][0:N-1];

    // Accumulators
    logic [ACC_BIT_SIZE-1:0] acc [0:N-1][0:N-1];

    // Current diagonal inputs for this cycle
    logic [DATA_BIT_SIZE-1:0] a_in [0:N-1];
    logic [DATA_BIT_SIZE-1:0] b_in [0:N-1];

    // Sequencing
    logic [T_BITS-1:0] t;
    logic running;

    integer r, c;
    integer k;

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
        for (r = 0; r < N; r = r + 1)
            a_in[r] = '0;

        for (c = 0; c < N; c = c + 1)
            b_in[c] = '0;

        // Only inject real data during the active wave phase
        if (running && (t < WAVE_CYCLES)) begin
            for (r = 0; r < N; r = r + 1) begin
                k = t - r;
                if ((k >= 0) && (k < N))
                    a_in[r] = A_matrix[r][k];
            end

            for (c = 0; c < N; c = c + 1) begin
                k = t - c;
                if ((k >= 0) && (k < N))
                    b_in[c] = B_matrix[k][c];
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

            for (r = 0; r < N; r = r + 1) begin
                for (c = 0; c < N; c = c + 1) begin
                    a_pipe[r][c] <= '0;
                    b_pipe[r][c] <= '0;
                    acc[r][c]    <= '0;
                end
            end
        end else begin
            done <= 1'b0;

            // Start a fresh multiply
            if (start && !running) begin
                running <= 1'b1;
                busy    <= 1'b1;
                t       <= '0;

                for (r = 0; r < N; r = r + 1) begin
                    for (c = 0; c < N; c = c + 1) begin
                        a_pipe[r][c] <= '0;
                        b_pipe[r][c] <= '0;
                        acc[r][c]    <= '0;
                    end
                end
            end
            else if (running) begin
                // Update all PEs once per cycle
                for (r = 0; r < N; r = r + 1) begin
                    for (c = 0; c < N; c = c + 1) begin
                        logic [DATA_BIT_SIZE-1:0] a_val;
                        logic [DATA_BIT_SIZE-1:0] b_val;
                        logic [PRODUCT_BIT_SIZE-1:0] product;

                        // A enters from left, then moves right
                        if (c == 0)
                            a_val = a_in[r];
                        else
                            a_val = a_pipe[r][c-1];

                        // B enters from top, then moves down
                        if (r == 0)
                            b_val = b_in[c];
                        else
                            b_val = b_pipe[r-1][c];

                        // Register propagated values
                        a_pipe[r][c] <= a_val;
                        b_pipe[r][c] <= b_val;

                        // Multiply-accumulate
                        product = a_val * b_val;
                        acc[r][c] <= acc[r][c] + {{(ACC_BIT_SIZE - PRODUCT_BIT_SIZE){1'b0}}, product};
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

    // Drive output matrix
    always_comb begin
        for (r = 0; r < N; r = r + 1) begin
            for (c = 0; c < N; c = c + 1) begin
                Out_matrix[r][c] = acc[r][c];
            end
        end
    end

    always_comb begin
        for (int i = 0; i < N; i++) begin
            for (int j = 0; j < N; j++) begin
                Out_matrix_flat[(i*N + j)*ACC_BIT_SIZE +: ACC_BIT_SIZE] = Out_matrix[i][j];
            end
        end
    end

endmodule