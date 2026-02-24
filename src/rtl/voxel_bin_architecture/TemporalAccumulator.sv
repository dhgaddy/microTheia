// Dual sliding-window accumulator for centroid computation.
// At each half-window boundary the phase toggles; at full-window end: early←late, late←0.

module TemporalAccumulator #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter HALF_WINDOW_MS  = 200,
    parameter CYCLES_PER_HALF = 2_400_000,
    parameter TIMER_BITS      = 22,
    parameter GRID_BITS       = 4,
    parameter ACC_SUM_BITS    = 18,
    parameter ACC_COUNT_BITS  = 12
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        event_valid,
    input  logic signed [GRID_BITS:0] event_x,
    input  logic signed [GRID_BITS:0] event_y,
    input  logic        event_polarity,
    output logic        window_phase,
    output logic        window_complete,
    output logic signed [ACC_SUM_BITS-1:0] early_sum_x,
    output logic signed [ACC_SUM_BITS-1:0] early_sum_y,
    output logic [ACC_COUNT_BITS-1:0]      early_count,
    output logic signed [ACC_SUM_BITS-1:0] late_sum_x,
    output logic signed [ACC_SUM_BITS-1:0] late_sum_y,
    output logic [ACC_COUNT_BITS-1:0]      late_count,
    output logic                           accum_valid,
    output logic [7:0]  debug_event_count
);

    logic [TIMER_BITS-1:0] timer;
    logic                  phase_advance;

    always_ff @(posedge clk) begin
        if (rst) begin
            timer           <= '0;
            window_phase    <= 1'b0;
            phase_advance   <= 1'b0;
            window_complete <= 1'b0;
        end else begin
            phase_advance   <= 1'b0;
            window_complete <= 1'b0;

            if (timer >= CYCLES_PER_HALF - 1) begin
                timer         <= '0;
                phase_advance <= 1'b1;

                if (window_phase == 1'b1) begin
                    window_complete <= 1'b1;
                    window_phase    <= 1'b0;
                end else begin
                    window_phase <= 1'b1;
                end
            end else begin
                timer <= timer + 1'b1;
            end
        end
    end

    logic signed [ACC_SUM_BITS-1:0] live_early_sum_x, live_early_sum_y;
    logic [ACC_COUNT_BITS-1:0]      live_early_count;
    logic signed [ACC_SUM_BITS-1:0] live_early_sum_x_on, live_early_sum_y_on;
    logic [ACC_COUNT_BITS-1:0]      live_early_count_on;
    logic signed [ACC_SUM_BITS-1:0] live_early_sum_x_off, live_early_sum_y_off;
    logic [ACC_COUNT_BITS-1:0]      live_early_count_off;
    logic signed [ACC_SUM_BITS-1:0] live_late_sum_x, live_late_sum_y;
    logic [ACC_COUNT_BITS-1:0]      live_late_count;
    logic signed [ACC_SUM_BITS-1:0] live_late_sum_x_on, live_late_sum_y_on;
    logic [ACC_COUNT_BITS-1:0]      live_late_count_on;
    logic signed [ACC_SUM_BITS-1:0] live_late_sum_x_off, live_late_sum_y_off;
    logic [ACC_COUNT_BITS-1:0]      live_late_count_off;

    logic [7:0] event_cnt;
    assign debug_event_count = event_cnt;

    // Sign-extend grid position to accumulator width
    wire signed [ACC_SUM_BITS-1:0] ext_x = {{(ACC_SUM_BITS-GRID_BITS-1){event_x[GRID_BITS]}}, event_x};
    wire signed [ACC_SUM_BITS-1:0] ext_y = {{(ACC_SUM_BITS-GRID_BITS-1){event_y[GRID_BITS]}}, event_y};

    always_ff @(posedge clk) begin
        if (rst) begin
            live_early_sum_x     <= '0; live_early_sum_y     <= '0; live_early_count     <= '0;
            live_early_sum_x_on  <= '0; live_early_sum_y_on  <= '0; live_early_count_on  <= '0;
            live_early_sum_x_off <= '0; live_early_sum_y_off <= '0; live_early_count_off <= '0;
            live_late_sum_x      <= '0; live_late_sum_y      <= '0; live_late_count      <= '0;
            live_late_sum_x_on   <= '0; live_late_sum_y_on   <= '0; live_late_count_on   <= '0;
            live_late_sum_x_off  <= '0; live_late_sum_y_off  <= '0; live_late_count_off  <= '0;
            event_cnt            <= '0;
        end else begin
            if (window_complete) begin
                live_early_sum_x     <= live_late_sum_x;
                live_early_sum_y     <= live_late_sum_y;
                live_early_count     <= live_late_count;
                live_early_sum_x_on  <= live_late_sum_x_on;
                live_early_sum_y_on  <= live_late_sum_y_on;
                live_early_count_on  <= live_late_count_on;
                live_early_sum_x_off <= live_late_sum_x_off;
                live_early_sum_y_off <= live_late_sum_y_off;
                live_early_count_off <= live_late_count_off;
                live_late_sum_x      <= '0;
                live_late_sum_y      <= '0;
                live_late_count      <= '0;
                live_late_sum_x_on   <= '0;
                live_late_sum_y_on   <= '0;
                live_late_count_on   <= '0;
                live_late_sum_x_off  <= '0;
                live_late_sum_y_off  <= '0;
                live_late_count_off  <= '0;
                event_cnt <= '0;
            end else if (event_valid) begin
                if (event_cnt < 8'hFF) event_cnt <= event_cnt + 1'b1;

                if (window_phase == 1'b0) begin
                    live_early_sum_x <= live_early_sum_x + ext_x;
                    live_early_sum_y <= live_early_sum_y + ext_y;
                    live_early_count <= live_early_count + 1'b1;

                    if (event_polarity) begin
                        live_early_sum_x_on  <= live_early_sum_x_on + ext_x;
                        live_early_sum_y_on  <= live_early_sum_y_on + ext_y;
                        live_early_count_on  <= live_early_count_on + 1'b1;
                    end else begin
                        live_early_sum_x_off <= live_early_sum_x_off + ext_x;
                        live_early_sum_y_off <= live_early_sum_y_off + ext_y;
                        live_early_count_off <= live_early_count_off + 1'b1;
                    end
                end else begin
                    live_late_sum_x <= live_late_sum_x + ext_x;
                    live_late_sum_y <= live_late_sum_y + ext_y;
                    live_late_count <= live_late_count + 1'b1;

                    if (event_polarity) begin
                        live_late_sum_x_on  <= live_late_sum_x_on + ext_x;
                        live_late_sum_y_on  <= live_late_sum_y_on + ext_y;
                        live_late_count_on  <= live_late_count_on + 1'b1;
                    end else begin
                        live_late_sum_x_off <= live_late_sum_x_off + ext_x;
                        live_late_sum_y_off <= live_late_sum_y_off + ext_y;
                        live_late_count_off <= live_late_count_off + 1'b1;
                    end
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            early_sum_x <= '0; early_sum_y <= '0; early_count <= '0;
            late_sum_x  <= '0; late_sum_y  <= '0; late_count  <= '0;
            accum_valid <= 1'b0;
        end else begin
            accum_valid <= 1'b0;

            if (window_complete) begin
                early_sum_x <= live_early_sum_x;
                early_sum_y <= live_early_sum_y;
                early_count <= live_early_count;
                late_sum_x  <= live_late_sum_x;
                late_sum_y  <= live_late_sum_y;
                late_count  <= live_late_count;
                accum_valid <= 1'b1;
            end
        end
    end

endmodule
