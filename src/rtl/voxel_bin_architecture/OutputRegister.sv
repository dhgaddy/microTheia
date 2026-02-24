// Persistence filter: requires PERSISTENCE_COUNT consecutive matching classifications
// before asserting gesture_valid. Confidence = min(15, dominant_magnitude >> 4).

module OutputRegister #(
    parameter ACC_SUM_BITS      = 18,
    parameter PERSISTENCE_COUNT = 2
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [1:0]  class_gesture,
    input  logic        class_valid,
    input  logic        class_pass,
    input  logic [ACC_SUM_BITS-1:0] abs_delta_x,
    input  logic [ACC_SUM_BITS-1:0] abs_delta_y,
    output logic [1:0]  gesture,
    output logic        gesture_valid,
    output logic [3:0]  gesture_confidence,
    output logic [2:0]  debug_state
);

    typedef enum logic [2:0] {
        ST_IDLE      = 3'd0,
        ST_TRACKING  = 3'd1,
        ST_CONFIRMED = 3'd2
    } state_t;

    state_t state;
    assign debug_state = state;

    logic [1:0] last_gesture;
    logic [2:0] match_count;
    logic [ACC_SUM_BITS-1:0] latched_abs_x, latched_abs_y;

    always_ff @(posedge clk) begin
        if (rst) begin
            state              <= ST_IDLE;
            last_gesture       <= '0;
            match_count        <= '0;
            gesture            <= '0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= '0;
            latched_abs_x      <= '0;
            latched_abs_y      <= '0;
        end else begin
            gesture_valid <= 1'b0;

            if (class_valid) begin
                if (class_pass) begin
                    if (class_gesture == last_gesture) begin
                        if (match_count < PERSISTENCE_COUNT)
                            match_count <= match_count + 1'b1;

                        if (match_count >= PERSISTENCE_COUNT - 1) begin
                            state         <= ST_CONFIRMED;
                            gesture       <= class_gesture;
                            gesture_valid <= 1'b1;
                            match_count   <= '0;

                            if (abs_delta_x > abs_delta_y)
                                gesture_confidence <= (abs_delta_x > 255) ? 4'd15 : abs_delta_x[7:4];
                            else
                                gesture_confidence <= (abs_delta_y > 255) ? 4'd15 : abs_delta_y[7:4];
                        end else begin
                            state <= ST_TRACKING;
                        end
                    end else begin
                        last_gesture <= class_gesture;
                        match_count  <= '0;
                        state        <= ST_TRACKING;
                    end

                    latched_abs_x <= abs_delta_x;
                    latched_abs_y <= abs_delta_y;
                end else begin
                    match_count <= '0;
                    state       <= ST_IDLE;
                end
            end
        end
    end

endmodule
