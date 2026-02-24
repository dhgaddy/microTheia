// Classifies motion vector into cardinal gesture direction.
// Gates: activity threshold → motion threshold → dominant axis → sign.

module GestureClassifier #(
    parameter ACC_SUM_BITS     = 18,
    parameter ACC_COUNT_BITS   = 12,
    parameter MIN_EVENT_THRESH = 20,
    parameter MOTION_THRESH    = 8
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        trigger,
    input  logic signed [ACC_SUM_BITS-1:0] delta_x,
    input  logic signed [ACC_SUM_BITS-1:0] delta_y,
    input  logic [ACC_SUM_BITS-1:0]        abs_delta_x,
    input  logic [ACC_SUM_BITS-1:0]        abs_delta_y,
    input  logic [ACC_COUNT_BITS-1:0]      total_events,
    output logic [1:0]  gesture,
    output logic        valid,
    output logic        pass
);

    localparam logic [1:0] GESTURE_UP    = 2'b00;
    localparam logic [1:0] GESTURE_DOWN  = 2'b01;
    localparam logic [1:0] GESTURE_LEFT  = 2'b10;
    localparam logic [1:0] GESTURE_RIGHT = 2'b11;

    always_ff @(posedge clk) begin
        if (rst) begin
            gesture <= '0;
            valid   <= 1'b0;
            pass    <= 1'b0;
        end else begin
            valid <= trigger;
            pass  <= 1'b0;

            if (trigger) begin
                if (total_events >= MIN_EVENT_THRESH) begin
                    if ((abs_delta_x >= MOTION_THRESH) || (abs_delta_y >= MOTION_THRESH)) begin
                        pass <= 1'b1;
                        if (abs_delta_y > abs_delta_x)
                            gesture <= (delta_y > 0) ? GESTURE_DOWN : GESTURE_UP;
                        else
                            gesture <= (delta_x > 0) ? GESTURE_RIGHT : GESTURE_LEFT;
                    end
                end
            end
        end
    end

endmodule
