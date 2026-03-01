`timescale 1ns/1ps

// Sends gesture ASCII strings ("UP\r\n", "DOWN\r\n", etc.) over UART.

module uart_debug #(
    parameter CLK_FREQ_HZ = 12_000_000,
    parameter BAUD_RATE   = 115200
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [1:0]  gesture_class,      // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    input  logic        gesture_valid,
    input  logic [7:0]  gesture_confidence,
    output logic        uart_tx
);

    localparam CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;

    localparam MSG_UP_LEN    = 4;
    localparam MSG_DOWN_LEN  = 6;
    localparam MSG_LEFT_LEN  = 6;
    localparam MSG_RIGHT_LEN = 7;

    logic [7:0] msg_up    [0:3];
    logic [7:0] msg_down  [0:5];
    logic [7:0] msg_left  [0:5];
    logic [7:0] msg_right [0:6];

    initial begin
        msg_up[0]     = "U"; msg_up[1]     = "P";
        msg_up[2]     = 8'h0D; msg_up[3]   = 8'h0A;

        msg_down[0]   = "D"; msg_down[1]   = "O";
        msg_down[2]   = "W"; msg_down[3]   = "N";
        msg_down[4]   = 8'h0D; msg_down[5] = 8'h0A;

        msg_left[0]   = "L"; msg_left[1]   = "E";
        msg_left[2]   = "F"; msg_left[3]   = "T";
        msg_left[4]   = 8'h0D; msg_left[5] = 8'h0A;

        msg_right[0]  = "R"; msg_right[1]  = "I";
        msg_right[2]  = "G"; msg_right[3]  = "H";
        msg_right[4]  = "T"; msg_right[5]  = 8'h0D;
        msg_right[6]  = 8'h0A;
    end

    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_LOAD = 2'd1,
        S_SEND = 2'd2,
        S_NEXT = 2'd3
    } state_t;

    state_t state;

    logic [1:0] current_gesture;
    logic [2:0] byte_idx;
    logic [2:0] msg_length;
    logic [7:0] current_byte;

    logic [7:0] tx_data;
    logic       tx_valid;
    logic       tx_busy;

    uart_tx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_tx (
        .clk   (clk),
        .rst   (rst),
        .data  (tx_data),
        .valid (tx_valid),
        .tx    (uart_tx),
        .busy  (tx_busy)
    );

    function automatic [2:0] get_msg_length(input [1:0] gesture);
        case (gesture)
            2'd0: get_msg_length = MSG_UP_LEN;
            2'd1: get_msg_length = MSG_DOWN_LEN;
            2'd2: get_msg_length = MSG_LEFT_LEN;
            default: get_msg_length = MSG_RIGHT_LEN;
        endcase
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            state           <= S_IDLE;
            current_gesture <= '0;
            byte_idx        <= '0;
            msg_length      <= '0;
            current_byte    <= '0;
            tx_data         <= '0;
            tx_valid        <= 1'b0;
        end else begin
            tx_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (gesture_valid && !tx_busy) begin
                        current_gesture <= gesture_class;
                        msg_length      <= get_msg_length(gesture_class);
                        byte_idx        <= '0;
                        state           <= S_LOAD;
                    end
                end

                S_LOAD: begin
                    case (current_gesture)
                        2'd0: current_byte <= msg_up[byte_idx];
                        2'd1: current_byte <= msg_down[byte_idx];
                        2'd2: current_byte <= msg_left[byte_idx];
                        default: current_byte <= msg_right[byte_idx];
                    endcase
                    state <= S_SEND;
                end

                S_SEND: begin
                    if (!tx_busy) begin
                        tx_data  <= current_byte;
                        tx_valid <= 1'b1;
                        state    <= S_NEXT;
                    end
                end

                S_NEXT: begin
                    if (tx_busy) begin
                        if (byte_idx >= msg_length - 1) begin
                            state <= S_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                            state    <= S_LOAD;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

