// Shallow FIFO for DVS event buffering. Combinational read output.
// Packed storage: [34:26]=x, [25:17]=y, [16]=polarity, [15:0]=timestamp

module InputFIFO #(
    parameter DEPTH    = 16,
    parameter PTR_BITS = 4
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        push_valid,
    input  logic [8:0]  push_x,
    input  logic [8:0]  push_y,
    input  logic        push_polarity,
    input  logic [15:0] push_ts,
    output logic        push_ready,
    input  logic        pop_req,
    output logic [8:0]  pop_x,
    output logic [8:0]  pop_y,
    output logic        pop_polarity,
    output logic [15:0] pop_ts,
    output logic        empty,
    output logic        full
);

    logic [34:0] fifo_mem [0:DEPTH-1];

    logic [PTR_BITS:0] wr_ptr;
    logic [PTR_BITS:0] rd_ptr;
    logic [PTR_BITS:0] count;

    assign empty      = (count == 0);
    assign full       = (count == DEPTH);
    assign push_ready = !full;

    wire [PTR_BITS-1:0] rd_addr = rd_ptr[PTR_BITS-1:0];
    assign pop_x        = fifo_mem[rd_addr][34:26];
    assign pop_y        = fifo_mem[rd_addr][25:17];
    assign pop_polarity = fifo_mem[rd_addr][16];
    assign pop_ts       = fifo_mem[rd_addr][15:0];

    wire do_push = push_valid && !full;
    wire do_pop  = pop_req && !empty;

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr <= '0;
        end else if (do_push) begin
            fifo_mem[wr_ptr[PTR_BITS-1:0]] <= {push_x, push_y, push_polarity, push_ts};
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            rd_ptr <= '0;
        else if (do_pop)
            rd_ptr <= rd_ptr + 1'b1;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            count <= '0;
        end else begin
            case ({do_push, do_pop})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
