`timescale 1ns/1ps

// Synchronous ring-buffer FIFO; infers iCE40 Block RAM

module input_fifo #(
    parameter DEPTH      = 256,
    parameter PTR_BITS   = 8,
    parameter DATA_WIDTH = 32
)(
    input  logic                    clk,
    input  logic                    rst,
    input  logic                    wr_en,
    input  logic [DATA_WIDTH-1:0]   wr_data,
    input  logic                    rd_en,
    output logic [DATA_WIDTH-1:0]   rd_data,
    output logic                    empty,
    output logic                    full,
    output logic [PTR_BITS:0]       count
);

    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    logic [PTR_BITS:0] wr_ptr;
    logic [PTR_BITS:0] rd_ptr;

    wire [PTR_BITS-1:0] wr_addr = wr_ptr[PTR_BITS-1:0];
    wire [PTR_BITS-1:0] rd_addr = rd_ptr[PTR_BITS-1:0];

    // Full: pointers differ only in MSB; empty: identical
    assign full  = (wr_ptr[PTR_BITS] != rd_ptr[PTR_BITS]) &&
                   (wr_ptr[PTR_BITS-1:0] == rd_ptr[PTR_BITS-1:0]);
    assign empty = (wr_ptr == rd_ptr);
    assign count = wr_ptr - rd_ptr;

    wire do_write = wr_en && !full;

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr <= '0;
        end else if (do_write) begin
            mem[wr_addr] <= wr_data;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    wire do_read = rd_en && !empty;

    always_ff @(posedge clk) begin
        if (rst) begin
            rd_ptr  <= '0;
            rd_data <= '0;
        end else begin
            rd_data <= mem[rd_addr];
            if (do_read)
                rd_ptr <= rd_ptr + 1'b1;
        end
    end

endmodule
