// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

module input_fifo #(
    parameter int FIFO_DEPTH = 256,
    parameter int DATA_WIDTH = 32
)(  input  logic                  clk_i,
    input  logic                  reset_i,
    input  logic [DATA_WIDTH-1:0] data_i,
    input  logic                  ready_i,
    input  logic                  valid_i,
    output logic                  ready_o,
    output logic                  valid_o,
    output logic [DATA_WIDTH-1:0] data_o,
    output logic [3:0]            in_fifo_dbg // debug bus
);

    localparam int FIFO_DEPTH_LOG2 = $clog2(FIFO_DEPTH);

    localparam int depth_p = (1 << FIFO_DEPTH_LOG2);

    logic [FIFO_DEPTH_LOG2-1:0] wr_ptr;
    logic [FIFO_DEPTH_LOG2-1:0] rd_ptr;
    logic [FIFO_DEPTH_LOG2:0]   tail_count;

    logic [DATA_WIDTH-1:0] out_data_r;
    logic                  out_valid_r;

    logic [DATA_WIDTH-1:0] ram_rd_data;
    logic                  rd_pending;
    // A pop collided with a simultaneous write — SRAM read was suppressed this
    // cycle and must be re-issued the next cycle when write_to_ram is deasserted.
    // rd_ptr and tail_count are NOT updated until the read actually fires.
    logic                  read_deferred;

    logic [FIFO_DEPTH_LOG2:0] total_count;
    logic push, pop;
    logic bypass_to_out;
    logic write_to_ram;
    logic issue_ram_read;

    assign total_count = tail_count + out_valid_r + rd_pending;
    assign ready_o     = (total_count < depth_p);
    assign valid_o     = out_valid_r;
    assign data_o      = out_data_r;

    assign push = valid_i & ready_o;
    assign pop  = out_valid_r & ready_i;

    // Directly refill output register on an empty tail instead of duplicating
    // that element into RAM.
    assign bypass_to_out = push & (
        (!out_valid_r && !rd_pending && (tail_count == 0)) ||
        (pop && (tail_count == 0))
    );

    assign write_to_ram = push & ~bypass_to_out;

    // GF180 SRAM macro constraint: one physical address bus — simultaneous R+W
    // to different addresses in synthesis causes the write to win and the read
    // to be silently lost.  Suppress the SRAM read whenever write_to_ram is
    // active; read_deferred re-fires it on the next cycle when the bus is free.
    assign issue_ram_read = ~write_to_ram & (
        (pop & (tail_count != 0)) | read_deferred
    );

    gf180_sram_1r1w #(
        .width_p(DATA_WIDTH),
        .depth_p(depth_p)
    ) u_fifo_mem (
        .clk_i      (clk_i),
        .reset_i    (reset_i),
        .wr_valid_i (write_to_ram),
        .wr_data_i  (data_i),
        .wr_addr_i  (wr_ptr),
        .rd_valid_i (issue_ram_read),
        .rd_addr_i  (rd_ptr),
        .rd_data_o  (ram_rd_data)
    );

    logic [FIFO_DEPTH_LOG2-1:0] wr_ptr_n;
    logic [FIFO_DEPTH_LOG2-1:0] rd_ptr_n;
    logic [FIFO_DEPTH_LOG2:0]   tail_count_n;
    logic [DATA_WIDTH-1:0]      out_data_n;
    logic                       out_valid_n;
    logic                       rd_pending_n;
    logic                       read_deferred_n;

    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            wr_ptr        <= '0;
            rd_ptr        <= '0;
            tail_count    <= '0;
            out_data_r    <= '0;
            out_valid_r   <= 1'b0;
            rd_pending    <= 1'b0;
            read_deferred <= 1'b0;
        end else begin

            wr_ptr_n        = wr_ptr;
            rd_ptr_n        = rd_ptr;
            tail_count_n    = tail_count;
            out_data_n      = out_data_r;
            out_valid_n     = out_valid_r;
            rd_pending_n    = rd_pending;
            read_deferred_n = read_deferred;

            // Complete a previous SRAM read.
            if (rd_pending) begin
                out_data_n   = ram_rd_data;
                out_valid_n  = 1'b1;
                rd_pending_n = 1'b0;
            end

            // Deferred read: the SRAM is now free — advance the pointer and
            // issue the read that was suppressed by a write collision.
            if (read_deferred && !write_to_ram) begin
                rd_pending_n    = 1'b1;
                rd_ptr_n        = rd_ptr_n + 1'b1;
                tail_count_n    = tail_count_n - 1'b1;
                read_deferred_n = 1'b0;
            end

            if (pop) begin
                if (tail_count != 0) begin
                    out_valid_n = 1'b0;
                    if (!write_to_ram) begin
                        // SRAM is free — issue the read immediately.
                        rd_pending_n    = 1'b1;
                        rd_ptr_n        = rd_ptr_n + 1'b1;
                        tail_count_n    = tail_count_n - 1'b1;
                    end else begin
                        // Write is using the SRAM — defer the read.
                        // rd_ptr and tail_count are held until the read fires.
                        read_deferred_n = 1'b1;
                    end
                end else if (push) begin
                    out_data_n  = data_i;
                    out_valid_n = 1'b1;
                end else begin
                    out_valid_n = 1'b0;
                end
            end else if (!out_valid_r && push && !rd_pending && (tail_count == 0)) begin
                out_data_n  = data_i;
                out_valid_n = 1'b1;
            end

            if (write_to_ram) begin
                wr_ptr_n     = wr_ptr_n + 1'b1;
                tail_count_n = tail_count_n + 1'b1;
            end

            wr_ptr        <= wr_ptr_n;
            rd_ptr        <= rd_ptr_n;
            tail_count    <= tail_count_n;
            out_data_r    <= out_data_n;
            out_valid_r   <= out_valid_n;
            rd_pending    <= rd_pending_n;
            read_deferred <= read_deferred_n;
        end
    end

    //debug bus connections
    assign in_fifo_dbg = {valid_o, ready_o, valid_i, ready_i};

endmodule
