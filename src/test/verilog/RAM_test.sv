`timescale 1ns/1ps

`define START_TESTBENCH error_o = 0; pass_o = 0; #1;
`define FINISH_WITH_FAIL error_o = 1; pass_o = 0; #1; $finish();
`define FINISH_WITH_PASS pass_o = 1; error_o = 0; #1; $finish();

module testbench
  (output logic error_o = 1'bx
  ,output logic pass_o  = 1'bx);

  logic [10:0] error;

  // Handshake + data
  logic        reset;
  logic        io_wr_valid_HS_Req, io_wr_valid_HS_Ack;
  logic        io_wr_valid_Data;
  logic        io_wr_data_HS_Req,  io_wr_data_HS_Ack;
  logic [7:0]  io_wr_data_Data;
  logic        io_wr_addr_HS_Req,  io_wr_addr_HS_Ack;
  logic [2:0]  io_wr_addr_Data;
  logic        io_rd_addr_HS_Req,  io_rd_addr_HS_Ack;
  logic [2:0]  io_rd_addr_Data;
  logic        io_rd_data_HS_Req,  io_rd_data_HS_Ack;
  logic [7:0]  io_rd_data_Data;

  // Unused clock input
  logic clk_dummy;
  initial clk_dummy = 1'b0;

  RAM RAM_inst (
    .clock              (clk_dummy),
    .reset              (reset),

    .io_wr_valid_HS_Req (io_wr_valid_HS_Req),
    .io_wr_valid_HS_Ack (io_wr_valid_HS_Ack),
    .io_wr_valid_Data   (io_wr_valid_Data),

    .io_wr_data_HS_Req  (io_wr_data_HS_Req),
    .io_wr_data_HS_Ack  (io_wr_data_HS_Ack),
    .io_wr_data_Data    (io_wr_data_Data),

    .io_wr_addr_HS_Req  (io_wr_addr_HS_Req),
    .io_wr_addr_HS_Ack  (io_wr_addr_HS_Ack),
    .io_wr_addr_Data    (io_wr_addr_Data),

    .io_rd_addr_HS_Req  (io_rd_addr_HS_Req),
    .io_rd_addr_HS_Ack  (io_rd_addr_HS_Ack),
    .io_rd_addr_Data    (io_rd_addr_Data),

    .io_rd_data_HS_Req  (io_rd_data_HS_Req),
    .io_rd_data_HS_Ack  (io_rd_data_HS_Ack),
    .io_rd_data_Data    (io_rd_data_Data)
  );

  // temp vars
  logic old_ack0, old_ack1, old_ack2, old_ack3;
  logic prev_out_req;
  integer i;
  bit timed_out;

  initial begin
    `START_TESTBENCH
    error = 0;

    // -------------------------
    // init + reset
    // -------------------------
    reset = 1'b1;

    io_wr_valid_HS_Req = 1'b0;
    io_wr_valid_Data   = 1'b0;

    io_wr_data_HS_Req  = 1'b0;
    io_wr_data_Data    = 8'd0;

    io_wr_addr_HS_Req  = 1'b0;
    io_wr_addr_Data    = 3'd0;

    io_rd_addr_HS_Req  = 1'b0;
    io_rd_addr_Data    = 3'd0;

    io_rd_data_HS_Ack  = 1'b0;

    #5;
    reset = 1'b0;
    #5;

    $display("==== T1: WRITE THEN READ (2 fires) ====");

    // ==========================================================
    // FIRE #1 : WRITE
    // ==========================================================
    // Provide stable inputs before toggling reqs
    io_wr_addr_Data  = 3'b000;
    io_wr_data_Data  = 8'd20;
    io_wr_valid_Data = 1'b1;      // enable write
    io_rd_addr_Data  = 3'b000;    // must provide (ACG expects 4 inputs)

    // Snapshot acks
    old_ack0 = io_wr_valid_HS_Ack;
    old_ack1 = io_wr_data_HS_Ack;
    old_ack2 = io_wr_addr_HS_Ack;
    old_ack3 = io_rd_addr_HS_Ack;

    prev_out_req = io_rd_data_HS_Req;

    $display("T1 pre-fire#1: wr_valid Req=%0b Ack=%0b | wr_addr Req=%0b Ack=%0b | wr_data Req=%0b Ack=%0b | rd_addr Req=%0b Ack=%0b | rd_data Req=%0b Ack=%0b",
            io_wr_valid_HS_Req, io_wr_valid_HS_Ack,
            io_wr_addr_HS_Req,  io_wr_addr_HS_Ack,
            io_wr_data_HS_Req,  io_wr_data_HS_Ack,
            io_rd_addr_HS_Req,  io_rd_addr_HS_Ack,
            io_rd_data_HS_Req,  io_rd_data_HS_Ack);

    // Toggle ALL 4 input reqs (ACG only fires once all have changed)
    io_wr_valid_HS_Req = ~io_wr_valid_HS_Req;
    io_wr_data_HS_Req  = ~io_wr_data_HS_Req;
    io_wr_addr_HS_Req  = ~io_wr_addr_HS_Req;
    io_rd_addr_HS_Req  = ~io_rd_addr_HS_Req;

    // Wait for all 4 acks to toggle (fire happened)
    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if ((io_wr_valid_HS_Ack !== old_ack0) &&
          (io_wr_data_HS_Ack  !== old_ack1) &&
          (io_wr_addr_HS_Ack  !== old_ack2) &&
          (io_rd_addr_HS_Ack  !== old_ack3)) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T1 ERROR: timeout waiting for input Acks on fire#1");
      error++;
      `FINISH_WITH_FAIL
    end

    // Disable write AFTER fire#1 accepted
    io_wr_valid_Data = 1'b0;

    $display("T1 post-fire#1: wr_valid Req=%0b Ack=%0b | wr_addr Req=%0b Ack=%0b | wr_data Req=%0b Ack=%0b | rd_addr Req=%0b Ack=%0b | rd_data Req=%0b Ack=%0b",
            io_wr_valid_HS_Req, io_wr_valid_HS_Ack,
            io_wr_addr_HS_Req,  io_wr_addr_HS_Ack,
            io_wr_data_HS_Req,  io_wr_data_HS_Ack,
            io_rd_addr_HS_Req,  io_rd_addr_HS_Ack,
            io_rd_data_HS_Req,  io_rd_data_HS_Ack);

    // IMPORTANT: allow next fire by consuming output token (toggle Out Ack once)
    // Even if you don't "use" this output, the ACG blocks until Out Ack changes.
    io_rd_data_HS_Ack = ~io_rd_data_HS_Ack;
    #1;

    // ==========================================================
    // FIRE #2 : READ (captures Memory[rd_addr] into io_rd_data_Data_REG)
    // ==========================================================
    // Provide stable inputs before toggling reqs
    io_wr_valid_Data = 1'b0;      // read-only
    io_wr_data_Data  = 8'd0;      // don't care
    io_wr_addr_Data  = 3'd0;      // don't care
    io_rd_addr_Data  = 3'b000;    // read from addr 0

    // Snapshot acks again
    old_ack0 = io_wr_valid_HS_Ack;
    old_ack1 = io_wr_data_HS_Ack;
    old_ack2 = io_wr_addr_HS_Ack;
    old_ack3 = io_rd_addr_HS_Ack;

    prev_out_req = io_rd_data_HS_Req;

    // Toggle ALL 4 input reqs again
    io_wr_valid_HS_Req = ~io_wr_valid_HS_Req;
    io_wr_data_HS_Req  = ~io_wr_data_HS_Req;
    io_wr_addr_HS_Req  = ~io_wr_addr_HS_Req;
    io_rd_addr_HS_Req  = ~io_rd_addr_HS_Req;

    // Wait for all 4 acks to toggle (fire#2 happened)
    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if ((io_wr_valid_HS_Ack !== old_ack0) &&
          (io_wr_data_HS_Ack  !== old_ack1) &&
          (io_wr_addr_HS_Ack  !== old_ack2) &&
          (io_rd_addr_HS_Ack  !== old_ack3)) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T1 ERROR: timeout waiting for input Acks on fire#2");
      error++;
      `FINISH_WITH_FAIL
    end

    // Wait for output Req toggle (sanity that output token updated on fire#2)
    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if (io_rd_data_HS_Req !== prev_out_req) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T1 ERROR: timeout waiting for rd_data Req toggle on fire#2");
      error++;
      `FINISH_WITH_FAIL
    end

    // Give a tiny settle time for the reg assignment to be visible
    #0.2;

    $display("T1 READ: rd_data=%0d exp=%0d", io_rd_data_Data, 8'd20);
    if (io_rd_data_Data !== 8'd20) begin
      $display("T1 ERROR: rd_data mismatch (got %0d)", io_rd_data_Data);
      error++;
    end

    // Consume output token (toggle Out Ack) so system isn't left blocked
    io_rd_data_HS_Ack = ~io_rd_data_HS_Ack;
    #1;

    // ==========================================================
    // T2: OVERWRITE SAME ADDRESS
    //   Write addr0=77 then read addr0, expect 77
    // ==========================================================
    $display("==== T2: OVERWRITE addr0 ====");

    // ---------- FIRE #3 : WRITE addr0 = 77 ----------
    io_wr_addr_Data  = 3'b000;
    io_wr_data_Data  = 8'd77;
    io_wr_valid_Data = 1'b1;
    io_rd_addr_Data  = 3'b000;   // still provide

    old_ack0 = io_wr_valid_HS_Ack;
    old_ack1 = io_wr_data_HS_Ack;
    old_ack2 = io_wr_addr_HS_Ack;
    old_ack3 = io_rd_addr_HS_Ack;

    prev_out_req = io_rd_data_HS_Req;

    io_wr_valid_HS_Req = ~io_wr_valid_HS_Req;
    io_wr_data_HS_Req  = ~io_wr_data_HS_Req;
    io_wr_addr_HS_Req  = ~io_wr_addr_HS_Req;
    io_rd_addr_HS_Req  = ~io_rd_addr_HS_Req;

    // wait for all 4 acks to toggle (timeout)
    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if ((io_wr_valid_HS_Ack !== old_ack0) &&
          (io_wr_data_HS_Ack  !== old_ack1) &&
          (io_wr_addr_HS_Ack  !== old_ack2) &&
          (io_rd_addr_HS_Ack  !== old_ack3)) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T2 ERROR: timeout waiting for input Acks on write fire");
      error++;
    end

    io_wr_valid_Data = 1'b0;

    // consume output token so next fire is allowed
    io_rd_data_HS_Ack = ~io_rd_data_HS_Ack;
    #1;

    // ---------- FIRE #4 : READ addr0, expect 77 ----------
    io_wr_valid_Data = 1'b0;
    io_wr_data_Data  = 8'd0;
    io_wr_addr_Data  = 3'd0;
    io_rd_addr_Data  = 3'b000;

    old_ack0 = io_wr_valid_HS_Ack;
    old_ack1 = io_wr_data_HS_Ack;
    old_ack2 = io_wr_addr_HS_Ack;
    old_ack3 = io_rd_addr_HS_Ack;

    prev_out_req = io_rd_data_HS_Req;

    io_wr_valid_HS_Req = ~io_wr_valid_HS_Req;
    io_wr_data_HS_Req  = ~io_wr_data_HS_Req;
    io_wr_addr_HS_Req  = ~io_wr_addr_HS_Req;
    io_rd_addr_HS_Req  = ~io_rd_addr_HS_Req;

    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if ((io_wr_valid_HS_Ack !== old_ack0) &&
          (io_wr_data_HS_Ack  !== old_ack1) &&
          (io_wr_addr_HS_Ack  !== old_ack2) &&
          (io_rd_addr_HS_Ack  !== old_ack3)) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T2 ERROR: timeout waiting for input Acks on read fire");
      error++;
    end

    // wait for output req toggle (timeout)
    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if (io_rd_data_HS_Req !== prev_out_req) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T2 ERROR: timeout waiting for rd_data Req toggle");
      error++;
    end

    #0.2;
    $display("T2 READ: rd_data=%0d exp=%0d", io_rd_data_Data, 8'd77);
    if (io_rd_data_Data !== 8'd77) begin
      $display("T2 ERROR: overwrite mismatch (got %0d)", io_rd_data_Data);
      error++;
    end

    // consume output
    io_rd_data_HS_Ack = ~io_rd_data_HS_Ack;
    #1;


    // ==========================================================
    // T3: "BREAK IT" PROTOCOL VIOLATION
    //   Don't consume output, then try to fire again.
    //   Expect: NO input Acks toggle (no fire) within timeout.
    // ==========================================================
    $display("==== T3: BREAK IT (no output consume) ====");

    // ---------- FIRE #5 : do a normal READ to create a pending output token ----------
    io_wr_valid_Data = 1'b0;
    io_wr_data_Data  = 8'd0;
    io_wr_addr_Data  = 3'd0;
    io_rd_addr_Data  = 3'b000;

    old_ack0 = io_wr_valid_HS_Ack;
    old_ack1 = io_wr_data_HS_Ack;
    old_ack2 = io_wr_addr_HS_Ack;
    old_ack3 = io_rd_addr_HS_Ack;

    prev_out_req = io_rd_data_HS_Req;

    io_wr_valid_HS_Req = ~io_wr_valid_HS_Req;
    io_wr_data_HS_Req  = ~io_wr_data_HS_Req;
    io_wr_addr_HS_Req  = ~io_wr_addr_HS_Req;
    io_rd_addr_HS_Req  = ~io_rd_addr_HS_Req;

    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if ((io_wr_valid_HS_Ack !== old_ack0) &&
          (io_wr_data_HS_Ack  !== old_ack1) &&
          (io_wr_addr_HS_Ack  !== old_ack2) &&
          (io_rd_addr_HS_Ack  !== old_ack3)) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T3 ERROR: timeout waiting for fire#5 acks");
      error++;
    end

    // Wait for output Req toggle so we know token is pending
    timed_out = 1;
    for (i = 0; i < 5000; i = i + 1) begin
      if (io_rd_data_HS_Req !== prev_out_req) begin
        timed_out = 0;
        i = 5000;
      end
      #0.1;
    end
    if (timed_out) begin
      $display("T3 ERROR: timeout waiting for output token on fire#5");
      error++;
    end

    // IMPORTANT: DO NOT consume output here (no io_rd_data_HS_Ack toggle)
    // Now attempt another "transaction" and expect it to NOT fire.

    old_ack0 = io_wr_valid_HS_Ack;
    old_ack1 = io_wr_data_HS_Ack;
    old_ack2 = io_wr_addr_HS_Ack;
    old_ack3 = io_rd_addr_HS_Ack;

    // toggle all 4 input reqs again
    io_wr_valid_HS_Req = ~io_wr_valid_HS_Req;
    io_wr_data_HS_Req  = ~io_wr_data_HS_Req;
    io_wr_addr_HS_Req  = ~io_wr_addr_HS_Req;
    io_rd_addr_HS_Req  = ~io_rd_addr_HS_Req;

    // Wait some time and ensure Acks do NOT change (meaning no fire)
    timed_out = 0; // reuse as "unexpected_fire" flag: 1 means it fired when it shouldn't
    for (i = 0; i < 2000; i = i + 1) begin
      if ((io_wr_valid_HS_Ack !== old_ack0) ||
          (io_wr_data_HS_Ack  !== old_ack1) ||
          (io_wr_addr_HS_Ack  !== old_ack2) ||
          (io_rd_addr_HS_Ack  !== old_ack3)) begin
        timed_out = 1; // unexpected ack toggle
        i = 2000;
      end
      #0.1;
    end

    if (timed_out) begin
      $display("T3 ERROR: Acks toggled even though output wasn't consumed (should have blocked fire)");
      error++;
    end else begin
      $display("T3 PASS: blocked as expected until output consumed");
    end

    // Now recover by consuming the pending output token
    io_rd_data_HS_Ack = ~io_rd_data_HS_Ack;
    #1;

    // (Optional) You could now do one more normal fire to prove it recovers.

    if (error > 0) begin
      `FINISH_WITH_FAIL
    end else begin
      `FINISH_WITH_PASS
    end
  end

  final begin
    $display("Simulation time is %t", $time);
    if(error_o === 1) begin
      $display("\033[0;31m    ______                    \033[0m");
      $display("\033[0;31m   / ____/_____________  _____\033[0m");
      $display("\033[0;31m  / __/ / ___/ ___/ __ \\/ ___/\033[0m");
      $display("\033[0;31m / /___/ /  / /  / /_/ / /    \033[0m");
      $display("\033[0;31m/_____/_/  /_/   \\____/_/     \033[0m");
      $display("Simulation Failed");
    end else if (pass_o === 1) begin
      $display("\033[0;32m    ____  ___   __________\033[0m");
      $display("\033[0;32m   / __ \\/   | / ___/ ___/\033[0m");
      $display("\033[0;32m  / /_/ / /| | \\__ \\\\__ \\ \033[0m");
      $display("\033[0;32m / ____/ ___ |___/ /__/ / \033[0m");
      $display("\033[0;32m/_/   /_/  |_/____/____/  \033[0m");
      $display();
      $display("Simulation Succeeded!");
    end else begin
      $display("   __  ___   ____ __ _   ______ _       ___   __");
      $display("  / / / / | / / //_// | / / __ \\ |     / / | / /");
      $display(" / / / /  |/ / ,<  /  |/ / / / / | /| / /  |/ / ");
      $display("/ /_/ / /|  / /| |/ /|  / /_/ /| |/ |/ / /|  /  ");
      $display("\\____/_/ |_/_/ |_/_/ |_/\\____/ |__/|__/_/ |_/   ");
      $display("Please set error_o or pass_o!");
    end
  end

endmodule


