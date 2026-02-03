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
  logic        io_In0_HS_Req, io_In0_HS_Ack;
  logic [7:0]  io_In0_Data;
  logic        io_In1_HS_Req, io_In1_HS_Ack;
  logic [7:0]  io_In1_Data;
  wire         io_Out_HS_Req;
  logic        io_Out_HS_Ack;
  wire  [7:0]  io_Out_Data;

  // Unused clock input (ADD ignores it)
  logic clk_dummy;
  initial clk_dummy = 1'b0;

  ADD ADD_inst (
    .clock        (clk_dummy),
    .reset        (reset),
    .io_In0_HS_Req(io_In0_HS_Req),
    .io_In0_HS_Ack(io_In0_HS_Ack),
    .io_In0_Data  (io_In0_Data),
    .io_In1_HS_Req(io_In1_HS_Req),
    .io_In1_HS_Ack(io_In1_HS_Ack),
    .io_In1_Data  (io_In1_Data),
    .io_Out_HS_Req(io_Out_HS_Req),
    .io_Out_HS_Ack(io_Out_HS_Ack),
    .io_Out_Data  (io_Out_Data)
  );

  // Keep previous Out Req to detect toggles
  logic prev_out_req;
  logic old_ack0, old_ack1;
  logic old_out_req;
  logic old_in0_ack, old_in1_ack;

  initial begin
    `START_TESTBENCH
    error = 0;

    // init
    reset        = 1'b1;
    io_In0_HS_Req = 1'b0;
    io_In1_HS_Req = 1'b0;
    io_Out_HS_Ack = 1'b0;
    io_In0_Data   = 8'd0;
    io_In1_Data   = 8'd0;
    prev_out_req  = 1'b0;

    #5;
    reset = 1'b0;
    #5;

    // -------------------------
    // TEST 1: 10 + 20 = 30
    // -------------------------
    prev_out_req = io_Out_HS_Req;

    // snapshot old acks before we change req
    old_ack0 = io_In0_HS_Ack;
    old_ack1 = io_In1_HS_Ack;

    io_In0_Data   = 8'd10;
    io_In1_Data   = 8'd20;
    io_In0_HS_Req = ~io_In0_HS_Req;
    io_In1_HS_Req = ~io_In1_HS_Req;

    // (Optional) immediate check: acks should NOT instantly jump to req
    #0.1;
    $display("T1 pre-fire: In0 Req=%0b Ack=%0b | In1 Req=%0b Ack=%0b | Out Req=%0b Ack=%0b",
            io_In0_HS_Req, io_In0_HS_Ack,
            io_In1_HS_Req, io_In1_HS_Ack,
            io_Out_HS_Req, io_Out_HS_Ack);

    if (io_In0_HS_Ack !== old_ack0) begin $display("T1 ERROR: In0 Ack changed too early"); error++; end
    if (io_In1_HS_Ack !== old_ack1) begin $display("T1 ERROR: In1 Ack changed too early"); error++; end

    wait (io_Out_HS_Req !== prev_out_req);
    #1;
    
    $display("T1 post-fire: In0 Req=%0b Ack=%0b | In1 Req=%0b Ack=%0b | Out Req=%0b Ack=%0b",
         io_In0_HS_Req, io_In0_HS_Ack,
         io_In1_HS_Req, io_In1_HS_Ack,
         io_Out_HS_Req, io_Out_HS_Ack);

    // after fire, input acks should match reqs
    if (io_In0_HS_Ack !== io_In0_HS_Req) begin $display("T1 ERROR: In0 Ack != Req after fire"); error++; end
    if (io_In1_HS_Ack !== io_In1_HS_Req) begin $display("T1 ERROR: In1 Ack != Req after fire"); error++; end

    // after fire, Out Req should be ~Out Ack (based on empty_ff <= ~OutAck)
    if (io_Out_HS_Req !== ~io_Out_HS_Ack) begin $display("T1 ERROR: Out Req != ~Ack after fire"); error++; end

    // data check
    $display("T1 out=%0d exp=%0d", io_Out_Data, 8'd30);
    if (io_Out_Data !== 8'd30) error++;

    io_Out_HS_Ack = ~io_Out_HS_Ack;  // consume

    // -------------------------
    // TEST 2: 5 + 7 = 12
    // -------------------------
    prev_out_req = io_Out_HS_Req;

    // snapshot old acks before we change req
    old_ack0 = io_In0_HS_Ack;
    old_ack1 = io_In1_HS_Ack;

    io_In0_Data   = 8'd5;
    io_In1_Data   = 8'd7;
    io_In0_HS_Req = ~io_In0_HS_Req;
    io_In1_HS_Req = ~io_In1_HS_Req;

    // (Optional) immediate check: acks should NOT instantly jump to req
    #0.1;
    $display("T2 pre-fire: In0 Req=%0b Ack=%0b | In1 Req=%0b Ack=%0b | Out Req=%0b Ack=%0b",
            io_In0_HS_Req, io_In0_HS_Ack,
            io_In1_HS_Req, io_In1_HS_Ack,
            io_Out_HS_Req, io_Out_HS_Ack);

    if (io_In0_HS_Ack !== old_ack0) begin $display("T2 ERROR: In0 Ack changed too early"); error++; end
    if (io_In1_HS_Ack !== old_ack1) begin $display("T2 ERROR: In1 Ack changed too early"); error++; end

    wait (io_Out_HS_Req !== prev_out_req);
    #1;
    
    $display("T2 post-fire: In0 Req=%0b Ack=%0b | In1 Req=%0b Ack=%0b | Out Req=%0b Ack=%0b",
         io_In0_HS_Req, io_In0_HS_Ack,
         io_In1_HS_Req, io_In1_HS_Ack,
         io_Out_HS_Req, io_Out_HS_Ack);

    // after fire, input acks should match reqs
    if (io_In0_HS_Ack !== io_In0_HS_Req) begin $display("T2 ERROR: In0 Ack != Req after fire"); error++; end
    if (io_In1_HS_Ack !== io_In1_HS_Req) begin $display("T2 ERROR: In1 Ack != Req after fire"); error++; end

    // after fire, Out Req should be ~Out Ack (based on empty_ff <= ~OutAck)
    if (io_Out_HS_Req !== ~io_Out_HS_Ack) begin $display("T2 ERROR: Out Req != ~Ack after fire"); error++; end

    // data check
    $display("T2 out=%0d exp=%0d", io_Out_Data, 8'd12);
    if (io_Out_Data !== 8'd12) error++;

    io_Out_HS_Ack = ~io_Out_HS_Ack;  // consume

    // -------------------------
    // TEST 3: 200 + 100 = 44 (mod 256)
    // -------------------------
    prev_out_req = io_Out_HS_Req;

    old_ack0 = io_In0_HS_Ack;
    old_ack1 = io_In1_HS_Ack;

    io_In0_Data   = 8'd200;
    io_In1_Data   = 8'd100;
    io_In0_HS_Req = ~io_In0_HS_Req;
    io_In1_HS_Req = ~io_In1_HS_Req;

    // (Optional) immediate check: acks should NOT instantly jump to req
    #0.1;
    $display("T3 pre-fire: In0 Req=%0b Ack=%0b | In1 Req=%0b Ack=%0b | Out Req=%0b Ack=%0b",
            io_In0_HS_Req, io_In0_HS_Ack,
            io_In1_HS_Req, io_In1_HS_Ack,
            io_Out_HS_Req, io_Out_HS_Ack);

    if (io_In0_HS_Ack !== old_ack0) begin $display("T3 ERROR: In0 Ack changed too early"); error++; end
    if (io_In1_HS_Ack !== old_ack1) begin $display("T3 ERROR: In1 Ack changed too early"); error++; end

    wait (io_Out_HS_Req !== prev_out_req);
    #1;
    
    $display("T3 post-fire: In0 Req=%0b Ack=%0b | In1 Req=%0b Ack=%0b | Out Req=%0b Ack=%0b",
         io_In0_HS_Req, io_In0_HS_Ack,
         io_In1_HS_Req, io_In1_HS_Ack,
         io_Out_HS_Req, io_Out_HS_Ack);

    // after fire, input acks should match reqs
    if (io_In0_HS_Ack !== io_In0_HS_Req) begin $display("T3 ERROR: In0 Ack != Req after fire"); error++; end
    if (io_In1_HS_Ack !== io_In1_HS_Req) begin $display("T3 ERROR: In1 Ack != Req after fire"); error++; end

    // after fire, Out Req should be ~Out Ack (based on empty_ff <= ~OutAck)
    if (io_Out_HS_Req !== ~io_Out_HS_Ack) begin $display("T3 ERROR: Out Req != ~Ack after fire"); error++; end

    // data check
    $display("T3 out=%0d exp=%0d", io_Out_Data, 8'd12);
    if (io_Out_Data !== 8'd44) error++;

    io_Out_HS_Ack = ~io_Out_HS_Ack;  // consume

    // -----------------------
    // Test: Toggle only In0 Req
    // -----------------------

    $display("\nNEG A: Toggle only In0 Req (should NOT fire)");

    prev_out_req = io_Out_HS_Req;

    old_in0_ack = io_In0_HS_Ack;
    old_in1_ack = io_In1_HS_Ack;

    io_In0_Data   = 8'd33;
    io_In1_Data   = 8'd44;      // doesn't matter if In1 doesn't request
    io_In0_HS_Req = ~io_In0_HS_Req;  // ONLY In0 requests
    // io_In1_HS_Req NOT toggled

    #10; // wait longer than your delay chain

    // Expect: no output request toggle
    if (io_Out_HS_Req !== prev_out_req) begin
    $display("NEG A ERROR: Out Req toggled but should NOT have");
    error++;
    end

    // Expect: no ack changes
    if (io_In0_HS_Ack !== old_in0_ack) begin
    $display("NEG A ERROR: In0 Ack changed but should NOT have");
    error++;
    end
    if (io_In1_HS_Ack !== old_in1_ack) begin
    $display("NEG A ERROR: In1 Ack changed but should NOT have");
    error++;
    end

    $display("NEG A OK: OutReq stayed %0b, In0Ack stayed %0b, In1Ack stayed %0b",
            io_Out_HS_Req, io_In0_HS_Ack, io_In1_HS_Ack);

    $display("NEG A state: OutReq=%0b OutAck=%0b (empty=%0b)",
         io_Out_HS_Req, io_Out_HS_Ack, (io_Out_HS_Req==io_Out_HS_Ack));

    // -----------------------
    // Test: Toggle only In0 Req twice
    // -----------------------

    $display("\nNEG A2: Toggle In0 Req twice (still should NOT fire)");
    prev_out_req = io_Out_HS_Req;
    old_in0_ack = io_In0_HS_Ack;
    old_in1_ack = io_In1_HS_Ack;

    io_In0_Data = 8'd11;
    io_In0_HS_Req = ~io_In0_HS_Req;
    #5;
    io_In0_Data = 8'd22;
    io_In0_HS_Req = ~io_In0_HS_Req;
    #10;

    if (io_Out_HS_Req !== prev_out_req) begin $display("NEG A2 ERROR: fired"); error++; end
    if (io_In0_HS_Ack !== old_in0_ack) begin $display("NEG A2 ERROR: In0Ack changed"); error++; end
    if (io_In1_HS_Ack !== old_in1_ack) begin $display("NEG A2 ERROR: In1Ack changed"); error++; end

    $display("NEG A2 OK: OutReq stayed %0b, In0Ack stayed %0b, In1Ack stayed %0b",
            io_Out_HS_Req, io_In0_HS_Ack, io_In1_HS_Ack);
    
    $display("NEG A2 state: OutReq=%0b OutAck=%0b (empty=%0b)",
         io_Out_HS_Req, io_Out_HS_Ack, (io_Out_HS_Req==io_Out_HS_Ack));


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
