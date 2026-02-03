`timescale 1ns/1ps

`define START_TESTBENCH    error_o = 0; pass_o = 0; #10;
`define FINISH_WITH_FAIL   error_o = 1; pass_o = 0; #10; $finish();
`define FINISH_WITH_PASS   pass_o = 1; error_o = 0; #10; $finish();

module testbench
  (output logic error_o = 1'bx
  ,output logic pass_o  = 1'bx);

  logic [10:0] error;

  // DUT I/O
  logic        clk_i;
  logic        reset_i;

  logic  [7:0] io_A_0, io_A_1, io_A_2, io_A_3;
  logic  [7:0] io_B_0, io_B_1, io_B_2, io_B_3;
  logic        io_start;

  wire [15:0]  io_C_0,  io_C_1,  io_C_2,  io_C_3,
               io_C_4,  io_C_5,  io_C_6,  io_C_7,
               io_C_8,  io_C_9,  io_C_10, io_C_11,
               io_C_12, io_C_13, io_C_14, io_C_15;

  // 100MHz clock -> period 10ns (toggle every 5ns)
  initial clk_i = 1'b0;
  always #5 clk_i = ~clk_i;

  // DUT instance (matches Chisel-generated names you showed)
  Sync_MatMul 
   #()
  Sync_MatMul_inst (
    .clock    (clk_i),
    .reset    (reset_i),

    .io_A_0   (io_A_0),
    .io_A_1   (io_A_1),
    .io_A_2   (io_A_2),
    .io_A_3   (io_A_3),

    .io_B_0   (io_B_0),
    .io_B_1   (io_B_1),
    .io_B_2   (io_B_2),
    .io_B_3   (io_B_3),

    .io_start (io_start),

    .io_C_0   (io_C_0),
    .io_C_1   (io_C_1),
    .io_C_2   (io_C_2),
    .io_C_3   (io_C_3),
    .io_C_4   (io_C_4),
    .io_C_5   (io_C_5),
    .io_C_6   (io_C_6),
    .io_C_7   (io_C_7),
    .io_C_8   (io_C_8),
    .io_C_9   (io_C_9),
    .io_C_10  (io_C_10),
    .io_C_11  (io_C_11),
    .io_C_12  (io_C_12),
    .io_C_13  (io_C_13),
    .io_C_14  (io_C_14),
    .io_C_15  (io_C_15)
  );

  logic [7:0] A [0:3][0:3]; 
  logic [7:0] B [0:3][0:3]; 
  logic [15:0] Cexp [0:3][0:3];

  always_comb begin
  for (int i = 0; i < 4; i++) begin
    for (int j = 0; j < 4; j++) begin
      int unsigned sum;
      sum = 0;
      for (int k = 0; k < 4; k++) begin
        sum += A[i][k] * B[k][j];
      end
      Cexp[i][j] = sum[15:0];
    end
  end
  end

  // t = 3n - 3
  initial begin
    `START_TESTBENCH
    error = 0;
    io_A_0 = 0; io_A_1 = 0; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 0; io_B_1 = 0; io_B_2 = 0; io_B_3 = 0;

    reset_i = 1'b1;                 // assert reset
    repeat (3) @(posedge clk_i);
    reset_i = 1'b0;                 // deassert reset
    @(posedge clk_i);

    // --------------------------------
    // Test: 2x2 Matrix Multiplication
    // --------------------------------

    // A
    A[0][0]=1; A[0][1]=2; A[0][2]=0; A[0][3]=0; 
    A[1][0]=3; A[1][1]=4; A[1][2]=0; A[1][3]=0; 
    A[2][0]=0; A[2][1]=0; A[2][2]=0; A[2][3]=0; 
    A[3][0]=0; A[3][1]=0; A[3][2]=0; A[3][3]=0; 
    // B
    B[0][0]= 5; B[0][1]= 6; B[0][2]= 0; B[0][3]= 0; 
    B[1][0]= 7; B[1][1]=8; B[1][2]=0; B[1][3]=0; 
    B[2][0]=0; B[2][1]=0; B[2][2]=0; B[2][3]=0; 
    B[3][0]=0; B[3][1]=0; B[3][2]=0; B[3][3]=0; 
    
    // Expected C = A*B
    Cexp[0][0]= 19; Cexp[0][1]= 22; Cexp[0][2]= 0; Cexp[0][3]= 0; 
    Cexp[1][0]= 43; Cexp[1][1]= 50; Cexp[1][2]= 0; Cexp[1][3]= 0; 
    Cexp[2][0]=0; Cexp[2][1]=0; Cexp[2][2]=0; Cexp[2][3]=0; 
    Cexp[3][0]=0; Cexp[3][1]=0; Cexp[3][2]=0; Cexp[3][3]=0;

    io_start = 1'b1;
    @(posedge clk_i);
    io_start = 1'b0;

    // t=0
    @(negedge clk_i);
    io_A_0 = 1; io_A_1 = 0; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 5; io_B_1 = 0; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    $display("t0 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=1
    @(negedge clk_i);
    io_A_0 = 2; io_A_1 = 3; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 7; io_B_1 = 6; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    $display("t1 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=2
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 4; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 0; io_B_1 = 8; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    $display("t2 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=3 flush
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 0; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 0; io_B_1 = 0; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    $display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);


    // checks (flattened: row*4 + col)
    if (io_C_0 !== Cexp[0][0]) begin $display("C00 mismatch got=%0d exp=%0d", io_C_0, Cexp[0][0]); error++; end
    if (io_C_1 !== Cexp[0][1]) begin $display("C01 mismatch got=%0d exp=%0d", io_C_1, Cexp[0][1]); error++; end
    if (io_C_4 !== Cexp[1][0]) begin $display("C10 mismatch got=%0d exp=%0d", io_C_4, Cexp[1][0]); error++; end
    if (io_C_5 !== Cexp[1][1]) begin $display("C11 mismatch got=%0d exp=%0d", io_C_5, Cexp[1][1]); error++; end

    // --------------------------------
    // Test: 4x4 Matrix Multiplication
    // --------------------------------

    // A
    A[0][0]=1; A[0][1]=2; A[0][2]=3; A[0][3]=4; 
    A[1][0]=5; A[1][1]=6; A[1][2]=7; A[1][3]=8; 
    A[2][0]=9; A[2][1]=10; A[2][2]=11; A[2][3]=12; 
    A[3][0]=13; A[3][1]=14; A[3][2]=15; A[3][3]=16; 
    // B
    B[0][0]= 17; B[0][1]= 18; B[0][2]= 19; B[0][3]= 20; 
    B[1][0]= 21; B[1][1]=22; B[1][2]=23; B[1][3]=24; 
    B[2][0]=25; B[2][1]=26; B[2][2]=27; B[2][3]=28; 
    B[3][0]=29; B[3][1]=30; B[3][2]=31; B[3][3]=32; 
    
    // Expected C = A*B
    Cexp[0][0]= 250; Cexp[0][1]= 260; Cexp[0][2]= 270; Cexp[0][3]= 280; 
    Cexp[1][0]= 618; Cexp[1][1]= 644; Cexp[1][2]= 670; Cexp[1][3]= 696; 
    Cexp[2][0]=986; Cexp[2][1]=1028; Cexp[2][2]=1070; Cexp[2][3]=1112; 
    Cexp[3][0]=1354; Cexp[3][1]=1412; Cexp[3][2]=1470; Cexp[3][3]=1528;

    io_start = 1'b1;
    repeat (2) @(posedge clk_i);
    io_start = 1'b0;

    // t=0
    @(negedge clk_i);
    io_A_0 = 1; io_A_1 = 0; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 17; io_B_1 = 0; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    //$display("t0 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=1
    @(negedge clk_i);
    io_A_0 = 2; io_A_1 = 5; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 21; io_B_1 = 18; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    //$display("t1 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=2
    @(negedge clk_i);
    io_A_0 = 3; io_A_1 = 6; io_A_2 = 9; io_A_3 = 0;
    io_B_0 = 25; io_B_1 = 22; io_B_2 = 19; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    //$display("t2 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=3
    @(negedge clk_i);
    io_A_0 = 4; io_A_1 = 7; io_A_2 = 10; io_A_3 = 13;
    io_B_0 = 29; io_B_1 = 26; io_B_2 = 23; io_B_3 = 20;
    @(posedge clk_i);
    #1;
    //$display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=4
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 8; io_A_2 = 11; io_A_3 = 14;
    io_B_0 = 0; io_B_1 = 30; io_B_2 = 27; io_B_3 = 24;
    @(posedge clk_i);
    #1;
    //$display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=5
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 0; io_A_2 = 12; io_A_3 = 15;
    io_B_0 = 0; io_B_1 = 0; io_B_2 = 31; io_B_3 = 28;
    @(posedge clk_i);
    #1;
    //$display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=6
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 0; io_A_2 = 0; io_A_3 = 16;
    io_B_0 = 0; io_B_1 = 0; io_B_2 = 0; io_B_3 = 32;
    @(posedge clk_i);
    #1;
    //$display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=7
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 0; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 0; io_B_1 = 0; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    //$display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=8
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 0; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 0; io_B_1 = 0; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    //$display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // t=9
    @(negedge clk_i);
    io_A_0 = 0; io_A_1 = 0; io_A_2 = 0; io_A_3 = 0;
    io_B_0 = 0; io_B_1 = 0; io_B_2 = 0; io_B_3 = 0;
    @(posedge clk_i);
    #1;
    //$display("t3 C00=%0d C01=%0d C10=%0d C11=%0d", io_C_0, io_C_1, io_C_4, io_C_5);

    // checks (flattened: row*4 + col)
    if (io_C_0 !== Cexp[0][0]) begin $display("C00 mismatch got=%0d exp=%0d", io_C_0, Cexp[0][0]); error++; end
    if (io_C_1 !== Cexp[0][1]) begin $display("C01 mismatch got=%0d exp=%0d", io_C_1, Cexp[0][1]); error++; end
    if (io_C_2 !== Cexp[0][2]) begin $display("C02 mismatch got=%0d exp=%0d", io_C_2, Cexp[0][2]); error++; end
    if (io_C_3 !== Cexp[0][3]) begin $display("C03 mismatch got=%0d exp=%0d", io_C_3, Cexp[0][3]); error++; end
    if (io_C_4 !== Cexp[1][0]) begin $display("C10 mismatch got=%0d exp=%0d", io_C_4, Cexp[1][0]); error++; end
    if (io_C_5 !== Cexp[1][1]) begin $display("C11 mismatch got=%0d exp=%0d", io_C_5, Cexp[1][1]); error++; end
    if (io_C_6 !== Cexp[1][2]) begin $display("C12 mismatch got=%0d exp=%0d", io_C_6, Cexp[1][2]); error++; end
    if (io_C_7 !== Cexp[1][3]) begin $display("C13 mismatch got=%0d exp=%0d", io_C_7, Cexp[1][3]); error++; end
    if (io_C_8 !== Cexp[2][0]) begin $display("C20 mismatch got=%0d exp=%0d", io_C_8, Cexp[2][0]); error++; end
    if (io_C_9 !== Cexp[2][1]) begin $display("C21 mismatch got=%0d exp=%0d", io_C_9, Cexp[2][1]); error++; end
    if (io_C_10 !== Cexp[2][2]) begin $display("C22 mismatch got=%0d exp=%0d", io_C_10, Cexp[2][2]); error++; end
    if (io_C_11 !== Cexp[2][3]) begin $display("C23 mismatch got=%0d exp=%0d", io_C_11, Cexp[2][3]); error++; end
    if (io_C_12 !== Cexp[3][0]) begin $display("C30 mismatch got=%0d exp=%0d", io_C_12, Cexp[3][0]); error++; end
    if (io_C_13 !== Cexp[3][1]) begin $display("C31 mismatch got=%0d exp=%0d", io_C_13, Cexp[3][1]); error++; end
    if (io_C_14 !== Cexp[3][2]) begin $display("C32 mismatch got=%0d exp=%0d", io_C_14, Cexp[3][2]); error++; end
    if (io_C_15 !== Cexp[3][3]) begin $display("C33 mismatch got=%0d exp=%0d", io_C_15, Cexp[3][3]); error++; end


    if (error > 0) begin
      `FINISH_WITH_FAIL
    end else begin
      `FINISH_WITH_PASS
    end
  end

  // pretty banner
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
