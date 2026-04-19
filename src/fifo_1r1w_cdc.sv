module fifo_1r1w_cdc
 #(parameter [31:0] width_p = 32
  ,parameter [31:0] depth_log2_p = 8
  )

   // "c" consumer, and "p" for producer. 
  (input [0:0] cclk_i
  ,input [0:0] creset_i
  ,input [width_p - 1:0] cdata_i
  ,input [0:0] cvalid_i
  ,output [0:0] cready_o 

  ,input [0:0] pclk_i
  ,input [0:0] preset_i
  ,output [0:0] pvalid_o 
  ,output [width_p - 1:0] pdata_o 
  ,input [0:0] pready_i
  );
   
  localparam int ptr_size = depth_log2_p + 1; 
  localparam int depth_p = 1 << depth_log2_p;
  /* verilator lint_off UNUSEDSIGNAL */
  logic [depth_log2_p:0] wptr, rptr, rptr_f;
  /* verilator lint_on UNUSEDSIGNAL */
  wire [depth_log2_p-1:0] w_addr, r_addr;
  wire [width_p - 1:0] data_w;

  assign w_addr = wptr[depth_log2_p-1:0];
  assign r_addr = rptr_f[depth_log2_p-1:0];

  wire [0:0] full, empty;
  logic [0:0] ready_o_w, valid_o_w;

//full pipe p2c domain crossing
logic [depth_log2_p:0] C_gray2buffer, C_buffer2bin, C_bin2check;
bin2gray #(.width_p(ptr_size)) cb2g(
  .bin_i(rptr),
  .gray_o(C_gray2buffer)
);
delaybuffer #(.width_p(ptr_size), .delay_p(1)) C_sync_buffer(
  .clk_i(cclk_i),
  .reset_i(creset_i),
  .data_i(C_gray2buffer),
  .valid_i(1'b1),
  .ready_o(),
  .valid_o(),
  .data_o(C_buffer2bin),
  .ready_i(1'b1)
);
gray2bin #(.width_p(ptr_size)) cg2b(
  .gray_i(C_buffer2bin),
  .bin_o(C_bin2check)
);
assign full  = (wptr[depth_log2_p-1:0] === C_bin2check[depth_log2_p-1:0]) &&
                (wptr[depth_log2_p] !== C_bin2check[depth_log2_p]);

// empty pipe c2p domain crossing
logic [depth_log2_p:0] wr_cside_reg;
always_ff @(posedge cclk_i) begin
  wr_cside_reg <= wptr;
end
logic [depth_log2_p:0] P_gray2buffer, P_buffer2bin, P_bin2check;
bin2gray #(.width_p(ptr_size)) pb2g(
  .bin_i(wr_cside_reg),
  .gray_o(P_gray2buffer)
);
delaybuffer #(.width_p(ptr_size), .delay_p(1)) P_sync_buffer(
  .clk_i(pclk_i),
  .reset_i(preset_i),
  .data_i(P_gray2buffer),
  .valid_i(1'b1),
  .ready_o(),
  .valid_o(),
  .data_o(P_buffer2bin),
  .ready_i(1'b1)
);
gray2bin #(.width_p(ptr_size)) pg2b(
  .gray_i(P_buffer2bin),
  .bin_o(P_bin2check)
);
assign empty = (P_bin2check === rptr);
wire [0:0] empty_f = (P_bin2check === rptr_f); //not used

//data out
assign pdata_o = data_w;


  // logic for  read/write controls
  logic [0:0] up_read, up_write;
  always_comb begin
    valid_o_w = ~empty;
    ready_o_w = ~full;
    up_write = ready_o_w & cvalid_i;
    up_read = pready_i & ~empty;
  end 
  assign cready_o = ready_o_w;
  assign pvalid_o = valid_o_w;
  wire [0:0] wr_valid_i_w;
  assign wr_valid_i_w = cvalid_i & cready_o; //valid in and not full

  counter #(.width_p(depth_log2_p + 1)) read_ptr(
    .clk_i(pclk_i),
    .reset_i(preset_i),
    .up_i(up_read),
    .down_i(1'b0),
    .count_o(rptr),
    .count_fast(rptr_f)
  );
  /* verilator lint_off PINMISSING */
  counter #(.width_p(depth_log2_p + 1)) write_ptr(
    .clk_i(cclk_i),
    .reset_i(creset_i),
    .up_i(up_write),
    .down_i(1'b0),
    .count_o(wptr)
  );
  /* verilator lint_on PINMISSING */
  
  ram_1r1w_sync #(.width_p(width_p), .depth_p(depth_p), .filename_p("")) fifo_ram(
    .cclk_i(cclk_i),
    .pclk_i(pclk_i),
    .reset_i(creset_i),
    .wr_valid_i(wr_valid_i_w), //valid in and not full
    .wr_data_i(cdata_i),
    .wr_addr_i(w_addr),
    .rd_addr_i(r_addr),
    .rd_valid_i(1'b1), //not empty
    .rd_data_o(data_w)
  );

   
endmodule

