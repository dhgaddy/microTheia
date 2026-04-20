// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors

//the idea is using registers for cdc that drain directly into a single clock domain sram fifo for input/output buffering

module reg_cdc_sram_buffer 
#(
    parameter DATA_WIDTH = 32, 
    parameter REG_FIFO_LOG2 = 6, 
    parameter FIFO_DEPTH = 256 
)
 //c is consumer/write side and p is producer/read side
(
   input [0:0] cclk_i //write side clock
  ,input [0:0] creset_i //write side rest
  ,input [DATA_WIDTH - 1:0] cdata_i //data in write side to cdc fifo
  ,input [0:0] cvalid_i //write side valid in to cdc fifo
  ,output [0:0] cdc_cready_o  //write side ready o from cdc fifo, should be tied to overflow flag

  ,input [0:0] pclk_i //read out side clock (input buffer and read side of cdc registers)
  ,input [0:0] preset_i //input buffer and cdc read side reset
  ,output [0:0] pvalid_o //comes from sram buffer fifo
  ,output [DATA_WIDTH - 1:0] pdata_o //from sram buffer fifo
  ,input [0:0] pready_i //to sram buffer fifo
  ,output [3:0] in_fifo_dbg // debug signals from sram fifo
  ,output logic [0:0] sram_ovfl
); 

//----------CDC WRITE IN
logic [DATA_WIDTH-1:0] cdc_write_data, read_between_data;
logic cdc_write_data_valid_i, cdc_write_side_ready_o, cdc_read_side_valid_o, cdc_read_side_ready_i;

assign cdc_write_data = cdata_i;
assign cdc_write_data_valid_i = cvalid_i;
assign cdc_cready_o = cdc_write_side_ready_o;

fifo_1r1w_cdc //my cdc fifo module from CSE 225, its internal ram module will just map to registers i think?
 #(.width_p(DATA_WIDTH),
  .depth_log2_p(REG_FIFO_LOG2) //this should be a very small number, as in 2-5 maybe 6
  )
   cdc_out          // consumer/write side 
   (.cclk_i(cclk_i)
  ,.creset_i(creset_i) //
  ,.cdata_i(cdc_write_data) // coming in from port cdata_i
  ,.cvalid_i(cdc_write_data_valid_i) // in from port cvalid_i
  ,.cready_o(cdc_write_side_ready_o) // out through port cdc_cready_o

  ,.pclk_i(pclk_i) //
  ,.preset_i(preset_i) //
  ,.pvalid_o(cdc_read_side_valid_o) // to sram buffer fifo
  ,.pdata_o(read_between_data) // to sram buffer fifo
  ,.pready_i(cdc_read_side_ready_i) // from sram buffer fifo
  );
//---------READ OUT FROM CDC REGISTERS TO INPUT BUFFER
/*
logic [DATA_WIDTH-1:0] cdc_read_data_r;
logic [0:0] cdc_read_valid_r;

always_ff @(posedge pclk_i) begin
    if (preset_i) begin
        cdc_read_data_r  <= '0;
        cdc_read_valid_r <= 1'b0;
    end
    else begin
        cdc_read_data_r  <= read_between_data;
        cdc_read_valid_r <= cdc_read_side_valid_o;
    end
end
*/


//---------SRAM BASED INPUT BUFFER WRITES IN FROM CDC REGISTERS
//this is the one that already uses the gf180 sram macro
input_fifo #(
        .FIFO_DEPTH(FIFO_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_input_fifo (
        .clk_i   (pclk_i),
        .reset_i (preset_i),
        .data_i  (read_between_data),
        .ready_i (pready_i),
        .valid_i (cdc_read_side_valid_o),
        .ready_o (cdc_read_side_ready_i),
        .valid_o (pvalid_o),
        .data_o  (pdata_o),
        .in_fifo_dbg(in_fifo_dbg)
    );
//---- READ OUT FROM SRAM BUFFER FIFO 

always_ff @(posedge pclk_i) begin
    if(preset_i) begin
        sram_ovfl <= 1'b0;
    end
    else if(cdc_read_side_valid_o && !cdc_read_side_ready_i) begin
        sram_ovfl <= 1'b1; //overflow flag for sram
    end      
end    
endmodule