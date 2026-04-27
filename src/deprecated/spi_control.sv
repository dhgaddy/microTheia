// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors

//SPI control interface module, type 0 SPI only 
//  type 0 means:
//  bit shifts out on negedge of SCLK (for both MISO and MOSI)
//  bit sampled on posedge of SCLK (again for both)

//  this module handles packing and unpacking bits in both directions (parameterized)
//  CDC is accomplished with a small register-based CDC FIFO that drains into a larger SRAM-based single clock domain buffering FIFO (one set in each direction)

//  calls to read from FIFOs are scheduled to maintain consistent serial output/input, but I would expect issues with attempting to pack less than 3 bits per write

module spi_control #(
    parameter DATA_WIDTH = 32,
    parameter REG_FIFO_LOG2 = 6,
    parameter FIFO_DEPTH = 256
)(
    input  logic [0:0] SCLK, //clock from master
    input  logic [0:0] MOSI, //master out slave in
    input  [0:0] CS, // chip select (or something similar, active low transaction enable signal)
    input  [0:0] clk_i, //clock from chip
    input  [0:0] reset_i, //reset from chip
    input  [DATA_WIDTH - 1:0] chip_out, //data from chip headed towards master, enters module in chip clock domain
    input  [0:0] chip_out_valid, //valid signal for data from chip headed towards master
    output [0:0] chip_out_ready_o, //ready_o for the output side fifo (as in this signal goes into the chip)
    output logic [0:0] MISO, //master in slave out
    output [DATA_WIDTH - 1:0] chip_in, //data from master that has been byte packed and crossed into chip clock domain, headed further into chip
    output [0:0] chip_in_valid, //valid signal for data from master headed into chip
    output logic [0:0] in_ovfl, out_ovfl, //overflow flag for cdc input/output register fifos, if their ready_o ever goes low then flag is set high and stays high until reset
    output [3:0] sram_in_fifo_dbg, sram_out_fifo_dbg,
    output logic [0:0] sram_out_ovfl, sram_in_ovfl
);

// ----------CHIP IN SECTION---------
localparam int DATA_WIDTH_W = $clog2(DATA_WIDTH);
logic [DATA_WIDTH -1:0] word_packer, packed_word;
logic [DATA_WIDTH_W - 1:0] packer_count; //tracking how many bits have been packed
logic [0:0] valid_word; //data exchange signal
logic cready_o_wire; // connect to overflow flag
logic in_fifo_valid_o; //intermediary signal for chip_in_valid
logic [DATA_WIDTH - 1:0] fifo_data_to_chip; //intermediary signal for chip_in

always_ff @(posedge SCLK) begin
    if(reset_i) begin
        packer_count <= '0;
        packed_word <= '0;
        word_packer <= '0;
        valid_word <= 1'b0;
        in_ovfl <= 1'b0;
    end
    else begin
        if(CS) begin //active low signal for transaction, so CS high is basically a reset
            packer_count <= '0; // 
            word_packer <= '0;
            valid_word <= 1'b0;
        end    
        else if(~CS) begin //CS active low transaction begins
            if(packer_count < DATA_WIDTH - 1) begin //if less than DATA WIDTH bits packed
                word_packer <= {word_packer [DATA_WIDTH - 2:0], MOSI}; //pack the serial bits MSB first
                packer_count <= packer_count + 1'd1; //keep track of bits packed
                valid_word <= 1'b0; //reset valid word signal
            end
            else if(packer_count == DATA_WIDTH - 1) begin //if DATA_WIDTH bits have been received
                packed_word <= {word_packer [DATA_WIDTH - 2:0], MOSI}; //extract word while capturing final bit
                word_packer <= '0; //start over packing bits
                valid_word <= 1'b1; //set valid word high, signal to be written into cdc fifo
                packer_count <= '0; //reset count
            end    
        end

        if(~cready_o_wire) begin //if unable to accept more writes trigger overflow flag
            in_ovfl <= 1'b1;
        end    
    end   

end    



reg_cdc_sram_buffer //module that handles cdc and input buffering
 #(.DATA_WIDTH(DATA_WIDTH),
  .REG_FIFO_LOG2(REG_FIFO_LOG2),
  .FIFO_DEPTH(FIFO_DEPTH)
  )
   cdc_in          // "c" consumer (write side), and "p" for producer (read side). 
   (.cclk_i(SCLK)
  ,.creset_i(reset_i) //connecting the same chip side reset to both ports, not sure about
  ,.cdata_i(packed_word) //writing in packed bytes
  ,.cvalid_i(valid_word) //using valid byte signal as write trigger
  ,.cdc_cready_o(cready_o_wire), //tieing to overflow signal

  .pclk_i(clk_i),
  .preset_i(reset_i),
  .pvalid_o(in_fifo_valid_o), //valid o signal for module output
  .pdata_o(fifo_data_to_chip), //output to a command/payload parser (different module, maybe straight to the decoder?)
  .pready_i(1'b1), //tieing to 1, just keeps it pumping
  .in_fifo_dbg(sram_in_fifo_dbg), //sram fifo debug bus
  .sram_ovfl(sram_in_ovfl) //sram overflow
  );


assign chip_in = fifo_data_to_chip;
assign chip_in_valid = in_fifo_valid_o;

//----------CHIP IN END-----------

//----------CHIP OUT BEGIN--------
//this side is a little more complicated because the bit on MISO must awlays be valid the very first posedge of SCLK after CS goes low

logic [DATA_WIDTH - 1:0] out_shifter, backup_shifter;
logic [DATA_WIDTH_W - 1:0] out_counter;
logic [DATA_WIDTH - 1:0] fifo_data_from_chip;
logic [0:0] fifo_from_chip_valid;
logic [0:0] need_read;

// one word holding register for the next word to send, in pursuit of having first bit ready at first posedge after CS goes low
logic [DATA_WIDTH - 1:0] next_tx_byte;

//flag indicating next_tx_byte currently holds a valid unused word
logic [0:0] next_tx_valid;
logic [0:0] consumed_during_idle;

always_ff @(negedge SCLK) begin
    if(reset_i) begin
        out_shifter   <= '0;
        backup_shifter <= '0;
        out_counter   <= '0;
        next_tx_byte  <= '0;
        next_tx_valid <= 1'b0;
        need_read     <= 1'b0;
        MISO          <= 1'b0;
        out_ovfl      <= 1'b0;
        consumed_during_idle <= 1'b0;
    end
    else begin
        //default is no read unless something calls for it
        need_read <= 1'b0;
        //if fifo has a valid byte and staging register isnt full then grab it and set valid flag
        if(fifo_from_chip_valid && !next_tx_valid) begin
            next_tx_byte  <= fifo_data_from_chip;
            need_read <= 1'b1;
            next_tx_valid <= 1'b1;
        end
        if(~chip_out_ready_o) begin
            out_ovfl <= 1'b1; //overflow flag for output fifo
        end

        if(CS)begin // cs high means not a transaction, but the out_shifter needs to stay loaded and ready
            if(!consumed_during_idle) begin //this check is important, fixed a bug
                out_counter <= '0;
            end
            else begin
                out_counter <= 1;
            end
            //below was causing problems, better to send the read request only when the register or the fifo data get used
            /*if(!next_tx_valid) begin // if no byte is buffered already ask fifo for one
                need_read <= 1'b1;
            end*/

            // preload shifter while idle so first bit is already valid before first posedge after CS goes low
            if(next_tx_valid&!consumed_during_idle) begin
                out_shifter <= {next_tx_byte, 1'b0};
                backup_shifter <= next_tx_byte;
                MISO        <= next_tx_byte[DATA_WIDTH - 1];
                //if(!consumed_during_idle) begin
                    out_counter <= out_counter + 1'd1; //preloaded bit needs to be counted
                    next_tx_valid <= 1'b0; //consume the byte? 
                //end    
                need_read <= 1'b1;
                consumed_during_idle <= 1'b1;
            end
            else if (!consumed_during_idle) begin //if nothing buffered and fifo has nothing then 0 is valid first bit
                out_shifter <= '0;
                MISO        <= 1'b0;
                //need_read <= 1'b1;
            end
        end
        else begin // CS low, a transaction is active or beginning
            MISO <= out_shifter[DATA_WIDTH - 1]; //msb first
            consumed_during_idle <= 1'b0;
            if(out_counter < DATA_WIDTH - 1) begin //a bit is shifted out no matter what if a transaction is active
                out_shifter <= {out_shifter[DATA_WIDTH - 2:0], 1'b0}; //shift left
                out_counter <= out_counter + 1'd1; //keep track of bits shifted

                // request next byte one cycle before current byte finishes
                if((out_counter == DATA_WIDTH - 2) && !next_tx_valid) begin
                    need_read <= 1'b1;
                end
            end

            else if (out_counter == DATA_WIDTH - 1) begin //if the final bit of the byte
                out_counter <= '0; //reset counter

                // reload directly from holding register if available
                if(next_tx_valid) begin
                    out_shifter   <= next_tx_byte;
                    backup_shifter <= next_tx_byte;
                    next_tx_valid <= 1'b0; //consume buffered byte
                    //need_read <= 1'b1;
                end
                else if(fifo_from_chip_valid) begin
                    out_shifter <= fifo_data_from_chip;
                    backup_shifter <= fifo_data_from_chip;
                    need_read <= 1'b1;
                end
                else begin
                    out_shifter <= '0; //otherwise shift out DATA_WIDTH 0s and then check again
                    backup_shifter <= '0;
                    need_read   <= 1'b1; // request read again to possibly be preloaded before next DATA_WIDTH bits finish shifting
                end
            end
        end
    end
end 

reg_cdc_sram_buffer //cdc and buffering for chip out side
 #(.DATA_WIDTH(DATA_WIDTH),
  .REG_FIFO_LOG2(REG_FIFO_LOG2),
  .FIFO_DEPTH(FIFO_DEPTH)
  )
   cdc_out          // consumer/write side is coming from in chip, producer/read side is headed out over spi
   (.cclk_i(clk_i)
  ,.creset_i(reset_i) //connecting the same chip side reset to both ports, not sure about
  ,.cdata_i(chip_out) //data from chip to be sent out over spi
  ,.cvalid_i(chip_out_valid) //signal comes from inside chip
  ,.cdc_cready_o(chip_out_ready_o) //signal to chip that there is room in output spi fifo, also tied to an overflow signal

  ,.pclk_i(SCLK) //using SPI master clock on the read side
  ,.preset_i(reset_i) //using same reset for everything
  ,.pvalid_o(fifo_from_chip_valid) //valid o signal for read from fifo
  ,.pdata_o(fifo_data_from_chip) //output to master over spi
  ,.pready_i(need_read) //request next byte when local holding register needs filling
  ,.in_fifo_dbg(sram_out_fifo_dbg) //sram fifo debug bus
  ,.sram_ovfl(sram_out_ovfl) //sram fifo overflow signal
  );

//----------CHIP OUT END----------

endmodule