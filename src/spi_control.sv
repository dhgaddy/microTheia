//SPI module, type 0 only 
// type 0 means:
// bit shifts out on negedge of SCLK (for both MISO and MOSI)
// bit sampled on posedge of SCLK (again for both)

// intended fixed input structure: 8 bit command possibly followed by 32 bit payload? (parsing in seperate module)

//handling CDC within this module

module spi_type_0_s #()(
    input  logic [0:0] SCLK, //clock from master
    input  logic [0:0] MOSI, //master out slave in
    input  [0:0] CS, // chip select (or something similar, active low transaction enable signal)
    input  [0:0] clk_i, //clock from chip
    input  [0:0] reset_i, //reset from chip
    input  [7:0] chip_out, //data from chip headed towards master, enters module in chip clock domain
    input  [0:0] chip_out_valid, //valid signal for data from chip headed towards master
    output [0:0] chip_out_ready_o, //ready_o for the output side fifo (as in this signal goes into the chip)
    output logic [0:0] MISO, //master in slave out
    output [7:0] chip_in, //data from master that has been byte packed and crossed into chip clock domain, headed further into chip
    output [0:0] chip_in_valid, //valid signal for data from master headed into chip
    output [0:0] in_ovfl, out_ovfl //overflow flag for cdc input/output fifos, if its ready_o ever goes low then flag is set high and stays high until reset
);

// ----------CHIP IN SECTION---------
logic [7:0] byte_packer, packed_byte;
logic [3:0] packer_count; //tracking how many bits have been packed
logic [0:0] valid_byte; //data exchange signal
logic cready_o_wire; // connect to overflow flag
logic in_fifo_valid_o; //intermediary signal for chip_in_valid
logic [7:0] fifo_data_to_chip; //intermediary signal for chip_in

always_ff @(posedge SCLK) begin
    if(reset_i) begin
        packer_count <= 4'd0;
        packed_byte <= 8'd0;
        byte_packer <= 8'd0;
        valid_byte <= 1'b0;
        in_ovfl <= 1'b0;
    end
    else begin
        if(CS) begin //active low signal for transaction, so CS high is basically a reset
            packer_count <= 4'd0;
            byte_packer <= 8'd0;
            valid_byte <= 1'b0;
        end    
        else if(~CS) begin //CS active low transaction begins
            if(packer_count < 4'd7) begin //if less than 8 bits packed
                byte_packer <= {byte_packer [6:0], MOSI}; //pack the serial bits MSB first
                packer_count <= packer_count + 4'd1; //keep track of bits packed
                valid_byte <= 1'b0; //reset valid byte signal
            end
            else if(packer_count == 4'd7) begin //if 8 bits have been received
                packed_byte <= {byte_packer [6:0], MOSI}; //extract byte while capturing final bit
                byte_packer <= 8'd0; //start over packing bits
                valid_byte <= 1'b1; //set valid byte high, signal to be written into cdc fifo
                packer_count <= 4'd0; //reset count
            end    
        end

        if(~cready_o_wire) begin //if unable to accept more writes trigger overflow flag
            in_ovfl <= 1'b1;
        end    
    end   

end    



fifo_1r1w_cdc //my cdc module from CSE 225, needs to have internal ram swapped out for gf180 macros
 #(.width_p(8),
  .depth_log2_p(9)
  )
   cdc_in          // "c" consumer (write side), and "p" for producer (read side). 
   (.cclk_i(SCLK)
  ,.creset_i(reset_i) //connecting the same chip side reset to both ports, not sure about
  ,.cdata_i(packed_byte) //writing in packed bytes
  ,.cvalid_i(valid_byte) //using valid byte signal as write trigger
  ,.cready_o(cready_o_wire) //tieing to overflow signal

  .pclk_i(clk_i),
  .preset_i(reset_i),
  .pvalid_o(in_fifo_valid_o), //valid o signal for module output
  .pdata_o(fifo_data_to_chip), //output to a command/payload parser (different module)
  .pready_i(1'b1) //tieing to 1, just keeps it pumping
  );


assign chip_in = fifo_data_to_chip;
assign chip_in_valid = in_fifo_valid_o;

//----------CHIP IN END-----------

//----------CHIP OUT BEGIN--------
//this side is a little more complicated because the bit on MISO must awlays be valid the very first posedge of SCLK after CS goes low

logic [7:0] out_shifter;
logic [3:0] out_counter;
logic [7:0] fifo_data_from_chip;
logic [0:0] fifo_from_chip_valid;
logic [0:0] need_read;

// one-byte holding register for the next byte to send, in between fifo and shifter, to be "always ready"
logic [7:0] next_tx_byte;

//flag indicating next_tx_byte currently holds a valid unused byte
logic [0:0] next_tx_valid;


always_ff @(negedge SCLK) begin
    if(reset_i) begin
        out_shifter   <= 8'd0;
        out_counter   <= 4'd0;
        next_tx_byte  <= 8'd0;
        next_tx_valid <= 1'b0;
        need_read     <= 1'b0;
        MISO          <= 1'b0;
        out_ovfl      <= 1'b0;
    end
    else begin
        //default is no read unless something calls for it
        need_read <= 1'b0;
        //if fifo has a valid byte and staging register isnt full then grab it and set valid flag
        if(fifo_from_chip_valid && !next_tx_valid) begin
            next_tx_byte  <= fifo_data_from_chip;
            next_tx_valid <= 1'b1;
        end
        if(~chip_out_ready_o) begin
            out_ovfl <= 1'b1; //overflow flag for output fifo
        end

        if(CS)begin // cs high means not a transaction, but the out_shifter needs to stay loaded and ready
            out_counter <= 4'd0;
            if(!next_tx_valid) begin // if no byte is buffered already ask fifo for one
                need_read <= 1'b1;
            end

            // preload shifter while idle so first bit is already valid before first posedge after CS goes low
            if(next_tx_valid) begin
                out_shifter <= next_tx_byte;
                MISO        <= next_tx_byte[7];
            end
            else begin //if nothing buffered and fifo has nothing then 0 is valid first bit
                out_shifter <= 8'd0;
                MISO        <= 1'b0;
            end
        end
        else begin // CS low, a transaction is active or beginning
            MISO <= out_shifter[7]; //msb first

            if(out_counter < 7) begin //a bit is shifted out no matter what if a transaction is active
                out_shifter <= {out_shifter[6:0], 1'b0}; //shift left
                out_counter <= out_counter + 4'd1; //keep track of bits shifted

                // request next byte one cycle before current byte finishes
                if((out_counter == 4'd6) && !next_tx_valid) begin
                    need_read <= 1'b1;
                end
            end

            else if (out_counter == 7) begin //if the final bit of the byte
                out_counter <= 4'd0; //reset counter

                // reload directly from holding register if available
                if(next_tx_valid) begin
                    out_shifter   <= next_tx_byte;
                    next_tx_valid <= 1'b0; //consume buffered byte
                end
                else if(fifo_from_chip_valid) begin
                    out_shifter <= fifo_data_from_chip;
                end
                else begin
                    out_shifter <= 8'd0; //otherwise shift out 8 0s and then check again
                    need_read   <= 1'b1; // request read again to possibly be preloaded before next 8 bits finish shifting
                end
            end
        end
    end
end 

fifo_1r1w_cdc //my cdc module from CSE 225, needs to have internal ram swapped out for gf180 macros
 #(.width_p(8),
  .depth_log2_p(9)
  )
   cdc_out          // consumer/write side is coming from in chip, producer/read side is headed out over spi
   (.cclk_i(clk_i)
  ,.creset_i(reset_i) //connecting the same chip side reset to both ports, not sure about
  ,.cdata_i(chip_out) //data from chip to be sent out over spi
  ,.cvalid_i(chip_out_valid) //signal comes from inside chip
  ,.cready_o(chip_out_ready_o) //signal to chip that there is room in output spi fifo, also tied to an overflow signal

  .pclk_i(SCLK), //using SPI master clock on the read side
  ,.preset_i(reset_i), //using same reset for everything
  ,.pvalid_o(fifo_from_chip_valid), //valid o signal for read from fifo
  ,.pdata_o(fifo_data_from_chip), //output to master over spi
  ,.pready_i(need_read) //request next byte when local holding register needs filling
  );

//----------CHIP OUT END----------

endmodule