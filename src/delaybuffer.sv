module delaybuffer
  #(parameter [31:0] width_p = 8
   ,parameter [31:0] delay_p = 8
   )
  (input [0:0] clk_i
  ,input [0:0] reset_i

  ,input [width_p - 1:0] data_i
  ,input logic [0:0] valid_i
  ,output logic  [0:0] ready_o 

  ,output logic [0:0] valid_o 
  ,output logic [width_p - 1:0] data_o 
  ,input logic [0:0] ready_i
  );
    wire transfer_signal = valid_i && ready_o;
    logic [width_p-1:0] datapath [delay_p-1:0];
    always_ff @(posedge clk_i) begin
      if (reset_i) begin
        valid_o <= 1'b0;
        data_o  <= 'x; 
        for (int i = 0; i < delay_p; i++) begin
           datapath[i] <= 'x;
        end   
      end else begin
        if (transfer_signal) begin
          datapath[0] <= data_i;
          for (int i = 1; i < delay_p; i++) begin
            datapath[i] <= datapath[i-1];
          end
        end
        if (ready_o) begin
          valid_o <= valid_i;
          data_o  <= datapath[delay_p-1];
        end

      end
    end
    assign ready_o = ~valid_o | ready_i;

endmodule