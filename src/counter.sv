module counter
  #(parameter width_p = 4,
    /* verilator lint_off WIDTHTRUNC */
    parameter [width_p-1:0] reset_val_p = '0)
    /* verilator lint_on WIDTHTRUNC */
   (input [0:0] clk_i
   ,input [0:0] reset_i
   ,input [0:0] up_i
   ,input [0:0] down_i
   ,output [width_p-1:0] count_o
   ,output [width_p - 1:0] count_fast);

   // Your code here:

   logic [width_p-1:0] q_l, d_l, count_next_w;

   
  always_ff @(posedge clk_i) begin
    if(reset_i) begin
      q_l <= reset_val_p;
    end else begin
      q_l <= d_l;
    end
  end

  always_comb begin
    case ({up_i, down_i})
      2'b01: d_l = q_l -1'b1;
      2'b10: d_l = q_l +1'b1;
      default: d_l = q_l;
    endcase 
    
    count_next_w = d_l;

  end

  assign count_o = q_l; 
  assign count_fast = count_next_w;            
  
       
endmodule