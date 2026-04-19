module bin2gray
  #(parameter width_p = 5)
   // You must fill these in with width_p
  (input [width_p - 1 : 0] bin_i
  ,output [width_p - 1: 0] gray_o);

   // Your code here

   logic [width_p-1:0] g_l;
   
   always_comb begin
    for(int i = 0; i < width_p - 1; i ++) begin
      g_l[i] = bin_i[i]^bin_i[i+1];
    end
    g_l[width_p-1] = bin_i[width_p-1];
   end

   assign gray_o = g_l;    

endmodule