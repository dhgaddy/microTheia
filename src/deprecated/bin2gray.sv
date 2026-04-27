// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
module bin2gray
  #(parameter width_p = 5)
  (input [width_p - 1 : 0] bin_i
  ,output [width_p - 1: 0] gray_o);

   logic [width_p-1:0] g_l;
   
   always_comb begin
    for(int i = 0; i < width_p - 1; i ++) begin
      g_l[i] = bin_i[i]^bin_i[i+1];
    end
    g_l[width_p-1] = bin_i[width_p-1];
   end

   assign gray_o = g_l;    

endmodule