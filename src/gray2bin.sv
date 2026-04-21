// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
module gray2bin
  #(parameter width_p = 5)
   (input [width_p-1 : 0] gray_i
    ,output [width_p-1: 0] bin_o);

    logic[width_p-1:0] b_l;

    always_comb begin
    b_l[width_p-1] = gray_i[width_p-1];
      for(int i = width_p - 2; i >= 0; i-- ) begin
        b_l[i] = b_l[i+1]^gray_i[i];
      end
    end

    assign bin_o = b_l;

endmodule