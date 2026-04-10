// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

module selectable_debug #()(
    output [31:0] debug_bus,
    input [10:0] classifier_dbg,
    input [14:0] mac_dbg,
    input [30:0] vox_bin_dbg,
    input [11:0] decoder_dbg,
    input [31:0] decoder_output,
    input [3:0] in_fifo_dbg,
    input [15:0] vox_core_debug,
    input [31:0] score_A, score_B, score_C, score_D, fifo_in, fifo_out,
    //input [31:0] control_dbg,
    input [3:0] debug_select
);

//select debug page to connect to 32 bit debug interface
//recieves debug select signal over SPI

/*
PAGE 0 (debug_select = 4'b0000): voxel_gesture_classifier and voxel_mac_engine debug
bit index       description
0               threshold_rd_valid   //start voxel_gesture_classifier
1               threshold_rd_addr[0]
2               threshold_rd_addr[1]
3               class_gesture[0]
4               class_gesture[1]
5               class_valid
6               class_pass
7               gesture[0]
8               gesture[1]
9               gesture_valid
10              gesture_confidence //end voxel_gesture_classifier
11              start              //start voxel_mac_engine
12              busy
13              rd_en
14              scores_valid
15              read_address[0]
16              read_address[0]
17              read_address[0]
18              read_address[0] 
19              read_address[0]
20              read_address[0]
21              read_address[0]
22              read_address[0]
23              read_address[0]
24              read_address[0]
25              tied to ground
26              tied to ground
27              tied to ground
28              tied to ground
29              tied to ground
30              tied to ground
31              tied to ground


PAGE 1 (debug_select = 4'b0001): voxel_binning
bit index       description
0               event_ready
1               readout_start
2               readout_valid
3               readout_last
4               readout_index[0] //which of 2048 voxel cells are being read
5               readout_index[1]
6               readout_index[2]
7               readout_index[3]
8               readout_index[4]
9               readout_index[5]
10              readout_index[6]
11              readout_index[7]
12              readout_index[8]
13              readout_index[9]
14              readout_index[10]
15              readout_data[0] //data being read out and into the mac engine
16              readout_data[1]
17              readout_data[2]
18              readout_data[3]
19              readout_data[4]
20              readout_data[5]
21              readout_data[6]
22              readout_data[7]
23              readout_data[8]
24              readout_data[9]
25              readout_data[10]
26              readout_data[11]
27              readout_data[12]
28              readout_data[13]
29              readout_data[14]
30              readout_data[15]
31              tied to ground

PAGE 2 (debug_select = 4'b0010): evt2_decoder and input_FIFO and voxel_core
bit index       description
0               data_ready         //start evt2_decoder
1               event_valid
2               x_out[0]
3               x_out[1]
4               x_out[2]
5               x_out[3]
6               y_out[0]
7               y_out[2]
8               y_out[2]
9               y_out[3]
10              data_valid 
11              event_ready_i       //end evt2_decoder
12              ready_i             //start input_FIFO
13              valid_i
14              ready_o
15              valid_o             //end input_FIFO
16              debug_event_count[0] //start voxel_bin_core
17              debug_event_count[1]
18              debug_event_count[2]
19              debug_event_count[3]
20              debug_event_count[4]
21              debug_event_count[5]
22              debug_event_count[6]
23              debug_event_count[7]
24              debug_fifo_empty
25              debug_fifo_full
26              debug_temporal_phase
27              debug_class_valid
28              debug_class_pass
29              debug_feature_window_ready
30              debug_capture_active
31              debug_score_busy

PAGE 3 (debug_select = 4'b0011): reserved for control module debug signals
bit index       description
0               
1               
2              
3          
4              
5              
6              
7               
8             
9               
10              
11              
12             
13              
14              
15              
16              
17              
18              
19              
20              
21              
22              
23              
24              
25              
26              
27              
28              
29              
30              
31              

PAGE 4 (debug_select = 4'b0100): evt2_decoder event output
bit index       description
0               decoder_out[0]
1               decoder_out[1]
2               decoder_out[2]
3               decoder_out[3]
4               decoder_out[4]
5               decoder_out[5]
6               decoder_out[6]
7               decoder_out[7]
8               decoder_out[8]
9               decoder_out[9]
10              decoder_out[10]
11              decoder_out[11]    
12              decoder_out[12]       
13              decoder_out[13]
14              decoder_out[14]
15              decoder_out[15]
16              decoder_out[16]
17              decoder_out[17]
18              decoder_out[18]
19              decoder_out[19]
20              decoder_out[20]
21              decoder_out[21]
22              decoder_out[22]
23              decoder_out[23]
24              decoder_out[24]
25              decoder_out[25]
26              decoder_out[26]
27              decoder_out[27]
28              decoder_out[28]
29              decoder_out[29]
30              decoder_out[30]
31              decoder_out[31]

PAGE 5 (debug_select = 4'b0101): input to input_FIFO
[31:0] = [31:0] input_FIFO_in

PAGE 6 (debug_select = 4'b0110): input_FIFO output
[31:0] = [31:0] input_FIFO_out

PAGE 7 (debug_select = 4'b0111): class score_A
[31:0] = [31:0] score_A

PAGE 8 (debug_select = 4'b1000): class score_B
[31:0] = [31:0] score_B

PAGE 9 (debug_select = 4'b1001): class score_C
[31:0] = [31:0] score_C

PAGE 10 (debug_select = 4'b1010): class score_D
[31:0] = [31:0] score_D
*/

localparam PAGE_0 = 4'b0000;
localparam PAGE_1 = 4'b0001;
localparam PAGE_2 = 4'b0010;
localparam PAGE_3 = 4'b0011;
localparam PAGE_4 = 4'b0100;
localparam PAGE_5 = 4'b0101;
localparam PAGE_6 = 4'b0110;
localparam PAGE_7 = 4'b0111;
localparam PAGE_8 = 4'b1000;
localparam PAGE_9 = 4'b1001;
localparam PAGE_10 = 4'b1010;

logic [31:0] selected_debug;
always_comb begin
    unique case (debug_select)
            PAGE_0: begin
                selected_debug = {6'b0, mac_dbg , classifier_dbg};
            end
            PAGE_1: begin
                selected_debug = {1'b0, vox_bin_dbg};
            end
            PAGE_2: begin
                selected_debug = {vox_core_debug, in_fifo_dbg, decoder_dbg};
            end
            /*PAGE_3: begin
                debug_bus = control_dbg
            end
            */
            PAGE_4: begin
                selected_debug = decoder_output;
            end
            PAGE_5: begin
                selected_debug = fifo_in;
            end
            PAGE_6: begin
                selected_debug = fifo_out;
            end
            PAGE_7: begin
                selected_debug = score_A;
            end
            PAGE_8: begin
                selected_debug = score_B;
            end
            PAGE_9: begin
                selected_debug = score_C;
            end
            PAGE_10: begin
                selected_debug = score_D;
            end
            default: selected_debug = {7'b0, mac_dbg , classifier_dbg};
    endcase
end

assign debug_bus = selected_debug;

endmodule