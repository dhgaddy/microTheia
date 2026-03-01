# Testing on ICE40 FPGA
uart_debug.sv
uart_rx.sv
uart_tx.sv

# Voxel Bin Architecture
src/rtl/voxel_bin_architecture/evt2_decoder.sv
src/rtl/voxel_bin_architecture/gesture_classifier.sv
src/rtl/voxel_bin_architecture/input_fifo.sv
src/rtl/voxel_bin_architecture/systolic_array.sv
src/rtl/voxel_bin_architecture/voxel_bin_core.sv
src/rtl/voxel_bin_architecture/voxel_bin_top.sv
src/rtl/voxel_bin_architecture/voxel_binning.sv
src/rtl/voxel_bin_architecture/weight_ram.sv

# Gradient Map Architecture
src/rtl/gradient_map_architecture/evt2_decoder.sv
src/rtl/gradient_map_architecture/gesture_classifier.sv
src/rtl/gradient_map_architecture/gradient_map_core.sv
src/rtl/gradient_map_architecture/gradient_map_top.sv
src/rtl/gradient_map_architecture/gradient_mapping.sv
src/rtl/gradient_map_architecture/input_fifo.sv
src/rtl/gradient_map_architecture/systolic_array.sv
src/rtl/gradient_map_architecture/weight_ram.sv

# Include chisel files
-f rtl/chisel/filelist.f