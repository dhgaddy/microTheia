# Testing on ICE40 FPGA
src/rtl/uart/uart_debug.sv
src/rtl/uart/uart_rx.sv
src/rtl/uart/uart_tx.sv

# Shared modules
src/rtl/evt2_decoder.sv
src/rtl/input_fifo.sv

# Voxel Bin Architecture
src/rtl/voxel_bin_architecture/voxel_gesture_classifier.sv
src/rtl/voxel_bin_architecture/voxel_systolic_array.sv
src/rtl/voxel_bin_architecture/voxel_bin_core.sv
src/rtl/voxel_bin_architecture/voxel_bin_top.sv
src/rtl/voxel_bin_architecture/voxel_binning.sv
src/rtl/voxel_bin_architecture/voxel_weight_ram.sv

# Gradient Map Architecture
src/rtl/gradient_map_architecture/gradient_gesture_classifier.sv
src/rtl/gradient_map_architecture/gradient_map_core.sv
src/rtl/gradient_map_architecture/gradient_map_top.sv
src/rtl/gradient_map_architecture/gradient_mapping.sv
src/rtl/gradient_map_architecture/gradient_systolic_array.sv
src/rtl/gradient_map_architecture/gradient_weight_ram.sv

# Include chisel files
-f rtl/chisel/filelist.f