# Testing on ICE40 FPGA
uart_debug.sv
uart_rx.sv
uart_tx.sv

# Voxel Bin Architecture
src/rtl/voxel_bin_architecture/dvs_gesture_accel.sv
src/rtl/voxel_bin_architecture/GestureClassifier.sv
src/rtl/voxel_bin_architecture/InputFIFO.sv
src/rtl/voxel_bin_architecture/MotionComputer.sv
src/rtl/voxel_bin_architecture/OutputRegister.sv
src/rtl/voxel_bin_architecture/SpatialCompressor.sv
src/rtl/voxel_bin_architecture/SystolicMatrixMultiply.sv
src/rtl/voxel_bin_architecture/TemporalAccumulator.sv
src/rtl/voxel_bin_architecture/TimeSurfaceBinning.sv
src/rtl/voxel_bin_architecture/voxel_bin_processed_top.sv
src/rtl/voxel_bin_architecture/voxel_bin_raw_top.sv
src/rtl/voxel_bin_architecture/WeightROM.sv

# Gradient Map Architecture
src/rtl/gradient_map_architecture/evt2_decoder.sv
src/rtl/gradient_map_architecture/flatten_buffer.sv
src/rtl/gradient_map_architecture/gradient_map_processed_top.sv
src/rtl/gradient_map_architecture/gradient_map_raw_top.sv
src/rtl/gradient_map_architecture/input_fifo.sv
src/rtl/gradient_map_architecture/spatio_temporal_classifier.sv
src/rtl/gradient_map_architecture/systolic_array.sv
src/rtl/gradient_map_architecture/time_surface_encoder.sv
src/rtl/gradient_map_architecture/time_surface_memory.sv
src/rtl/gradient_map_architecture/weight_rom.sv

# Include chisel files
-f rtl/chisel/filelist.f