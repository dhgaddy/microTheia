# sources
src/chip_core.sv
src/control_fsm.sv
# src/chip_top.sv  -- requires IO pad models; use 'make sim-chip-top' instead
src/evt2_decoder.sv
src/gf180_sram_1r1w.sv
src/input_fifo.sv
src/selectable_debug.sv
# src/slot_defines.svh  -- header only, included by chip_top.sv
src/spi_wrapper.sv
src/soc.sv
src/voxel_bin_core.sv
src/voxel_binning.sv
src/voxel_gesture_classifier.sv
src/voxel_mac_engine.sv

# third party
third_party/verilog_spi/spi_module.v
third_party/verilog_spi/pos_edge_det.v
third_party/verilog_spi/neg_edge_det.v