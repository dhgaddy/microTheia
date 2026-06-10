# SRAM macros

# Dump all block (macro) instance names to macro_instances.txt in the run
# directory so the exact elaborated names (including [] from generate loops)
# are visible for PDN pattern debugging.
set macro_instances_file [file join [pwd] "macro_instances.txt"]
set fp [open $macro_instances_file w]
foreach inst [[ord::get_db_block] getInsts] {
    if { [$inst isBlock] } {
        puts $fp "[$inst getName]"
    }
}
close $fp

# Weight SRAMs (col0, x=500) — 8 x sram1024x8
define_pdn_grid \
    -macro \
    -instances i_chip_core.u_soc.u_core.gen_weight_ram \
    -name sram_weight \
    -starts_with POWER \
    -halo "$::env(PDN_HORIZONTAL_HALO) $::env(PDN_VERTICAL_HALO)"

add_pdn_connect \
    -grid sram_weight \
    -layers "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"

add_pdn_connect \
    -grid sram_weight \
    -layers "$::env(PDN_VERTICAL_LAYER) Metal3"

# Feature RAM (col1, x=2800) — 4 x sram1024x8
define_pdn_grid \
    -macro \
    -instances i_chip_core.u_soc.u_core.u_feature_ram \
    -name sram_feature \
    -starts_with POWER \
    -halo "$::env(PDN_HORIZONTAL_HALO) $::env(PDN_VERTICAL_HALO)"

add_pdn_connect \
    -grid sram_feature \
    -layers "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"

add_pdn_connect \
    -grid sram_feature \
    -layers "$::env(PDN_VERTICAL_LAYER) Metal3"

# Counter mem (col1 continued) — 4 x sram1024x8
define_pdn_grid \
    -macro \
    -instances i_chip_core.u_soc.u_core.u_voxel_binning.u_counter_mem \
    -name sram_counter \
    -starts_with POWER \
    -halo "$::env(PDN_HORIZONTAL_HALO) $::env(PDN_VERTICAL_HALO)"

add_pdn_connect \
    -grid sram_counter \
    -layers "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"

add_pdn_connect \
    -grid sram_counter \
    -layers "$::env(PDN_VERTICAL_LAYER) Metal3"

# Input FIFO (4 x sram256x8)
define_pdn_grid \
    -macro \
    -instances i_chip_core.u_soc.u_core.u_input_fifo.u_fifo_mem \
    -name sram_fifo \
    -starts_with POWER \
    -halo "$::env(PDN_HORIZONTAL_HALO) $::env(PDN_VERTICAL_HALO)"

add_pdn_connect \
    -grid sram_fifo \
    -layers "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"

add_pdn_connect \
    -grid sram_fifo \
    -layers "$::env(PDN_VERTICAL_LAYER) Metal3"

# Threshold RAM (5 x sram256x8)
define_pdn_grid \
    -macro \
    -instances "i_chip_core.u_soc.u_core.u_thresh_ram" \
    -name sram_thresh \
    -starts_with POWER \
    -halo "$::env(PDN_HORIZONTAL_HALO) $::env(PDN_VERTICAL_HALO)"

add_pdn_connect \
    -grid sram_thresh \
    -layers "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"

add_pdn_connect \
    -grid sram_thresh \
    -layers "$::env(PDN_VERTICAL_LAYER) Metal3"




# # Add stripes on W/E edges of SRAM
# add_pdn_stripe \
#     -grid sram_macros_NS \
#     -layer Metal4 \
#     -width 1.36 \
#     -offset 0.68 \
#     -spacing 0.28 \
#     -pitch 298.30 \
#     -starts_with GROUND \
#     -number_of_straps 2

# # Since the above stripes block the top level PDN at Metal4, add some more stripes
# # to improve the PDN's integrity and ensure a better connection for the macro.
# add_pdn_stripe \
#     -grid sram_macros_NS \
#     -layer Metal4 \
#     -width 4.00 \
#     -offset 50.80 \
#     -spacing 0.28 \
#     -pitch 48.86 \
#     -starts_with GROUND \
#     -number_of_straps 5

# # Add stripes on W/E edges of SRAM
# add_pdn_stripe \
#     -grid sram_macros_WE \
#     -layer Metal4 \
#     -width 1.36 \
#     -offset 0.68 \
#     -spacing 0.28 \
#     -pitch 319.09 \
#     -starts_with POWER \
#     -number_of_straps 2

# # Since the above stripes block the top level PDN at Metal4, add some more stripes
# # to improve the PDN's integrity and ensure a better connection for the macro.
# add_pdn_stripe \
#     -grid sram_macros_WE \
#     -layer Metal4 \
#     -width 4.00 \
#     -offset 28.0 \
#     -spacing 0.28 \
#     -pitch 43.50 \
#     -starts_with GROUND \
#     -number_of_straps 7
