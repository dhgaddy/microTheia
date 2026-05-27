current_design $::env(DESIGN_NAME)
set_units -time ns

set clock_port __VIRTUAL_CLK__
if { [info exists ::env(CLOCK_PORT)] } {
    set port_count [llength $::env(CLOCK_PORT)]

    if { $port_count == "0" } {
        puts "\[WARNING] No CLOCK_PORT found. A dummy clock will be used."
    } elseif { $port_count != "1" } {
        puts "\[WARNING] Multi-clock files are not currently supported by the base SDC file. Only the first clock will be constrained."
    }

    if { $port_count > "0" } {
        set ::clock_port [lindex $::env(CLOCK_PORT) 0]
    }
}

if { $::env(CLOCK_PORT) == $::env(CLOCK_NET) } {
    set port_args [get_ports $clock_port]
} else {
    # This should actually use CLOCK_PIN?
    set port_args [get_pins [lindex $::env(CLOCK_NET) 0]]
}

puts "\[INFO] Using clock $clock_port…"
create_clock {*}$port_args -name $clock_port -period $::env(CLOCK_PERIOD)

set input_delay_value [expr $::env(CLOCK_PERIOD) * $::env(IO_DELAY_CONSTRAINT) / 100]
set output_delay_value [expr $::env(CLOCK_PERIOD) * $::env(IO_DELAY_CONSTRAINT) / 100]
puts "\[INFO] Setting output delay to: $output_delay_value"
puts "\[INFO] Setting input delay to: $input_delay_value"

set_max_fanout $::env(MAX_FANOUT_CONSTRAINT) [current_design]
if { [info exists ::env(MAX_TRANSITION_CONSTRAINT)] } {
    set_max_transition $::env(MAX_TRANSITION_CONSTRAINT) [current_design]
}
if { [info exists ::env(MAX_CAPACITANCE_CONSTRAINT)] } {
    set_max_capacitance $::env(MAX_CAPACITANCE_CONSTRAINT) [current_design]
}

set clocks [get_clocks $clock_port]

# Bidirectional pads
set clk_core_inout_ports [get_ports {
    bidir_PAD[*]
}]

set_input_delay -min 0 -clock $clocks $clk_core_inout_ports
set_input_delay -max $input_delay_value -clock $clocks $clk_core_inout_ports
set_output_delay $output_delay_value -clock $clocks $clk_core_inout_ports

# Input-only pads
set clk_core_input_ports [get_ports {
    rst_n_PAD
    input_PAD[*]
}]

set_input_delay -min 0 -clock $clocks $clk_core_input_ports
set_input_delay -max $input_delay_value -clock $clocks $clk_core_input_ports

# Output load
set cap_load [expr $::env(OUTPUT_CAP_LOAD) / 1000.0]
puts "\[INFO] Setting load to: $cap_load"
set_load $cap_load [all_outputs]

puts "\[INFO] Setting clock uncertainty to: $::env(CLOCK_UNCERTAINTY_CONSTRAINT)"
set_clock_uncertainty $::env(CLOCK_UNCERTAINTY_CONSTRAINT) $clocks

puts "\[INFO] Setting clock transition to: $::env(CLOCK_TRANSITION_CONSTRAINT)"
set_clock_transition $::env(CLOCK_TRANSITION_CONSTRAINT) $clocks

puts "\[INFO] Setting timing derate to: $::env(TIME_DERATING_CONSTRAINT)%"
set_timing_derate -early [expr 1-[expr $::env(TIME_DERATING_CONSTRAINT) / 100]]
set_timing_derate -late [expr 1+[expr $::env(TIME_DERATING_CONSTRAINT) / 100]]

if { [info exists ::env(OPENLANE_SDC_IDEAL_CLOCKS)] && $::env(OPENLANE_SDC_IDEAL_CLOCKS) } {
    unset_propagated_clock [all_clocks]
} else {
    set_propagated_clock [all_clocks]
}

# SPI slave clock — 16 MHz, asynchronous to core clock.
# SCLK is sampled by single-flop edge detectors clocked by the 64 MHz core
# clock (4:1 ratio gives 2 core cycles per SCLK half-period).
# Default pin mapping (alt_select=0): SCLK=input_PAD[5], MOSI=input_PAD[6],
# CS=input_PAD[7], MISO=bidir_PAD[38].
create_clock -period 62.5 -name SCLK [get_ports {input_PAD[5]}]

# Declare the two clock domains asynchronous so the tool does not attempt
# cross-domain timing analysis between them.
set_clock_groups -asynchronous \
    -group [get_clocks $clock_port] \
    -group [get_clocks SCLK]

# SPI master drives MOSI/CS stable before the SCLK edge (SPI mode 0).
# Allow up to 5 ns setup margin at the pad.
set_input_delay -clock SCLK -max 5.0 [get_ports {input_PAD[6]}]
set_input_delay -clock SCLK -min 0   [get_ports {input_PAD[6]}]
set_input_delay -clock SCLK -max 5.0 [get_ports {input_PAD[7]}]
set_input_delay -clock SCLK -min 0   [get_ports {input_PAD[7]}]

# MISO must be valid before the next SCLK edge the master samples on.
# Allow up to 10 ns output delay (half SCLK period = 31.25 ns leaves margin).
set_output_delay -clock SCLK -max 10.0 [get_ports {bidir_PAD[38]}]
set_output_delay -clock SCLK -min 0    [get_ports {bidir_PAD[38]}]
