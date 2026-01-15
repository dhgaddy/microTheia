.PHONY: lint synth clean_synth clean

# Lint the RTL
lint:
	verilator lint.vlt -f rtl/rtl.f --lint-only --top top

# Synthesize for iCEBreaker
synth: synth/icestorm_icebreaker/build/icebreaker.bit

# Build UART echo bitstream (top_uart_echo)
.PHONY: synth-echo
synth-echo: synth/icestorm_icebreaker/build/icebreaker_echo.bit

synth/build/rtl.sv2v.v: rtl/rtl.f
	mkdir -p $(dir $@)
	sv2v $$(cat rtl/rtl.f) -w $@ -DSYNTHESIS

synth/icestorm_icebreaker/build/synth.json: synth/build/rtl.sv2v.v synth/icestorm_icebreaker/yosys.tcl
	mkdir -p $(dir $@)
	yosys -p 'tcl synth/icestorm_icebreaker/yosys.tcl' -l synth/icestorm_icebreaker/build/yosys.log

synth/icestorm_icebreaker/build/icebreaker.asc: synth/icestorm_icebreaker/build/synth.json synth/icestorm_icebreaker/icebreaker.pcf
	nextpnr-ice40 \
	  --json synth/icestorm_icebreaker/build/synth.json \
	  --up5k \
	  --package sg48 \
	  --pcf synth/icestorm_icebreaker/icebreaker.pcf \
	  --asc synth/icestorm_icebreaker/build/icebreaker.asc

synth/icestorm_icebreaker/build/icebreaker.bit: synth/icestorm_icebreaker/build/icebreaker.asc
	icepack synth/icestorm_icebreaker/build/icebreaker.asc synth/icestorm_icebreaker/build/icebreaker.bit

# Echo flow
synth/icestorm_icebreaker/build/synth_echo.json: synth/build/rtl.sv2v.v synth/icestorm_icebreaker/yosys_echo.tcl
	mkdir -p $(dir $@)
	yosys -p 'tcl synth/icestorm_icebreaker/yosys_echo.tcl' -l synth/icestorm_icebreaker/build/yosys_echo.log

synth/icestorm_icebreaker/build/icebreaker_echo.asc: synth/icestorm_icebreaker/build/synth_echo.json synth/icestorm_icebreaker/icebreaker.pcf
	nextpnr-ice40 \
	  --json synth/icestorm_icebreaker/build/synth_echo.json \
	  --up5k \
	  --package sg48 \
	  --pcf synth/icestorm_icebreaker/icebreaker.pcf \
	  --asc synth/icestorm_icebreaker/build/icebreaker_echo.asc

synth/icestorm_icebreaker/build/icebreaker_echo.bit: synth/icestorm_icebreaker/build/icebreaker_echo.asc
	icepack synth/icestorm_icebreaker/build/icebreaker_echo.asc synth/icestorm_icebreaker/build/icebreaker_echo.bit

# Convenience: build + flash sorter
.PHONY: run-sorter
run-sorter: synth/icestorm_icebreaker/build/icebreaker.bit
	iceprog synth/icestorm_icebreaker/build/icebreaker.bit

# Convenience: build + flash echo
.PHONY: run-echo
run-echo: synth/icestorm_icebreaker/build/icebreaker_echo.bit
	iceprog synth/icestorm_icebreaker/build/icebreaker_echo.bit

clean_synth:
	rm -rf synth/build synth/icestorm_icebreaker/build

clean: clean_synth
	rm -rf *.log *.rpt