.PHONY: help lint sim synth clean_synth chisel-test chisel-verilog clean-chisel clean synth-echorun-sorter run-echo

# gf180 tags
.PHONY: all sim-gl clone-pdk librelane librelane-nodrc librelane-klayoutdrc librelane-magicdrc librelane-openroad librelane-klayout librelane-padring sim-gl sim-view copy-final render-image

RTL_DIR := src/rtl
CHISEL_RTL_DIR := src/rtl/chisel-verilog

# GF180 Stuff
MAKEFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

RUN_TAG = $(shell ls librelane/runs/ | tail -n 1)
TOP = chip_top

PDK_ROOT ?= $(MAKEFILE_DIR)/gf180mcu
PDK ?= gf180mcuD
PDK_TAG ?= 1.6.6

AVAILABLE_SLOTS = 1x1 0p5x1 1x0p5 0p5x0p5
DEFAULT_SLOT = 1x1

# Slot can be any of AVAILABLE_SLOTS
SLOT ?= $(DEFAULT_SLOT)

ifeq ($(SLOT),default)        
    SLOT = $(DEFAULT_SLOT)
endif

ifeq ($(filter $(SLOT),$(AVAILABLE_SLOTS)),)
    $(error $(SLOT) does not exist in AVAILABLE_SLOTS: $(AVAILABLE_SLOTS))
endif

.DEFAULT_GOAL := help

# Help, needs to finish
help: ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-20s %s\n", $$1, $$2}'

lint: ## Lint the RTL
	verilator lint.vlt -f $(RTL_DIR)/rtl.f --lint-only --top top

sim: ## Run RTL simulation with cocotb
	cd cocotb; PDK_ROOT=${PDK_ROOT} PDK=${PDK} SLOT=${SLOT} python3 chip_top_tb.py

synth: synth/icestorm_icebreaker/build/icebreaker.bit ## Synthesize for iCEBreaker

synth-echo: synth/icestorm_icebreaker/build/icebreaker_echo.bit ## Build UART echo bitstream (top_uart_echo)

synth/build/rtl.sv2v.v: $(RTL_DIR)/rtl.f
	mkdir -p $(dir $@)
	sv2v $$(cat $(RTL_DIR)/rtl.f) -w $@ -DSYNTHESIS

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

run-sorter: synth/icestorm_icebreaker/build/icebreaker.bit ## Convenience: build + flash sorter
	iceprog synth/icestorm_icebreaker/build/icebreaker.bit

run-echo: synth/icestorm_icebreaker/build/icebreaker_echo.bit ## Convenience: build + flash echo
	iceprog synth/icestorm_icebreaker/build/icebreaker_echo.bit

chisel-test: ## uses the chisel tests, runs on all
	sbt test

chisel-verilog: ## generates the verilog, choose the file
	sbt run

clean_synth: ## removes out all synth files
	rm -rf synth/build synth/icestorm_icebreaker/build

clean_chisel: ## removes all generated verilog files
	rm -rf $(CHISEL_RTL_DIR)/*.sv $(CHISEL_RTL_DIR)/*.v $(CHISEL_RTL_DIR)/filelist.f

clean: clean_synth clean_chisel ## cleans out all generated files
	rm -rf *.log *.rpt 

all: librelane ## Build the project (runs LibreLane)

clone-pdk: ## Clone the GF180MCU PDK repository
	rm -rf $(MAKEFILE_DIR)/gf180mcu
	git clone https://github.com/wafer-space/gf180mcu.git $(MAKEFILE_DIR)/gf180mcu --depth 1 --branch ${PDK_TAG}

librelane: ## Run LibreLane flow (synthesis, PnR, verification)
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk

librelane-nodrc: ## Run LibreLane flow without DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --skip KLayout.Antenna --skip KLayout.DRC --skip Magic.DRC

librelane-klayoutdrc: ## Run LibreLane flow without magic DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --skip Magic.DRC

librelane-magicdrc: ## Run LibreLane flow without KLayout DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --skip KLayout.DRC

librelane-openroad: ## Open the last run in OpenROAD
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --last-run --flow OpenInOpenROAD

librelane-klayout: ## Open the last run in KLayout
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --last-run --flow OpenInKLayout

librelane-padring: ## Only create the padring
	PDK_ROOT=${PDK_ROOT} PDK=${PDK} python3 scripts/padring.py librelane/slots/slot_${SLOT}.yaml librelane/config.yaml

sim-gl: ## Run gate-level simulation with cocotb (after copy-final)
	cd cocotb; GL=1 PDK_ROOT=${PDK_ROOT} PDK=${PDK} SLOT=${SLOT} python3 chip_top_tb.py

sim-view: ## View simulation waveforms in GTKWave
	gtkwave cocotb/sim_build/chip_top.fst

copy-final: ## Copy final output files from the last run
	rm -rf final/
	cp -r librelane/runs/${RUN_TAG}/final/ final/

render-image: ## Render an image from the final layout (after copy-final)
	mkdir -p img/
	PDK_ROOT=${PDK_ROOT} PDK=${PDK} python3 scripts/lay2img.py final/gds/${TOP}.gds img/${TOP}.png --width 2048 --oversampling 4
