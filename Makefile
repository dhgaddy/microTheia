MAKEFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

RUN_TAG = $(shell ls librelane/runs/ | tail -n 1)
TOP = voxel_bin_top

PDK_ROOT ?= $(MAKEFILE_DIR)/dependencies/pdks
PDK ?= gf180mcuD
PDK_TAG ?= 1.8.0
SCL ?= gf180mcu_as_sc_mcu7t3v3
AVALON_REPO ?= https://github.com/AvalonSemiconductors/gf180mcu_as_sc_mcu7t3v3.git

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

SIM_DUTS = $(strip $(DUT))

SV_SRCS := $(shell find src -name "*.sv")

help: ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@grep -E '^[a-zA-Z0-9_-]+:[^#]*## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":[^#]*## "}; {printf "  %-20s %s\n", $$1, $$2}'
.PHONY: help

# Simulation configuration
CONFIG ?= voxel_default
CONFIG_FILE := configs/$(CONFIG).txt

# iCE40 FPGA Flow Wrapper
ICE40_MAKEFILE := ice40/ice40.mk

all: librelane ## Build the project (runs LibreLane)
.PHONY: all

clone-pdk: ## Clone the GF180MCU base PDK into dependencies/pdks/gf180mcuD/
	mkdir -p $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD
	git clone https://github.com/wafer-space/gf180mcu.git $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD --depth 1 --branch ${PDK_TAG} --no-local 2>/dev/null || \
		(cd $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD && git fetch --depth 1 origin tag ${PDK_TAG} && git checkout ${PDK_TAG})
.PHONY: clone-pdk

OCD_SRAM_REPO ?= https://github.com/RTimothyEdwards/gf180mcu_ocd_ip_sram.git

clone-ocd-sram: ## Clone 3.3V OCD SRAM macros and install into dependencies/pdks/gf180mcuD/libs.ref/gf180mcu_ocd_ip_sram/
	@if [ ! -d $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD/libs.ref ]; then \
		echo "ERROR: Run 'make clone-pdk' first."; exit 1; \
	fi
	$(eval OCD_TMP := $(shell mktemp -d))
	git clone $(OCD_SRAM_REPO) $(OCD_TMP) --depth 1
	$(eval OCD_DST := $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD/libs.ref/gf180mcu_ocd_ip_sram)
	mkdir -p $(OCD_DST)/gds $(OCD_DST)/lef $(OCD_DST)/lib $(OCD_DST)/verilog $(OCD_DST)/spice
	for cell in sram256x8m8wm1 sram512x8m8wm1 sram1024x8m8wm1; do \
		src=$(OCD_TMP)/cells/gf180mcu_ocd_ip_sram__$$cell; \
		cp $$src/gf180mcu_ocd_ip_sram__$$cell.gds      $(OCD_DST)/gds/; \
		cp $$src/gf180mcu_ocd_ip_sram__$$cell.lef      $(OCD_DST)/lef/; \
		cp $$src/gf180mcu_ocd_ip_sram__$$cell.blackbox.v $(OCD_DST)/verilog/; \
		cp $$src/gf180mcu_ocd_ip_sram__$$cell.spice    $(OCD_DST)/spice/; \
		cp $$src/gf180mcu_ocd_ip_sram__$$cell__*.lib   $(OCD_DST)/lib/; \
	done
	rm -rf $(OCD_TMP)
	@echo "OCD 3.3V SRAMs installed at $(OCD_DST)"
.PHONY: clone-ocd-sram

clone-avalon-pdk: ## Merge Avalon 3.3V std cell library into dependencies/pdks/gf180mcuD/
	@if [ ! -d $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD/libs.ref ]; then \
		echo "ERROR: Run 'make clone-pdk' first to fetch the base GF180MCU PDK."; exit 1; \
	fi
	$(eval AVALON_TMP := $(shell mktemp -d))
	git clone $(AVALON_REPO) $(AVALON_TMP) --depth 1
	cp -r $(AVALON_TMP)/pdk/libs.ref/. $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD/libs.ref/
	cp -r $(AVALON_TMP)/pdk/libs.tech/. $(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD/libs.tech/
	rm -rf $(AVALON_TMP)
	cp $(MAKEFILE_DIR)/librelane/gf180mcu_as_sc_mcu7t3v3_config.tcl \
		$(MAKEFILE_DIR)/dependencies/pdks/gf180mcuD/libs.tech/librelane/gf180mcu_as_sc_mcu7t3v3/config.tcl
	@echo "Avalon 3.3V library merged into dependencies/pdks/gf180mcuD/"
.PHONY: clone-avalon-pdk

librelane: ## Run LibreLane flow (synthesis, PnR, verification)
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk
.PHONY: librelane

librelane-nodrc: ## Run LibreLane flow without DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --skip KLayout.Antenna --skip KLayout.DRC --skip Magic.DRC
.PHONY: librelane-nodrc

librelane-klayoutdrc: ## Run LibreLane flow without magic DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --skip Magic.DRC
.PHONY: librelane-klayoutdrc

librelane-magicdrc: ## Run LibreLane flow without KLayout DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --skip KLayout.DRC
.PHONY: librelane-magicdrc

librelane-openroad: ## Open the last run in OpenROAD
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --last-run --flow OpenInOpenROAD
.PHONY: librelane-openroad

librelane-klayout: ## Open the last run in KLayout
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --last-run --flow OpenInKLayout
.PHONY: librelane-klayout

librelane-padring: ## Only create the padring
	PDK_ROOT=${PDK_ROOT} PDK=${PDK} python3 scripts/padring.py librelane/slots/slot_${SLOT}.yaml librelane/config.yaml
.PHONY: librelane-padring

lint: ## Lint all SystemVerilog files in src
	verilator --lint-only \
	          -Wall \
	          -Wno-fatal \
	          -flife lint.vlt \
	          $(SV_SRCS)
.PHONY: lint

# Default testbench module name is <DUT>_tb, but you can override it:
# Example: make sim DUT=voxel_bin_core_parallel TB=voxel_bin_core
TB ?= $(DUT)

sim: ## Run RTL simulation with cocotb
	@if [ -z "$(DUT)" ]; then \
		echo "Error: You must specify DUT=<module_name>"; \
		echo "Example: make sim DUT=voxel_bin_top"; \
		exit 1; \
	fi; \
	for d in $(SIM_DUTS); do \
		echo "===================================================="; \
		echo " Running DUT=$$d with CONFIG=$(CONFIG)"; \
		echo "===================================================="; \
		if [ ! -f "cocotb/$(TB)_tb.py" ]; then \
			echo "Skipping $$d (no testbench found: cocotb/$(TB)_tb.py)"; \
			continue; \
		fi; \
		rm -rf cocotb/sim_build/$$d; \
		\
		SRCS=$$(grep -v '^[[:space:]]*$$' src/rtl.f | grep -v '^[[:space:]]*#' | tr -d '\r' | tr '\n' ' '); \
		\
		PARAMS=$$(PYTHONPATH=cocotb SIM_CONFIG=$(CONFIG_FILE) python3 -m util.config_parser $$d); \
		export SIM_CONFIG=$(CONFIG_FILE); \
		\
		TOPLEVEL=$$d \
		TOPLEVEL_LANG=verilog \
		COCOTB_TEST_MODULES=$(TB)_tb \
		VERILOG_SOURCES="$$SRCS" \
		COMPILE_ARGS="$$PARAMS" \
		WAVES=1 \
		SIM_BUILD=cocotb/sim_build/$$d \
		PYTHONPATH=cocotb \
		make -f $$(cocotb-config --makefiles)/Makefile.sim results.xml; \
	done
.PHONY: sim

# SRCS="src/gf180_sram_1r1w.sv src/voxel_*.sv src/input_fifo.sv src/evt2_decoder.sv src/control_fsm.sv src/selectable_debug.sv src/spi_wrapper.sv src/verilog_spi/*.v"; \

sim-fast: ## Run voxel_bin_core sim with small fast-sim config (8x8 grid, N=8, 4 bins)
	$(MAKE) sim DUT=voxel_bin_core CONFIG=voxel_sim_fast
.PHONY: sim-fast

sim-all: ## Test all the modules against Makefile compile args
	$(MAKE) sim DUT=gf180_sram_1r1w CONFIG=gf180_sram_1r1w
	$(MAKE) sim DUT=input_fifo
	$(MAKE) sim DUT=evt2_decoder
	$(MAKE) sim DUT=voxel_gesture_classifier
	$(MAKE) sim DUT=voxel_mac_engine
	$(MAKE) sim DUT=voxel_binning
	$(MAKE) sim DUT=voxel_bin_core
.PHONY: sim-all

sim-chip-top: ## Run chip_top RTL simulation with cocotb (uses Python runner; PDK optional)
	cd cocotb; PDK=${PDK} SLOT=${SLOT} python3 chip_top_tb.py
.PHONY: sim-chip-top

sim-gl: ## Run gate-level simulation with cocotb (after copy-final)
	cd cocotb; GL=1 PDK_ROOT=${PDK_ROOT} PDK=${PDK} SLOT=${SLOT} python3 chip_top_tb.py
.PHONY: sim-gl

sim-view: ## View simulation waveforms in GTKWave
	gtkwave cocotb/sim_build/chip_top.fst
.PHONY: sim-view

copy-final: ## Copy final output files from the last run
	rm -rf final/
	cp -r librelane/runs/${RUN_TAG}/final/ final/
.PHONY: copy-final

render-image: ## Render an image from the final layout (after copy-final)
	mkdir -p img/
	PDK_ROOT=${PDK_ROOT} PDK=${PDK} python3 scripts/lay2img.py final/gds/${TOP}.gds img/${TOP}.png --width 2048 --oversampling 4
.PHONY: copy-final

ice40: ## Run ice40 FPGA build
	$(MAKE) -C ice40 -f ice40.mk ARCH=$(ARCH)

ice40-prog: ## Program ice40 board
	$(MAKE) -C ice40 -f ice40.mk prog ARCH=$(ARCH)

ice40-timing: ## Timing report for ice40 build
	$(MAKE) -C ice40 -f ice40.mk timing ARCH=$(ARCH)

ice40-clean: ## Cleans out all ice40 logic
	$(MAKE) -C ice40 -f ice40.mk clean

clean: ## Cleans the generated files
	rm -rf results.xml sim_build/
.PHONY: clean
