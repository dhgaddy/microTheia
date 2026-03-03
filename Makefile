MAKEFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

RUN_TAG = $(shell ls librelane/runs/ | tail -n 1)
TOP = voxel_bin_top

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

# NEW ----------------------------------------------------------------------------
SIM_DUTS = $(if $(strip $(DUT)),$(DUT),$(basename $(notdir $(wildcard src/*.sv))))

SV_SRCS := $(shell find src -name "*.sv")

help: ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-20s %s\n", $$1, $$2}'
.PHONY: help

# Optional configuration file
CONFIG ?=
CONFIG_FILE := $(if $(CONFIG),configs/$(CONFIG).txt,)

HAS_CONFIG := $(if $(CONFIG),1,0)
# ---------------------------------------------------------------------------------

all: librelane ## Build the project (runs LibreLane)
.PHONY: all

clone-pdk: ## Clone the GF180MCU PDK repository
	rm -rf $(MAKEFILE_DIR)/gf180mcu
	git clone https://github.com/wafer-space/gf180mcu.git $(MAKEFILE_DIR)/gf180mcu --depth 1 --branch ${PDK_TAG}
.PHONY: clone-pdk

librelane: ## Run LibreLane flow (synthesis, PnR, verification)
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk
.PHONY: librelane

librelane-nodrc: ## Run LibreLane flow without DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --skip KLayout.Antenna --skip KLayout.DRC --skip Magic.DRC
.PHONY: librelane-nodrc

librelane-klayoutdrc: ## Run LibreLane flow without magic DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --skip Magic.DRC
.PHONY: librelane-klayoutdrc

librelane-magicdrc: ## Run LibreLane flow without KLayout DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --skip KLayout.DRC
.PHONY: librelane-magicdrc

librelane-openroad: ## Open the last run in OpenROAD
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --last-run --flow OpenInOpenROAD
.PHONY: librelane-openroad

librelane-klayout: ## Open the last run in KLayout
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --manual-pdk --last-run --flow OpenInKLayout
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

sim: ## Run RTL simulation with cocotb
	@for d in $(SIM_DUTS); do \
		echo "===================================================="; \
		echo " Running DUT=$$d"; \
		echo "===================================================="; \
		if [ ! -f "cocotb/$${d}_tb.py" ]; then \
			echo "Skipping $$d (no testbench found)"; \
			continue; \
		fi; \
		\
		# ------------------ Default CARGS ------------------ \
		if [ "$$d" = "evt2_decoder" ]; then \
			CARGS="-P$$d.GRID_SIZE=16"; \
		elif [ "$$d" = "input_fifo" ]; then \
			CARGS="-P$$d.FIFO_DEPTH=8 -P$$d.DATA_WIDTH=32"; \
		elif [ "$$d" = "uart_debug" ]; then \
			CARGS="-P$$d.CLK_FREQ_HZ=12000000 -P$$d.BAUD_RATE=3000000"; \
		elif [ "$$d" = "uart_rx" ]; then \
			CARGS="-P$$d.CLK_FREQ_HZ=12000000 -P$$d.BAUD_RATE=3000000"; \
		elif [ "$$d" = "uart_tx" ]; then \
			CARGS="-P$$d.CLK_FREQ_HZ=12000000 -P$$d.BAUD_RATE=3000000"; \
		elif [ "$$d" = "MatMul" ]; then \
			CARGS="-P$$d.N=8 -P$$d.DATA_BIT_SIZE=16"; \
		elif [ "$$d" = "voxel_gesture_classifier" ]; then \
			CARGS="-P$$d.ACC_SUM_BITS=18 -P$$d.PERSISTENCE_COUNT=2"; \
		elif [ "$$d" = "voxel_systolic_array" ]; then \
			CARGS="-P$$d.GRID_SIZE=16 -P$$d.NUM_BINS=4 -P$$d.NUM_CLASSES=4 -P$$d.VALUE_BITS=6 -P$$d.WEIGHT_BITS=8 -P$$d.ACC_BITS=24 -P$$d.PARALLEL_READS=4"; \
		elif [ "$$d" = "voxel_weight_ram" ]; then \
			CARGS="-P$$d.CLASS_IDX=0 -P$$d.GRID_SIZE=16 -P$$d.NUM_BINS=4 -P$$d.WEIGHT_BITS=8"; \
		elif [ "$$d" = "voxel_binning" ]; then \
			CARGS="-P$$d.CYCLES_PER_BIN=100"; \
		elif [ "$$d" = "voxel_bin_core" ]; then \
			CARGS="-P$$d.CYCLES_PER_BIN=100"; \
		elif [ "$$d" = "voxel_bin_top" ]; then \
			CARGS="-P$$d.CYCLES_PER_BIN=100 -P$$d.CLK_FREQ_HZ=1000000 -P$$d.BAUD_RATE=250000 -P$$d.PARALLEL_READS=4"; \
		else \
			CARGS=""; \
		fi; \
		\
		rm -rf sim_build; \
		\
		if echo $$d | grep -q gradient; then \
			SRCS="src/gradient_*.sv src/input_fifo.sv src/evt2_decoder.sv src/uart_*.sv src/MatMul.sv"; \
		else \
			SRCS="src/voxel_*.sv src/input_fifo.sv src/evt2_decoder.sv src/uart_*.sv src/MatMul.sv"; \
		fi; \
		\
		# ------------------ CONFIG override ------------------ \
		if [ "$(HAS_CONFIG)" = "1" ]; then \
			PARAMS=$$(PYTHONPATH=cocotb SIM_CONFIG=$(CONFIG_FILE) python3 -m config_parser $$d); \
			COMPILE_ARGS="$$PARAMS"; \
			export SIM_CONFIG=$(CONFIG_FILE); \
		else \
			COMPILE_ARGS="$$CARGS"; \
			unset SIM_CONFIG; \
		fi; \
		\
		TOPLEVEL=$$d \
		TOPLEVEL_LANG=verilog \
		COCOTB_TEST_MODULES=$${d}_tb \
		VERILOG_SOURCES="$$SRCS" \
		COMPILE_ARGS="$$COMPILE_ARGS" \
		SIM_CARGS="$$COMPILE_ARGS" \
		PYTHONPATH=cocotb \
		make -f $$(cocotb-config --makefiles)/Makefile.sim; \
	done
.PHONY: sim

sim-gl: ## Run gate-level simulation with cocotb (after copy-final)
	cd cocotb; GL=1 PDK_ROOT=${PDK_ROOT} PDK=${PDK} SLOT=${SLOT} python3 chip_top_tb.py
.PHONY: sim-gl

sim-view: ## View simulation waveforms in GTKWave
	gtkwave cocotb/sim_build/chip_top.fst
.PHONY: sim-view

sim-test:
	$(MAKE) sim DUT=evt2_decoder
	$(MAKE) sim DUT=input_fifo
	$(MAKE) sim DUT=uart_debug
	$(MAKE) sim DUT=uart_rx
	$(MAKE) sim DUT=uart_tx
	$(MAKE) sim DUT=MatMul
	$(MAKE) sim DUT=voxel_gesture_classifier
	$(MAKE) sim DUT=voxel_systolic_array
	$(MAKE) sim DUT=voxel_weight_ram
	$(MAKE) sim DUT=voxel_binning
	$(MAKE) sim DUT=voxel_bin_core
	$(MAKE) sim DUT=voxel_bin_top

# 	$(MAKE) sim DUT=input_fifo CONFIG=voxel_default
# 	$(MAKE) sim DUT=uart_debug CONFIG=voxel_default
# # 	$(MAKE) sim DUT=uart_rx CONFIG=voxel_default 				  # failing
# 	$(MAKE) sim DUT=uart_tx CONFIG=voxel_default
# 	$(MAKE) sim DUT=MatMul CONFIG=voxel_default
# 	$(MAKE) sim DUT=evt2_decoder CONFIG=voxel_default
# 	$(MAKE) sim DUT=voxel_gesture_classifier CONFIG=voxel_default
# 	$(MAKE) sim DUT=voxel_systolic_array CONFIG=voxel_default
# 	$(MAKE) sim DUT=voxel_weight_ram CONFIG=voxel_default
# 	$(MAKE) sim DUT=voxel_binning CONFIG=voxel_default 			  # takes a while to run
# 	$(MAKE) sim DUT=voxel_bin_core CONFIG=voxel_default
# # 	$(MAKE) sim DUT=voxel_bin_top CONFIG=voxel_default  		  # failing

#	$(MAKE) sim DUT=gradient_map_core CONFIG=gradient_default# still broken
#	$(MAKE) sim DUT=gradient_map_top CONFIG=gradient_default# still broken
#	$(MAKE) sim DUT=gradient_gesture_classifier CONFIG=gradient_default# takes a while to run
#	$(MAKE) sim DUT=gradient_mapping CONFIG=gradient_default
#	$(MAKE) sim DUT=gradient_systolic_array CONFIG=gradient_default
#	$(MAKE) sim DUT=gradient_weight_ram CONFIG=gradient_default
.PHONY: sim-test

copy-final: ## Copy final output files from the last run
	rm -rf final/
	cp -r librelane/runs/${RUN_TAG}/final/ final/
.PHONY: copy-final

render-image: ## Render an image from the final layout (after copy-final)
	mkdir -p img/
	PDK_ROOT=${PDK_ROOT} PDK=${PDK} python3 scripts/lay2img.py final/gds/${TOP}.gds img/${TOP}.png --width 2048 --oversampling 4
.PHONY: copy-final

clean: ## Cleans the generated files
	rm -rf results.xml sim_build/
.PHONY: clean