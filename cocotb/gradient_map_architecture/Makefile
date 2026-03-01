TOPLEVEL_LANG = verilog

RTL_DIR = ../../rtl

GRADIENT_MAP_COMMON_SOURCES = $(RTL_DIR)/uart_tx.sv \
                              $(RTL_DIR)/uart_rx.sv \
                              $(RTL_DIR)/uart_debug.sv \
                              $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                              $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                              $(RTL_DIR)/gradient_map_architecture/gradient_mapping.sv \
                              $(RTL_DIR)/gradient_map_architecture/gradient_map_core.sv \
                              $(RTL_DIR)/gradient_map_architecture/weight_ram.sv \
                              $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                              $(RTL_DIR)/gradient_map_architecture/gesture_classifier.sv

# Default test target
# "core" is the integration-style unit testbench available in this directory.
TEST ?= core

ifeq ($(TEST),core)
  VERILOG_SOURCES = $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                    $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                    $(RTL_DIR)/gradient_map_architecture/gradient_mapping.sv \
                    $(RTL_DIR)/gradient_map_architecture/gradient_map_core.sv \
                    $(RTL_DIR)/gradient_map_architecture/weight_ram.sv \
                    $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                    $(RTL_DIR)/gradient_map_architecture/gesture_classifier.sv
  TOPLEVEL = gradient_map_core
  COCOTB_TEST_MODULES = test_gradient_map_core
  COMPILE_ARGS += -Pgradient_map_core.CLK_FREQ_HZ=1000000
  COMPILE_ARGS += -Pgradient_map_core.FRAME_PERIOD_MS=1
  COMPILE_ARGS += -Pgradient_map_core.MIN_MASS_THRESH=20
  COMPILE_ARGS += -Pgradient_map_core.DECAY_SHIFT=15
else ifeq ($(TEST),input_fifo)
  VERILOG_SOURCES = $(RTL_DIR)/gradient_map_architecture/input_fifo.sv
  TOPLEVEL = input_fifo
  COCOTB_TEST_MODULES = test_input_fifo
  COMPILE_ARGS += -Pinput_fifo.DEPTH=8
  COMPILE_ARGS += -Pinput_fifo.PTR_BITS=3
else ifeq ($(TEST),evt2_decoder)
  VERILOG_SOURCES = $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv
  TOPLEVEL = evt2_decoder
  COCOTB_TEST_MODULES = test_evt2_decoder
else ifeq ($(TEST),gradient_mapping)
  VERILOG_SOURCES = $(RTL_DIR)/gradient_map_architecture/gradient_mapping.sv
  TOPLEVEL = gradient_mapping
  COCOTB_TEST_MODULES = test_gradient_mapping
else ifeq ($(TEST),systolic_array)
  VERILOG_SOURCES = $(RTL_DIR)/gradient_map_architecture/systolic_array.sv
  TOPLEVEL = systolic_array
  COCOTB_TEST_MODULES = test_systolic_array
  COMPILE_ARGS += -Psystolic_array.NUM_CELLS=16
else ifeq ($(TEST),gesture_classifier)
  VERILOG_SOURCES = $(RTL_DIR)/gradient_map_architecture/gesture_classifier.sv \
                    $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                    $(RTL_DIR)/gradient_map_architecture/weight_ram.sv
  TOPLEVEL = gesture_classifier
  COCOTB_TEST_MODULES = test_gesture_classifier
  COMPILE_ARGS += -Pgesture_classifier.FRAME_PERIOD_MS=1
  COMPILE_ARGS += -Pgesture_classifier.MIN_MASS_THRESH=20
else ifeq ($(TEST),top)
  VERILOG_SOURCES = $(GRADIENT_MAP_COMMON_SOURCES) \
                    $(RTL_DIR)/gradient_map_architecture/gradient_map_top.sv
  TOPLEVEL = gradient_map_top
  COCOTB_TEST_MODULES = test_gradient_map_top
  COMPILE_ARGS += -Pgradient_map_top.CLK_FREQ_HZ=1000000
  COMPILE_ARGS += -Pgradient_map_top.BAUD_RATE=250000
  COMPILE_ARGS += -Pgradient_map_top.FRAME_PERIOD_MS=1
  COMPILE_ARGS += -Pgradient_map_top.MIN_MASS_THRESH=20
  COMPILE_ARGS += -Pgradient_map_top.DECAY_SHIFT=6
else ifeq ($(TEST),weight_ram)
  VERILOG_SOURCES = $(RTL_DIR)/gradient_map_architecture/weight_ram.sv
  TOPLEVEL = weight_ram
  COCOTB_TEST_MODULES = test_weight_ram
  COMPILE_ARGS += -Pweight_ram.CLASS_IDX=0
else
  $(error Unknown TEST=$(TEST). Use: core, input_fifo, evt2_decoder, gradient_mapping, systolic_array, gesture_classifier, top, weight_ram)
endif

SIM = icarus
# Keep per-target build artifacts separate so switching TEST values does not
# reuse an incompatible sim.vvp from a previous run.
SIM_BUILD ?= sim_build_$(if $(TEST),$(TEST),default)

COMPILE_ARGS += -g2012

# Enable waveform dumping by default so each simulation emits a trace.
export WAVES ?= 1
export COCOTB_LOG_LEVEL ?= INFO
export COCOTB_REDUCED_LOG_FMT ?= 1
export GPI_LOG_LEVEL ?= CRITICAL
WAVE_FILE ?= $(SIM_BUILD)/$(TOPLEVEL).fst
GTKWAVE ?= gtkwave

WINDOWS_HOST := $(filter Windows_NT Msys MSYS,$(OS))

# On Windows, locate the venv and OSS CAD Suite relative to the project root,
# falling back to any installation already on PATH.
ifneq ($(WINDOWS_HOST),)
  # GNU Make defaults to sh.exe which is often absent on Windows PATH.
  # Bootstrap a POSIX shell + utilities from Git-for-Windows (via 8.3 path
  # to avoid spaces) before any $(shell) call.
  _GIT_USR_BIN := $(firstword $(wildcard C:/PROGRA~1/Git/usr/bin) \
                               $(wildcard C:/PROGRA~2/Git/usr/bin))
  ifneq ($(_GIT_USR_BIN),)
    SHELL := $(_GIT_USR_BIN)/sh.exe
  endif

  PROJECT_ROOT := $(realpath ../..)
  VENV_SCRIPTS := $(PROJECT_ROOT)/.venv/Scripts

  # Detect oss-cad-suite: project-local, nested project-local, or user-level
  _OSS_PROJ     := $(PROJECT_ROOT)/oss-cad-suite
  _OSS_NESTED   := $(PROJECT_ROOT)/oss-cad-suite/oss-cad-suite
  _OSS_USER     := $(subst \,/,$(USERPROFILE))/Documents/oss-cad-suite
  ifneq ($(wildcard $(_OSS_PROJ)/bin/iverilog.exe),)
    _OSS_ROOT := $(_OSS_PROJ)
  else ifneq ($(wildcard $(_OSS_NESTED)/bin/iverilog.exe),)
    _OSS_ROOT := $(_OSS_NESTED)
  else ifneq ($(wildcard $(_OSS_USER)/bin/iverilog.exe),)
    _OSS_ROOT := $(_OSS_USER)
  else
    _OSS_ROOT :=
  endif

  ifneq ($(_OSS_ROOT),)
    OSS_CAD_BIN := $(_OSS_ROOT)/bin
    OSS_CAD_LIB := $(_OSS_ROOT)/lib
  else
    OSS_CAD_BIN :=
    OSS_CAD_LIB :=
  endif

  export PYTHONNOUSERSITE := 1
  export PYTHONPATH := $(PROJECT_ROOT)/.venv/Lib/site-packages
  export PATH := $(VENV_SCRIPTS):$(OSS_CAD_BIN):$(OSS_CAD_LIB):$(_GIT_USR_BIN):$(PATH)
  COCOTB_PYTHON := $(VENV_SCRIPTS)/python.exe
else
  PROJECT_ROOT := $(realpath ../..)
  WSL_VENV_BIN := $(PROJECT_ROOT)/.wsl-venv/bin
  VENV_BIN     := $(PROJECT_ROOT)/.venv/bin
  ifneq ($(wildcard $(WSL_VENV_BIN)/python),)
    COCOTB_PYTHON := $(WSL_VENV_BIN)/python
    export PATH := $(WSL_VENV_BIN):$(PATH)
  else ifneq ($(wildcard $(VENV_BIN)/python),)
    COCOTB_PYTHON := $(VENV_BIN)/python
    export PATH := $(VENV_BIN):$(PATH)
  else
    COCOTB_PYTHON := python3
  endif
endif

COCOTB_MAKEFILES := $(shell "$(COCOTB_PYTHON)" -m cocotb_tools.config --makefiles)
PYTHON_BIN := $(COCOTB_PYTHON)

include $(COCOTB_MAKEFILES)/Makefile.inc

SIM_LOWERCASE := $(shell echo $(SIM) | tr A-Z a-z)
ifeq ($(wildcard $(COCOTB_MAKEFILES)/simulators/Makefile.$(SIM_LOWERCASE)),)
  $(error Couldn't find cocotb simulator makefile for SIM=$(SIM))
endif
include $(COCOTB_MAKEFILES)/simulators/Makefile.$(SIM_LOWERCASE)

UNIT_TESTS = core input_fifo evt2_decoder gradient_mapping systolic_array gesture_classifier top weight_ram
UNIT_TEST_TARGETS = $(addprefix test_,$(UNIT_TESTS))

.PHONY: test test_full test_unit view_wave clean_all $(UNIT_TEST_TARGETS)

test:
	$(MAKE) sim

test_unit: $(UNIT_TEST_TARGETS)

$(UNIT_TEST_TARGETS):
	@echo "======== Running TEST=$(@:test_%=%) ========"
	$(MAKE) clean TEST=$(@:test_%=%) SIM_BUILD=sim_build_$(@:test_%=%)
	$(MAKE) sim TEST=$(@:test_%=%) SIM_BUILD=sim_build_$(@:test_%=%)

test_full:
	$(MAKE) clean && $(MAKE) sim

view_wave:
	@if [ -f "$(WAVE_FILE)" ]; then \
		echo "Opening waveform: $(WAVE_FILE)"; \
		$(GTKWAVE) "$(WAVE_FILE)"; \
	else \
		echo "Waveform not found: $(WAVE_FILE)"; \
		echo "Run: make sim TEST=$(TEST)"; \
		exit 1; \
	fi

clean_all:
ifneq ($(WINDOWS_HOST),)
	@powershell -NoProfile -Command "Get-ChildItem -Path . -Filter 'sim_build*' -Directory -ErrorAction SilentlyContinue | Remove-Item -Recurse -Force"
	@powershell -NoProfile -Command "if (Test-Path 'results.xml') { Remove-Item -Force 'results.xml' }"
	@powershell -NoProfile -Command "if (Test-Path '__pycache__') { Remove-Item -Recurse -Force '__pycache__' }"
else
	rm -rf sim_build* results.xml __pycache__
endif


