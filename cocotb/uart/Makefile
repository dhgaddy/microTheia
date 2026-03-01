TOPLEVEL_LANG = verilog
SIM = icarus

RTL_DIR = ../../rtl

COMPILE_ARGS += -g2012

export COCOTB_LOG_LEVEL ?= INFO
export COCOTB_REDUCED_LOG_FMT ?= 1
export GPI_LOG_LEVEL ?= CRITICAL
export WAVES ?= 1
WAVE_FILE ?= $(SIM_BUILD)/$(TOPLEVEL).fst
GTKWAVE ?= gtkwave

TEST ?= uart_rx
SIM_BUILD ?= sim_build_$(TEST)

ifeq ($(TEST),uart_rx)
  VERILOG_SOURCES = $(RTL_DIR)/uart_rx.sv
  TOPLEVEL = uart_rx
  COCOTB_TEST_MODULES = test_uart_rx
  COMPILE_ARGS += -Puart_rx.CLKS_PER_BIT=4

else ifeq ($(TEST),uart_tx)
  VERILOG_SOURCES = $(RTL_DIR)/uart_tx.sv
  TOPLEVEL = uart_tx
  COCOTB_TEST_MODULES = test_uart_tx
  COMPILE_ARGS += -Puart_tx.CLKS_PER_BIT=4

else ifeq ($(TEST),uart_debug)
  VERILOG_SOURCES = $(RTL_DIR)/uart_tx.sv $(RTL_DIR)/uart_debug.sv
  TOPLEVEL = uart_debug
  COCOTB_TEST_MODULES = test_uart_debug
  COMPILE_ARGS += -Puart_debug.CLK_FREQ_HZ=12000000 -Puart_debug.BAUD_RATE=3000000

else
  $(error Unknown TEST=$(TEST). Use: uart_rx, uart_tx, uart_debug)
endif

ifeq ($(OS),Windows_NT)
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

  COCOTB_MAKEFILES := $(shell "$(VENV_SCRIPTS)/cocotb-config.exe" --makefiles 2>/dev/null)
  include $(COCOTB_MAKEFILES)/Makefile.sim
else
  include $(shell cocotb-config --makefiles)/Makefile.sim
endif

.PHONY: all test_all view_wave clean_all

all: test_all

test_all:
	@for t in uart_rx uart_tx uart_debug; do \
		echo "======== Running TEST=$$t ========"; \
		$(MAKE) clean TEST=$$t SIM_BUILD=sim_build_$$t > /dev/null 2>&1; \
		$(MAKE) sim TEST=$$t SIM_BUILD=sim_build_$$t || exit 1; \
	done

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
	rm -rf sim_build* results.xml __pycache__


