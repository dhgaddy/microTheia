PCF    = icebreaker.pcf
DEVICE = up5k
PACKAGE = sg48

RTL_DIR = ../rtl
PROJECT_ROOT := $(realpath ..)

# Select architecture:
#   ARCH=gradient_map  -> gradient_map_top (UART EVT2.0 words)
#   ARCH=voxel_bin     -> voxel_bin_top    (UART EVT2.0 words)
ARCH   ?= gradient_map
TARGET ?= raw

# ── gradient_map sources ────────────────────────────────────────────────────
# Raw pipeline: UART RX -> 5-byte assembler -> gradient_map_core -> UART TX
GRADIENT_MAP_CORE = $(RTL_DIR)/uart_tx.sv \
                    $(RTL_DIR)/uart_rx.sv \
                    $(RTL_DIR)/uart_debug.sv \
                    $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                    $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                    $(RTL_DIR)/gradient_map_architecture/gradient_mapping.sv \
                    $(RTL_DIR)/gradient_map_architecture/gradient_map_core.sv \
                    $(RTL_DIR)/gradient_map_architecture/weight_ram.sv \
                    $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                    $(RTL_DIR)/gradient_map_architecture/gesture_classifier.sv

# Processed pipeline: preprocessed events -> gradient_map_core -> UART TX
GRADIENT_MAP_PROCESSED_CORE = $(RTL_DIR)/uart_tx.sv \
                               $(RTL_DIR)/uart_rx.sv \
                               $(RTL_DIR)/uart_debug.sv \
                               $(RTL_DIR)/gradient_map_architecture/input_fifo.sv \
                               $(RTL_DIR)/gradient_map_architecture/evt2_decoder.sv \
                               $(RTL_DIR)/gradient_map_architecture/gradient_mapping.sv \
                               $(RTL_DIR)/gradient_map_architecture/gradient_map_core.sv \
                               $(RTL_DIR)/gradient_map_architecture/weight_ram.sv \
                               $(RTL_DIR)/gradient_map_architecture/systolic_array.sv \
                               $(RTL_DIR)/gradient_map_architecture/gesture_classifier.sv

# ── voxel_bin sources ───────────────────────────────────────────────────────
VOXEL_BIN_CORE = $(RTL_DIR)/uart_tx.sv \
                 $(RTL_DIR)/uart_rx.sv \
                 $(RTL_DIR)/voxel_bin_architecture/input_fifo.sv \
                 $(RTL_DIR)/voxel_bin_architecture/evt2_decoder.sv \
                 $(RTL_DIR)/voxel_bin_architecture/voxel_binning.sv \
                 $(RTL_DIR)/voxel_bin_architecture/systolic_array.sv \
                 $(RTL_DIR)/voxel_bin_architecture/gesture_classifier.sv \
                 $(RTL_DIR)/voxel_bin_architecture/weight_ram.sv \
                 $(RTL_DIR)/voxel_bin_architecture/voxel_bin_core.sv

# ── select top module ────────────────────────────────────────────────────────
ifeq ($(ARCH),gradient_map)
  ifeq ($(TARGET),raw)
    PROJ              = gradient_map_top
    PCF               = icebreaker.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(GRADIENT_MAP_CORE) \
                $(RTL_DIR)/gradient_map_architecture/gradient_map_top.sv
  else
    $(error Unknown TARGET=$(TARGET). Only TARGET=raw is supported)
  endif
else ifeq ($(ARCH),voxel_bin)
  ifeq ($(TARGET),raw)
    PROJ              = voxel_bin_top
    PCF               = icebreaker.pcf
    ALLOW_UNCONSTRAINED =
    RTL_FILES = $(VOXEL_BIN_CORE) \
                $(RTL_DIR)/voxel_bin_architecture/voxel_bin_top.sv
  else
    $(error Unknown TARGET=$(TARGET). Only TARGET=raw is supported)
  endif
else
  $(error Unknown ARCH=$(ARCH). Use ARCH=gradient_map or ARCH=voxel_bin)
endif

WINDOWS_HOST := $(filter Windows_NT Msys MSYS,$(OS))

# Locate oss-cad-suite automatically so synthesis works from PowerShell and WSL
# without requiring manual PATH setup.
ifneq ($(WINDOWS_HOST),)
  _GIT_USR_BIN := $(firstword $(wildcard C:/PROGRA~1/Git/usr/bin) \
                               $(wildcard C:/PROGRA~2/Git/usr/bin))
  ifneq ($(_GIT_USR_BIN),)
    SHELL := $(_GIT_USR_BIN)/sh.exe
  endif

  _OSS_PROJ   := $(PROJECT_ROOT)/oss-cad-suite
  _OSS_NESTED := $(PROJECT_ROOT)/oss-cad-suite/oss-cad-suite
  _OSS_USER   := $(subst \,/,$(USERPROFILE))/Documents/oss-cad-suite
  ifneq ($(wildcard $(_OSS_PROJ)/bin/yosys.exe),)
    _OSS_ROOT := $(_OSS_PROJ)
  else ifneq ($(wildcard $(_OSS_NESTED)/bin/yosys.exe),)
    _OSS_ROOT := $(_OSS_NESTED)
  else ifneq ($(wildcard $(_OSS_USER)/bin/yosys.exe),)
    _OSS_ROOT := $(_OSS_USER)
  else
    _OSS_ROOT :=
  endif
else
  _OSS_PROJ   := $(PROJECT_ROOT)/oss-cad-suite
  _OSS_NESTED := $(PROJECT_ROOT)/oss-cad-suite/oss-cad-suite
  _OSS_HOME   := $(HOME)/oss-cad-suite
  _OSS_OPT    := /opt/oss-cad-suite
  ifneq ($(wildcard $(_OSS_PROJ)/bin/yosys),)
    _OSS_ROOT := $(_OSS_PROJ)
  else ifneq ($(wildcard $(_OSS_NESTED)/bin/yosys),)
    _OSS_ROOT := $(_OSS_NESTED)
  else ifneq ($(wildcard $(_OSS_HOME)/bin/yosys),)
    _OSS_ROOT := $(_OSS_HOME)
  else ifneq ($(wildcard $(_OSS_OPT)/bin/yosys),)
    _OSS_ROOT := $(_OSS_OPT)
  else
    _OSS_ROOT :=
  endif
endif

ifneq ($(_OSS_ROOT),)
ifneq ($(WINDOWS_HOST),)
  export PATH := $(_OSS_ROOT)/bin:$(_OSS_ROOT)/lib:$(PATH)
else
  # On Linux/WSL, keep system tools first and append OSS CAD Suite so missing
  # tools (e.g. nextpnr) are still found without forcing an alternate yosys.
  export PATH := $(PATH):$(_OSS_ROOT)/bin:$(_OSS_ROOT)/lib
endif
endif

.PHONY: all all_arch clean prog report timing

all: $(PROJ).bit

all_arch:
	$(MAKE) clean ARCH=gradient_map TARGET=raw
	$(MAKE) all ARCH=gradient_map TARGET=raw
	$(MAKE) clean ARCH=voxel_bin TARGET=raw
	$(MAKE) all ARCH=voxel_bin TARGET=raw

$(PROJ).json: $(RTL_FILES)
	yosys -p "read_verilog -sv $(RTL_FILES); synth_ice40 -top $(PROJ) -json $@"

$(PROJ).asc: $(PROJ).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --json $< --pcf $(PCF) --asc $@ --freq 12 $(ALLOW_UNCONSTRAINED)

$(PROJ).bit: $(PROJ).asc
	icepack $< $@

prog: $(PROJ).bit
	iceprog $<

report: $(PROJ).json
	yosys -p "read_json $<; stat"

timing: $(PROJ).asc
	icetime -d $(DEVICE) -m -r $(PROJ).timing $<

clean:
ifneq ($(WINDOWS_HOST),)
	@powershell -NoProfile -Command "if (Test-Path '$(PROJ).json') { Remove-Item -Force '$(PROJ).json' }"
	@powershell -NoProfile -Command "if (Test-Path '$(PROJ).asc') { Remove-Item -Force '$(PROJ).asc' }"
	@powershell -NoProfile -Command "if (Test-Path '$(PROJ).bit') { Remove-Item -Force '$(PROJ).bit' }"
	@powershell -NoProfile -Command "if (Test-Path '$(PROJ).timing') { Remove-Item -Force '$(PROJ).timing' }"
else
	rm -f $(PROJ).json $(PROJ).asc $(PROJ).bit $(PROJ).timing
endif
