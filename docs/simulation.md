# Simulation Guide (`cocotb/`)

μTheia uses [cocotb](https://www.cocotb.org/) 2.0.1 with [Icarus Verilog](https://github.com/steveicarus/iverilog) as the underlying simulator.

---

## Quick Start

```bash
# Single module test
make sim DUT=voxel_bin_core CONFIG=voxel_default

# All modules
make sim-all

# Gate-level simulation (requires make copy-final first)
make sim-gl

# View waveform
make sim-view   # opens GTKWave on cocotb/sim_build/chip_top.fst
```

---

## Directory Layout

```
cocotb/
├── util/                       # Shared utilities (imported by all TBs)
│   ├── config_parser.py        # Parse configs/*.txt into Verilog -P flags
│   ├── utilities.py            # File I/O, EVT2 helpers, test runners
│   ├── get_top.py              # Extract top module name from project
│   ├── get_filelist.py         # Build source file list for simulator
│   └── test_logging.py         # Test result logging helpers
│
├── evt2_decoder_tb.py          # evt2_decoder tests
├── input_fifo_tb.py            # input_fifo tests
├── sram_wrapper_tb.py          # sram_wrapper tests
├── voxel_binning_tb.py         # voxel_binning tests
├── voxel_mac_engine_tb.py      # voxel_mac_engine tests
├── voxel_gesture_classifier_tb.py  # voxel_gesture_classifier tests
├── control_fsm_tb.py           # control_fsm tests
├── spi_wrapper_tb.py           # spi_wrapper tests
├── voxel_bin_core_tb.py        # Full pipeline integration
├── soc_tb.py                   # SOC-level tests
├── chip_top_tb.py              # Full chip with IO pads
└── chip_top_tb_new.py          # Advanced chip-level gesture tests
```

---

## Configuration Files (`configs/`)

Configuration files map module parameters to Verilog `-P` overrides. The `config_parser.py` utility reads them and passes the resulting flags to Icarus.

| Config | GRID_SIZE | NUM_BINS | Clock | Notes |
|--------|-----------|----------|-------|-------|
| `voxel_default.txt` | 16 | 16 | 64 MHz | Full-scale default |
| `voxel_sim_fast.txt` | 8 | 8 | 64 MHz | Fast simulation (less memory) |
| `voxel_8x8_4bins.txt` | 8 | 4 | 64 MHz | Minimal config |
| `voxel_modded_config.txt` | varies | varies | — | Development config |
| `voxel_mac_engine.txt` | — | — | — | MAC-only tests |
| `sram_wrapper.txt` | — | — | — | Single-bank SRAM tests |
| `sram_wrapper_2bank.txt` | — | — | — | Multi-bank SRAM tests |

Config format (example):
```
GRID_SIZE=16
NUM_BINS=16
CLK_FREQ_HZ=64000000
SENSOR_WIDTH=320
SENSOR_HEIGHT=320
```

If `CONFIG` is not specified on the `make sim` command line, the Makefile falls back to its own default `-P` flags.

---

## Utility Modules (`cocotb/util/`)

### `config_parser.py`

Reads a config file and returns a dictionary of parameter overrides. Used at testbench startup to parameterize the DUT through cocotb's `plusargs` mechanism.

```python
from util.config_parser import parse_config
params = parse_config("configs/voxel_default.txt")
```

### `utilities.py`

Collection of helper functions shared across testbenches:

| Function | Description |
|----------|-------------|
| `send_evt2_event(dut, x, y, ts)` | Drive a single CD event onto the SPI input |
| `send_evt2_word(dut, word)` | Drive a raw 32-bit word |
| `load_weights_from_file(dut, path)` | Parse a weight file and send EVT_WEIGHT commands |
| `wait_for_gesture(dut, timeout_cycles)` | Poll for `gesture_valid` with timeout |
| `reset_dut(dut, clk)` | Assert/deassert reset, advance clock |

### `get_filelist.py`

Builds the ordered list of RTL source files (reads `rtl.f`) to pass to the Icarus `-f` flag. Handles the `chip_top.sv` / `slot_defines.svh` special cases for simulation vs synthesis.

### `test_logging.py`

Wraps cocotb's logging to emit per-test pass/fail summaries compatible with the `results.xml` JUnit format that cocotb generates.

---

## Testbench Descriptions

### `input_fifo_tb.py`

Tests:
- Write to empty FIFO (bypass path).
- Fill FIFO to capacity, verify `ready_o` goes low.
- Simultaneous read/write (collision deferral).
- Read from full FIFO until empty.

### `evt2_decoder_tb.py`

Tests:
- CD event decoding: X/Y coordinate mapping from sensor space to grid space.
- TIME_HIGH packet handling; 34-bit timestamp reconstruction.
- Weight command decoding; verify SRAM write signals.
- Threshold command decoding (upper + lower packet assembly).
- Bin length command decoding.
- Control commands (BOOT_REQ, RELOAD_REQ, DEBUG_REQ, EVT_READS_DONE).

### `sram_wrapper_tb.py`

Tests:
- Write then read at various addresses.
- Multi-bank address space (depth > 1024).
- Hazard case: read and write to different addresses in same cycle (verifies warning in simulation).

### `voxel_binning_tb.py`

Tests:
- Single event accumulation, RMW correctness.
- Saturation at `2^COUNTER_BITS − 1`.
- Bin rollover on timestamp boundary.
- Full window readout (READOUT_BINS bins) into feature RAM.
- Ring buffer wraparound (more than NUM_BINS bins processed).

### `voxel_mac_engine_tb.py`

Tests:
- Dot product correctness for known feature/weight pairs.
- Negative weight handling.
- All-zero features produce zero scores.
- FEATURE_COUNT + 2 cycle latency.

### `voxel_gesture_classifier_tb.py`

Tests:
- Argmax correctness for all four classes.
- Threshold pass/fail (`gesture_valid` asserted or suppressed).
- Confidence pass/fail (`gesture_confidence` asserted or suppressed).
- Signed score edge cases (negative max score).

### `control_fsm_tb.py`

Tests:
- Boot sequence: BOOT → LOAD → RUN on receipt of BOOT_REQ + EVT_READS_DONE.
- Reload sequence: RUN → BOOT → LOAD → RUN.
- Debug entry/exit.
- 1024-cycle SRAM power-up delay enforcement.

### `spi_wrapper_tb.py`

Tests:
- 32-bit word transfer, verify `evt_word_valid` pulse.
- Partial word (CS abort mid-transfer), verify word is dropped.
- MISO output: verify gesture bits appear in upper bits of response.

### `voxel_bin_core_tb.py`

Integration test of the full pipeline:
1. Boot sequence.
2. Load weights via EVT_WEIGHT commands.
3. Send TIME_HIGH + CD events over a 1-second window.
4. Verify gesture output.

### `chip_top_tb.py` / `chip_top_tb_new.py`

End-to-end tests including IO pad models. `chip_top_tb_new.py` drives complete gesture recordings from `EVT2_gesture_set/` and verifies the correct class is output.

---

## Gate-Level Simulation

Gate-level simulation uses the post-PnR netlist from `final/` and the GF180MCU standard cell timing models.

```bash
make copy-final   # copies final GDS + netlist
make sim-gl       # GL functional simulation
make sim-sdf      # GL simulation with SDF timing back-annotation
```

`make sim-gl-parallel` runs four gesture tests simultaneously (one per gesture class). Each run takes approximately **7 hours** on a modern workstation.

**Known issue:** SDF back-annotation (`sim-sdf`) requires the Verilog timing models for GF180MCU SRAM macros to be present in the PDK installation. If macros are missing, the simulation will emit warnings and timing violations will not be reported.

---

## Waveform Viewing

```bash
make sim-view      # GTKWave on cocotb/sim_build/chip_top.fst
```

[Surfer](https://gitlab.com/surfer-project/surfer) is also available inside the devcontainer.

Waveform files are generated in `cocotb/sim_build/<module_name>/`. The FST format is used by default (compact binary).

---

## Adding a New Test

1. Create `cocotb/<module>_tb.py` following the cocotb 2.0 coroutine pattern.
2. Import shared utilities: `from util.utilities import reset_dut, send_evt2_event`.
3. Add the module to the `SIM_ALL_TARGETS` list in the Makefile.
4. Create a config in `configs/<module>.txt` if non-default parameters are needed.
5. Run: `make sim DUT=<module> CONFIG=<config>`.
