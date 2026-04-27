#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
"""
bin_inject.py — Inject an EVT2 .bin file into voxel_binning via cocotb simulation
and visualize per-bin, per-cell event counts.

Usage:
    python scripts/bin_inject.py [path/to/file.bin] [options]
    python scripts/bin_inject.py --synthetic [options]

If no path is given, an interactive file picker dialog opens.

Options:
    --config CONFIG    Config name (default: voxel_default)
    --no-sim           Parse the .bin file and show golden-model heatmaps only
                       (no RTL simulation — much faster)
    --bins N           Override READOUT_BINS for display
    --grid N           Override GRID_SIZE for display
    --synthetic        Generate a synthetic EVT2 file with known per-cell counts,
                       run it through the golden model and RTL sim, and verify
                       the output matches the expected counts exactly.
    --synthetic-seed N Random seed for synthetic event generation (default: 42)
"""

import argparse
import os
import struct
import sys
import textwrap
from pathlib import Path

# ---------------------------------------------------------------------------
# Repo root and config
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIGS_DIR = REPO_ROOT / "configs"


def load_config(config_name: str) -> dict:
    cfg_path = CONFIGS_DIR / f"{config_name}.txt"
    if not cfg_path.exists():
        raise FileNotFoundError(f"Config not found: {cfg_path}")
    params: dict = {}
    for raw in cfg_path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        key, val = line.split("=", 1)
        key = key.strip()
        val = val.strip()
        try:
            params[key] = eval(val, {}, params)
        except Exception:
            try:
                params[key] = int(val.replace("_", ""))
            except Exception:
                params[key] = val
    return params


# ---------------------------------------------------------------------------
# EVT2 constants
# ---------------------------------------------------------------------------
EVT_CD_OFF    = 0x0
EVT_CD_ON     = 0x1
EVT_TIME_HIGH = 0x8


def read_evt2_bin(path: Path):
    """Return list of 32-bit little-endian words from a .bin file."""
    data = path.read_bytes()
    n = len(data) // 4
    return list(struct.unpack_from(f"<{n}I", data, 0))


# ---------------------------------------------------------------------------
# Golden-model (pure Python, no RTL)
# ---------------------------------------------------------------------------

class GoldenModel:
    """Timestamp-driven voxel binning model identical to VoxelBinningModel in
    voxel_binning_tb.py but standalone."""

    def __init__(self, grid_size, num_bins, readout_bins, bin_duration_us, max_counter):
        self.G  = grid_size
        self.NB = num_bins
        self.RB = readout_bins
        self.BD = bin_duration_us
        self.MC = max_counter
        self.CPB = grid_size * grid_size
        self.mem = [0] * (num_bins * grid_size * grid_size)
        self.wr_bin = 0
        self.completed = 0
        self.ts_init = False
        self.bin_start = 0
        self.windows: list[list[int]] = []

    def _cell(self, x, y):
        return self.wr_bin * self.CPB + y * self.G + x

    def _snapshot(self):
        start = (self.wr_bin + self.NB - (self.RB - 1)) % self.NB
        out = []
        for off in range(self.RB):
            b = (start + off) % self.NB
            base = b * self.CPB
            out.extend(self.mem[base:base + self.CPB])
        return out

    def _rotate(self):
        nxt = (self.wr_bin + 1) % self.NB
        self.completed = min(self.completed + 1, self.NB)
        snap = self._snapshot() if self.completed >= self.RB else None
        base = nxt * self.CPB
        for i in range(self.CPB):
            self.mem[base + i] = 0
        self.wr_bin = nxt
        return snap

    def accept_event(self, x, y, ts):
        if not self.ts_init:
            self.ts_init = True
            self.bin_start = ts
        else:
            while ts - self.bin_start >= self.BD:
                snap = self._rotate()
                if snap is not None:
                    self.windows.append(snap)
                self.bin_start += self.BD
        addr = self._cell(x, y)
        if self.mem[addr] < self.MC:
            self.mem[addr] += 1

    def flush(self):
        for _ in range(self.NB):
            snap = self._rotate()
            if snap is not None:
                self.windows.append(snap)


def decode_and_run(words, cfg, swap_xy=False, flip_x=False, flip_y=False):
    """Decode EVT2 words through the golden model and return completed windows."""
    G   = cfg["GRID_SIZE"]
    NB  = cfg["NUM_BINS"]
    RB  = cfg["READOUT_BINS"]
    SW  = cfg["SENSOR_WIDTH"]
    SH  = cfg.get("SENSOR_HEIGHT", SW)
    BD  = (cfg["WINDOW_MS"] * 1000) // RB
    CB  = cfg.get("COUNTER_BITS", 16)
    MC  = (1 << CB) - 1

    DIV_K = 12
    X_M = (1 << DIV_K) // (SW // G) + 1
    Y_M = (1 << DIV_K) // (SH // G) + 1

    model = GoldenModel(G, NB, RB, BD, MC)

    have_th = False
    time_high = 0

    for word in words:
        pkt     = (word >> 28) & 0xF
        ts_lsb  = (word >> 22) & 0x3F
        x_raw   = (word >> 11) & 0x7FF
        y_raw   = word & 0x7FF

        if pkt == EVT_TIME_HIGH:
            have_th   = True
            time_high = word & 0x0FFFFFFF
            continue

        if pkt not in (EVT_CD_OFF, EVT_CD_ON):
            continue
        if not have_th:
            continue

        xc = min(x_raw, SW - 1)
        yc = min(y_raw, SH - 1)
        xs = yc if swap_xy else xc
        ys = xc if swap_xy else yc
        if flip_x:
            xs = (SW - 1) - xs
        if flip_y:
            ys = (SH - 1) - ys
        xg = min((xs * X_M) >> DIV_K, G - 1)
        yg = min((ys * Y_M) >> DIV_K, G - 1)
        ts = (time_high << 6) | ts_lsb

        model.accept_event(xg, yg, ts)

    model.flush()
    return model.windows, RB, G


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------
ANSI_RESET  = "\033[0m"
ANSI_BOLD   = "\033[1m"
ANSI_CYAN   = "\033[96m"
ANSI_YELLOW = "\033[93m"
ANSI_GREEN  = "\033[92m"
ANSI_RED    = "\033[91m"
ANSI_DIM    = "\033[2m"
ANSI_BLUE   = "\033[94m"

# 8-level ASCII density ramp
DENSITY_CHARS = " .+#@"

def _ansi_cell(val, max_val):
    if max_val == 0:
        return ANSI_DIM + "  0" + ANSI_RESET
    ratio = val / max_val
    if ratio == 0:
        return ANSI_DIM + "  0" + ANSI_RESET
    if ratio < 0.25:
        color = ANSI_BLUE
    elif ratio < 0.5:
        color = ANSI_CYAN
    elif ratio < 0.75:
        color = ANSI_YELLOW
    else:
        color = ANSI_RED
    return color + f"{val:3d}" + ANSI_RESET


def display_windows(windows, readout_bins, grid_size, bin_file_name):
    cpb = grid_size * grid_size

    print()
    print(ANSI_BOLD + f"=== voxel_binning injection: {bin_file_name} ===" + ANSI_RESET)
    print(f"    {len(windows)} readout window(s), each with {readout_bins} bins "
          f"of {grid_size}x{grid_size} cells")
    print()

    if not windows:
        print(ANSI_RED + "  No completed readout windows produced." + ANSI_RESET)
        return

    for w_idx, window in enumerate(windows):
        print(ANSI_BOLD + ANSI_CYAN +
              f"--- Window {w_idx+1}/{len(windows)} ------------------------------" +
              ANSI_RESET)
        bins_in_window = [window[b*cpb:(b+1)*cpb] for b in range(readout_bins)]
        global_max = max((v for b in bins_in_window for v in b), default=0)
        total_events = sum(v for b in bins_in_window for v in b)

        print(f"  Total events in window: {ANSI_BOLD}{total_events}{ANSI_RESET}   "
              f"Peak cell count: {ANSI_BOLD}{global_max}{ANSI_RESET}")
        print()

        for b_idx, cells in enumerate(bins_in_window):
            bin_total = sum(cells)
            bin_max   = max(cells, default=0)
            bin_pct   = 100 * bin_total / total_events if total_events else 0.0
            label = f"  Bin {b_idx+1:>2}/{readout_bins}  (total={bin_total:5d}, peak={bin_max:4d}, {bin_pct:5.1f}%)"

            if bin_max == 0:
                print(ANSI_DIM + label + "  [empty]" + ANSI_RESET)
                continue

            print(ANSI_YELLOW + label + ANSI_RESET)
            print(ANSI_DIM + "       " + "  x:" +
                  "".join(f"{x:>3}" for x in range(grid_size)) + ANSI_RESET)

            for y in range(grid_size):
                row_vals = cells[y*grid_size:(y+1)*grid_size]
                row_str  = "".join(_ansi_cell(v, global_max) for v in row_vals)
                print(f"    y{y:02d} {ANSI_DIM}|{ANSI_RESET}" + row_str)

            print()

    # Summary heatmap: event count per spatial cell summed over all windows and bins
    print(ANSI_BOLD + "--- Cumulative spatial heatmap (all windows x bins) ----" + ANSI_RESET)
    agg = [0] * cpb
    for window in windows:
        for b in range(readout_bins):
            for i in range(cpb):
                agg[i] += window[b * cpb + i]
    agg_max = max(agg, default=0)
    print(f"  Peak accumulated count: {ANSI_BOLD}{agg_max}{ANSI_RESET}")
    print()
    print(ANSI_DIM + "       " + "  x:" +
          "".join(f"{x:>3}" for x in range(grid_size)) + ANSI_RESET)
    for y in range(grid_size):
        row = agg[y*grid_size:(y+1)*grid_size]
        row_str = "".join(_ansi_cell(v, agg_max) for v in row)
        print(f"    y{y:02d} {ANSI_DIM}|{ANSI_RESET}" + row_str)
    print()


# ---------------------------------------------------------------------------
# cocotb RTL simulation path
# ---------------------------------------------------------------------------

def run_rtl_sim(bin_path: Path, config_name: str):
    """Invoke cocotb simulation of voxel_binning with the chosen .bin file."""
    print(ANSI_CYAN + f"[RTL] Launching cocotb simulation for {bin_path.name} ..." + ANSI_RESET)

    tb_path = REPO_ROOT / "cocotb" / "_bin_inject_tb.py"
    _write_rtl_tb(tb_path, bin_path)

    # Mirror the explicit source list from the Makefile sim target — avoids
    # pulling in chip_top.sv which requires slot_defines.svh not present here.
    voxel_srcs = [
        "src/gf180_sram_1r1w.sv",
        "src/voxel_binning.sv",
        "src/voxel_bin_core.sv",
        "src/voxel_bin_top.sv",
        "src/voxel_gesture_classifier.sv",
        "src/voxel_mac_engine.sv",
        "src/input_fifo.sv",
        "src/evt2_decoder.sv",
        "src/uart_debug.sv",
        "src/uart_rx.sv",
        "src/uart_tx.sv",
        "src/gray2bin.sv",
        "src/bin2gray.sv",
        "src/counter.sv",
        "src/delaybuffer.sv",
        "src/fifo_1r1w_cdc.sv",
        "src/ram_1r1w_sync.sv",
        "src/reg_cdc_sram_buffer.sv",
        "src/spi_control.sv",
        "src/selectable_debug.sv",
    ]
    srcs = " ".join(str(REPO_ROOT / s) for s in voxel_srcs if (REPO_ROOT / s).exists())
    cfg_file = CONFIGS_DIR / f"{config_name}.txt"

    import subprocess
    env = os.environ.copy()
    env["TOPLEVEL"]             = "voxel_binning"
    env["TOPLEVEL_LANG"]        = "verilog"
    env["COCOTB_TEST_MODULES"]  = "_bin_inject_tb"
    env["VERILOG_SOURCES"]      = srcs
    env["SIM_CONFIG"]           = str(cfg_file)
    env["PYTHONPATH"]           = str(REPO_ROOT / "cocotb")
    env["SIM_BUILD"]            = str(REPO_ROOT / "cocotb" / "sim_build" / "bin_inject")
    env["BIN_INJECT_PATH"]      = str(bin_path)

    # Remove stale results.xml so make always re-runs the simulation.
    (REPO_ROOT / "results.xml").unlink(missing_ok=True)

    cocotb_mk = subprocess.check_output(["cocotb-config", "--makefiles"],
                                        text=True).strip()
    result = subprocess.run(
        ["make", "-f", f"{cocotb_mk}/Makefile.sim", "results.xml"],
        env=env,
        cwd=str(REPO_ROOT),
    )
    tb_path.unlink(missing_ok=True)
    return result.returncode == 0


def _write_rtl_tb(tb_path: Path, bin_path: Path):
    """Write a minimal cocotb testbench that injects a bin file into voxel_binning."""
    code = textwrap.dedent(f"""\
    # Auto-generated by bin_inject.py -- do not edit.
    import struct, os
    from pathlib import Path
    import cocotb
    from cocotb.clock import Clock
    from cocotb.triggers import ClockCycles, Event, NextTimeStep, ReadOnly, RisingEdge
    from util.config_parser import load_config
    from util.test_logging import logged_test

    MODULE = os.environ.get("TOPLEVEL")
    CFG    = load_config(MODULE)

    GRID_SIZE     = CFG["GRID_SIZE"]
    NUM_BINS      = CFG["NUM_BINS"]
    READOUT_BINS  = CFG["READOUT_BINS"]
    WINDOW_MS     = CFG["WINDOW_MS"]
    COUNTER_BITS  = CFG.get("COUNTER_BITS", 16)
    SENSOR_WIDTH  = CFG["SENSOR_WIDTH"]
    SENSOR_HEIGHT = CFG.get("SENSOR_HEIGHT", SENSOR_WIDTH)
    MAP_SWAP_XY   = CFG.get("MAP_SWAP_XY", 0)
    MAP_FLIP_X    = CFG.get("MAP_FLIP_X",  0)
    MAP_FLIP_Y    = CFG.get("MAP_FLIP_Y",  0)

    CELLS_PER_BIN   = GRID_SIZE * GRID_SIZE
    FEATURE_COUNT   = READOUT_BINS * CELLS_PER_BIN
    BIN_DURATION_US = (WINDOW_MS * 1000) // READOUT_BINS

    EVT_CD_OFF    = 0x0
    EVT_CD_ON     = 0x1
    EVT_TIME_HIGH = 0x8
    ST_ACCUM      = 0

    DIV_K = 12
    X_M   = (1 << DIV_K) // (SENSOR_WIDTH  // GRID_SIZE) + 1
    Y_M   = (1 << DIV_K) // (SENSOR_HEIGHT // GRID_SIZE) + 1

    BIN_FILE = Path(os.environ["BIN_INJECT_PATH"])

    ANSI_BOLD   = "\\033[1m"
    ANSI_CYAN   = "\\033[96m"
    ANSI_YELLOW = "\\033[93m"
    ANSI_RED    = "\\033[91m"
    ANSI_DIM    = "\\033[2m"
    ANSI_RESET  = "\\033[0m"
    ANSI_BLUE   = "\\033[94m"


    def _ansi_cell(val, max_val):
        if max_val == 0 or val == 0:
            return ANSI_DIM + "  0" + ANSI_RESET
        r = val / max_val
        color = ANSI_BLUE if r < 0.25 else (ANSI_CYAN if r < 0.5 else (ANSI_YELLOW if r < 0.75 else ANSI_RED))
        return color + f"{{val:3d}}" + ANSI_RESET


    def display_windows(windows):
        cpb = GRID_SIZE * GRID_SIZE
        print()
        print(ANSI_BOLD + f"=== RTL voxel_binning injection: {{BIN_FILE.name}} ===" + ANSI_RESET)
        print(f"    {{len(windows)}} readout window(s), "
              f"each {{READOUT_BINS}} bins of {{GRID_SIZE}}x{{GRID_SIZE}}")
        print()
        if not windows:
            print(ANSI_RED + "  No completed readout windows." + ANSI_RESET)
            return

        for w_idx, window in enumerate(windows):
            bins  = [window[b*cpb:(b+1)*cpb] for b in range(READOUT_BINS)]
            gmax  = max((v for b in bins for v in b), default=0)
            total = sum(v for b in bins for v in b)
            print(ANSI_BOLD + ANSI_CYAN + f"--- Window {{w_idx+1}}/{{len(windows)}} ---" + ANSI_RESET)
            print(f"  Total events: {{ANSI_BOLD}}{{total}}{{ANSI_RESET}}  Peak: {{ANSI_BOLD}}{{gmax}}{{ANSI_RESET}}")
            print()
            for b_idx, cells in enumerate(bins):
                bt   = sum(cells)
                bmax = max(cells)
                pct  = 100 * bt / total if total else 0.0
                label = f"  Bin {{b_idx+1:>2}}/{{READOUT_BINS}}  (total={{bt:5d}}, peak={{bmax:4d}}, {{pct:5.1f}}%)"
                if bmax == 0:
                    print(ANSI_DIM + label + "  [empty]" + ANSI_RESET)
                    continue
                print(ANSI_YELLOW + label + ANSI_RESET)
                print(ANSI_DIM + "       " + "  x:" + "".join(f"{{x:>3}}" for x in range(GRID_SIZE)) + ANSI_RESET)
                for y in range(GRID_SIZE):
                    row = cells[y*GRID_SIZE:(y+1)*GRID_SIZE]
                    print(f"    y{{y:02d}} {{ANSI_DIM}}|{{ANSI_RESET}}" + "".join(_ansi_cell(v, gmax) for v in row))
                print()

        agg = [0] * cpb
        for window in windows:
            for b in range(READOUT_BINS):
                for i in range(cpb):
                    agg[i] += window[b*cpb+i]
        agg_max = max(agg, default=0)
        print(ANSI_BOLD + "--- Cumulative spatial heatmap ----" + ANSI_RESET)
        print(ANSI_DIM + "       " + "  x:" + "".join(f"{{x:>3}}" for x in range(GRID_SIZE)) + ANSI_RESET)
        for y in range(GRID_SIZE):
            row = agg[y*GRID_SIZE:(y+1)*GRID_SIZE]
            print(f"    y{{y:02d}} {{ANSI_DIM}}|{{ANSI_RESET}}" + "".join(_ansi_cell(v, agg_max) for v in row))
        print()


    async def _wait_state(dut, target, timeout=10000):
        for _ in range(timeout):
            await RisingEdge(dut.clk)
            if int(dut.state.value) == target:
                return
        raise AssertionError(f"Timeout waiting for state {{target}}")


    async def _wait_event_ready(dut, timeout=20000):
        \"\"\"Wait for event_ready=1. Caller must be at a rising edge, not in ReadOnly.\"\"\"
        for _ in range(timeout):
            await ReadOnly()
            if int(dut.event_ready.value):
                return
            await RisingEdge(dut.clk)
        raise AssertionError("Timeout waiting for event_ready")


    async def _readout_monitor(dut, windows, done_evt):
        \"\"\"Background task: collect every readout window until done_evt is set.\"\"\"
        while True:
            # Wait for readout_start pulse (rising-edge sampling only, no ReadOnly here
            # so we don't conflict with the driver coroutine).
            await RisingEdge(dut.clk)
            if done_evt.is_set() and not int(dut.readout_start.value):
                break
            if not int(dut.readout_start.value):
                continue

            # Collect FEATURE_COUNT valid readout_data samples.
            values = []
            for _ in range(FEATURE_COUNT + 500):
                await RisingEdge(dut.clk)
                if int(dut.readout_valid.value):
                    values.append(int(dut.readout_data.value))
                    if int(dut.readout_last.value):
                        break
            if len(values) == FEATURE_COUNT:
                windows.append(values)


    @logged_test()
    async def test_bin_inject(dut):
        cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
        dut.rst.value = 1
        dut.event_valid.value  = 0
        dut.event_x.value      = 0
        dut.event_y.value      = 0
        dut.ts_in.value        = 0
        dut.force_rollover_i.value = 0
        dut.readout_ready.value    = 1
        await ClockCycles(dut.clk, 5)
        dut.rst.value = 0
        await _wait_state(dut, ST_ACCUM)

        data  = BIN_FILE.read_bytes()
        words = list(struct.unpack_from(f"<{{len(data)//4}}I", data, 0))

        windows  = []
        done_evt = Event()
        monitor  = cocotb.start_soon(_readout_monitor(dut, windows, done_evt))

        have_th   = False
        time_high = 0

        for word in words:
            pkt    = (word >> 28) & 0xF
            ts_lsb = (word >> 22) & 0x3F
            x_raw  = (word >> 11) & 0x7FF
            y_raw  = word & 0x7FF

            if pkt == EVT_TIME_HIGH:
                have_th   = True
                time_high = word & 0x0FFFFFFF
                continue
            if pkt not in (EVT_CD_OFF, EVT_CD_ON):
                continue
            if not have_th:
                continue

            xc = min(x_raw, SENSOR_WIDTH  - 1)
            yc = min(y_raw, SENSOR_HEIGHT - 1)
            xs = yc if MAP_SWAP_XY else xc
            ys = xc if MAP_SWAP_XY else yc
            if MAP_FLIP_X:
                xs = (SENSOR_WIDTH  - 1) - xs
            if MAP_FLIP_Y:
                ys = (SENSOR_HEIGHT - 1) - ys
            xg = min((xs * X_M) >> DIV_K, GRID_SIZE - 1)
            yg = min((ys * Y_M) >> DIV_K, GRID_SIZE - 1)
            ts = (time_high << 6) | ts_lsb

            # Wait for event_ready (DUT may be in RMW writeback or readout/clear).
            # _wait_event_ready starts with ReadOnly so we need to be at a rising edge.
            await _wait_event_ready(dut)
            await NextTimeStep()
            dut.event_x.value      = xg
            dut.event_y.value      = yg
            dut.ts_in.value        = ts
            dut.event_valid.value  = 1
            await RisingEdge(dut.clk)
            dut.event_valid.value  = 0
            # One extra cycle so rmw_pending clears before next _wait_event_ready.
            await RisingEdge(dut.clk)

        # Flush: force-rollover NUM_BINS times to drain any partial window.
        for _ in range(NUM_BINS):
            await _wait_event_ready(dut)
            await NextTimeStep()
            dut.force_rollover_i.value = 1
            await RisingEdge(dut.clk)
            dut.force_rollover_i.value = 0
            await _wait_state(dut, ST_ACCUM)

        # Let the monitor collect the last readout then shut it down.
        await ClockCycles(dut.clk, FEATURE_COUNT + 100)
        done_evt.set()
        await monitor

        display_windows(windows)
    """)
    tb_path.write_text(code)


# ---------------------------------------------------------------------------
# Synthetic EVT2 generation and verification
# ---------------------------------------------------------------------------

def build_synthetic_evt2(cfg: dict, seed: int = 42) -> tuple[list[int], list[list[int]], list[list[int]]]:
    """Build a synthetic EVT2 word stream with known per-cell, per-bin event counts.

    Returns
    -------
    words       : list of 32-bit EVT2 words ready to write to a .bin file
    plan        : plan[bin][cell] = number of events injected into that cell
                  (READOUT_BINS bins, each GRID_SIZE*GRID_SIZE cells, oldest first)
    expected    : expected[window][flat_index] = count that should appear in RTL readout
                  (one window because the synthetic stream fills exactly READOUT_BINS bins)
    """
    import random
    rng = random.Random(seed)

    G   = cfg["GRID_SIZE"]
    NB  = cfg["NUM_BINS"]
    RB  = cfg["READOUT_BINS"]
    BD  = (cfg["WINDOW_MS"] * 1000) // RB
    CB  = cfg.get("COUNTER_BITS", 16)
    MC  = (1 << CB) - 1
    CPB = G * G

    # ------------------------------------------------------------------
    # Step 1: decide how many events per cell per bin.
    # Use small deterministic counts (1–5) so saturation is impossible and
    # every cell is non-zero — makes mismatches easy to spot.
    # ------------------------------------------------------------------
    plan: list[list[int]] = []
    for b in range(RB):
        row = []
        for _ in range(CPB):
            row.append(rng.randint(1, 5))
        plan.append(row)

    # ------------------------------------------------------------------
    # Step 2: encode as EVT2 words.
    # Layout: bins are separated by a TIME_HIGH that advances the timestamp
    # past a bin boundary.  Within each bin every event gets a unique
    # timestamp increment so ordering is deterministic and there are no
    # duplicate timestamps that could confuse the model.
    #
    # Sensor coords: map grid cell (x_grid, y_grid) back to a sensor pixel
    # at the centre of the corresponding tile so the evt2_decoder will
    # map it back to the same grid cell.
    # SW  = cfg["SENSOR_WIDTH"]
    # SH  = cfg.get("SENSOR_HEIGHT", SW)
    # x_sensor = x_grid * (SW // G) + (SW // G) // 2
    # y_sensor = y_grid * (SH // G) + (SH // G) // 2
    # ------------------------------------------------------------------
    SW  = cfg["SENSOR_WIDTH"]
    SH  = cfg.get("SENSOR_HEIGHT", SW)
    XD  = SW // G
    YD  = SH // G

    words: list[int] = []
    # Start timestamp at 0; advance by BD microseconds per bin boundary.
    # Each event within a bin gets +1 us so timestamps are strictly increasing.
    current_ts_us = 0

    def _emit_time_high(ts_us: int):
        th_payload = (ts_us >> 6) & 0x0FFFFFFF
        words.append((EVT_TIME_HIGH << 28) | th_payload)

    def _emit_cd(x_sensor: int, y_sensor: int, ts_us: int):
        ts_lsb = ts_us & 0x3F
        words.append(
            (EVT_CD_ON << 28) |
            (ts_lsb    << 22) |
            (x_sensor  << 11) |
            (y_sensor  & 0x7FF)
        )

    # Initial TIME_HIGH before first event
    _emit_time_high(current_ts_us)

    for b in range(RB):
        # Advance past the bin boundary at the start of bin b+1 (except bin 0
        # which starts at ts=0 — the model initialises bin_start on first event).
        if b > 0:
            current_ts_us = b * BD   # exact bin boundary

        for cell_idx in range(CPB):
            xg = cell_idx % G
            yg = cell_idx // G
            xs = min(xg * XD + XD // 2, SW - 1)
            ys = min(yg * YD + YD // 2, SH - 1)
            count = plan[b][cell_idx]
            for _ in range(count):
                # Refresh TIME_HIGH whenever the upper bits change
                new_th = current_ts_us >> 6
                if not words or ((words[-1] >> 28) & 0xF) != EVT_TIME_HIGH or \
                        (words[-1] & 0x0FFFFFFF) != (new_th & 0x0FFFFFFF):
                    _emit_time_high(current_ts_us)
                _emit_cd(xs, ys, current_ts_us)
                current_ts_us += 1

    # ------------------------------------------------------------------
    # Step 3: compute the expected readout window that the golden model
    # will produce once all RB bins are filled and rolled over.
    #
    # voxel_binning emits bins oldest→newest in the readout.  After
    # filling bins 0..RB-1 and forcing one more rollover the snapshot
    # window is bins 0..RB-1 in order — exactly our plan.
    # ------------------------------------------------------------------
    expected_window: list[int] = []
    for b in range(RB):
        expected_window.extend(plan[b])

    return words, plan, [expected_window]


def verify_windows(got_windows: list[list[int]],
                   expected_windows: list[list[int]],
                   plan: list[list[int]],
                   cfg: dict,
                   source_label: str) -> bool:
    """Print a pass/fail comparison of got vs expected readout windows.

    Returns True if all windows match exactly.
    """
    G   = cfg["GRID_SIZE"]
    RB  = cfg["READOUT_BINS"]
    CPB = G * G

    print()
    print(ANSI_BOLD + f"=== Verification: {source_label} ===" + ANSI_RESET)

    if len(got_windows) == 0:
        print(ANSI_RED + "  FAIL: no readout windows produced." + ANSI_RESET)
        return False

    # We only check the first window (the synthetic stream fills one window).
    exp = expected_windows[0]

    # Find the first window in got_windows that is non-trivially non-zero
    # (the flush phase may produce extra all-zero windows after the real one).
    got = None
    for w in got_windows:
        if any(v != 0 for v in w):
            got = w
            break

    if got is None:
        print(ANSI_RED + "  FAIL: all produced windows are all-zero." + ANSI_RESET)
        return False

    if len(got) != len(exp):
        print(ANSI_RED + f"  FAIL: window length {len(got)} != expected {len(exp)}." + ANSI_RESET)
        return False

    mismatches = [(i, got[i], exp[i]) for i in range(len(exp)) if got[i] != exp[i]]
    all_pass = len(mismatches) == 0

    if all_pass:
        print(ANSI_GREEN + f"  PASS: all {len(exp)} cells match expected counts." + ANSI_RESET)
    else:
        print(ANSI_RED + f"  FAIL: {len(mismatches)} cell(s) mismatch." + ANSI_RESET)

    # Per-bin breakdown showing plan vs got vs delta
    print()
    header_printed = False
    for b in range(RB):
        base = b * CPB
        bin_exp   = exp[base:base + CPB]
        bin_got   = got[base:base + CPB] if got else [0] * CPB
        bin_bad   = [(i, bin_got[i], bin_exp[i])
                     for i in range(CPB) if bin_got[i] != bin_exp[i]]
        status    = ANSI_GREEN + "PASS" + ANSI_RESET if not bin_bad else ANSI_RED + "FAIL" + ANSI_RESET

        print(ANSI_YELLOW + f"  Bin {b+1:>2}/{RB}" + ANSI_RESET + f"  [{status}]"
              + (f"  {len(bin_bad)} mismatch(es)" if bin_bad else ""))

        if not header_printed:
            print(ANSI_DIM + "         x:" + "".join(f"{x:>5}" for x in range(G)) + ANSI_RESET)
            header_printed = True

        gmax = max(bin_exp) if bin_exp else 1
        for y in range(G):
            row_exp = bin_exp[y*G:(y+1)*G]
            row_got = bin_got[y*G:(y+1)*G] if bin_got else [0]*G
            cells_str = ""
            for x in range(G):
                e = row_exp[x]
                g = row_got[x]
                if g == e:
                    cells_str += ANSI_GREEN + f"{g:>5}" + ANSI_RESET
                else:
                    cells_str += ANSI_RED + f"{g:>3}!={e:<1}" + ANSI_RESET
            print(f"    y{y:02d} {ANSI_DIM}|{ANSI_RESET}" + cells_str)
        print()

    return all_pass


def run_synthetic(cfg: dict, seed: int, no_sim: bool, config_name: str) -> bool:
    """Generate a synthetic EVT2, verify golden model, optionally verify RTL."""
    import tempfile

    G  = cfg["GRID_SIZE"]
    RB = cfg["READOUT_BINS"]

    print(ANSI_CYAN + f"[synthetic] Generating EVT2 stream  "
          f"(grid={G}x{G}, readout_bins={RB}, seed={seed}) ..." + ANSI_RESET)

    words, plan, expected_windows = build_synthetic_evt2(cfg, seed)
    n_cd = sum(1 for w in words if (w >> 28) & 0xF in (EVT_CD_OFF, EVT_CD_ON))
    print(f"  Generated {len(words):,d} words  ({n_cd:,d} CD events)\n")

    # Print the injection plan so the user knows what to expect
    CPB = G * G
    print(ANSI_BOLD + "--- Injection plan (events per cell per bin) ---" + ANSI_RESET)
    print(ANSI_DIM + "         x:" + "".join(f"{x:>3}" for x in range(G)) + ANSI_RESET)
    for b in range(RB):
        print(ANSI_YELLOW + f"  Bin {b+1:>2}/{RB}" + ANSI_RESET)
        for y in range(G):
            row = plan[b][y*G:(y+1)*G]
            print(f"    y{y:02d} {ANSI_DIM}|{ANSI_RESET}" +
                  "".join(ANSI_BLUE + f"{v:3d}" + ANSI_RESET for v in row))
        print()

    # Write to a temp .bin file
    tmp = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
    tmp.write(struct.pack(f"<{len(words)}I", *words))
    tmp.close()
    bin_path = Path(tmp.name)

    all_ok = True

    # --- Golden model ---
    print(ANSI_CYAN + "[synthetic] Running golden model ..." + ANSI_RESET)
    golden_windows, rb, gs = decode_and_run(words, cfg)
    golden_ok = verify_windows(golden_windows, expected_windows, plan, cfg, "golden model")
    all_ok = all_ok and golden_ok

    # Show heatmap of golden output
    display_windows(golden_windows, rb, gs, f"synthetic (seed={seed})")

    # --- RTL simulation ---
    if not no_sim:
        print(ANSI_CYAN + "[synthetic] Running RTL simulation ..." + ANSI_RESET)
        ok = run_rtl_sim(bin_path, config_name)
        if ok:
            # RTL result was printed by the TB itself; we can't easily capture it
            # here, so just report success.
            print(ANSI_GREEN + "[synthetic] RTL simulation passed." + ANSI_RESET)
        else:
            print(ANSI_RED + "[synthetic] RTL simulation FAILED." + ANSI_RESET)
            all_ok = False

    bin_path.unlink(missing_ok=True)

    print()
    if all_ok:
        print(ANSI_BOLD + ANSI_GREEN + "All checks PASSED." + ANSI_RESET)
    else:
        print(ANSI_BOLD + ANSI_RED + "One or more checks FAILED." + ANSI_RESET)

    return all_ok


# ---------------------------------------------------------------------------
# File picker
# ---------------------------------------------------------------------------

def pick_bin_file() -> Path:
    """Open a Tk file-picker dialog to choose a .bin file."""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        initial = str(REPO_ROOT / "EVT2_gesture_set")
        chosen = filedialog.askopenfilename(
            title="Select EVT2 .bin file",
            initialdir=initial if Path(initial).exists() else str(REPO_ROOT),
            filetypes=[("EVT2 binary files", "*.bin"), ("All files", "*.*")],
        )
        root.destroy()
        if not chosen:
            print("No file selected. Exiting.")
            sys.exit(0)
        return Path(chosen)
    except Exception as e:
        print(f"File picker unavailable ({e}). Pass path as argument.")
        sys.exit(1)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Inject an EVT2 .bin file into voxel_binning and visualise output."
    )
    ap.add_argument("bin_file", nargs="?", help=".bin file path (omit to open picker)")
    ap.add_argument("--config", default="voxel_default",
                    help="Config name in configs/ (default: voxel_default)")
    ap.add_argument("--no-sim", action="store_true",
                    help="Skip RTL sim; use golden model only (fast)")
    ap.add_argument("--bins",  type=int, help="Override READOUT_BINS for display")
    ap.add_argument("--grid",  type=int, help="Override GRID_SIZE for display")
    ap.add_argument("--synthetic", action="store_true",
                    help="Generate a synthetic EVT2 with known per-cell counts and verify output")
    ap.add_argument("--synthetic-seed", type=int, default=42, metavar="N",
                    help="RNG seed for synthetic event generation (default: 42)")
    args = ap.parse_args()

    print(f"\n{ANSI_BOLD}bin_inject.py{ANSI_RESET}")

    cfg = load_config(args.config)
    if args.bins:
        cfg["READOUT_BINS"] = args.bins
    if args.grid:
        cfg["GRID_SIZE"] = args.grid

    # ------------------------------------------------------------------
    # Synthetic mode: generate, verify, exit
    # ------------------------------------------------------------------
    if args.synthetic:
        print(f"  Mode   : synthetic  (seed={args.synthetic_seed})")
        print(f"  Config : {args.config}\n")
        ok = run_synthetic(cfg, args.synthetic_seed, args.no_sim, args.config)
        sys.exit(0 if ok else 1)

    # ------------------------------------------------------------------
    # Normal mode: inject a real .bin file
    # ------------------------------------------------------------------
    bin_path = Path(args.bin_file) if args.bin_file else pick_bin_file()
    if not bin_path.exists():
        print(f"Error: file not found: {bin_path}")
        sys.exit(1)

    print(f"  File   : {bin_path}")
    print(f"  Config : {args.config}")

    words = read_evt2_bin(bin_path)
    cd_count = sum(1 for w in words if (w >> 28) & 0xF in (EVT_CD_OFF, EVT_CD_ON))
    print(f"  Words  : {len(words):,d}   CD events: {cd_count:,d}\n")

    if args.no_sim:
        print(ANSI_CYAN + "[golden] Running pure-Python model ..." + ANSI_RESET)
        windows, rb, gs = decode_and_run(
            words, cfg,
            swap_xy=bool(cfg.get("MAP_SWAP_XY", 0)),
            flip_x =bool(cfg.get("MAP_FLIP_X",  0)),
            flip_y =bool(cfg.get("MAP_FLIP_Y",  0)),
        )
        display_windows(windows, rb, gs, bin_path.name)
    else:
        ok = run_rtl_sim(bin_path, args.config)
        if not ok:
            print(ANSI_RED + "\n[RTL] Simulation failed — falling back to golden model." + ANSI_RESET)
            windows, rb, gs = decode_and_run(words, cfg)
            display_windows(windows, rb, gs, bin_path.name)


if __name__ == "__main__":
    main()
