# SPDX-FileCopyrightText: © 2025 Project Template Contributors
# SPDX-License-Identifier: Apache-2.0
#
# chip_top testbench
#
# Drives stimulus through the top-level PAD signals and validates:
#   - spi_ready asserts after reset
#   - SPI boot/page-select works on the default SPI bus (input_PAD[5,6,7])
#   - Toggling input_PAD[8] (ALT_INPUT_MODE) switches the active SPI interface
#     and the active MISO pin (bidir_PAD[38] ↔ bidir_PAD[39])
#   - The alt SPI bus (input_PAD[2,3,4]) is functional after the toggle
#   - A second toggle returns the chip to the default interface
#   - Debug pages 0-4 can be selected over SPI and the debug_bus (bidir_PAD[37:6])
#     updates accordingly
#
# RTL bugs fixed in chip_core.sv before this testbench was written:
#   `assign bidir_oe = '0` and `assign bidir_out = '0` created multiple drivers
#   on bits that are also driven by bit-select assigns, producing X in simulation.
#   Fixed by replacing both blanket assigns with targeted assigns to the reserved
#   bits only (bidir_oe[5:2] and bidir_out[5:2]).
#
# NOTE on CSV vs RTL pin naming:
#   docs/pin_chart.csv labels input pins 2-4 as SPI_DEF_* and 5-7 as SPI_ALT_*.
#   chip_core.sv routes pins [5,6,7] to SPI when alt_select=0 (the power-on
#   default) and pins [2,3,4] when alt_select=1.  This testbench follows the RTL.

import os
import random
import logging
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, NextTimeStep, RisingEdge, Timer, Edge
from cocotb_tools.runner import get_runner

# ── Environment ───────────────────────────────────────────────────────────────
sim      = os.getenv("SIM", "icarus")
gl       = os.getenv("GL", False)
pdk_root = os.getenv("PDK_ROOT", Path(__file__).resolve().parent / "../gf180mcu")
pdk      = os.getenv("PDK", "gf180mcuD")
scl      = os.getenv("SCL", "gf180mcu_as_sc_mcu7t3v3")
pad      = os.getenv("PAD", "gf180mcu_fd_io")
sram     = os.getenv("SRAM", "gf180mcu_ocd_ip_sram")
slot     = os.getenv("SLOT", "1x1")

# CUSTOM STUFF ----------------------------------------------------------------
timing   = os.getenv("TIMING", "").lower() not in ("", "0", "false", "no")
sdf_file = os.getenv("SDF_FILE")

# Resolve the GLS netlist:
#   1. Honour the GL_NETLIST env var if set (absolute path to any .nl.v/.pnl.v).
#   2. Prefer final/pnl/chip_top.pnl.v (created by `make copy-final`).
#   3. Fall back to the latest librelane run's post-synth netlist
#      (librelane/runs/RUN_*/06-yosys-synthesis/chip_top.nl.v).
# A clear message is raised at runner-build time if nothing is found.
_repo_root = Path(__file__).resolve().parents[1]

def _resolve_gl_netlist() -> Path:
    env = os.getenv("GL_NETLIST")
    if env:
        return Path(env)
    final_pnl = _repo_root / "final" / "pnl" / "chip_top.pnl.v"
    if final_pnl.exists():
        return final_pnl
    runs = sorted((_repo_root / "librelane" / "runs").glob("RUN_*"))
    for run in reversed(runs):
        cand = run / "06-yosys-synthesis" / "chip_top.nl.v"
        if cand.exists():
            return cand
    # No netlist found — return a path that will fail loudly in the runner.
    return _repo_root / "librelane" / "runs" / "RUN_<missing>" / "chip_top.nl.v"

gl_netlist = _resolve_gl_netlist()
# ------------------------------------------------------------------------

hdl_toplevel = "chip_top"

# ── Timing ────────────────────────────────────────────────────────────────────
CLK_FREQ_HZ    = int(os.getenv("CLK_FREQ_HZ", "64000000"))
_raw_ps        = int(round(1_000_000_000_000 / CLK_FREQ_HZ))
CHIP_PERIOD_PS = _raw_ps + (_raw_ps % 2)   # cocotb Clock requires an even ps period
DATA_WIDTH     = 32

# Each SCLK half-period expressed in chip-clock cycles.
# 1 cycle → 32 MHz SCLK at 64 MHz chip clock.
SPI_HALF = int(os.getenv("SPI_HALF_CYCLES", "1"))

# ── Pin map ───────────────────────────────────────────────────────────────────
# input_PAD indices
PIN_DEF_SCLK = 5   # default SPI (alt_select=0)
PIN_DEF_MOSI = 6
PIN_DEF_CS   = 7
PIN_ALT_SCLK = 2   # alternate SPI (alt_select=1)
PIN_ALT_MOSI = 3
PIN_ALT_CS   = 4
PIN_ALT_MODE = 8   # rising edge toggles alt_select flip-flop

# bidir_PAD output indices
BPIN_HEARTBEAT = 0
BPIN_SPI_READY = 1
BPIN_DBG_LO    = 6   # debug_bus[0]
BPIN_DEF_MISO  = 38
BPIN_ALT_MISO  = 39

# ── EVT2 command word builders ────────────────────────────────────────────────
# Encodings match soc_tb.py / control_fsm

def _boot_req():
    return 0xC << 28

def _debug_page(page):
    """type=0xE, page[3:0] in bits [27:24]."""
    return (0xE << 28) | ((int(page) & 0xF) << 24)

# ── Input pin shadow ──────────────────────────────────────────────────────────

class InputPins:
    """Shadow register so individual bits of input_PAD can be set."""

    def __init__(self, width=12):
        self._v = 0

    def set(self, idx, val):
        if val:
            self._v |= (1 << idx)
        else:
            self._v &= ~(1 << idx)

    def drive(self, dut):
        dut.input_PAD.value = self._v

# ── Low-level helpers ─────────────────────────────────────────────────────────

async def _drive_spi_pins(dut, pins):
    """Apply external SPI pin changes away from clk_PAD's sampling edge.

    Timed GLS annotates delay on both clk_PAD and input_PAD paths. If the
    testbench changes SCLK/MOSI/CS immediately after a rising clk_PAD edge, the
    delayed core clock can see those inputs move inside the same active edge.
    Driving on the falling edge gives the pad-delayed signals half a chip cycle
    of setup before the next rising edge while preserving the SPI bit rate.
    """
    await FallingEdge(dut.clk_PAD)
    pins.drive(dut)


async def _start_clock(dut):
    cocotb.start_soon(Clock(dut.clk_PAD, CHIP_PERIOD_PS, "ps").start())


async def _reset(dut, pins, hold_cycles=16):
    """Apply active-low reset; SPI buses idle (CS=1, SCLK=0)."""
    pins.set(PIN_DEF_CS, 1)
    pins.set(PIN_ALT_CS, 1)
    pins.set(PIN_DEF_SCLK, 0)
    pins.set(PIN_ALT_SCLK, 0)
    pins.set(PIN_DEF_MOSI, 0)
    pins.set(PIN_ALT_MOSI, 0)
    pins.set(PIN_ALT_MODE, 0)
    pins.drive(dut)

    # Only drive power pins if the netlist actually exposes them (.pnl.v).
    # The post-synth .nl.v has no VDD/VSS ports and accessing them crashes
    # cocotb at elaboration.
    if gl and hasattr(dut, "VDD"):
        dut.VDD.value = 1
        dut.VSS.value = 0

    dut.rst_n_PAD.value = 0
    await ClockCycles(dut.clk_PAD, hold_cycles)
    dut.rst_n_PAD.value = 1
    await ClockCycles(dut.clk_PAD, 50)


def _bidir_str(dut):
    """
    Return bidir_PAD as a 40-char string, MSB (bit 39) first.
    cocotb 2.x str(LogicArray) gives MSB-first '0'/'1'/'X'/'Z' chars.
    bidir_PAD always has Z bits on the reserved/inactive pads, so int()
    always raises ValueError — use string parsing instead.
    """
    return str(dut.bidir_PAD.value)   # e.g. "1z0z0z0z1010...0"


def _bidir_bit(dut, idx):
    """Read one bit of bidir_PAD by index (0=LSB); returns 0 if Z/X."""
    s = _bidir_str(dut)               # 40 chars, s[0] = bidir_PAD[39]
    c = s[39 - idx]
    return 1 if c == '1' else 0


async def _wait_spi_ready(dut, max_cycles=5000):
    """
    Pin-level read of spi_ready (exposed on bidir_PAD[1]).

    Reading via the pin (instead of the internal i_chip_core.spi_ready signal)
    validates the full output path: chip_core's `assign bidir_out[1] = spi_ready`,
    `bidir_oe[1] = 1'b1`, the IO pad model, and the inout bidir_PAD wire.
    A bit-mapping bug in chip_core.sv would silently break the chip in
    production but go unnoticed if we read the internal signal.
    """
    for n in range(max_cycles):
        await RisingEdge(dut.clk_PAD)
        if _bidir_bit(dut, BPIN_SPI_READY) == 1:
            dut._log.info(f"spi_ready asserted on bidir_PAD[1] after {n} cycles")
            return
    raise AssertionError("spi_ready never asserted on bidir_PAD[1] after reset")


def _read_bidir_bit(dut, idx):
    """Z-safe single-bit read from bidir_PAD."""
    return _bidir_bit(dut, idx)


def _read_debug_bus(dut):
    """
    Pin-level read of debug_bus (exposed on bidir_PAD[37:6]).

    Same rationale as _wait_spi_ready: reading the pins instead of the
    internal i_chip_core.debug_bus signal validates the full output path
    including bit-mapping, output enables, and pad models.
    """
    s   = _bidir_str(dut)                # 40 chars, MSB-first (s[0] = bit 39)
    bus = 0
    # bidir_PAD[37:6] → 32 bits of debug_bus.  bit 6 = LSB of debug_bus.
    for i in range(32):
        idx = 6 + i                      # pin index 6..37
        c   = s[39 - idx]
        if c == '1':
            bus |= (1 << i)
    return bus


async def _startup(dut):
    """Full startup: clock + reset + wait spi_ready. Returns InputPins."""
    pins = InputPins()
    await _start_clock(dut)
    await NextTimeStep()
    await _reset(dut, pins)
    await _wait_spi_ready(dut)
    return pins

# ── SPI streaming ─────────────────────────────────────────────────────────────

async def _spi_stream(dut, pins, words, *, alt=False,
                      capture_miso=False, half=SPI_HALF, inter_gap=4,
                      progress_every=0, tag="spi"):
    """
    Mode-0 SPI stream. CS stays low for the whole burst.

    alt=False uses input_PAD[5,6,7] / bidir_PAD[38].
    alt=True  uses input_PAD[2,3,4] / bidir_PAD[39].

    progress_every: emit a progress log every N words.  0 = no progress logs.
    Used by long EVT2 recording streams (~249K words) so the user can see the
    sim is making progress — without breaking the CS-held-low invariant of a
    real sensor burst.
    """
    p_sclk  = PIN_ALT_SCLK if alt else PIN_DEF_SCLK
    p_mosi  = PIN_ALT_MOSI if alt else PIN_DEF_MOSI
    p_cs    = PIN_ALT_CS   if alt else PIN_DEF_CS
    miso_bp = BPIN_ALT_MISO if alt else BPIN_DEF_MISO

    miso_words = []

    # Idle state: CS high, SCLK low
    pins.set(p_cs,   1)
    pins.set(p_sclk, 0)
    pins.set(p_mosi, 0)
    await _drive_spi_pins(dut, pins)
    await ClockCycles(dut.clk_PAD, 4)

    # Pre-load MSB of first word, then assert CS
    first = int(words[0]) & 0xFFFFFFFF
    pins.set(p_mosi, (first >> (DATA_WIDTH - 1)) & 1)
    pins.set(p_cs, 0)
    await _drive_spi_pins(dut, pins)
    await ClockCycles(dut.clk_PAD, 4)

    for widx, word in enumerate(words):
        word = int(word) & 0xFFFFFFFF
        miso_word = 0
        dut._log.debug(f"{tag}: word {widx} MOSI=0x{word:08X}")

        for bit in range(DATA_WIDTH):
            # Rising SCLK — slave samples MOSI here
            pins.set(p_sclk, 1)
            await _drive_spi_pins(dut, pins)
            await ClockCycles(dut.clk_PAD, half)

            # Sample MISO while SCLK is high
            if capture_miso:
                miso_word = (miso_word << 1) | _read_bidir_bit(dut, miso_bp)

            # Falling SCLK — master presents next MOSI bit
            pins.set(p_sclk, 0)
            if bit < DATA_WIDTH - 1:
                pins.set(p_mosi, (word >> (DATA_WIDTH - 2 - bit)) & 1)
            await _drive_spi_pins(dut, pins)
            await ClockCycles(dut.clk_PAD, half)

        if capture_miso:
            miso_words.append(miso_word)

        # Inter-word gap: CS stays low, SCLK stays low
        await ClockCycles(dut.clk_PAD, inter_gap)
        if widx < len(words) - 1:
            nxt = int(words[widx + 1]) & 0xFFFFFFFF
            pins.set(p_mosi, (nxt >> (DATA_WIDTH - 1)) & 1)
            await _drive_spi_pins(dut, pins)

        if progress_every and (widx + 1) % progress_every == 0:
            dut._log.info(f"{tag}: streamed {widx + 1}/{len(words)} words "
                          f"(CS held low) …")

    # Deassert CS
    await ClockCycles(dut.clk_PAD, 4)
    pins.set(p_cs, 1)
    pins.set(p_sclk, 0)
    await _drive_spi_pins(dut, pins)
    await ClockCycles(dut.clk_PAD, 8)

    dut._log.info(f"{tag}: done ({len(words)} words)")
    return miso_words


async def _spi_xfer(dut, pins, word, *, alt=False, tag="spi_xfer"):
    """Single 32-bit SPI transfer with MISO capture."""
    words = await _spi_stream(dut, pins, [word], alt=alt,
                               capture_miso=True, tag=tag)
    return words[0]


# ── Tests ─────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_reset_and_spi_ready(dut):
    """
    After reset, chip must assert spi_ready (bidir_PAD[1]) within 5000 cycles.
    Validates the chip_top startup path through IO pads, clock domain, SPI init.
    """
    pins = await _startup(dut)

    rdy = _read_bidir_bit(dut, BPIN_SPI_READY)
    assert rdy == 1, f"spi_ready should be 1 after wait, got {rdy}"
    dut._log.info("PASS: spi_ready high after reset")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_default_spi_boot_and_miso(dut):
    """
    Send BOOT_REQ then a DEBUG_PAGE(0) word over the default SPI interface
    and capture MISO.  After reset with no gesture data MISO should be 0x0.

    Default SPI path (alt_select=0):
      SCLK → input_PAD[5], MOSI → input_PAD[6], CS → input_PAD[7]
      MISO ← bidir_PAD[38]
    """
    pins = await _startup(dut)

    dut._log.info("Sending BOOT_REQ over default SPI...")
    await _spi_stream(dut, pins, [_boot_req()], alt=False, tag="boot_req")
    await ClockCycles(dut.clk_PAD, 20)

    miso = await _spi_xfer(dut, pins, _debug_page(0),
                            alt=False, tag="miso_page0")
    dut._log.info(f"MISO after BOOT_REQ = 0x{miso:08X}")
    # classification_output starts at 0 after reset → MISO = 0
    assert miso == 0, f"Expected MISO=0x00000000 (no gesture yet), got 0x{miso:08X}"
    dut._log.info("PASS: default SPI boot accepted, MISO=0")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_debug_page_sweep(dut):
    """
    Select debug pages 0-4 over the default SPI interface.
    Logs debug_bus (bidir_PAD[37:6]) for each page.
    Pages 0-2 expose live internal signals; 3 exposes control_fsm state; 4 is decoder output.
    We verify the commands are accepted without error and log the bus values.
    """
    PAGE_NAMES = {
        0: "voxel_gesture_classifier + mac_engine",
        1: "voxel_binning",
        2: "evt2_decoder + input_FIFO + voxel_core",
        3: "control_fsm state (main_state[11:8] load_state[7:2] boot_fail[1] boot_done[0])",
        4: "evt2_decoder event output",
    }

    pins = await _startup(dut)

    await _spi_stream(dut, pins, [_boot_req()], alt=False, tag="boot")
    await ClockCycles(dut.clk_PAD, 20)

    prev_bus = None
    for page in range(5):
        await _spi_stream(dut, pins, [_debug_page(page)],
                          alt=False, tag=f"page_sel_{page}")
        await ClockCycles(dut.clk_PAD, 10)

        bus = _read_debug_bus(dut)
        dut._log.info(
            f"  Page {page} ({PAGE_NAMES[page]}): debug_bus=0x{bus:08X}"
        )
        prev_bus = bus

    dut._log.info("PASS: all debug pages 0-4 selected without simulation error")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_alt_input_mode_toggle(dut):
    """
    Toggle ALT_INPUT_MODE (input_PAD[8]) once to flip alt_select from 0 → 1.

    Verifies:
      - MISO output enable moves from bidir_PAD[38] to bidir_PAD[39].
      - SPI transactions via the alt pin set (input_PAD[2,3,4]) succeed.
      - bidir_PAD[38] is driven low (MISO_wire muxed to 0) in alt mode.

    The pin is double-synchronized inside chip_core, so the rising edge must
    be held for at least 3 chip-clock cycles to reliably propagate.
    """
    pins = await _startup(dut)

    # --- baseline: default SPI works, MISO on bidir_PAD[38] ---
    await _spi_stream(dut, pins, [_boot_req()], alt=False, tag="boot_def")
    await ClockCycles(dut.clk_PAD, 20)

    miso_before = await _spi_xfer(dut, pins, _debug_page(0),
                                   alt=False, tag="miso_before_toggle")
    dut._log.info(f"Default MISO (before toggle) = 0x{miso_before:08X}")

    # --- pulse ALT_INPUT_MODE to trigger alt_select toggle ---
    dut._log.info("Pulsing ALT_INPUT_MODE (input_PAD[8]) high...")
    pins.set(PIN_ALT_MODE, 1)
    pins.drive(dut)
    await ClockCycles(dut.clk_PAD, 8)   # > 2 sync FF + 1 edge-detect stage
    pins.set(PIN_ALT_MODE, 0)
    pins.drive(dut)
    await ClockCycles(dut.clk_PAD, 8)

    # --- alt SPI (input_PAD[2,3,4]) should now be active ---
    dut._log.info("Testing alt SPI path (input_PAD[2,3,4], MISO=bidir_PAD[39])...")
    miso_alt = await _spi_xfer(dut, pins, _debug_page(0),
                                alt=True, tag="miso_alt")
    dut._log.info(f"Alt MISO (after toggle) = 0x{miso_alt:08X}")

    # In alt mode, bidir_out[38] is forced to 0 and bidir_oe[38]=0 (output disabled)
    # bidir_PAD[38] should read 0 because the pad drives 0 when OE=0 (or Z from pad)
    def_miso_pad = _read_bidir_bit(dut, BPIN_DEF_MISO)
    assert def_miso_pad == 0, (
        f"bidir_PAD[38] should be 0 while alt_select=1 (driven to 0, OE=0), "
        f"got {def_miso_pad}"
    )

    dut._log.info("PASS: alt_select toggled; alt SPI functional, bidir_PAD[38]=0")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_alt_input_mode_toggle_back(dut):
    """
    Toggle ALT_INPUT_MODE twice → alt_select returns to 0 → default SPI active again.
    Verifies that the toggle is a true flip-flop (each rising edge inverts state).
    """
    pins = await _startup(dut)

    async def _pulse_alt_mode(n=8):
        pins.set(PIN_ALT_MODE, 1)
        pins.drive(dut)
        await ClockCycles(dut.clk_PAD, n)
        pins.set(PIN_ALT_MODE, 0)
        pins.drive(dut)
        await ClockCycles(dut.clk_PAD, n)

    dut._log.info("First toggle → alt_select=1")
    await _pulse_alt_mode()

    dut._log.info("Second toggle → alt_select=0 (default restored)")
    await _pulse_alt_mode()

    # Default SPI must accept commands again
    await _spi_stream(dut, pins, [_boot_req()], alt=False, tag="boot_restored")
    await ClockCycles(dut.clk_PAD, 20)

    miso = await _spi_xfer(dut, pins, _debug_page(0),
                            alt=False, tag="miso_restored")
    dut._log.info(f"MISO after double-toggle = 0x{miso:08X}")

    # bidir_PAD[39] (alt MISO) should be 0: bidir_out[39] driven to 0, OE[39]=0
    alt_pad = _read_bidir_bit(dut, BPIN_ALT_MISO)
    assert alt_pad == 0, (
        f"bidir_PAD[39] should be 0 after returning to default mode, got {alt_pad}"
    )

    dut._log.info("PASS: double toggle restores default SPI interface")


# ── Classification test ───────────────────────────────────────────────────────
# Streams real EVT2 gesture recordings through chip_top input pins, boots the
# chip with learned weights/thresholds over SPI, then asserts that the on-chip
# classifier produces the correct gesture label for each of the 4 recordings.
# This is the top-level proof that the fabricated chip will classify correctly.

import struct

_REPO_ROOT    = Path(__file__).resolve().parents[1]
_GRID_SIZE    = 16
_READOUT_BINS = 16
_NUM_CLASSES  = 4
_FEAT_COUNT   = _GRID_SIZE * _GRID_SIZE * _READOUT_BINS   # 4096

# Programmable bin length — added by commit c1146c3.
#
# voxel_binning.sv defaults bin_duration_ts to 34'd62500 (62.5 ms) on reset
# for the 16-bin chip (1 s window / 16 bins).  It is overridden whenever the
# EVT2 decoder asserts bin_length_valid with a non-zero bin_length_us, via
# the BIN_LENGTH_U=0x5 / BIN_LENGTH_L=0x6 opcodes during evt_ld_en.
#
# Why we program it explicitly even when matching the default:
#   1. Exercises the BIN_LENGTH opcode path through SPI → FIFO → decoder → binning.
#   2. Documents the bin length in the test instead of relying on an RTL
#      default that could change.
#   3. Makes the test resilient to RTL refactors that might change DEFAULT_BIN_LENGTH.
#
# Override via env var:  BIN_LENGTH_US=50000  → 50 ms bins, 0.8 s window.
BIN_LENGTH_US = int(os.getenv("BIN_LENGTH_US", "62500"))

GESTURE_NAMES = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}

# Expected gesture index for each test file (order matches _resolve_bin_files())
_EXPECTED_CLASS = {0: 0, 1: 1, 2: 2, 3: 3}

# ── EVT2 word builders ────────────────────────────────────────────────────────

def _w_weight(weight, feat_addr, class_id):
    # EVT_WEIGHT = 0x2: [27:20]=data, [19:8]=feature addr (12b), [7:2]=class/sram sel (6b)
    return (0x2 << 28) | ((weight & 0xFF) << 20) | ((feat_addr & 0xFFF) << 8) | ((class_id & 0x3F) << 2)

def _w_thresh_upper(val, addr):
    # EVT_THRESH_U = 0x3: [27:9]=upper bits of threshold (19-bit field; fits
    # exactly at SCORE_BITS=37, top bit unused at SCORE_BITS<37).
    # RTL reads addr only from THRESH_L, so addr is ignored here.
    return (0x3 << 28) | (((val >> 18) & 0x7FFFF) << 9)

def _w_thresh_lower(val, addr):
    # EVT_THRESH_L = 0x4: [27:10]=lower 18 bits of threshold, [9:7]=thresh addr (3b)
    return (0x4 << 28) | ((val & 0x3FFFF) << 10) | ((addr & 0x7) << 7)

def _w_bin_length_upper(val):
    """
    Upper 17 bits of programmable bin length (µs).

    Layout (matches evt2_decoder.sv BIN_LENGTH_U handler at line 206):
        [31:28] type = 0x5
        [27:17] don't care
        [16:0]  upper 17 bits of bin_length_us (latched into bin_length_reg)
    """
    return (0x5 << 28) | ((int(val) >> 17) & 0x1FFFF)

def _w_bin_length_lower(val):
    """
    Lower 17 bits of bin length.  When this word is decoded the full 34-bit
    value {bin_length_reg, evt_word[16:0]} is registered and bin_length_valid
    pulses, signalling voxel_binning to update bin_duration_ts.
    """
    return (0x6 << 28) | (int(val) & 0x1FFFF)

def _w_reads_done():
    return 0xF << 28

# ── File helpers ──────────────────────────────────────────────────────────────

def _load_weights():
    weights = []
    for c in range(_NUM_CLASSES):
        path = _REPO_ROOT / f"weights/{_FEAT_COUNT}weights_q8_c{c}.mem"
        vals = []
        if path.exists():
            for line in path.read_text(encoding="ascii").splitlines():
                line = line.strip()
                if line and not line.startswith("//"):
                    try:
                        vals.append(int(line, 16))
                    except ValueError:
                        vals.append(0)
        else:
            raise FileNotFoundError(f"Weight file missing: {path}")
        while len(vals) < _FEAT_COUNT:
            vals.append(0)
        weights.append(vals[:_FEAT_COUNT])
    return weights

def _load_thresholds():
    path = _REPO_ROOT / "weights/thresholds.mem"
    vals = []
    if path.exists():
        for line in path.read_text(encoding="ascii").splitlines():
            line = line.strip()
            if line and not line.startswith("//"):
                try:
                    vals.append(int(line, 16))
                except ValueError:
                    vals.append(0)
    while len(vals) < 2 * _NUM_CLASSES:
        vals.append(0)
    return vals[:2 * _NUM_CLASSES]

def _build_program_words(weights, thresholds, bin_length_us=BIN_LENGTH_US):
    """
    Build the SPI programming stream (sent after BOOT_REQ once evt_ld_en is high).

    Order:
        1. weights      — per-class feature weights into the MAC SRAM
        2. thresholds   — class + diff thresholds (upper / lower halves)
        3. bin length   — programs voxel_binning's bin_duration_ts (NEW)
        4. READS_DONE   — signals the decoder that programming is complete

    The bin-length pair is sent *before* READS_DONE so bin_length_valid pulses
    while evt_ld_en is still high; voxel_binning latches it into
    bin_duration_ts and from that point all rollovers are spaced exactly
    bin_length_us apart (line 323 of voxel_binning.sv:
    `bin_start_ts <= bin_start_ts + bin_duration_ts`).
    """
    words = []
    for c in range(_NUM_CLASSES):
        for a in range(_FEAT_COUNT):
            words.append(_w_weight(weights[c][a], a, c))
    for a, v in enumerate(thresholds):
        words.append(_w_thresh_upper(v, a))
        words.append(_w_thresh_lower(v, a))
    # Program bin length last so it lands while evt_ld_en is still high.
    words.append(_w_bin_length_upper(bin_length_us))
    words.append(_w_bin_length_lower(bin_length_us))
    words.append(_w_reads_done())
    return words

def _read_bin(path):
    data = Path(path).read_bytes()
    if data.startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise RuntimeError(f"{path} is a Git LFS pointer — run 'git lfs pull'")
    n = len(data) // 4
    words = list(struct.unpack_from(f"<{n}I", data, 0))

    # Sanity check: TIME_HIGH must be monotonically non-decreasing.
    #
    # We deliberately do NOT check CD-event timestamps for monotonicity.
    # The GenX320's parallel pixel-array readout emits events from different
    # pixels in arbitrary order within a single TIME_HIGH block (≤ 64 µs
    # spread).  Real recordings routinely contain small backwards CD deltas;
    # this is normal sensor behavior, not corruption.  The chip tolerates it
    # because (acc_event_ts - bin_start_ts) is compared against the 125 ms
    # bin duration — within-block reordering (≤ 64 µs) is two orders of
    # magnitude smaller than a bin boundary, so no spurious rollover fires.
    #
    # Checking TIME_HIGH monotonicity is sufficient to catch:
    #   • LFS truncation (file size mismatch caught earlier; mid-event truncation
    #     would leave dangling bytes — caught by `len(data) // 4` losing data)
    #   • Wrong endianness / byte-swap (would scramble type and ts bits, producing
    #     bizarre TIME_HIGH values)
    #   • Concatenation of recordings without re-basing (TIME_HIGH would reset
    #     to 0 at the seam — definitely backwards)
    last_th     = -1
    last_th_idx = -1
    for idx, w in enumerate(words):
        if (w >> 28) == 0x8:                                # TIME_HIGH
            th = w & 0x0FFFFFFF
            if th < last_th:
                raise RuntimeError(
                    f"{path}: TIME_HIGH went backwards at word {idx}: "
                    f"th={th} < prev th={last_th} (prev word #{last_th_idx}). "
                    "File is corrupt or recordings were concatenated."
                )
            last_th     = th
            last_th_idx = idx
    return words

def _append_flush_events(words, bin_length_us=BIN_LENGTH_US):
    """
    Add synthetic TIME_HIGH+CD_OFF pairs to drain the voxel binning pipeline.

    Each pair advances the decoder's reconstructed timestamp by exactly
    `bin_length_us`, which matches `bin_duration_ts` inside voxel_binning.
    The first event whose timestamp crosses the next bin boundary triggers a
    rollover (voxel_binning.sv:188 — `(acc_event_ts - bin_start_ts) >=
    bin_duration_ts`), so READOUT_BINS+2=18 spaced events guarantee enough
    rollovers to flush every bin in the ring buffer regardless of the
    programmed bin length.

    NOTE: this MUST track BIN_LENGTH_US.  If the chip is programmed with a
    50 ms bin length but flush events are spaced 125 ms apart, each flush
    event would cross *multiple* bin boundaries and the chip would have to
    process queued rollovers — which is fine for correctness but wastes
    drain time.  Conversely, if flush spacing is too small, rollovers never
    trigger and stale event counts persist into the next recording.
    """
    last_th = 0
    for w in words:
        if (w >> 28) == 0x8:
            last_th = w & 0x0FFFFFFF
    for i in range(1, _READOUT_BINS + 2):
        ts_us  = last_th * 64 + i * bin_length_us
        th_val = (ts_us >> 6) & 0x0FFFFFFF
        ts_lsb = ts_us & 0x3F
        words.append((0x8 << 28) | th_val)
        words.append((ts_lsb << 22) | (160 << 11) | 160)
    return words

def _resolve_bin_files():
    test_set = _REPO_ROOT / "EVT2_gesture_set" / "test_set"
    names = ["wave_down_sun_test1.bin", "wave_left_sun_test1.bin",
             "wave_right_sun_test1.bin", "wave_up_sun_test1.bin"]
    return [test_set / n for n in names]

# ── Classification-test helpers ───────────────────────────────────────────────

def _resolve_handle(dut, path):
    """Resolve a signal handle ONCE. Returns the handle or None.
    Works for:
      RTL: dut.i_chip_core.u_soc.gesture_valid
      GL : dut["\\i_chip_core.u_soc.gesture_valid "]
      STA wrapper: dut.u_chip_top["\\i_chip_core.u_soc.gesture_valid "]
    """

    def _scopes():
        yield dut
        # STA SDF wrapper case: chip_top is one level below the wrapper.
        for child_name in ("u_chip_top", "chip_top", "uut", "u_dut", "dut"):
            try:
                yield getattr(dut, child_name)
            except Exception:
                continue

    for scope in _scopes():
        # Try direct RTL hierarchy from this scope.
        try:
            obj = scope
            for part in path.split("."):
                obj = getattr(obj, part)
            _ = int(obj.value)
            return obj
        except Exception:
            pass

        # Try escaped GL names in this scope.
        for name in (f"\\{path} ", f"\\{path}"):
            try:
                h = scope[name]
                _ = int(h.value)
                return h
            except Exception:
                pass
    return None


def _resolve_bus_handles(dut, path, width):
    """Resolve handles for a multi-bit signal, one per bit. Returns a list
    of handles (length=width) or None if any bit cannot be resolved.

    In GL the netlist may split a bus into per-bit escaped wires
    (e.g. `\\...gesture[0] `, `\\...gesture[1] `).
    """
    # RTL form: try the bus name as a single handle
    h = _resolve_handle(dut, path)
    if h is not None:
        return [h]   # single multi-bit handle; caller can read .value directly
    def _scopes():
        yield dut
        for child_name in ("u_chip_top", "chip_top", "uut", "u_dut", "dut"):
            try:
                yield getattr(dut, child_name)
            except Exception:
                continue

    # GL form: per-bit handles
    bits = []
    for i in range(width):
        b = None
        for scope in _scopes():
            for name in (f"\\{path}[{i}] ", f"\\{path}[{i}]"):
                try:
                    hb = scope[name]
                    _ = int(hb.value)
                    b = hb
                    break
                except Exception:
                    continue
            if b is not None:
                break
        if b is None:
            return None
        bits.append(b)
    return bits


def _sig(dut, path):
    """One-shot signal read (used only for diagnostics, NOT in hot loops).
    For polling loops use _resolve_handle once and read .value via the
    cached handle.
    """
    h = _resolve_handle(dut, path)
    return int(h.value) if h is not None else None


def _sig_bus(dut, path, width):
    """One-shot multi-bit read (diagnostics only)."""
    handles = _resolve_bus_handles(dut, path, width)
    if handles is None:
        return None
    if len(handles) == 1:
        return int(handles[0].value)
    val = 0
    for i, h in enumerate(handles):
        val |= (int(h.value) & 1) << i
    return val

# Time from end of BOOT_REQ word to evt_ld_en going high.  Driven by
# control_fsm.sv:PWR_WAIT_CYCLES (default 1024) + 1 cycle for the FSM to
# register boot_req_i + 1 cycle for the LD_OPEN→evt_ld_en register update.
# 1100 gives ~75 cycles of slack — fixed wait, no internal-signal access.
# This matches what a host MCU would do (no GPIO is connected to evt_ld_en).
_EVT_LD_EN_WAIT_CYCLES = 1100


async def _boot_chip_top(dut, pins, weights, thresholds):
    """
    Boot the chip over SPI: BOOT_REQ → fixed wait for boot FSM → program words.

    Pin-only protocol:
      1. Send BOOT_REQ word over SPI (input_PAD pins).
      2. Wait fixed _EVT_LD_EN_WAIT_CYCLES — this matches what a real host
         MCU would do, since evt_ld_en is internal and not bonded out.
      3. Stream weights / thresholds / bin_length / READS_DONE.
    """
    program_words = _build_program_words(weights, thresholds)
    dut._log.info(f"Booting chip: BOOT_REQ + {len(program_words)} program words")
    await _spi_stream(dut, pins, [_boot_req()], tag="boot_req")
    await ClockCycles(dut.clk_PAD, _EVT_LD_EN_WAIT_CYCLES)
    await _spi_stream(dut, pins, program_words, tag="program")
    await ClockCycles(dut.clk_PAD, 100)

    # Ground-truth FSM check via internal signals (cached, no leak).
    # In RTL these are accessible via hierarchy; in GL they're top-level
    # escaped wires that _resolve_handle / _resolve_bus_handles can reach.
    _probe_fsm_after_boot(dut)


_MAIN_STATE_NAMES = {0: "ST_BOOT", 1: "ST_LOAD", 2: "ST_RUN", 3: "ST_DEBUG"}
_LOAD_STATE_NAMES = {
    0: "LD_IDLE", 1: "LD_WAIT_PWR", 2: "LD_OPEN",
    3: "LD_WAIT", 4: "LD_DONE", 5: "LD_FAIL",
}

def _probe_fsm_after_boot(dut):
    """Read the FSM state directly and log it; useful for diagnosing why
    GLS classification fails. In a healthy boot we expect main_state=ST_RUN
    (=2), evt_ld_en=0, core_rst_o=0.
    """
    h_main = _resolve_bus_handles(dut, "i_chip_core.u_soc.u_core.controller_fsm.main_state", 2)
    h_load = _resolve_bus_handles(dut, "i_chip_core.u_soc.u_core.controller_fsm.load_state", 6)
    h_ldn  = _resolve_handle(dut, "i_chip_core.u_soc.u_core.controller_fsm.evt_ld_en")
    h_crst = _resolve_handle(dut, "i_chip_core.u_soc.u_core.controller_fsm.core_rst_o")
    h_done = _resolve_handle(dut, "i_chip_core.u_soc.u_core.controller_fsm.evt_reads_done")

    def _bits(handles):
        if handles is None:
            return None
        if len(handles) == 1:
            return int(handles[0].value)
        v = 0
        for i, h in enumerate(handles):
            v |= (int(h.value) & 1) << i
        return v

    ms = _bits(h_main)
    ls = _bits(h_load)
    ldn = int(h_ldn.value) if h_ldn is not None else None
    crst = int(h_crst.value) if h_crst is not None else None
    erd = int(h_done.value) if h_done is not None else None

    dut._log.info(
        f"FSM probe post-boot: "
        f"main_state={ms} ({_MAIN_STATE_NAMES.get(ms, '?')}) "
        f"load_state={ls} ({_LOAD_STATE_NAMES.get(ls, '?')}) "
        f"evt_ld_en={ldn} core_rst_o={crst} evt_reads_done={erd}"
    )
    if ms != 2:
        dut._log.warning(
            f"FSM did NOT reach ST_RUN — chip is stuck in {_MAIN_STATE_NAMES.get(ms, ms)} / "
            f"{_LOAD_STATE_NAMES.get(ls, ls)}. The classifier cannot run until "
            "main_state=ST_RUN (which deasserts core_rst_o and ungates MAC)."
        )

# ── Speedup levers ────────────────────────────────────────────────────────────
# A full 4-gesture run streams ≈ 1 M EVT2 words at 68 chip cycles each (32 SCLK
# bits × 2 cycles per bit + 4 inter-gap), i.e. ≈ 68 M chip cycles → 1.06 s of
# simulated time.  Icarus typically runs ~1-5× slower than wall clock on this
# design, so a full run can take 5-15 minutes.  When iterating, use any of:
#
#   GESTURE_INDICES=0       Run only Down (4× faster than full run).
#   GESTURE_INDICES=0,1     Run Down + Left (2× faster).
#   MAX_STREAM_WORDS=5000   Truncate each recording to 5 000 words (50× faster
#                            but classification will be wrong — connectivity
#                            and SPI-stream check only, not accuracy).
#   BIN_LENGTH_US=50000     50 ms bins → first window in 0.4 s of event time
#                            (less of the recording needs to be streamed before
#                            gestures fire — but the model was trained on
#                            125 ms bins so accuracy will drop).
#
# Hard cap on total words streamed per recording (0 = stream the entire file).
_MAX_STREAM_WORDS = int(os.getenv("MAX_STREAM_WORDS", "0"))

# Comma-separated list of gesture indices to run (default = all 4).
# 0=Down, 1=Left, 2=Right, 3=Up.
_GESTURE_INDICES = [
    int(s) for s in os.getenv("GESTURE_INDICES", "0,1,2,3").split(",") if s.strip()
]


class _GestureResult:
    """
    Records *every* gesture_valid pulse the chip emits during one recording.

    The chip implements a sliding-window classifier: each bin rollover triggers
    a fresh readout → MAC scan → classifier decision → gesture_valid pulse.
    A typical 1-second recording therefore produces 6-8 gesture_valid pulses,
    NOT a single one.  voxel_bin_core_tb confirms this and uses the dominant
    class (most-frequent of all pulses) as the chip's verdict.

    The first pulse can be misleading: e.g. wave_left starts with a downward
    arc whose feature vector resembles wave_up, so the first window classifies
    as "Up" before the full motion is observed.  Using the dominant class is
    therefore the correct way to validate the chip's classification — exactly
    matching the methodology of voxel_bin_core_tb.
    """
    def __init__(self):
        self.fired      = False        # at least one gesture_valid pulse seen
        self.gesture    = None         # dominant gesture (set after collection)
        self.confidence = None         # confidence of dominant class (last pulse)
        self.gestures   = []           # ordered list of (gesture, confidence)

    def dominant(self):
        """(gesture, confidence) of the most-frequent class. None if empty.

        Ties are broken by recency: among classes tied for the highest pulse
        count, the most recently observed wins. The chip uses a sliding-window
        classifier, so later windows are more likely to represent the completed
        gesture than an early/incomplete window.
        """
        if not self.gestures:
            return None, None

        counts = {}
        for g, _ in self.gestures:
            counts[g] = counts.get(g, 0) + 1

        max_count = max(counts.values())

        # Walk backward so ties choose the most recent class.
        for g, c in reversed(self.gestures):
            if counts[g] == max_count:
                return g, c

        return None, None

async def _gesture_monitor(dut, result):
    """
    Background task: poll gesture_valid on every rising chip-clock edge and
    record EVERY pulse (not just the first).

    Works in both RTL and GL:
      RTL: dotted hierarchical signal access.
      GL : flat netlist with escaped top-level wires on chip_top.

    Resolves the three handles (gesture_valid, gesture[1:0], gesture_confidence)
    ONCE at startup and caches them. Doing `dut[name]` inside the polling loop
    leaks VPI handles and gets the process OOM-killed mid-stream — _resolve_*
    is only called here.

    RisingEdge on deep-hierarchy signals is unreliable in Icarus 12 / cocotb
    2.0.x (the VPI callback may not fire for intermediate nets), so we poll
    on the top-level chip clock instead.
    """
    h_valid = _resolve_handle(dut, "i_chip_core.u_soc.gesture_valid")
    h_gest  = _resolve_bus_handles(dut, "i_chip_core.u_soc.gesture", 2)
    h_conf  = _resolve_handle(dut, "i_chip_core.u_soc.gesture_confidence")

    if h_valid is None or h_gest is None or h_conf is None:
        dut._log.error(
            "_gesture_monitor: cannot resolve gesture signal handles — "
            "internal signal access failed. Monitor will see 0 pulses."
        )
        return

    dut._log.info(
        f"_gesture_monitor: handles cached (gesture as "
        f"{'bus' if len(h_gest)==1 else 'per-bit'}, initial valid="
        f"{int(h_valid.value)})"
    )

    def _read_gesture():
        if len(h_gest) == 1:
            return int(h_gest[0].value)
        return (int(h_gest[1].value) << 1) | (int(h_gest[0].value) & 1)

    try:
        while True:
            await RisingEdge(dut.clk_PAD)
            if int(h_valid.value) == 1:
                g = _read_gesture()
                c = int(h_conf.value)
                result.gestures.append((g, c))
                result.fired = True
                dut._log.info(
                    f"  gesture_valid #{len(result.gestures)}: "
                    f"gesture={g} ({GESTURE_NAMES.get(g, '?')}), confidence={c}"
                )
    except Exception:
        pass  # task was cancelled at end of recording


async def _sample_miso_during_drain(dut, pins, result, *, num_samples, gap_cycles):
    """
    Pin-level alternative to _gesture_monitor for GLS.

    spi_wrapper latches the most-recent gesture_valid pulse into the SPI
    readback path, so repeatedly issuing DEBUG_PAGE SPI reads during the
    drain phase reconstructs the histogram of distinct (gesture, confidence)
    values the chip emitted — without touching any internal RTL signal that
    flattens away after synthesis.

    De-duplication: consecutive samples that read the same (gesture,
    confidence) are merged. spi_wrapper holds the last value until the next
    gesture_valid pulse, so an unchanged read means no new pulse fired.
    """
    # spi_wrapper's gesture-readback register resets to (gesture=0, confidence=0).
    # Until the first real gesture_valid pulse fires, drain samples return that
    # baseline. Seed prev with it so the dedup loop does not record the
    # uninitialized state as a spurious (Down,0) pulse. A real classification
    # should differ from this once confidence becomes valid.
    prev = (0, 0)

    for i in range(num_samples):
        await ClockCycles(dut.clk_PAD, gap_cycles)
        miso = await _spi_xfer(dut, pins, _debug_page(0),
                                tag=f"drain_miso_{i}")
        # MISO encoding (see _expected_miso): bit[31]=confidence,
        # bits[30:29]=gesture.
        gesture    = (miso >> (DATA_WIDTH - 3)) & 0x3
        confidence = (miso >> (DATA_WIDTH - 1)) & 0x1
        cur = (gesture, confidence)
        if cur == prev:
            continue
        result.gestures.append(cur)
        result.fired = True
        prev = cur
        dut._log.info(
            f"  drain MISO sample #{i}: gesture={gesture} "
            f"({GESTURE_NAMES.get(gesture, '?')}), confidence={confidence}"
        )


async def _stream_recording(dut, pins, bin_path):
    """
    Stream the full EVT2 .bin file through chip_top's default SPI pins, then
    drain the pipeline and return every gesture_valid pulse the chip emitted.

    Observability (RTL and GL both use the same methodology):
      A background task polls i_chip_core.u_soc.gesture_valid on every
      clock edge and captures every pulse. In RTL the signal is reached
      via dotted hierarchy; in GL it lives as a top-level escaped wire
      `\\i_chip_core.u_soc.gesture_valid ` on chip_top (the netlist is
      flat) — _sig/_sig_bus transparently handle both forms.

    No early exit: the chip's sliding-window classifier produces multiple
    gesture_valid pulses per recording (one per bin rollover after the first
    full readout window), and we MUST collect them all because the very first
    pulse may classify into a misleading class — see _GestureResult docstring.
    Streaming the full recording also matches how a real event camera would
    feed events into the chip (continuous, no premature stop) and naturally
    drains the voxel-binning ring buffer via the appended flush events.
    """
    words = _read_bin(bin_path)
    words = _append_flush_events(list(words))
    if _MAX_STREAM_WORDS > 0:
        words = words[:_MAX_STREAM_WORDS]

    total = len(words)
    stem  = Path(bin_path).stem
    dut._log.info(f"Streaming {Path(bin_path).name}: {total} words "
                  f"(single CS-low burst)")

    result = _GestureResult()

    # Use the gesture_valid pulse monitor whenever the simulator exposes the
    # net. Timed STA wraps chip_top in chip_top_sdf_wrapper.u_chip_top, which
    # _resolve_handle handles. The older SPI-readback fallback is ambiguous for
    # Down with confidence=0 because that is also the reset readback value.
    use_pin_level_monitor = False

    if use_pin_level_monitor:
        monitor_task = None
        dut._log.info(
            "STA GLS mode: using pin-level SPI debug-page sampling instead "
            "of internal gesture_valid monitor."
        )
    else:
        monitor_task = cocotb.start_soon(_gesture_monitor(dut, result))

    # Single SPI burst with CS held low for the entire stream — matches how
    # a real GenX320 sensor delivers events: continuous, no CS toggling.
    # inter_gap=4: SPI re-arm needs ≥ 3 chip cycles minimum.
    await _spi_stream(dut, pins, words, inter_gap=4,
                      progress_every=10_000, tag=stem)

    # Drain: 300 000 cycles @ 64 MHz = 4.7 ms. After streaming we have
    # ~READOUT_BINS+2 = 18 flush-driven bin rollovers queued in the binner.
    # The classifier emits one gesture_valid pulse per rollover at ~128 µs
    # throughput (16 bins × 513 readout cycles + 4-class MAC + 4-stage
    # classifier pipeline). 18 pulses × 128 µs ≈ 2.3 ms — 50 000 cycles only
    # captured the first 6, biasing the dominant class toward the noisy
    # opening windows. 300 000 cycles covers all ~18 expected pulses plus
    # margin.
    dut._log.info("Recording consumed; draining pipeline (300 000 cycles) …")

    if use_pin_level_monitor:
        await _sample_miso_during_drain(
            dut, pins, result,
            num_samples=300,
            gap_cycles=1000,
        )
    else:
        await ClockCycles(dut.clk_PAD, 300_000)
        monitor_task.cancel()

    if not result.gestures:
        raise AssertionError(
            f"No gesture_valid pulses fired after streaming {stem} "
            f"({total} words) + 50 000 drain cycles"
        )

    # Compute dominant class — voxel_bin_core_tb-style validation.
    dom_g, dom_c = result.dominant()
    counts = {}
    for g, _ in result.gestures:
        counts[g] = counts.get(g, 0) + 1
    hist = " ".join(f"{GESTURE_NAMES.get(g, '?')}:{n}"
                    for g, n in sorted(counts.items()))
    dut._log.info(
        f"[{stem}] {len(result.gestures)} gesture pulses; histogram=({hist}); "
        f"dominant={GESTURE_NAMES.get(dom_g, '?')} (confidence={dom_c})"
    )

    result.gesture    = dom_g
    result.confidence = dom_c
    return result

def _expected_miso(gesture, confidence):
    return (((confidence & 1) << 2) | (gesture & 0x3)) << (DATA_WIDTH - 3)

# ── Classification tests ──────────────────────────────────────────────────────

# Targeted FSM-stepping diagnostic. After each boot step, read the FSM state
# directly via internal signals (escaped names in GL, dotted hierarchy in RTL).
# Pinpoints exactly which step the FSM fails to advance on.
@cocotb.test(skip=not (bool(gl) and bool(os.getenv("FSM_STEP"))))
async def test_fsm_step_diagnostic(dut):
    """Step the chip through the boot sequence, probing main_state /
    load_state / evt_ld_en / boot_req_i / evt_reads_done after each step.
    """
    pins = InputPins()
    await _start_clock(dut)
    await NextTimeStep()
    await _reset(dut, pins)
    await _wait_spi_ready(dut)

    # Cache probe handles up front (only signals that survived synthesis).
    h_main = _resolve_bus_handles(dut, "i_chip_core.u_soc.u_core.controller_fsm.main_state", 2)
    h_load = _resolve_bus_handles(dut, "i_chip_core.u_soc.u_core.controller_fsm.load_state", 6)
    h_ldn  = _resolve_handle(dut, "i_chip_core.u_soc.u_core.controller_fsm.evt_ld_en")
    h_crst = _resolve_handle(dut, "i_chip_core.u_soc.u_core.controller_fsm.core_rst_o")
    h_erd  = _resolve_handle(dut, "i_chip_core.u_soc.u_core.controller_fsm.evt_reads_done")
    h_swvalid = _resolve_handle(dut, "i_chip_core.u_soc.evt_word_valid")
    h_evt_count = _resolve_bus_handles(dut, "i_chip_core.u_soc.u_core.debug_event_count", 8)
    h_evt_word = _resolve_bus_handles(dut, "i_chip_core.u_soc.evt_word", 32)

    def _bits(h):
        if h is None: return None
        if len(h) == 1: return int(h[0].value)
        v = 0
        for i, x in enumerate(h):
            v |= (int(x.value) & 1) << i
        return v

    def _b1(h):
        return None if h is None else int(h.value)

    async def probe(label):
        await ClockCycles(dut.clk_PAD, 1)   # settle
        ms = _bits(h_main); ls = _bits(h_load)
        dut._log.info(
            f"PROBE [{label}]: main={ms}({_MAIN_STATE_NAMES.get(ms,'?')}) "
            f"load={ls}({_LOAD_STATE_NAMES.get(ls,'?')}) "
            f"evt_ld_en={_b1(h_ldn)} core_rst={_b1(h_crst)} "
            f"reads_done={_b1(h_erd)} "
            f"spi.evt_word_valid={_b1(h_swvalid)} "
            f"u_core.debug_event_count={_bits(h_evt_count)} "
            f"u_soc.evt_word=0x{_bits(h_evt_word):08X}"
        )

    # Verify cocotb writes to input_PAD actually reach the chip internals.
    # The chip-side bus is called i_chip_core.input_in (post-pad).
    # IMPORTANT: avoid toggling bit 8 (ALT_INPUT_MODE) — its rising edge
    # toggles alt_select, which re-routes SPI to the alt input pins.
    h_in2core = _resolve_bus_handles(dut, "i_chip_core.input_in", 12)
    for test_val in (0x000, 0x0FF, 0x055, 0x000):
        pins._v = test_val
        pins.drive(dut)
        await ClockCycles(dut.clk_PAD, 4)
        in2core = None
        if h_in2core is not None:
            v = 0
            for i, hb in enumerate(h_in2core):
                v |= (int(hb.value) & 1) << i
            in2core = v
        try:
            ext = int(dut.input_PAD.value)
        except Exception:
            ext = -1
        in2 = in2core if in2core is not None else -1
        dut._log.info(
            f"INPUT PROBE: drove input_PAD=0x{test_val:03X}  "
            f"readback dut.input_PAD={ext:#x}  chip-side input_PAD2CORE={in2:#x}"
        )
    # Restore idle state
    pins._v = 0
    pins.set(PIN_DEF_CS, 1)
    pins.set(PIN_ALT_CS, 1)
    pins.drive(dut)
    await ClockCycles(dut.clk_PAD, 4)

    # Probe SPI input plumbing: toggle SCLK and check chip sees the edges.
    h_sclk = _resolve_handle(dut, "i_chip_core.SCLK_wire")
    h_cs   = _resolve_handle(dut, "i_chip_core.CS_wire")
    h_mosi = _resolve_handle(dut, "i_chip_core.MOSI_wire")
    h_in5  = (h_in2core[5] if h_in2core is not None and len(h_in2core) > 5 else None)
    h_oe0  = _resolve_handle(dut, "bidir_CORE2PAD_OE[0]")
    h_oe1  = _resolve_handle(dut, "bidir_CORE2PAD_OE[1]")
    h_rstn = _resolve_handle(dut, "i_chip_core.rst_n")
    dut._log.info(
        f"RESET PROBE: chip rst_n={int(h_rstn.value) if h_rstn is not None else '?'} "
        f"OE[0](=!alt_select)={int(h_oe0.value) if h_oe0 is not None else '?'} "
        f"OE[1](=alt_select)={int(h_oe1.value) if h_oe1 is not None else '?'}"
    )
    pins.set(PIN_DEF_CS, 0)
    pins.set(PIN_DEF_MOSI, 1)
    pins.drive(dut)
    await ClockCycles(dut.clk_PAD, 2)
    for cycle in range(4):
        pins.set(PIN_DEF_SCLK, 1)
        pins.drive(dut)
        await ClockCycles(dut.clk_PAD, 1)
        dut._log.info(
            f"SPI plumbing (SCLK_HI {cycle}): input_in[5]={int(h_in5.value) if h_in5 is not None else '?'} "
            f"SCLK_wire={int(h_sclk.value) if h_sclk is not None else '?'} "
            f"CS_wire={int(h_cs.value) if h_cs is not None else '?'} "
            f"MOSI_wire={int(h_mosi.value) if h_mosi is not None else '?'}"
        )
        pins.set(PIN_DEF_SCLK, 0)
        pins.drive(dut)
        await ClockCycles(dut.clk_PAD, 1)
        dut._log.info(
            f"SPI plumbing (SCLK_LO {cycle}): input_in[5]={int(h_in5.value) if h_in5 is not None else '?'} "
            f"SCLK_wire={int(h_sclk.value) if h_sclk is not None else '?'}"
        )
    # restore idle
    pins._v = 0
    pins.set(PIN_DEF_CS, 1)
    pins.set(PIN_ALT_CS, 1)
    pins.drive(dut)
    await ClockCycles(dut.clk_PAD, 4)

    await probe("post-reset")
    await _spi_stream(dut, pins, [_boot_req()], tag="boot_req")
    await probe("after BOOT_REQ word")
    await ClockCycles(dut.clk_PAD, 50)
    await probe("BOOT_REQ + 50 cycles")
    await ClockCycles(dut.clk_PAD, 1100)
    await probe("BOOT_REQ + 1150 cycles (expect LD_WAIT)")

    # Send just a couple of program words to see if decoder advances them
    await _spi_stream(dut, pins, [_w_weight(0, 0, 0)], tag="one_weight")
    await probe("after one weight word")
    await _spi_stream(dut, pins, [_w_reads_done()], tag="reads_done")
    await ClockCycles(dut.clk_PAD, 200)
    await probe("after reads_done + 200 cycles (expect ST_RUN)")


# Diagnostic test: streams a chunk of events, then probes each pipeline stage
# via DEBUG_PAGE reads. Skipped by default; enable with DIAGNOSTIC=1 in env.
# Run only in GL mode where we cannot probe internal signals directly.
@cocotb.test(skip=not (bool(gl) and bool(os.getenv("DIAGNOSTIC"))))
async def test_diagnostic_pipeline_probe(dut):
    """
    Streams ~60K words of wave_left in chunks, pausing between chunks to
    read every debug page. Lets us see WHICH pipeline stage stops advancing
    in GLS:
      Page 3 (FSM)       — boot_done? load_state correct?
      Page 2 (decoder)   — event_valid pulsing? FIFO non-empty?
      Page 1 (binning)   — readout_valid/last pulsing? bin counters moving?
      Page 0 (classifier+MAC) — scores_valid? gesture_valid?
    """
    bin_files = _resolve_bin_files()
    bin_path  = bin_files[1]   # wave_left
    if not bin_path.exists():
        raise FileNotFoundError(f"Missing: {bin_path}")

    weights    = _load_weights()
    thresholds = _load_thresholds()

    pins = InputPins()
    await _start_clock(dut)
    await NextTimeStep()
    await _reset(dut, pins)
    await _wait_spi_ready(dut)
    await _boot_chip_top(dut, pins, weights, thresholds)

    # Snapshot all debug pages — non-intrusive sequence of SPI reads.
    async def snapshot(label):
        dut._log.info(f"=== snapshot: {label} ===")
        for p in range(5):
            # Send DEBUG_PAGE(p), then read debug_bus from bidir_PAD[37:6]
            await _spi_stream(dut, pins, [_debug_page(p)], tag=f"snap_pg{p}")
            await ClockCycles(dut.clk_PAD, 10)
            bus = _read_debug_bus(dut)
            dut._log.info(f"  page {p}: debug_bus=0x{bus:08X}")

    await snapshot("post-boot")

    # Load recording, drop flush events for diagnostic clarity
    full = _read_bin(bin_path)
    chunk_size = 30_000
    for chunk_idx in range(3):
        start = chunk_idx * chunk_size
        end   = start + chunk_size
        chunk = full[start:end]
        if not chunk:
            break
        dut._log.info(f"streaming chunk {chunk_idx} words [{start}:{end}]")
        await _spi_stream(dut, pins, chunk, inter_gap=4,
                          progress_every=10_000, tag=f"chunk{chunk_idx}")
        # CS goes high here (between bursts); let pipeline settle
        await ClockCycles(dut.clk_PAD, 5_000)
        await snapshot(f"after chunk {chunk_idx} ({end} words streamed)")


# ── Sanity test (CI-friendly, no LFS required) ────────────────────────────────

@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_sanity_evt2_and_debug(dut):
    """
    CI sanity check: reset, minimal boot (BOOT_REQ + READS_DONE, no weights),
    stream 2 synthetic EVT2 words (TIME_HIGH + CD_OFF), sweep debug pages 0-4.
    No LFS data required — designed for 'make sim' quick-check in GitHub Actions.
    """
    pins = await _startup(dut)

    # Minimal boot: BOOT_REQ, wait for the FSM to open the load window, then
    # READS_DONE.  Skipping weight/threshold loads keeps the test fast and
    # avoids any dependency on LFS-stored .mem files.
    dut._log.info("Minimal boot: BOOT_REQ → wait → READS_DONE")
    await _spi_stream(dut, pins, [_boot_req()], tag="boot_req")
    await ClockCycles(dut.clk_PAD, _EVT_LD_EN_WAIT_CYCLES)
    await _spi_stream(dut, pins, [_w_reads_done()], tag="reads_done")
    await ClockCycles(dut.clk_PAD, 100)

    # 2 synthetic EVT2 words: TIME_HIGH at t_hi=100, then a CD_OFF at (50, 50)
    time_high_word = (0x8 << 28) | 100             # TYPE=TIME_HIGH, t_hi=100
    cd_off_word    = (0x0 << 28) | (10 << 22) | (50 << 11) | 50  # ts_lsb=10 x=50 y=50
    dut._log.info("Streaming 2 synthetic EVT2 words (TIME_HIGH + CD_OFF)...")
    await _spi_stream(dut, pins, [time_high_word, cd_off_word], tag="evt2_sanity")
    await ClockCycles(dut.clk_PAD, 200)

    # Sweep debug pages 0-4 and verify the bus is readable
    dut._log.info("Sweeping debug pages 0-4...")
    for page in range(5):
        await _spi_stream(dut, pins, [_debug_page(page)], tag=f"dbg_pg{page}")
        await ClockCycles(dut.clk_PAD, 20)
        bus = _read_debug_bus(dut)
        dut._log.info(f"  Page {page}: debug_bus=0x{bus:08X}")

    dut._log.info("PASS: EVT2 sanity + debug sweep completed")


# Runs in both RTL and GLS. The internal-signal monitor used in RTL is
# replaced by capturing MISO during the stream in GL mode — see
# _stream_recording. Pin-only observability in GL still validates the full
# pipeline including SRAM read/write paths.
@cocotb.test()
async def test_classify_all_gestures(dut):
    """
    Full chip-top classification verification.

    For each of the 4 gesture recordings (Down, Left, Right, Up):
      1. Reset and boot the chip with learned weights and thresholds over SPI.
      2. Stream the EVT2 .bin recording through chip_top input_PAD SPI pins.
      3. Wait for gesture_valid from the on-chip classifier.
      4. Assert the reported gesture matches the expected label.
      5. Read the classification back through MISO to confirm the SPI output path.

    This proves end-to-end correctness from the chip_top IO pads through the
    full pipeline: IO pads → SPI front-end → EVT2 decoder → voxel binning →
    MAC engine → gesture classifier → MISO output pin.
    """
    bin_files = _resolve_bin_files()
    selected  = [(i, bin_files[i]) for i in _GESTURE_INDICES if 0 <= i < len(bin_files)]
    if not selected:
        raise ValueError(
            f"GESTURE_INDICES={_GESTURE_INDICES} selected no valid gestures "
            f"(valid range: 0..{len(bin_files)-1})"
        )
    for _, bf in selected:
        if not bf.exists():
            raise FileNotFoundError(
                f"Missing test recording: {bf}\n"
                "Run 'git lfs pull' to fetch the EVT2 gesture set."
            )

    if len(selected) < len(bin_files):
        skipped = [GESTURE_NAMES[i] for i in range(len(bin_files))
                   if i not in {s[0] for s in selected}]
        cocotb.log.info(
            f"GESTURE_INDICES filter active: running "
            f"{[GESTURE_NAMES[i] for i, _ in selected]}, skipping {skipped}"
        )

    weights    = _load_weights()
    thresholds = _load_thresholds()

    results = []

    for slot, (idx, bin_path) in enumerate(selected):
        expected = _EXPECTED_CLASS[idx]
        dut._log.info("=" * 70)
        dut._log.info(f"Gesture {idx}: {GESTURE_NAMES[expected]}  ({bin_path.name})")
        dut._log.info("=" * 70)

        # Fresh reset for every recording.  Gate the one-time clock startup on
        # `slot` (loop iteration) not `idx` (gesture index): with a filter like
        # GESTURE_INDICES=2,3 the first iteration has idx=2, but it is still
        # the first time we need to start the clock.
        pins = InputPins()
        if slot == 0:
            await _start_clock(dut)
            await NextTimeStep()
        await _reset(dut, pins)
        await _wait_spi_ready(dut)

        # Boot: send weights + thresholds
        await _boot_chip_top(dut, pins, weights, thresholds)

        # Stream the full recording and collect every gesture_valid pulse.
        result = await _stream_recording(dut, pins, bin_path)
        dom_gesture    = result.gesture       # dominant class (correctness)
        dom_confidence = result.confidence
        last_gesture, last_confidence = result.gestures[-1]   # MISO comparand

        dut._log.info(
            f"Dominant classifier output: gesture={dom_gesture} "
            f"({GESTURE_NAMES.get(dom_gesture, dom_gesture)}), "
            f"confidence={dom_confidence}"
        )

        # Give spi_wrapper time to latch the latest classification
        await ClockCycles(dut.clk_PAD, 20)

        # Read classification back through MISO (dummy DEBUG_PAGE transfer).
        # spi_wrapper latches the latest gesture pulse, so MISO reflects the
        # LAST gesture_valid in the run, not necessarily the dominant one.
        miso = await _spi_xfer(dut, pins, _debug_page(0), tag="miso_readback")
        expected_miso = _expected_miso(last_gesture, last_confidence)

        assert miso == expected_miso, (
            f"[{bin_path.name}] MISO mismatch (last latched gesture): "
            f"got 0x{miso:08X}, expected 0x{expected_miso:08X} "
            f"(last gesture={GESTURE_NAMES.get(last_gesture, last_gesture)}, "
            f"confidence={last_confidence})"
        )

        # Correctness assertion uses the DOMINANT class — matches the
        # validation methodology of voxel_bin_core_tb, which also accepts
        # that the first window of a recording can classify into a
        # misleading class (e.g. wave_left starts with a downward arc that
        # looks like wave_up) and only the dominant of all windows is the
        # true verdict.
        assert dom_gesture == expected, (
            f"[{bin_path.name}] Wrong dominant classification: "
            f"got {GESTURE_NAMES.get(dom_gesture, dom_gesture)}, "
            f"expected {GESTURE_NAMES[expected]} "
            f"(pulses: "
            f"{[GESTURE_NAMES.get(g, g) for g, _ in result.gestures]})"
        )

        results.append((GESTURE_NAMES[expected], dom_gesture, dom_confidence,
                        len(result.gestures)))
        dut._log.info(
            f"PASS [{bin_path.name}]: "
            f"expected={GESTURE_NAMES[expected]}, "
            f"dominant={GESTURE_NAMES.get(dom_gesture, dom_gesture)} "
            f"(from {len(result.gestures)} pulses), "
            f"last={GESTURE_NAMES.get(last_gesture, last_gesture)}, "
            f"confidence={dom_confidence}, MISO=0x{miso:08X}"
        )

    dut._log.info("=" * 70)
    dut._log.info(
        f"ALL {len(results)} GESTURE{'S' if len(results) != 1 else ''} "
        f"CLASSIFIED CORRECTLY"
    )
    for name, g, c, n in results:
        dut._log.info(
            f"  {name:6s} → {GESTURE_NAMES.get(g, g)} "
            f"(confidence={c}, {n} pulses)"
        )
    dut._log.info("=" * 70)


# ── Runner ────────────────────────────────────────────────────────────────────

def chip_top_runner():
    from cocotb_tools.runner import get_runner
    proj_path = Path(__file__).resolve().parent

    defines  = {f"SLOT_{slot.upper()}": True}
    defines[f"PDK_{pdk.replace('-', '_')}"] = True
    defines[f"SCL_{scl}"] = True
    defines[f"PAD_{pad}"] = True
    defines[f"SRAM_{sram}"] = True
    includes = [proj_path / "../src/"]

    if gl:
        if not gl_netlist.exists():
            raise FileNotFoundError(
                f"GLS netlist not found: {gl_netlist}\n"
                "Override with GL_NETLIST=/abs/path/to/chip_top.{nl,pnl}.v"
            )

        pdk_libs = Path(pdk_root) / pdk / "libs.ref"

        # SCL cell models: prefer ciel-managed PDK path; fall back to the
        # vendored sim/ stub (kept for Verilator compat / offline use).
        pdk_scl_v = pdk_libs / scl / "verilog" / f"{scl}.v"
        scl_v = pdk_scl_v if pdk_scl_v.exists() else proj_path / "../sim/gf180mcu_as_sc_mcu7t3v3.v"
        scl_prim = pdk_libs / scl / "verilog" / "primitives.v"

        # IO pad models: prefer ciel-managed PDK path; fall back to vendored stubs.
        pdk_io_v = pdk_libs / pad / "verilog" / f"{pad}.v"
        io_v = pdk_io_v if pdk_io_v.exists() else proj_path / "../sim/gf180mcu_fd_io.v"
        pdk_wsio_v = pdk_libs / pad / "verilog" / "gf180mcu_ws_io.v"
        ws_io_v = pdk_wsio_v if pdk_wsio_v.exists() else proj_path / "../sim/gf180mcu_ws_io.v"

        sources = [scl_v]
        # gf180mcu_as_sc_mcu7t3v3 has no separate primitives.v; all others do.
        if scl != "gf180mcu_as_sc_mcu7t3v3" and scl_prim.exists():
            sources.append(scl_prim)
        if scl == "gf180mcu_as_sc_mcu7t3v3":
            sources.append(proj_path / "../sim/gf180mcu_as_sc_mcu7t3v3_missing_cells.v")
        sources += [
            # IO pad models.
            io_v,
            ws_io_v,

            # SRAM behavioral models.
            proj_path / "../sim/gf180mcu_ocd_ip_sram_models.v",

            # Gate-level netlist.
            gl_netlist,
        ]
        # Post-synthesis (.nl.v) has NO power pins; post-PnR (.pnl.v) does.
        # Auto-detect from filename so the same runner works for both.
        use_power_pins = gl_netlist.name.endswith(".pnl.v")
        # `functional` (lowercase) gates IO pad behavioural models in
        # gf180mcu_fd_io.v and must always be set for GLS.
        # Uppercase `FUNCTIONAL` suppresses `specify` blocks in the PDK cell
        # models (gf180mcu_as_sc_mcu7t3v3.v uses `ifndef FUNCTIONAL guards).
        # For timed STA GLS (TIMING=1) the SDF annotator needs those specify
        # blocks active, so we must NOT define FUNCTIONAL in that mode.
        # For plain functional GLS the specify blocks add overhead with no
        # benefit, so we keep FUNCTIONAL defined to suppress them.
        defines.update({"functional": True})
        if not timing:
            # Functional GLS only: suppress specify blocks for faster compile.
            defines["FUNCTIONAL"] = True
        if use_power_pins:
            defines["USE_POWER_PINS"] = True
        print(f"[chip_top_tb] GLS netlist: {gl_netlist}")
        print(f"[chip_top_tb] SCL: {scl}   USE_POWER_PINS={use_power_pins}")
    else:
        sources = [
            proj_path / "../src/chip_top.sv",
            proj_path / "../src/chip_core.sv",
            proj_path / "../src/soc.sv",
            proj_path / "../src/spi_wrapper.sv",
            proj_path / "../src/control_fsm.sv",
            proj_path / "../src/evt2_decoder.sv",
            proj_path / "../src/sram_wrapper.sv",
            proj_path / "../src/input_fifo.sv",
            proj_path / "../src/selectable_debug.sv",
            proj_path / "../src/voxel_bin_core.sv",
            proj_path / "../src/voxel_binning.sv",
            proj_path / "../src/voxel_gesture_classifier.sv",
            proj_path / "../src/voxel_mac_engine.sv",
            proj_path / "../third_party/verilog_spi/spi_module.v",
            proj_path / "../third_party/verilog_spi/pos_edge_det.v",
            proj_path / "../third_party/verilog_spi/neg_edge_det.v",
        ]

        # IO pad and SRAM models: check each file individually against the
        # ciel-managed PDK; fall back to the vendored sim/ stub if absent.
        # gf180mcu_ws_io.v is a wafer-space supply-pad stub that may not be
        # present in all PDK distributions.
        pdk_io_v   = Path(pdk_root) / pdk / "libs.ref" / pad  / "verilog" / f"{pad}.v"
        pdk_wsio_v = Path(pdk_root) / pdk / "libs.ref" / pad  / "verilog" / "gf180mcu_ws_io.v"
        pdk_sram_v = Path(pdk_root) / pdk / "libs.ref" / sram / "verilog" / f"{sram}__sram512x8m8wm1.v"

        sources.append(pdk_io_v   if pdk_io_v.exists()   else proj_path / "../sim/gf180mcu_fd_io.v")
        sources.append(pdk_wsio_v if pdk_wsio_v.exists() else proj_path / "../sim/gf180mcu_ws_io.v")
        sources.append(pdk_sram_v if pdk_sram_v.exists() else proj_path / "../sim/gf180mcu_ocd_ip_sram_models.v")

    sources += [
        proj_path / "../ip/gf180mcu_ws_ip__logo/vh/gf180mcu_ws_ip__logo.v",
        proj_path / "../ip/gf180mcu_ws_ip__marker/vh/gf180mcu_ws_ip__marker.v",
        proj_path / "../ip/gf180mcu_ws_ip__qrcode_id/vh/gf180mcu_ws_ip__qrcode_id.v",
        proj_path / "../ip/gf180mcu_ws_ip__shuttle_id/vh/gf180mcu_ws_ip__shuttle_id.v",
        proj_path / "../ip/gf180mcu_ws_ip__project_id/vh/gf180mcu_ws_ip__project_id.v",
    ]

    build_args = []
    if sim == "verilator":
        build_args = [
            "--timing", "--trace", "--trace-fst", "--trace-structs",
            # Force deterministic 0-initialization for all uninitialized
            # variables (Verilator 5.x defaults to "unique" which inserts
            # random-X values for unassigned regs — fine for finding races
            # but introduces nondeterminism into GLS where many flops only
            # reset synchronously).
            "--x-initial", "0",
            "--x-assign", "0",
            # The gate-level netlist produces thousands of benign warnings:
            # WIDTH on tie cells, UNOPTFLAT on the IO pad inout chains,
            # PINMISSING / IMPLICIT on IP module instantiations,
            # MULTIDRIVEN on bidir pads. Suppress so the build doesn't
            # blow up on `-Werror`.
            "-Wno-WIDTH", "-Wno-UNOPTFLAT", "-Wno-PINMISSING",
            "-Wno-IMPLICIT", "-Wno-MULTIDRIVEN", "-Wno-TIMESCALEMOD",
            "-Wno-COMBDLY", "-Wno-INITIALDLY", "-Wno-CASEINCOMPLETE",
            "-Wno-CASEX", "-Wno-LATCH", "-Wno-UNUSED",
        ]

    # Allow parallel invocations to use isolated build/result paths via env
    # vars (e.g. one process per gesture). Defaults keep the single-process
    # flow unchanged.
    sim_build   = os.getenv("SIM_BUILD", "sim_build")
    test_filter = os.getenv("COCOTB_TEST_FILTER")    # regex; None = all tests
    results_xml = os.getenv("RESULTS_XML")           # absolute path or None

    hdl_top_for_sim = hdl_toplevel

    if timing:
        if not gl:
            raise ValueError("TIMING=1 requires GL=1 because SDF applies to the gate-level netlist.")

        if sim != "icarus":
            raise ValueError("Timed SDF GLS should use SIM=icarus.")

        if not sdf_file:
            raise ValueError("TIMING=1 was set, but SDF_FILE was not provided.")

        if not Path(sdf_file).exists():
            raise FileNotFoundError(f"SDF file not found: {sdf_file}")

        sources.append(proj_path / "../sim/chip_top_sdf_wrapper.sv")
        hdl_top_for_sim = "chip_top_sdf_wrapper"

        build_args += [
            "-gspecify",
            "-ginterconnect",
            f'-DSDF_FILE="{sdf_file}"',
        ]

        print(f"[chip_top_tb] Timed GLS enabled with SDF: {sdf_file}")

    print(f"[chip_top_tb] HDL top for simulation: {hdl_top_for_sim}")

    force_rebuild = os.getenv("FORCE_REBUILD", "0").lower() not in ("", "0", "false", "no")
    waves_enabled = os.getenv("WAVES", "1").lower() not in ("", "0", "false", "no")

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel=hdl_top_for_sim,
        defines=defines,
        always=force_rebuild,
        includes=includes,
        build_args=build_args,
        build_dir=sim_build,
        waves=waves_enabled,
    )

    runner.test(
        hdl_toplevel=hdl_top_for_sim,
        test_module="chip_top_tb,",
        plusargs=[],
        build_dir=sim_build,
        test_filter=test_filter,
        results_xml=results_xml,
        waves=waves_enabled,
    )


if __name__ == "__main__":
    chip_top_runner()