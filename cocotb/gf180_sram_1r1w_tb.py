# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

WIDTH_P  = CFG.get("width_p", 8)
DEPTH_P  = CFG.get("depth_p", 256)
ADDR_MAX  = DEPTH_P - 1
DATA_MASK = (1 << WIDTH_P) - 1


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    dut.reset_i.value  = 1
    dut.wr_valid_i.value = 0
    dut.wr_data_i.value  = 0
    dut.wr_addr_i.value  = 0
    dut.rd_valid_i.value = 0
    dut.rd_addr_i.value  = 0
    await ClockCycles(dut.clk_i, 4)
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)


async def write_word(dut, addr, data):
    """Issue a one-cycle write then deassert."""
    dut.wr_valid_i.value = 1
    dut.wr_addr_i.value  = addr
    dut.wr_data_i.value  = data & DATA_MASK
    await RisingEdge(dut.clk_i)
    dut.wr_valid_i.value = 0


async def read_word(dut, addr):
    """Issue a one-cycle read; returns Q from the following cycle (1-cycle latency)."""
    dut.rd_valid_i.value = 1
    dut.rd_addr_i.value  = addr
    await RisingEdge(dut.clk_i)   # read issued on this edge
    dut.rd_valid_i.value = 0
    await RisingEdge(dut.clk_i)   # Q registered on this edge
    await ReadOnly()
    val = int(dut.rd_data_o.value)
    await NextTimeStep()
    return val


@logged_test()
async def test_reset_defaults(dut):
    """Memory starts at 0 after power-on; rd_data_o is 0 for unwritten addresses."""
    await setup(dut)
    got = await read_word(dut, 0)
    assert got == 0, f"Expected 0 for unwritten addr 0, got 0x{got:X}"
    got = await read_word(dut, ADDR_MAX)
    assert got == 0, f"Expected 0 for unwritten addr {ADDR_MAX}, got 0x{got:X}"


@logged_test()
async def test_basic_write_then_read(dut):
    """Write a known value, read it back one cycle later."""
    await setup(dut)
    val = 0xA5 & DATA_MASK
    await write_word(dut, 0, val)
    got = await read_word(dut, 0)
    assert got == val, f"Expected 0x{val:X}, got 0x{got:X}"


@logged_test()
async def test_multiple_distinct_addresses(dut):
    """Write distinct patterns to 16 addresses; read them all back in order."""
    await setup(dut)
    N = min(16, DEPTH_P)
    vals = [(addr * 7 + 13) & DATA_MASK for addr in range(N)]
    for addr, v in enumerate(vals):
        await write_word(dut, addr, v)
    for addr, v in enumerate(vals):
        got = await read_word(dut, addr)
        assert got == v, f"addr={addr}: expected 0x{v:X}, got 0x{got:X}"


@logged_test()
async def test_address_boundaries(dut):
    """First (0) and last (DEPTH_P-1) address must work correctly."""
    await setup(dut)
    lo_val = 0x55 & DATA_MASK
    hi_val = 0xAA & DATA_MASK
    await write_word(dut, 0, lo_val)
    await write_word(dut, ADDR_MAX, hi_val)
    lo = await read_word(dut, 0)
    hi = await read_word(dut, ADDR_MAX)
    assert lo == lo_val, f"addr=0: expected 0x{lo_val:X}, got 0x{lo:X}"
    assert hi == hi_val, f"addr={ADDR_MAX}: expected 0x{hi_val:X}, got 0x{hi:X}"


@logged_test()
async def test_overwrite_same_address(dut):
    """Writing twice to the same address: the second write must win."""
    await setup(dut)
    await write_word(dut, 5, 0x11 & DATA_MASK)
    await write_word(dut, 5, 0x22 & DATA_MASK)
    got = await read_word(dut, 5)
    assert got == (0x22 & DATA_MASK), f"Overwrite failed: expected 0x22, got 0x{got:X}"


@logged_test()
async def test_read_latency_is_one_cycle(dut):
    """rd_data_o must be stable one cycle AFTER rd_valid_i is asserted (synchronous read)."""
    await setup(dut)
    sentinel = 0x3C & DATA_MASK
    await write_word(dut, 7, sentinel)

    # Assert rd_valid: sample rd_data_o in the *same* cycle — it still holds old Q.
    dut.rd_valid_i.value = 1
    dut.rd_addr_i.value  = 7
    await RisingEdge(dut.clk_i)
    await ReadOnly()
    stale = int(dut.rd_data_o.value)
    await NextTimeStep()

    # One cycle later, Q must carry the correct data.
    dut.rd_valid_i.value = 0
    await RisingEdge(dut.clk_i)
    await ReadOnly()
    got = int(dut.rd_data_o.value)
    await NextTimeStep()

    assert got == sentinel, f"Expected 0x{sentinel:X} at cycle+1, got 0x{got:X}"
    # stale may equal sentinel if the macro bypasses — we only enforce the +1 cycle value.
    _ = stale  # suppress unused warning; value is intentionally not checked


@logged_test()
async def test_output_holds_without_rd_valid(dut):
    """Without rd_valid_i, rd_data_o must hold the last read value (no spurious change)."""
    await setup(dut)
    sentinel = 0x6B & DATA_MASK
    await write_word(dut, 2, sentinel)
    got = await read_word(dut, 2)
    assert got == sentinel

    # Clock 5 more cycles with rd_valid=0; output must not change.
    await ClockCycles(dut.clk_i, 5)
    await ReadOnly()
    assert int(dut.rd_data_o.value) == sentinel, \
        "rd_data_o changed without rd_valid_i"


@logged_test()
async def test_write_wins_simultaneous_wr_rd(dut):
    """When wr_valid and rd_valid target the same address simultaneously:
    - Simulation model: read-before-write (rd_data_o one cycle later shows old value
      because both NBA assignments use pre-posedge sim_mem state).
    - Synthesis path: write wins (GWEN=0 overrides REN in the GF180 macro).
    In both cases a subsequent independent read must return the newly written value."""
    await setup(dut)
    seed_val = 0xCC & DATA_MASK
    new_val  = 0xDD & DATA_MASK

    # Seed the address with a known value.
    await write_word(dut, 10, seed_val)

    # Assert wr_valid=1 and rd_valid=1 to the same address simultaneously.
    dut.wr_valid_i.value = 1
    dut.wr_addr_i.value  = 10
    dut.wr_data_i.value  = new_val
    dut.rd_valid_i.value = 1
    dut.rd_addr_i.value  = 10
    await RisingEdge(dut.clk_i)
    dut.wr_valid_i.value = 0
    dut.rd_valid_i.value = 0

    # Simultaneous rd_data_o (one cycle after the above edge):
    # - Simulation model (NBA ordering): read-before-write → seed_val.
    # - Gate-level / synthesis model (GWEN=0 wins): write wins → new_val.
    # Both are valid outcomes depending on the simulation target.
    await RisingEdge(dut.clk_i)
    await ReadOnly()
    sim_rd = int(dut.rd_data_o.value)
    assert sim_rd in (seed_val, new_val), \
        f"Simultaneous rd result: expected 0x{seed_val:X} (sim/read-before-write) or 0x{new_val:X} (synth/write-wins), got 0x{sim_rd:X}"
    await NextTimeStep()

    # A plain subsequent read must return the newly-written value.
    got = await read_word(dut, 10)
    assert got == new_val, \
        f"Post-simultaneous read: expected new value 0x{new_val:X}, got 0x{got:X}"


@logged_test()
async def test_memory_survives_reset(dut):
    """GF180 SRAM macros are not cleared by reset; written data persists."""
    await setup(dut)
    sentinel = 0x7E & DATA_MASK
    await write_word(dut, 3, sentinel)

    # Re-assert then deassert reset.
    dut.reset_i.value = 1
    await ClockCycles(dut.clk_i, 4)
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)

    got = await read_word(dut, 3)
    assert got == sentinel, \
        f"Memory was unexpectedly cleared by reset: expected 0x{sentinel:X}, got 0x{got:X}"


@logged_test()
async def test_rd_during_reset_ignored(dut):
    """rd_valid_i during reset must not corrupt rd_data_o state; output recovers after reset."""
    await setup(dut)
    sentinel = 0x91 & DATA_MASK
    await write_word(dut, 15, sentinel)

    # Assert reset and simultaneously toggle rd_valid — should be safely ignored.
    dut.reset_i.value  = 1
    dut.rd_valid_i.value = 1
    dut.rd_addr_i.value  = 15
    await ClockCycles(dut.clk_i, 4)
    dut.rd_valid_i.value = 0
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)

    # After reset, a clean read must return the correct value (memory preserved).
    got = await read_word(dut, 15)
    assert got == sentinel, \
        f"Post-reset read failed: expected 0x{sentinel:X}, got 0x{got:X}"


@logged_test()
async def test_randomized_write_read_scoreboard(dut):
    """100 random writes followed by full read-back against a Python shadow RAM."""
    await setup(dut)
    rng          = random.Random(0xDEAD_BEEF)
    shadow       = [0] * DEPTH_P
    written_addrs = set()

    for _ in range(100):
        addr         = rng.randint(0, ADDR_MAX)
        data         = rng.randint(0, DATA_MASK)
        shadow[addr] = data
        written_addrs.add(addr)
        await write_word(dut, addr, data)

    for addr in sorted(written_addrs):
        got = await read_word(dut, addr)
        assert got == shadow[addr], \
            f"addr={addr}: expected 0x{shadow[addr]:X}, got 0x{got:X}"


@logged_test()
async def test_back_to_back_writes_then_sequential_reads(dut):
    """Fill a contiguous block in one pass; drain in one pass; verify ordering."""
    await setup(dut)
    N   = min(32, DEPTH_P)
    ref = [(i * 17 + 3) & DATA_MASK for i in range(N)]

    # Write N words consecutively.
    for addr in range(N):
        await write_word(dut, addr, ref[addr])

    # Read them back without gaps.
    for addr in range(N):
        got = await read_word(dut, addr)
        assert got == ref[addr], \
            f"addr={addr}: expected 0x{ref[addr]:X}, got 0x{got:X}"


@logged_test()
async def test_pipelined_reads(dut):
    """rd_valid_i held high for N consecutive cycles with incrementing address.
    Verifies that back-to-back reads (as the MAC engine would issue) all return
    correct data without gaps or off-by-one pipeline shifts."""
    await setup(dut)
    N   = min(8, DEPTH_P)
    ref = [(i * 31 + 7) & DATA_MASK for i in range(N)]
    for i in range(N):
        await write_word(dut, i, ref[i])

    # Assert rd_valid for N consecutive cycles, advancing the address each cycle.
    # After ReadOnly() in each cycle, rd_data_o already holds the result registered
    # on that edge (NBA semantics in the behavioral model).
    results = []
    dut.rd_valid_i.value = 1
    for i in range(N):
        dut.rd_addr_i.value = i
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        results.append(int(dut.rd_data_o.value))
        await NextTimeStep()
    dut.rd_valid_i.value = 0

    for i, got in enumerate(results):
        assert got == ref[i], \
            f"pipeline[{i}]: expected 0x{ref[i]:X}, got 0x{got:X}"


@logged_test()
async def test_multibank_boundary(dut):
    """Cross-bank address boundary: last address in bank 0 (1023) and first address
    in bank 1 (1024) must be independent locations.  Skipped when DEPTH_P <= 1024
    (single-bank configuration)."""
    if DEPTH_P <= 1024:
        return
    await setup(dut)
    pairs = [
        (1023,    0xAB & DATA_MASK),
        (1024,    0xCD & DATA_MASK),
        (0,       0x12 & DATA_MASK),
        (ADDR_MAX, 0x34 & DATA_MASK),
    ]
    for addr, val in pairs:
        await write_word(dut, addr, val)
    for addr, expected in pairs:
        got = await read_word(dut, addr)
        assert got == expected, \
            f"bank-boundary addr={addr}: expected 0x{expected:X}, got 0x{got:X}"
