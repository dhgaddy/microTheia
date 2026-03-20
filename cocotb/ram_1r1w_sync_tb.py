# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2025 Group G Contributors
import os
import random

import cocotb
from util.test_logging import logged_test
from util.config_parser import load_config
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

MODULE = os.environ.get("TOPLEVEL", "ram_1r1w_sync")
CFG = load_config(MODULE)

WIDTH = CFG.get("width_p", 8)
DEPTH = CFG.get("depth_p", 512)
ADDR_MASK = DEPTH - 1
DATA_MASK = (1 << WIDTH) - 1


class Ram1R1WSyncModel:
    """Cycle-accurate model for ram_1r1w_sync parameterized by WIDTH/DEPTH."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.mem = [0] * DEPTH
        self.rd_data = 0

    def step(self, reset_i, wr_valid_i, wr_data_i, wr_addr_i, rd_valid_i, rd_addr_i):
        old_mem = self.mem.copy()

        if reset_i:
            self.rd_data = 0
        elif rd_valid_i:
            self.rd_data = old_mem[rd_addr_i & ADDR_MASK]

        if wr_valid_i:
            self.mem[wr_addr_i & ADDR_MASK] = wr_data_i & DATA_MASK

        return self.rd_data


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, units="ns").start())
    dut.reset_i.value = 1
    dut.wr_valid_i.value = 0
    dut.wr_data_i.value = 0
    dut.wr_addr_i.value = 0
    dut.rd_valid_i.value = 0
    dut.rd_addr_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)


async def drive_and_check(dut, model, reset_i, wr_valid_i, wr_data_i, wr_addr_i, rd_valid_i, rd_addr_i, tag):
    dut.reset_i.value = reset_i
    dut.wr_valid_i.value = wr_valid_i
    dut.wr_data_i.value = wr_data_i
    dut.wr_addr_i.value = wr_addr_i
    dut.rd_valid_i.value = rd_valid_i
    dut.rd_addr_i.value = rd_addr_i

    exp = model.step(reset_i, wr_valid_i, wr_data_i, wr_addr_i, rd_valid_i, rd_addr_i)

    await RisingEdge(dut.clk_i)
    await ReadOnly()

    got = int(dut.rd_data_o.value)
    assert got == exp, f"{tag}: rd_data_o DUT=0x{got:02X} model=0x{exp:02X}"

    await NextTimeStep()


@logged_test()
async def test_default_init_all_zeros(dut):
    """Unwritten RAM locations must read back as 0 (default filename_p="" initialises to zero).
    MUST run first — before any other test writes to the RAM."""
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        model.step(1, 0, 0, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0, 0, 0)

    # Sample a spread of addresses that were never written.
    sample_addrs = [0, 1, DEPTH // 4, DEPTH // 2, DEPTH // 2 + 1, DEPTH - 2, DEPTH - 1]
    for addr in sample_addrs:
        await drive_and_check(dut, model, 0, 0, 0, 0, 1, addr, f"init-zero-{addr}")
        got = int(dut.rd_data_o.value)
        assert got == 0, f"Unwritten addr {addr}: expected 0, got 0x{got:02X}"


@logged_test()
async def test_reset_and_basic_read_write(dut):
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    await drive_and_check(dut, model, 0, 1, 0xEF, 7, 0, 0, "write")
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 7, "read")
    assert int(dut.rd_data_o.value) == 0xEF


@logged_test()
async def test_read_before_write_same_cycle_same_addr(dut):
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    # Prime location.
    await drive_and_check(dut, model, 0, 1, 0x11, 12, 0, 0, "prime")

    # Same-cycle read and write same addr: rd_data should return old value.
    await drive_and_check(dut, model, 0, 1, 0x22, 12, 1, 12, "rw-same")
    assert int(dut.rd_data_o.value) == 0x11

    # Next read sees new value.
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 12, "read-new")
    assert int(dut.rd_data_o.value) == 0x22


@logged_test()
async def test_write_during_reset(dut):
    """RTL keeps writes active during reset; verify this edge behavior."""
    await setup(dut)
    model = Ram1R1WSyncModel()

    # Keep reset high and write.
    await drive_and_check(dut, model, 1, 1, 0xA5, 33, 0, 0, "wr-in-rst")

    # Deassert reset and read back written value.
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 33, "read-after-rst")
    assert int(dut.rd_data_o.value) == 0xA5


@logged_test()
async def test_randomized_golden_scoreboard(dut):
    await setup(dut)
    model = Ram1R1WSyncModel()
    rng = random.Random(0xAA551234)

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    for cycle in range(3000):
        reset_i = 1 if rng.randint(0, 199) == 0 else 0
        wr_valid_i = rng.randint(0, 1)
        rd_valid_i = rng.randint(0, 1)
        wr_data_i = rng.getrandbits(WIDTH)
        wr_addr_i = rng.randint(0, DEPTH - 1)
        rd_addr_i = rng.randint(0, DEPTH - 1)

        await drive_and_check(
            dut,
            model,
            reset_i,
            wr_valid_i,
            wr_data_i,
            wr_addr_i,
            rd_valid_i,
            rd_addr_i,
            f"rnd-{cycle}",
        )


@logged_test()
async def test_rd_valid_low_holds_output(dut):
    """When rd_valid_i=0, rd_data_o must hold its previously latched value."""
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    await drive_and_check(dut, model, 0, 1, 0xBE, 5, 0, 0, "write")
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 5, "read")
    latched = int(dut.rd_data_o.value)
    assert latched == 0xBE, f"Expected 0xBE, got 0x{latched:02X}"

    for addr in [0, 100, 255, DEPTH - 1]:
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, addr, f"hold-{addr}")
        assert int(dut.rd_data_o.value) == 0xBE, \
            f"rd_data_o changed while rd_valid_i=0 (addr={addr})"


@logged_test()
async def test_boundary_addresses(dut):
    """Write then read at address 0, DEPTH-1, and DEPTH//2."""
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    test_cases = [
        (0,          0xAA),
        (DEPTH - 1,  0x55),
        (DEPTH // 2, 0xDE),
    ]
    for addr, data in test_cases:
        await drive_and_check(dut, model, 0, 1, data, addr, 0, 0, f"wr-{addr}")
        await drive_and_check(dut, model, 0, 0, 0, 0, 1, addr, f"rd-{addr}")
        got = int(dut.rd_data_o.value)
        assert got == data, \
            f"Addr {addr}: expected 0x{data:02X}, got 0x{got:02X}"


@logged_test()
async def test_all_ones_data(dut):
    """All-ones data word (DATA_MASK) stored and read back intact."""
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    await drive_and_check(dut, model, 0, 1, DATA_MASK, 0, 0, 0, "wr-max")
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 0, "rd-max")
    got = int(dut.rd_data_o.value)
    assert got == DATA_MASK, f"All-ones data corrupted: 0x{got:08X}"


@logged_test()
async def test_overwrite_same_address(dut):
    """Overwriting the same address multiple times; final value must be last write."""
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    addr = 42
    values = [0x11, 0x22, 0x33, 0x44]
    for v in values:
        await drive_and_check(dut, model, 0, 1, v, addr, 0, 0, f"wr-{v:08X}")

    await drive_and_check(dut, model, 0, 0, 0, 0, 1, addr, "rd-final")
    got = int(dut.rd_data_o.value)
    assert got == values[-1], f"Expected 0x{values[-1]:02X}, got 0x{got:02X}"
