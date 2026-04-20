# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors

import os

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly, NextTimeStep

MODULE = os.environ.get("TOPLEVEL")

SPI_PERIOD_NS = 20   # 50 MHz
CHIP_PERIOD_NS = 40  # 25 MHz
DATA_WIDTH = 32      # use module defaults for first sanity test


async def setup(dut):
    cocotb.start_soon(Clock(dut.SCLK, SPI_PERIOD_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.clk_i, CHIP_PERIOD_NS, unit="ns").start())

    dut.CS.value = 1
    dut.MOSI.value = 0
    dut.reset_i.value = 1
    dut.chip_out.value = 0
    dut.chip_out_valid.value = 0

    await ClockCycles(dut.clk_i, 5)
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)


async def spi_send_word(dut, word, width=DATA_WIDTH):
    """Send one word MSB-first over MOSI using SPI mode 0 timing."""
    dut._log.info(f"Sending 0x{word:0{width // 4}X} over MOSI")

    dut.CS.value = 0

    for bit_idx in range(width):
        bit = (word >> (width - 1 - bit_idx)) & 1

        # mode 0: drive MOSI while clock is low so it is stable by next posedge
        await FallingEdge(dut.SCLK)
        dut.MOSI.value = bit

        # slave samples on posedge
        await RisingEdge(dut.SCLK)

    # return to idle after final sampled bit
    await FallingEdge(dut.SCLK)
    dut.CS.value = 1
    dut.MOSI.value = 0


@logged_test()
async def test_minimal_rx_sanity(dut):
    """Minimal sanity test: send one 32-bit word over MOSI and check chip_in."""
    await setup(dut)

    test_word = 0xDEADBEEF

    await spi_send_word(dut, test_word, DATA_WIDTH)

    observed = None
    for cycle in range(200):
        await RisingEdge(dut.clk_i)
        await ReadOnly()

        if int(dut.chip_in_valid.value) == 1:
            val = dut.chip_in.value
            dut._log.info(f"chip_in_valid=1 on chip cycle {cycle}, chip_in={val}")

            if not val.is_resolvable:
                dut._log.warning(f"chip_in contains X/Z on chip cycle {cycle}: {val}")
                await NextTimeStep()
                continue

            observed = int(val)
            dut._log.info(f"Observed resolved chip_in=0x{observed:08X}")
            break

        await NextTimeStep()

    assert observed is not None, "Did not observe resolved chip_in when chip_in_valid was asserted"
    assert observed == test_word, (
        f"chip_in mismatch: got 0x{observed:08X}, expected 0x{test_word:08X}"
    )