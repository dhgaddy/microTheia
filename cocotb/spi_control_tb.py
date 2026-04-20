# SPDX-License-Identifier: Apache-2.0

import os

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly, NextTimeStep

MODULE = os.environ.get("TOPLEVEL")

SPI_PERIOD_NS = 20   # 50 MHz
CHIP_PERIOD_NS = 40  # 25 MHz
DATA_WIDTH = 32


async def setup(dut):
    cocotb.start_soon(Clock(dut.SCLK, SPI_PERIOD_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.clk_i, CHIP_PERIOD_NS, unit="ns").start())

    await NextTimeStep()

    dut.CS.value = 1
    dut.MOSI.value = 0
    dut.reset_i.value = 1
    dut.chip_out.value = 0
    dut.chip_out_valid.value = 0

    await ClockCycles(dut.clk_i, 5)
    await NextTimeStep()
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)


async def push_chip_out_word(dut, word):
    dut._log.info(f"Pushing 0x{word:08X} into chip_out")

    for _ in range(50):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if int(dut.chip_out_ready_o.value):
            break
    else:
        raise AssertionError("chip_out_ready_o never asserted")

    await NextTimeStep()
    dut.chip_out.value = word
    dut.chip_out_valid.value = 1

    await RisingEdge(dut.clk_i)
    await ReadOnly()

    await NextTimeStep()
    dut.chip_out_valid.value = 0
    dut.chip_out.value = 0


async def preload_tx_path(dut):
    # keep CS high and give chip-side TX path time to preload
    dut.CS.value = 1
    dut.MOSI.value = 0

    for _ in range(20):
        await RisingEdge(dut.clk_i)

    # let a few SPI half-cycles happen while idle/high-CS
    for _ in range(4):
        await FallingEdge(dut.SCLK)


async def spi_mode0_transfer_word(dut, mosi_word, width=DATA_WIDTH):
    """
    Simple SPI Mode 0 transfer:
      - CPOL=0, CPHA=0
      - data is sampled on rising edge
      - data changes on falling edge
    """
    dut._log.info(f"SPI mode 0 full-duplex transfer MOSI=0x{mosi_word:08X}")

    miso_word = 0

    # Align to low phase first so we can place first MOSI bit cleanly
    await FallingEdge(dut.SCLK)

    # Put first MOSI bit on the line while SCLK is low
    first_bit = (mosi_word >> (width - 1)) & 1
    dut.MOSI.value = first_bit

    # Start transaction
    dut.CS.value = 0

    for bit_idx in range(width):
        # Rising edge: both sides sample
        await RisingEdge(dut.SCLK)
        await ReadOnly()
        miso_bit = int(dut.MISO.value)
        miso_word = (miso_word << 1) | miso_bit

        # Falling edge: prepare next MOSI bit, except after last bit
        if bit_idx != width - 1:
            await FallingEdge(dut.SCLK)
            next_bit = (mosi_word >> (width - 2 - bit_idx)) & 1
            dut.MOSI.value = next_bit

    # Finish transaction on the next falling edge
    await FallingEdge(dut.SCLK)
    dut.CS.value = 1
    dut.MOSI.value = 0

    dut._log.info(f"Observed MISO=0x{miso_word:08X}")
    return miso_word


async def wait_for_chip_in_word(dut, expected_word=None, max_cycles=200):
    observed = None

    for cycle in range(max_cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()

        if int(dut.chip_in_valid.value):
            val = dut.chip_in.value
            dut._log.info(f"chip_in_valid=1 cycle {cycle}, chip_in={val}")

            if not val.is_resolvable:
                await NextTimeStep()
                continue

            observed = int(val)
            break

        await NextTimeStep()

    assert observed is not None, "Did not observe chip_in"

    if expected_word is not None:
        assert observed == expected_word, (
            f"chip_in mismatch: got 0x{observed:08X}, expected 0x{expected_word:08X}"
        )

    return observed


@logged_test()
async def test_minimal_full_duplex_sanity(dut):
    await setup(dut)

    tx_word = 0xCAFEBABE
    rx_word = 0xDEADBEEF

    await push_chip_out_word(dut, tx_word)
    await preload_tx_path(dut)

    miso_word = await spi_mode0_transfer_word(dut, rx_word, DATA_WIDTH)
    observed = await wait_for_chip_in_word(dut, expected_word=rx_word)

    assert miso_word == tx_word, (
        f"MISO mismatch: got 0x{miso_word:08X}, expected 0x{tx_word:08X}"
    )

    dut._log.info(
        f"PASS full duplex: chip_in=0x{observed:08X}, MISO=0x{miso_word:08X}"
    )