# SPDX-License-Identifier: Apache-2.0

import os

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import (
    ClockCycles,
    RisingEdge,
    FallingEdge,
    ReadOnly,
    NextTimeStep,
)

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


async def wait_for_ready_high(dut, max_cycles=200):
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if int(dut.chip_out_ready_o.value):
            return cycle
        await NextTimeStep()
    raise AssertionError("chip_out_ready_o never asserted")


async def push_chip_out_word(dut, word):
    dut._log.info(f"Pushing 0x{word:08X} into chip_out")

    accept_cycle = await wait_for_ready_high(dut)

    await NextTimeStep()
    dut.chip_out.value = word
    dut.chip_out_valid.value = 1

    await RisingEdge(dut.clk_i)
    await ReadOnly()

    await NextTimeStep()
    dut.chip_out_valid.value = 0
    dut.chip_out.value = 0

    return accept_cycle


async def push_chip_out_word_until_accepted(dut, word, max_cycles=200):
    dut._log.info(f"Pushing-with-backpressure 0x{word:08X} into chip_out")

    await NextTimeStep()
    dut.chip_out.value = word
    dut.chip_out_valid.value = 1

    for cycle in range(max_cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        if int(dut.chip_out_ready_o.value):
            await NextTimeStep()
            dut.chip_out_valid.value = 0
            dut.chip_out.value = 0
            return cycle

    await NextTimeStep()
    dut.chip_out_valid.value = 0
    dut.chip_out.value = 0
    raise AssertionError("chip_out transfer was never accepted")


async def preload_tx_path(dut, chip_cycles=20, spi_half_cycles=4):
    dut.CS.value = 1
    dut.MOSI.value = 0

    for _ in range(chip_cycles):
        await RisingEdge(dut.clk_i)

    for _ in range(spi_half_cycles):
        await FallingEdge(dut.SCLK)


async def spi_mode0_transfer_word(dut, mosi_word, width=DATA_WIDTH):
    """
    Simple SPI mode 0 full-duplex transfer.
    Returns (miso_word, last_sample_time_ns).
    Transaction ends immediately after the final sampled rising edge.
    """
    dut._log.info(f"SPI mode 0 full-duplex transfer MOSI=0x{mosi_word:08X}")

    miso_word = 0

    await FallingEdge(dut.SCLK)
    await NextTimeStep()
    dut.MOSI.value = (mosi_word >> (width - 1)) & 1
    dut.CS.value = 0

    last_sample_time_ns = None

    for bit_idx in range(width):
        await RisingEdge(dut.SCLK)
        await ReadOnly()

        miso_bit = int(dut.MISO.value)
        miso_word = (miso_word << 1) | miso_bit
        last_sample_time_ns = cocotb.utils.get_sim_time(unit="ns")

        if bit_idx != width - 1:
            await FallingEdge(dut.SCLK)
            await NextTimeStep()
            next_bit = (mosi_word >> (width - 2 - bit_idx)) & 1
            dut.MOSI.value = next_bit

    await NextTimeStep()
    dut.CS.value = 1
    dut.MOSI.value = 0

    dut._log.info(f"Observed MISO=0x{miso_word:08X}")
    return miso_word, last_sample_time_ns


async def spi_mode0_transfer_stream_words(dut, mosi_words, width=DATA_WIDTH):
    """
    Continuous SPI mode 0 transfer with CS held low across multiple words.
    Returns (miso_words, last_sample_time_ns).
    """
    dut._log.info(
        "SPI mode 0 streaming transfer MOSI words="
        + str([f"0x{w:08X}" for w in mosi_words])
    )

    total_bits = len(mosi_words) * width
    flat_mosi = []

    for word in mosi_words:
        for bit_idx in range(width):
            flat_mosi.append((word >> (width - 1 - bit_idx)) & 1)

    miso_bits = []

    await FallingEdge(dut.SCLK)
    await NextTimeStep()
    dut.MOSI.value = flat_mosi[0]
    dut.CS.value = 0

    last_sample_time_ns = None

    for bit_idx in range(total_bits):
        await RisingEdge(dut.SCLK)
        await ReadOnly()

        miso_bits.append(int(dut.MISO.value))
        last_sample_time_ns = cocotb.utils.get_sim_time(unit="ns")

        if bit_idx != total_bits - 1:
            await FallingEdge(dut.SCLK)
            await NextTimeStep()
            dut.MOSI.value = flat_mosi[bit_idx + 1]

    await NextTimeStep()
    dut.CS.value = 1
    dut.MOSI.value = 0

    miso_words = []
    for word_idx in range(len(mosi_words)):
        value = 0
        start = word_idx * width
        end = start + width
        for bit in miso_bits[start:end]:
            value = (value << 1) | bit
        miso_words.append(value)

    dut._log.info(
        "Observed streaming MISO words="
        + str([f"0x{w:08X}" for w in miso_words])
    )

    return miso_words, last_sample_time_ns


async def spi_send_partial_word_then_abort(dut, word, nbits, width=DATA_WIDTH):
    assert 0 < nbits < width, "nbits must be between 1 and width-1"

    dut._log.info(f"Sending partial SPI word 0x{word:08X} for {nbits} bits then aborting")

    await FallingEdge(dut.SCLK)
    await NextTimeStep()
    dut.MOSI.value = (word >> (width - 1)) & 1
    dut.CS.value = 0

    await RisingEdge(dut.SCLK)

    for bit_idx in range(1, nbits):
        await FallingEdge(dut.SCLK)
        await NextTimeStep()
        bit = (word >> (width - 1 - bit_idx)) & 1
        dut.MOSI.value = bit
        await RisingEdge(dut.SCLK)

    await NextTimeStep()
    dut.CS.value = 1
    dut.MOSI.value = 0


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


async def wait_for_chip_in_words(dut, expected_words=None, max_cycles_per_word=200):
    observed_words = []
    target_count = len(expected_words) if expected_words is not None else 1

    for word_idx in range(target_count):
        observed = None

        for cycle in range(max_cycles_per_word):
            await RisingEdge(dut.clk_i)
            await ReadOnly()

            if int(dut.chip_in_valid.value):
                val = dut.chip_in.value
                dut._log.info(
                    f"chip_in_valid=1 word_idx={word_idx} cycle {cycle}, chip_in={val}"
                )

                if not val.is_resolvable:
                    await NextTimeStep()
                    continue

                observed = int(val)
                break

            await NextTimeStep()

        assert observed is not None, f"Did not observe chip_in word {word_idx}"
        observed_words.append(observed)

        if expected_words is not None:
            assert observed == expected_words[word_idx], (
                f"chip_in word {word_idx} mismatch: got 0x{observed:08X}, "
                f"expected 0x{expected_words[word_idx]:08X}"
            )

    return observed_words


async def drain_chip_in_words_best_effort(dut, expected_count=None, max_chip_cycles=5000):
    """
    Best-effort draining of chip_in while a burst is in flight.
    Collects words whenever chip_in_valid asserts.
    If expected_count is provided, stops after collecting that many words.
    """
    observed_words = []

    for cycle in range(max_chip_cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()

        if int(dut.chip_in_valid.value):
            val = dut.chip_in.value
            if val.is_resolvable:
                observed_words.append(int(val))
                dut._log.info(
                    f"drain_chip_in_words_best_effort: captured word_idx={len(observed_words)-1} "
                    f"value=0x{int(val):08X}"
                )

                if expected_count is not None and len(observed_words) >= expected_count:
                    return observed_words

        await NextTimeStep()

    return observed_words


async def assert_no_chip_in_valid(dut, cycles, tag):
    for cycle in range(cycles):
        await RisingEdge(dut.clk_i)
        await ReadOnly()
        assert int(dut.chip_in_valid.value) == 0, (
            f"{tag}: chip_in_valid unexpectedly high on cycle {cycle}"
        )
        await NextTimeStep()


async def run_full_duplex_case(dut, tx_word, rx_word, tag):
    dut._log.info(
        f"{tag}: starting case with chip_out=0x{tx_word:08X}, MOSI=0x{rx_word:08X}"
    )

    await push_chip_out_word(dut, tx_word)
    await preload_tx_path(dut)

    miso_word, _ = await spi_mode0_transfer_word(dut, rx_word, DATA_WIDTH)
    observed = await wait_for_chip_in_word(dut, expected_word=rx_word)

    assert miso_word == tx_word, (
        f"{tag}: MISO mismatch: got 0x{miso_word:08X}, expected 0x{tx_word:08X}"
    )

    dut._log.info(
        f"{tag}: PASS chip_in=0x{observed:08X}, MISO=0x{miso_word:08X}"
    )


@logged_test()
async def test_spi_control_multi_full_duplex(dut):
    await setup(dut)

    # -------------------------------------------------
    # 1. Minimal full-duplex sanity
    # -------------------------------------------------
    await run_full_duplex_case(
        dut,
        tx_word=0xCAFEBABE,
        rx_word=0xDEADBEEF,
        tag="scenario_1_minimal_sanity",
    )

    # -------------------------------------------------
    # 2. Back-to-back style directed cases in same run
    # -------------------------------------------------
    directed_cases = [
        (0x11111111, 0xAAAAAAAA, "scenario_2_case_0"),
        (0x22222222, 0x55555555, "scenario_2_case_1"),
        (0x33333333, 0x0F0F0F0F, "scenario_2_case_2"),
    ]

    for tx_word, rx_word, tag in directed_cases:
        await run_full_duplex_case(dut, tx_word, rx_word, tag)

    # -------------------------------------------------
    # 3. Pattern coverage
    # -------------------------------------------------
    pattern_cases = [
        (0x00000000, 0xFFFFFFFF, "scenario_3_all0_all1"),
        (0xFFFFFFFF, 0x00000000, "scenario_3_all1_all0"),
        (0xAAAAAAAA, 0x55555555, "scenario_3_alt_a5"),
        (0x55555555, 0xAAAAAAAA, "scenario_3_alt_5a"),
        (0x80000001, 0x7FFFFFFE, "scenario_3_edges"),
    ]

    for tx_word, rx_word, tag in pattern_cases:
        await run_full_duplex_case(dut, tx_word, rx_word, tag)

    # -------------------------------------------------
    # 4. A few arbitrary mixed values
    # -------------------------------------------------
    mixed_cases = [
        (0x01234567, 0x89ABCDEF, "scenario_4_mixed_0"),
        (0x13579BDF, 0x2468ACE0, "scenario_4_mixed_1"),
        (0xFEEDC0DE, 0x12345678, "scenario_4_mixed_2"),
    ]

    for tx_word, rx_word, tag in mixed_cases:
        await run_full_duplex_case(dut, tx_word, rx_word, tag)

    # -------------------------------------------------
    # 5. Early-CS abort / partial transfer
    # -------------------------------------------------
    partial_word = 0xABCDEF01
    dut._log.info("scenario_5_partial_abort: sending partial word then aborting")
    await spi_send_partial_word_then_abort(dut, partial_word, nbits=13)

    await assert_no_chip_in_valid(dut, cycles=20, tag="scenario_5_partial_abort")

    await run_full_duplex_case(
        dut,
        tx_word=0x0BADF00D,
        rx_word=0x1234ABCD,
        tag="scenario_5_followup_clean_word",
    )

    # -------------------------------------------------
    # 6. Minimal-idle back-to-back transactions
    # -------------------------------------------------
    dut._log.info("scenario_6_min_idle_back_to_back: two transfers with minimal idle spacing")
    await push_chip_out_word(dut, 0x89ABCDEF)
    await preload_tx_path(dut, chip_cycles=20, spi_half_cycles=1)
    miso_0, _ = await spi_mode0_transfer_word(dut, 0x10203040, DATA_WIDTH)
    observed_0 = await wait_for_chip_in_word(dut, expected_word=0x10203040)
    assert miso_0 == 0x89ABCDEF, (
        f"scenario_6 transfer 0 MISO mismatch: got 0x{miso_0:08X}, expected 0x89ABCDEF"
    )

    await push_chip_out_word(dut, 0x76543210)
    await preload_tx_path(dut, chip_cycles=10, spi_half_cycles=2)
    miso_1, _ = await spi_mode0_transfer_word(dut, 0x55667788, DATA_WIDTH)
    observed_1 = await wait_for_chip_in_word(dut, expected_word=0x55667788)
    assert miso_1 == 0x76543210, (
        f"scenario_6 transfer 1 MISO mismatch: got 0x{miso_1:08X}, expected 0x76543210"
    )
    dut._log.info(
        f"scenario_6_min_idle_back_to_back: PASS "
        f"chip_in0=0x{observed_0:08X}, chip_in1=0x{observed_1:08X}"
    )

    # -------------------------------------------------
    # 7. No-TX-data-available case
    # -------------------------------------------------
    dut._log.info("scenario_7_no_tx_data: transfer without pushing chip_out first")
    miso_word, _ = await spi_mode0_transfer_word(dut, 0xCAFED00D)
    observed = await wait_for_chip_in_word(dut, expected_word=0xCAFED00D)

    assert miso_word == 0x00000000, (
        f"scenario_7_no_tx_data: expected zero-filled MISO, got 0x{miso_word:08X}"
    )
    dut._log.info(
        f"scenario_7_no_tx_data: PASS chip_in=0x{observed:08X}, MISO=0x{miso_word:08X}"
    )

    # -------------------------------------------------
    # 8. TX queueing / ordering of multiple words with continuous CS low streaming
    # -------------------------------------------------
    dut._log.info("scenario_8_tx_queueing: queue multiple TX words and stream them out under one CS-low window")
    queued_words = [0xAAAABBBB, 0xCCCCDDDD, 0xEEEEFFFF]
    rx_words =     [0x11112222, 0x33334444, 0x55556666]

    for word in queued_words:
        await push_chip_out_word(dut, word)

    await preload_tx_path(dut, chip_cycles=30, spi_half_cycles=4)

    rx_task = cocotb.start_soon(wait_for_chip_in_words(dut, expected_words=rx_words))
    miso_words, _ = await spi_mode0_transfer_stream_words(dut, rx_words)
    observed_words = await rx_task

    assert miso_words == queued_words, (
        f"scenario_8_tx_queueing: MISO stream mismatch: "
        f"got {[f'0x{x:08X}' for x in miso_words]}, "
        f"expected {[f'0x{x:08X}' for x in queued_words]}"
    )

    dut._log.info(
        "scenario_8_tx_queueing: PASS "
        f"chip_in={[f'0x{x:08X}' for x in observed_words]}, "
        f"MISO={[f'0x{x:08X}' for x in miso_words]}"
    )

    # -------------------------------------------------
    # 9. Latency measurement: push accepted -> last MISO bit sampled
    # -------------------------------------------------
    dut._log.info("scenario_9_latency_measurement: measuring push-to-last-MISO latency")

    tx_word = 0x13572468
    rx_word = 0x89ABCDEF

    await wait_for_ready_high(dut)
    await NextTimeStep()
    dut.chip_out.value = tx_word
    dut.chip_out_valid.value = 1
    push_accept_time_ns = None

    await RisingEdge(dut.clk_i)
    await ReadOnly()
    push_accept_time_ns = cocotb.utils.get_sim_time(unit="ns")

    await NextTimeStep()
    dut.chip_out_valid.value = 0
    dut.chip_out.value = 0

    await preload_tx_path(dut)
    miso_word, last_sample_time_ns = await spi_mode0_transfer_word(dut, rx_word)
    observed = await wait_for_chip_in_word(dut, expected_word=rx_word)

    assert miso_word == tx_word, (
        f"scenario_9_latency_measurement: MISO mismatch: "
        f"got 0x{miso_word:08X}, expected 0x{tx_word:08X}"
    )

    latency_ns = last_sample_time_ns - push_accept_time_ns
    chip_cycles_latency = latency_ns / CHIP_PERIOD_NS
    spi_cycles_latency = latency_ns / SPI_PERIOD_NS

    dut._log.info(
        "scenario_9_latency_measurement: "
        f"push_accept_time_ns={push_accept_time_ns:.1f}, "
        f"last_sample_time_ns={last_sample_time_ns:.1f}, "
        f"latency_ns={latency_ns:.1f}, "
        f"latency_chip_cycles={chip_cycles_latency:.2f}, "
        f"latency_spi_cycles={spi_cycles_latency:.2f}, "
        f"chip_in=0x{observed:08X}, MISO=0x{miso_word:08X}"
    )

    # -------------------------------------------------
    # 10. Overflow behavior (last scenario) -- RX/MOSI side while actively draining
    # -------------------------------------------------
    dut._log.info("scenario_10_overflow: attempting to trigger RX-side overflow while chip_in drains as fast as possible")

    overflow_rx_words = [0xA0000000 + i for i in range((1 << 6) + 80)]
    await NextTimeStep()
    dut.chip_out.value = 0
    dut.chip_out_valid.value = 0

    drain_task = cocotb.start_soon(
        drain_chip_in_words_best_effort(
            dut,
            expected_count=len(overflow_rx_words),
            max_chip_cycles=8000
        )
    )

    _miso_words, _ = await spi_mode0_transfer_stream_words(dut, overflow_rx_words)
    drained_words = await drain_task

    for _ in range(20):
        await RisingEdge(dut.clk_i)

    await ReadOnly()
    in_ovfl = int(dut.in_ovfl.value)
    sram_in_ovfl = int(dut.sram_in_ovfl.value)

    dut._log.info(
        f"scenario_10_overflow: drained_count={len(drained_words)}, "
        f"in_ovfl={in_ovfl}, sram_in_ovfl={sram_in_ovfl}"
    )

    # Primary correctness check: no data loss
    assert len(drained_words) == len(overflow_rx_words), (
    f"scenario_10_overflow: data loss under load "
    f"(got {len(drained_words)}, expected {len(overflow_rx_words)})"
    )

    # Informational only (not a failure condition)
    if (in_ovfl == 0) and (sram_in_ovfl == 0):
        dut._log.info("scenario_10_overflow: PASS (no overflow, sustained throughput)")
    else:
        dut._log.info("scenario_10_overflow: PASS (overflow observed under stress)")

        dut._log.info("All sequential full-duplex scenarios passed")