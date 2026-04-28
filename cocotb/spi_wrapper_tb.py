# SPDX-License-Identifier: Apache-2.0

import os

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import (
    ClockCycles,
    RisingEdge,
    ReadOnly,
    NextTimeStep,
)

MODULE = os.environ.get("TOPLEVEL")

CHIP_PERIOD_NS = 31.25  # 32 MHz
DATA_WIDTH = 32

# Manual SCLK. Trying two real "clock" signals in cocotb was causing problems.
# I set default test to chip at 32 MHz and SCLk at 8MHz
# Each SCLK half-cycle is this many chip clk cycles.
# SCLK period = 2 * SPI_HALF_CHIP_CYCLES * CHIP_PERIOD_NS
# With 4: SCLK ~= 4 MHz.
SPI_HALF_CHIP_CYCLES_DEFAULT = 4


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, CHIP_PERIOD_NS, unit="ns").start())

    await NextTimeStep()

    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0
    dut.reset.value = 1

    dut.gesture.value = 0
    dut.gesture_valid.value = 0
    dut.gesture_confidence.value = 0

    await ClockCycles(dut.clk, 8)
    await NextTimeStep()

    dut.reset.value = 0

    await ClockCycles(dut.clk, 20)


async def wait_for_spi_ready(dut, max_cycles=500):
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()

        if int(dut.spi_ready.value):
            dut._log.info(f"spi_ready asserted after {cycle} clk cycles")
            await NextTimeStep()
            return cycle

        await NextTimeStep()

    raise AssertionError("spi_ready never asserted")


async def pulse_classification(dut, gesture, confidence):
    dut._log.info(
        f"Pulsing classification gesture={gesture:02b}, confidence={confidence}"
    )

    await NextTimeStep()
    dut.gesture.value = gesture
    dut.gesture_confidence.value = confidence
    dut.gesture_valid.value = 1

    await RisingEdge(dut.clk)
    await NextTimeStep()

    dut.gesture_valid.value = 0


def expected_miso_from_classification(gesture, confidence):
    classification = ((confidence & 1) << 2) | (gesture & 0b11)
    return classification << (DATA_WIDTH - 3)


async def spi_mode0_transfer_word(
    dut,
    mosi_word,
    width=DATA_WIDTH,
    half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT,
    pre_start_cycles=4,
    post_finish_cycles=4,
    cs_high_gap_cycles=4,
):
    """
    Manual SPI mode 0 full-duplex transfer.

    Important for this third-party SPI IP:
    - SCLK is sampled by clk, so SCLK transitions must be held for multiple clk cycles.
    - MOSI is sampled on rising SCLK.
    - The internal bit counter advances/completes on falling SCLK.
    - The word completes after the final falling SCLK edge.
    """
    dut._log.info(f"SPI mode 0 full-duplex transfer MOSI=0x{mosi_word:08X}")

    miso_word = 0

    # Idle before transaction
    await NextTimeStep()
    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0

    await ClockCycles(dut.clk, cs_high_gap_cycles)

    # Assert CS while SCLK is low and present first MOSI bit.
    await NextTimeStep()
    dut.CS.value = 0
    dut.MOSI.value = (mosi_word >> (width - 1)) & 1

    # Give wrapper time to pulse process_next_word while CS low and SCLK low.
    await ClockCycles(dut.clk, pre_start_cycles)

    for bit_idx in range(width):
        # Rising edge: master samples MISO, slave samples MOSI.
        await NextTimeStep()
        dut.SCLK.value = 1

        await ClockCycles(dut.clk, half_cycles)
        await ReadOnly()

        miso_bit = int(dut.MISO.value)
        miso_word = (miso_word << 1) | miso_bit

        await NextTimeStep()

        # Falling edge: SPI IP advances bit counter / completes final bit.
        dut.SCLK.value = 0

        await ClockCycles(dut.clk, half_cycles)

        # Present next MOSI bit while SCLK is low.
        if bit_idx != width - 1:
            await NextTimeStep()
            dut.MOSI.value = (mosi_word >> (width - 2 - bit_idx)) & 1

    # Keep CS low briefly after final falling edge so wrapper sees completion.
    await ClockCycles(dut.clk, post_finish_cycles)

    await NextTimeStep()
    dut.CS.value = 1
    dut.MOSI.value = 0

    await ClockCycles(dut.clk, 4)

    dut._log.info(f"Observed MISO=0x{miso_word:08X}")
    return miso_word


async def spi_mode0_stream_words(
    dut,
    mosi_words,
    width=DATA_WIDTH,
    half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT,
    pre_start_cycles=4,
    inter_word_low_cycles=4,
    post_finish_cycles=4,
    cs_high_gap_cycles=4,
):
    """
    Manual SPI mode 0 streaming transfer.

    CS is held low across all words:
        CS low -> word0 -> word1 -> ... -> wordN -> CS high

    Returns list of observed MISO words.

    This test intentionally leaves SCLK low for inter_word_low_cycles after
    each completed word before starting the next word. That gives the wrapper
    time to observe processing_word fall and re-assert process_next_word while
    CS remains low.
    """
    dut._log.info(
        "SPI mode 0 streaming transfer MOSI words="
        + str([f"0x{w:08X}" for w in mosi_words])
    )

    assert len(mosi_words) > 0, "mosi_words must not be empty"

    miso_words = []

    # Idle before stream
    await NextTimeStep()
    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0

    await ClockCycles(dut.clk, cs_high_gap_cycles)

    # Assert CS once for entire stream.
    await NextTimeStep()
    dut.CS.value = 0
    dut.MOSI.value = (mosi_words[0] >> (width - 1)) & 1

    # Give wrapper time to request first word while CS low and SCLK low.
    await ClockCycles(dut.clk, pre_start_cycles)

    for word_idx, mosi_word in enumerate(mosi_words):
        dut._log.info(f"Streaming word_idx={word_idx}, MOSI=0x{mosi_word:08X}")

        miso_word = 0

        for bit_idx in range(width):
            # Rising edge: master samples MISO, slave samples MOSI.
            await NextTimeStep()
            dut.SCLK.value = 1

            await ClockCycles(dut.clk, half_cycles)
            await ReadOnly()

            miso_bit = int(dut.MISO.value)
            miso_word = (miso_word << 1) | miso_bit

            await NextTimeStep()

            # Falling edge: SPI IP advances bit counter / completes final bit.
            dut.SCLK.value = 0

            await ClockCycles(dut.clk, half_cycles)

            # Present next bit while SCLK is low.
            if bit_idx != width - 1:
                await NextTimeStep()
                dut.MOSI.value = (mosi_word >> (width - 2 - bit_idx)) & 1

        miso_words.append(miso_word)

        # Keep CS low and SCLK low between words so the wrapper can re-arm.
        await ClockCycles(dut.clk, inter_word_low_cycles)

        # Present first bit of next word before the next rising edge.
        if word_idx != len(mosi_words) - 1:
            next_word = mosi_words[word_idx + 1]
            await NextTimeStep()
            dut.MOSI.value = (next_word >> (width - 1)) & 1

    await ClockCycles(dut.clk, post_finish_cycles)

    await NextTimeStep()
    dut.CS.value = 1
    dut.MOSI.value = 0
    dut.SCLK.value = 0

    await ClockCycles(dut.clk, 8)

    dut._log.info(
        "Observed streaming MISO words="
        + str([f"0x{w:08X}" for w in miso_words])
    )

    return miso_words


async def wait_for_evt_word(dut, expected_word=None, max_cycles=1000):
    observed = None

    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()

        if int(dut.evt_valid.value):
            val = dut.evt_word.value
            dut._log.info(f"evt_valid=1 cycle={cycle}, evt_word={val}")

            if val.is_resolvable:
                observed = int(val)
                break

        await NextTimeStep()

    assert observed is not None, "Did not observe evt_valid"

    if expected_word is not None:
        assert observed == expected_word, (
            f"evt_word mismatch: got 0x{observed:08X}, "
            f"expected 0x{expected_word:08X}"
        )

    return observed


async def wait_for_evt_words(dut, expected_words, max_cycles_per_word=1000):
    observed_words = []

    for word_idx, expected_word in enumerate(expected_words):
        observed = None

        for cycle in range(max_cycles_per_word):
            await RisingEdge(dut.clk)
            await ReadOnly()

            if int(dut.evt_valid.value):
                val = dut.evt_word.value
                dut._log.info(
                    f"evt_valid=1 word_idx={word_idx} cycle={cycle}, evt_word={val}"
                )

                if val.is_resolvable:
                    observed = int(val)
                    break

            await NextTimeStep()

        assert observed is not None, f"Did not observe evt_valid for word_idx={word_idx}"

        assert observed == expected_word, (
            f"evt_word mismatch word_idx={word_idx}: "
            f"got 0x{observed:08X}, expected 0x{expected_word:08X}"
        )

        observed_words.append(observed)

    return observed_words


async def assert_no_evt_valid(dut, cycles, tag):
    for cycle in range(cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()

        assert int(dut.evt_valid.value) == 0, (
            f"{tag}: evt_valid unexpectedly asserted at cycle {cycle}"
        )

        await NextTimeStep()


async def run_duplex_case(
    dut,
    gesture,
    confidence,
    mosi_word,
    tag,
    pulse_new_classification=True,
    half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT,
    pre_start_cycles=4,
    cs_high_gap_cycles=4,
):
    expected_miso = expected_miso_from_classification(gesture, confidence)

    dut._log.info(
        f"{tag}: gesture={gesture:02b}, confidence={confidence}, "
        f"expected_miso=0x{expected_miso:08X}, MOSI=0x{mosi_word:08X}, "
        f"pulse_new_classification={pulse_new_classification}"
    )

    if pulse_new_classification:
        await pulse_classification(dut, gesture, confidence)
        await ClockCycles(dut.clk, 10)

    rx_task = cocotb.start_soon(wait_for_evt_word(dut, expected_word=mosi_word))
    miso_word = await spi_mode0_transfer_word(
        dut,
        mosi_word,
        half_cycles=half_cycles,
        pre_start_cycles=pre_start_cycles,
        cs_high_gap_cycles=cs_high_gap_cycles,
    )
    observed_evt_word = await rx_task

    assert miso_word == expected_miso, (
        f"{tag}: MISO mismatch: got 0x{miso_word:08X}, "
        f"expected 0x{expected_miso:08X}"
    )

    dut._log.info(
        f"{tag}: PASS evt_word=0x{observed_evt_word:08X}, "
        f"MISO=0x{miso_word:08X}"
    )


async def run_streaming_case(
    dut,
    mosi_words,
    gesture,
    confidence,
    tag,
    half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT,
    inter_word_low_cycles=4,
):
    expected_miso_word = expected_miso_from_classification(gesture, confidence)
    expected_miso_words = [expected_miso_word for _ in mosi_words]

    dut._log.info(
        f"{tag}: streaming {len(mosi_words)} words under one CS-low window, "
        f"gesture={gesture:02b}, confidence={confidence}, "
        f"expected_miso_each=0x{expected_miso_word:08X}"
    )

    await pulse_classification(dut, gesture, confidence)
    await ClockCycles(dut.clk, 10)

    rx_task = cocotb.start_soon(wait_for_evt_words(dut, expected_words=mosi_words))

    miso_words = await spi_mode0_stream_words(
        dut,
        mosi_words,
        half_cycles=half_cycles,
        inter_word_low_cycles=inter_word_low_cycles,
    )

    observed_evt_words = await rx_task

    assert miso_words == expected_miso_words, (
        f"{tag}: streaming MISO mismatch: "
        f"got {[f'0x{x:08X}' for x in miso_words]}, "
        f"expected {[f'0x{x:08X}' for x in expected_miso_words]}"
    )

    dut._log.info(
        f"{tag}: PASS evt_words={[f'0x{x:08X}' for x in observed_evt_words]}, "
        f"MISO={[f'0x{x:08X}' for x in miso_words]}"
    )


async def spi_partial_word_then_abort(
    dut,
    mosi_word,
    nbits,
    width=DATA_WIDTH,
    half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT,
):
    assert 0 < nbits < width, "nbits must be between 1 and width-1"

    dut._log.info(
        f"Sending partial SPI word 0x{mosi_word:08X} for {nbits} bits, then aborting"
    )

    await NextTimeStep()
    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0
    await ClockCycles(dut.clk, 4)

    await NextTimeStep()
    dut.CS.value = 0
    dut.MOSI.value = (mosi_word >> (width - 1)) & 1
    await ClockCycles(dut.clk, 4)

    for bit_idx in range(nbits):
        await NextTimeStep()
        dut.SCLK.value = 1
        await ClockCycles(dut.clk, half_cycles)

        await NextTimeStep()
        dut.SCLK.value = 0
        await ClockCycles(dut.clk, half_cycles)

        if bit_idx != nbits - 1:
            await NextTimeStep()
            dut.MOSI.value = (mosi_word >> (width - 2 - bit_idx)) & 1

    await NextTimeStep()
    dut.CS.value = 1
    dut.MOSI.value = 0
    dut.SCLK.value = 0

    # Give wrapper enough time to issue abort reset and for SPI IP to reinitialize.
    await ClockCycles(dut.clk, 50)


async def toggle_sclk_with_cs_high(dut, cycles, half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT):
    dut._log.info(f"Toggling SCLK {cycles} cycles while CS remains high")

    await NextTimeStep()
    dut.CS.value = 1
    dut.MOSI.value = 0
    dut.SCLK.value = 0

    await ClockCycles(dut.clk, 4)

    for _ in range(cycles):
        await NextTimeStep()
        dut.SCLK.value = 1
        await ClockCycles(dut.clk, half_cycles)

        await NextTimeStep()
        dut.SCLK.value = 0
        await ClockCycles(dut.clk, half_cycles)

    await ClockCycles(dut.clk, 10)


@logged_test()
async def test_spi_wrapper_duplex_sanity(dut):
    await setup(dut)
    await wait_for_spi_ready(dut)

    # -------------------------------------------------
    # 1. Existing minimal full-duplex sanity
    # -------------------------------------------------
    await run_duplex_case(
        dut,
        gesture=0b01,
        confidence=1,
        mosi_word=0xDEADBEEF,
        tag="scenario_1_minimal_duplex_sanity",
    )

    # -------------------------------------------------
    # 2. Existing directed classification patterns
    # -------------------------------------------------
    directed_cases = [
        (0b00, 0, 0x00000000, "scenario_2_class_000"),
        (0b01, 0, 0x12345678, "scenario_2_class_001"),
        (0b10, 1, 0xA5A51234, "scenario_2_class_110"),
        (0b11, 1, 0xFFFFFFFF, "scenario_2_class_111"),
    ]

    for gesture, confidence, mosi_word, tag in directed_cases:
        await run_duplex_case(
            dut,
            gesture=gesture,
            confidence=confidence,
            mosi_word=mosi_word,
            tag=tag,
        )

    # -------------------------------------------------
    # 3. More MOSI bit-pattern coverage
    # -------------------------------------------------
    pattern_cases = [
        (0b00, 1, 0xAAAAAAAA, "scenario_3_mosi_alt_a"),
        (0b01, 1, 0x55555555, "scenario_3_mosi_alt_5"),
        (0b10, 0, 0x80000001, "scenario_3_mosi_edge_bits"),
        (0b11, 0, 0x7FFFFFFE, "scenario_3_mosi_inverse_edge_bits"),
        (0b00, 0, 0x0F0F0F0F, "scenario_3_mosi_nibbles_0f"),
        (0b11, 1, 0xF0F0F0F0, "scenario_3_mosi_nibbles_f0"),
    ]

    for gesture, confidence, mosi_word, tag in pattern_cases:
        await run_duplex_case(
            dut,
            gesture=gesture,
            confidence=confidence,
            mosi_word=mosi_word,
            tag=tag,
        )

    # -------------------------------------------------
    # 4. Classification hold behavior
    # If gesture_valid is not pulsed, MISO should keep last classification.
    # Previous case leaves classification at gesture=11, confidence=1 -> 0xE0000000.
    # -------------------------------------------------
    await run_duplex_case(
        dut,
        gesture=0b11,
        confidence=1,
        mosi_word=0xCAFEBABE,
        tag="scenario_4_hold_previous_classification",
        pulse_new_classification=False,
    )

    # -------------------------------------------------
    # 5. Rapid classification update before transaction
    # Only the latest valid classification should be shifted out.
    # -------------------------------------------------
    dut._log.info("scenario_5_rapid_classification_update: pulsing two classifications")
    await pulse_classification(dut, gesture=0b00, confidence=0)
    await ClockCycles(dut.clk, 2)
    await pulse_classification(dut, gesture=0b10, confidence=1)
    await ClockCycles(dut.clk, 10)

    rx_task = cocotb.start_soon(wait_for_evt_word(dut, expected_word=0x13579BDF))
    miso_word = await spi_mode0_transfer_word(dut, 0x13579BDF)
    observed_evt_word = await rx_task

    expected_miso = expected_miso_from_classification(0b10, 1)
    assert miso_word == expected_miso, (
        f"scenario_5_rapid_classification_update: MISO mismatch: "
        f"got 0x{miso_word:08X}, expected 0x{expected_miso:08X}"
    )
    dut._log.info(
        f"scenario_5_rapid_classification_update: PASS "
        f"evt_word=0x{observed_evt_word:08X}, MISO=0x{miso_word:08X}"
    )

    # -------------------------------------------------
    # 6. CS high SCLK activity should not create evt_valid
    # -------------------------------------------------
    dut._log.info("scenario_6_cs_high_ignored")
    await toggle_sclk_with_cs_high(dut, cycles=40)
    await assert_no_evt_valid(dut, cycles=20, tag="scenario_6_cs_high_ignored")

    # Follow with a clean word to confirm recovery.
    await run_duplex_case(
        dut,
        gesture=0b01,
        confidence=1,
        mosi_word=0x2468ACE0,
        tag="scenario_6_followup_clean_word",
    )

    # -------------------------------------------------
    # 7. Partial word then CS abort should not create evt_valid
    # -------------------------------------------------
    dut._log.info("scenario_7_partial_abort")
    await spi_partial_word_then_abort(dut, mosi_word=0xABCDEF01, nbits=13)
    await assert_no_evt_valid(dut, cycles=40, tag="scenario_7_partial_abort")

    # Follow with a clean word to confirm recovery.
    await run_duplex_case(
        dut,
        gesture=0b10,
        confidence=0,
        mosi_word=0x10203040,
        tag="scenario_7_followup_clean_word",
    )

    # -------------------------------------------------
    # 8. Minimal CS-high idle gap between transactions
    # -------------------------------------------------
    minimal_gap_cases = [
        (0b00, 1, 0x11111111, "scenario_8_min_gap_0"),
        (0b01, 0, 0x22222222, "scenario_8_min_gap_1"),
        (0b10, 1, 0x33333333, "scenario_8_min_gap_2"),
        (0b11, 0, 0x44444444, "scenario_8_min_gap_3"),
    ]

    for gesture, confidence, mosi_word, tag in minimal_gap_cases:
        await run_duplex_case(
            dut,
            gesture=gesture,
            confidence=confidence,
            mosi_word=mosi_word,
            tag=tag,
            cs_high_gap_cycles=1,
        )

    # -------------------------------------------------
    # 9. Slower SPI timing
    # -------------------------------------------------
    await run_duplex_case(
        dut,
        gesture=0b01,
        confidence=1,
        mosi_word=0x55AA55AA,
        tag="scenario_9_slow_spi",
        half_cycles=8,
    )

    # -------------------------------------------------
    # 10. Faster SPI timing still safely sampled by clk
    # half_cycles=2 means SCLK half-period is 2 clk cycles.
    # -------------------------------------------------
    await run_duplex_case(
        dut,
        gesture=0b10,
        confidence=1,
        mosi_word=0xAA55AA55,
        tag="scenario_10_faster_spi",
        half_cycles=2,
    )

    # -------------------------------------------------
    # 11. Repeated identical MOSI words should each create evt_valid
    # -------------------------------------------------
    repeated_word = 0x0BADF00D
    for idx in range(3):
        await run_duplex_case(
            dut,
            gesture=idx & 0b11,
            confidence=idx & 1,
            mosi_word=repeated_word,
            tag=f"scenario_11_repeated_word_{idx}",
        )

    # -------------------------------------------------
    # 12. Final all-ones classification and all-zero MOSI
    # -------------------------------------------------
    await run_duplex_case(
        dut,
        gesture=0b11,
        confidence=1,
        mosi_word=0x00000000,
        tag="scenario_12_final_all_ones_class_all_zero_mosi",
    )

    # -------------------------------------------------
    # 13. Streaming: multiple words in one CS-low transaction
    # -------------------------------------------------
    await run_streaming_case(
        dut,
        mosi_words=[
            0x01020304,
            0x11223344,
            0x55667788,
        ],
        gesture=0b01,
        confidence=1,
        tag="scenario_13_streaming_three_words",
    )

    # -------------------------------------------------
    # 14. Streaming: pattern-heavy burst in one CS-low transaction
    # -------------------------------------------------
    await run_streaming_case(
        dut,
        mosi_words=[
            0x00000000,
            0xFFFFFFFF,
            0xAAAAAAAA,
            0x55555555,
            0x80000001,
            0x7FFFFFFE,
        ],
        gesture=0b10,
        confidence=1,
        tag="scenario_14_streaming_pattern_burst",
    )

    # -------------------------------------------------
    # 15. Streaming: faster timing burst
    # -------------------------------------------------
    await run_streaming_case(
        dut,
        mosi_words=[
            0xABCDEF01,
            0x12345678,
            0xDEADBEEF,
            0x0BADF00D,
        ],
        gesture=0b11,
        confidence=0,
        tag="scenario_15_streaming_faster_spi",
        half_cycles=2,
        inter_word_low_cycles=4,
    )

    # -------------------------------------------------
    # 16. Streaming: minimal inter-word SCLK-low gap
    # This is the tightest streaming case in this TB.
    # If this fails, the wrapper/IP likely needs more idle-low time between words.
    # -------------------------------------------------
    await run_streaming_case(
        dut,
        mosi_words=[
            0x13572468,
            0x24681357,
            0xCAFEBABE,
        ],
        gesture=0b00,
        confidence=1,
        tag="scenario_16_streaming_min_inter_word_gap",
        half_cycles=4,
        inter_word_low_cycles=1,
    )

    dut._log.info("All sequential spi_wrapper duplex/control/streaming scenarios passed")