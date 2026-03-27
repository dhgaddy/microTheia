# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2025 Group G Contributors
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.test_logging import logged_test
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

FEATURE_COUNT = CFG.get("FEATURE_COUNT", 2048)
COUNTER_BITS  = CFG.get("COUNTER_BITS",  16)
WEIGHT_BITS   = CFG.get("WEIGHT_BITS",   8)
NUM_CLASSES   = CFG.get("NUM_CLASSES",   4)

COUNTER_MASK = (1 << COUNTER_BITS) - 1
WEIGHT_MASK  = (1 << WEIGHT_BITS)  - 1

SCORE_BITS = None
SCORE_MASK = None

EXPECTED_LATENCY = FEATURE_COUNT + 2


def configure_from_dut(dut):
    global SCORE_BITS, SCORE_MASK, EXPECTED_LATENCY
    SCORE_BITS = len(dut.scores_flat) // NUM_CLASSES
    SCORE_MASK = (1 << SCORE_BITS) - 1
    EXPECTED_LATENCY = FEATURE_COUNT + 2
    dut._log.info(
        f"FEATURE_COUNT={FEATURE_COUNT} COUNTER_BITS={COUNTER_BITS} "
        f"WEIGHT_BITS={WEIGHT_BITS} NUM_CLASSES={NUM_CLASSES} SCORE_BITS={SCORE_BITS}"
    )


def golden_gemv(features, weights):
    return [
        sum(int(features[i]) * int(weights[g][i]) for i in range(FEATURE_COUNT)) & SCORE_MASK
        for g in range(NUM_CLASSES)
    ]


def unpack_scores(val):
    return [(val >> (g * SCORE_BITS)) & SCORE_MASK for g in range(NUM_CLASSES)]


def pack_weight_slice(w_at_addr):
    packed = 0
    for g in range(NUM_CLASSES):
        packed |= (int(w_at_addr[g]) & WEIGHT_MASK) << (g * WEIGHT_BITS)
    return packed


class RamModel:
    """Models caller-side feature and weight RAMs with 1-cycle synchronous read latency."""

    def __init__(self, dut, features, weights):
        self.dut      = dut
        self.features = features
        self.weights  = weights
        self._task    = None

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        if self._task is not None:
            self._task.cancel()
            self._task = None

    async def _run(self):
        self.dut.feature_data.value     = 0
        self.dut.weight_data_flat.value = 0

        # Pipeline register: data captured this cycle is driven NEXT cycle.
        # This matches gf180_sram_1r1w behaviour — rd_valid_i/rd_addr_i are
        # registered at posedge, data appears one cycle later.  ReadOnly()
        # sees post-NBA state (rd_en goes high in the *same* timestep as
        # the start pulse), so without this pipeline the model is 1 cycle
        # too fast vs real synchronous SRAM.
        pipe_feat  = 0
        pipe_wflat = 0

        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            # Sample this cycle's read request (delivered next cycle).
            if int(self.dut.rd_en.value):
                addr = int(self.dut.rd_addr.value)
                new_feat = int(self.features[addr])
                new_wflat = pack_weight_slice(
                    [self.weights[g][addr] for g in range(NUM_CLASSES)]
                )
            else:
                new_feat  = 0
                new_wflat = 0

            # Drive data captured in the PREVIOUS cycle.
            await NextTimeStep()
            self.dut.feature_data.value     = pipe_feat
            self.dut.weight_data_flat.value = pipe_wflat

            pipe_feat  = new_feat
            pipe_wflat = new_wflat


async def setup(dut):
    configure_from_dut(dut)
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value           = 1
    dut.start.value         = 0
    dut.feature_data.value  = 0
    dut.weight_data_flat.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def run_gemv(dut, features, weights, tag):
    ram = RamModel(dut, features, weights)
    ram.start()

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    await ReadOnly()
    assert int(dut.busy.value) == 1, \
        f"{tag}: busy must be 1 after the start edge"

    cycles_to_valid = 0
    saw_valid = False

    for _ in range(EXPECTED_LATENCY + 5):
        await RisingEdge(dut.clk)
        cycles_to_valid += 1
        await ReadOnly()

        if int(dut.scores_valid.value):
            saw_valid = True
            assert int(dut.busy.value) == 0, \
                f"{tag}: busy must be 0 on the scores_valid cycle"
            break
        else:
            assert int(dut.busy.value) == 1, \
                f"{tag}: busy dropped prematurely at cycle {cycles_to_valid}"

    ram.stop()
    assert saw_valid, \
        f"{tag}: timed out after {cycles_to_valid} cycles without seeing scores_valid"

    got = unpack_scores(int(dut.scores_flat.value))
    exp = golden_gemv(features, weights)
    assert got == exp, (
        f"{tag}: score mismatch\n"
        f"  got={got}\n"
        f"  exp={exp}"
    )

    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.scores_valid.value) == 0, \
        f"{tag}: scores_valid must drop after one cycle"

    await NextTimeStep()

    return cycles_to_valid


@logged_test()
async def test_reset_defaults(dut):
    """After reset busy=0, scores_valid=0, no spurious outputs."""
    await setup(dut)
    await ReadOnly()
    assert int(dut.busy.value)        == 0, "busy should be 0 after reset"
    assert int(dut.scores_valid.value) == 0, "scores_valid should be 0 after reset"


@logged_test()
async def test_all_zeros(dut):
    """All-zero features and weights produce zero scores for every class."""
    await setup(dut)
    f = [0] * FEATURE_COUNT
    w = [[0] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    await run_gemv(dut, f, w, "all-zeros")
    got = unpack_scores(int(dut.scores_flat.value))
    for g in range(NUM_CLASSES):
        assert got[g] == 0, f"class {g}: expected 0, got {got[g]}"


@logged_test()
async def test_single_nonzero_feature(dut):
    """F[k]=MAX with W[g][k]=g+1 (all else 0): S[g] = MAX*(g+1) & SCORE_MASK."""
    await setup(dut)
    k = FEATURE_COUNT // 2
    v = COUNTER_MASK
    f = [0] * FEATURE_COUNT
    f[k] = v
    w = [[0] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    for g in range(NUM_CLASSES):
        w[g][k] = g + 1
    await run_gemv(dut, f, w, "single-nonzero")
    got = unpack_scores(int(dut.scores_flat.value))
    for g in range(NUM_CLASSES):
        exp_val = (v * (g + 1)) & SCORE_MASK
        assert got[g] == exp_val, \
            f"class {g}: expected {exp_val}, got {got[g]}"


@logged_test()
async def test_class_independence(dut):
    """Non-zero weights in class 0 only: S[0] nonzero, S[1..NUM_CLASSES-1] = 0."""
    await setup(dut)
    rng = random.Random(0xC1A55)
    f = [rng.randint(1, COUNTER_MASK) for _ in range(FEATURE_COUNT)]
    w = [[0] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    for i in range(FEATURE_COUNT):
        w[0][i] = rng.randint(1, WEIGHT_MASK)
    await run_gemv(dut, f, w, "class-indep")
    got = unpack_scores(int(dut.scores_flat.value))
    exp = golden_gemv(f, w)
    assert got[0] == exp[0], f"class 0 mismatch: got {got[0]}, exp {exp[0]}"
    for g in range(1, NUM_CLASSES):
        assert got[g] == 0, f"class {g} should be 0, got {got[g]}"


@logged_test()
async def test_unit_weights(dut):
    """W[g][g % FC] = 1 (all else 0): S[g] = F[g % FC]."""
    await setup(dut)
    rng = random.Random(0x1D3E7A)
    f = [rng.randint(0, COUNTER_MASK) for _ in range(FEATURE_COUNT)]
    w = [[0] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    for g in range(NUM_CLASSES):
        w[g][g % FEATURE_COUNT] = 1
    await run_gemv(dut, f, w, "unit-weights")
    got = unpack_scores(int(dut.scores_flat.value))
    for g in range(NUM_CLASSES):
        exp_val = f[g % FEATURE_COUNT] & SCORE_MASK
        assert got[g] == exp_val, \
            f"class {g}: expected {exp_val}, got {got[g]}"


@logged_test()
async def test_all_max_values(dut):
    """Max feature and weight values: verify no overflow truncation vs golden."""
    await setup(dut)
    f = [COUNTER_MASK] * FEATURE_COUNT
    w = [[WEIGHT_MASK] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    await run_gemv(dut, f, w, "max-max")
    got = unpack_scores(int(dut.scores_flat.value))
    exp = golden_gemv(f, w)
    assert got == exp, f"max*max mismatch\n  got={got}\n  exp={exp}"
    for g in range(NUM_CLASSES):
        assert got[g] > 0, f"class {g}: expected nonzero for all-max inputs"


@logged_test()
async def test_accumulator_clears_between_runs(dut):
    """Run with all-max then all-zero; second result must be exactly zero."""
    await setup(dut)
    f_large = [COUNTER_MASK] * FEATURE_COUNT
    w_large = [[WEIGHT_MASK] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    await run_gemv(dut, f_large, w_large, "fill-accum")

    f_zero = [0] * FEATURE_COUNT
    w_zero = [[0] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    await run_gemv(dut, f_zero, w_zero, "zero-after-fill")

    got = unpack_scores(int(dut.scores_flat.value))
    for g in range(NUM_CLASSES):
        assert got[g] == 0, \
            f"class {g}: accumulator bleed detected ({got[g]} != 0)"


@logged_test()
async def test_start_ignored_while_busy(dut):
    """A start pulse issued while busy must be ignored; first run completes correctly."""
    await setup(dut)
    rng = random.Random(0xB00B5)
    f = [rng.randint(0, COUNTER_MASK) for _ in range(FEATURE_COUNT)]
    w = [[rng.randint(0, WEIGHT_MASK) for _ in range(FEATURE_COUNT)]
         for _ in range(NUM_CLASSES)]

    ram = RamModel(dut, f, w)
    ram.start()

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    await ClockCycles(dut.clk, max(3, FEATURE_COUNT // 4))
    await ReadOnly()
    assert int(dut.busy.value) == 1, "engine should still be busy mid-run"
    await NextTimeStep()
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    saw_valid = False
    for _ in range(EXPECTED_LATENCY + 10):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.scores_valid.value):
            saw_valid = True
            break

    ram.stop()
    assert saw_valid, "run did not complete after spurious start pulse"

    got = unpack_scores(int(dut.scores_flat.value))
    exp = golden_gemv(f, w)
    assert got == exp, (
        "spurious start corrupted result\n"
        f"  got={got}\n  exp={exp}"
    )


@logged_test()
async def test_back_to_back_runs(dut):
    """Start a new run immediately after the previous one completes (no idle gap)."""
    await setup(dut)
    rng = random.Random(0xB2B2B2B2)

    for trial in range(5):
        f = [rng.randint(0, COUNTER_MASK) for _ in range(FEATURE_COUNT)]
        w = [[rng.randint(0, WEIGHT_MASK) for _ in range(FEATURE_COUNT)]
             for _ in range(NUM_CLASSES)]
        cyc = await run_gemv(dut, f, w, f"b2b-{trial}")
        assert cyc == EXPECTED_LATENCY, \
            f"trial {trial}: latency {cyc} != expected {EXPECTED_LATENCY}"


@logged_test()
async def test_scores_valid_exact_timing(dut):
    """scores_valid must fire at exactly FEATURE_COUNT + 2 clock edges after start."""
    await setup(dut)
    f = [1] * FEATURE_COUNT
    w = [[1] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    ram = RamModel(dut, f, w)
    ram.start()

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    cycle_count = 0
    saw_valid = False
    for _ in range(EXPECTED_LATENCY + 5):
        await RisingEdge(dut.clk)
        cycle_count += 1
        await ReadOnly()
        if int(dut.scores_valid.value):
            saw_valid = True
            break

    ram.stop()
    assert saw_valid, f"scores_valid never fired (checked {cycle_count} cycles)"
    assert cycle_count == EXPECTED_LATENCY, (
        f"scores_valid at cycle {cycle_count}, expected {EXPECTED_LATENCY} "
        f"(FEATURE_COUNT={FEATURE_COUNT})"
    )


@logged_test()
async def test_randomized_golden(dut):
    """10 random trials: scores must match the Python golden model exactly."""
    await setup(dut)
    rng = random.Random(0x5A57A11C)
    for trial in range(10):
        f = [rng.randint(0, COUNTER_MASK) for _ in range(FEATURE_COUNT)]
        w = [[rng.randint(0, WEIGHT_MASK) for _ in range(FEATURE_COUNT)]
             for _ in range(NUM_CLASSES)]
        await run_gemv(dut, f, w, f"rnd-{trial}")


@logged_test()
async def test_stress_high_range(dut):
    """15 trials with values near the max of each field; stresses the accumulator."""
    await setup(dut)
    rng = random.Random(0xFACEFEED)
    for trial in range(15):
        f = [rng.randint(COUNTER_MASK // 2, COUNTER_MASK) for _ in range(FEATURE_COUNT)]
        w = [[rng.randint(WEIGHT_MASK  // 2, WEIGHT_MASK)  for _ in range(FEATURE_COUNT)]
             for _ in range(NUM_CLASSES)]
        await run_gemv(dut, f, w, f"stress-{trial}")


@logged_test()
async def test_all_ones(dut):
    """F=1, W=1: every score must equal FEATURE_COUNT (masked to SCORE_BITS)."""
    await setup(dut)
    f = [1] * FEATURE_COUNT
    w = [[1] * FEATURE_COUNT for _ in range(NUM_CLASSES)]
    await run_gemv(dut, f, w, "all-ones")
    got = unpack_scores(int(dut.scores_flat.value))
    exp_val = FEATURE_COUNT & SCORE_MASK
    for g in range(NUM_CLASSES):
        assert got[g] == exp_val, \
            f"class {g}: expected {exp_val}, got {got[g]}"


@logged_test()
async def test_asymmetric_classes(dut):
    """Each class gets a different constant weight; scores must scale proportionally."""
    await setup(dut)
    rng = random.Random(0xA5CC77)
    f = [rng.randint(1, COUNTER_MASK) for _ in range(FEATURE_COUNT)]
    w = [
        [g + 1] * FEATURE_COUNT
        for g in range(NUM_CLASSES)
    ]
    await run_gemv(dut, f, w, "asymmetric")
    got = unpack_scores(int(dut.scores_flat.value))
    exp = golden_gemv(f, w)
    assert got == exp, f"asymmetric mismatch\n  got={got}\n  exp={exp}"
    for g in range(1, NUM_CLASSES):
        assert got[g] >= got[g - 1], \
            f"class {g} score {got[g]} not >= class {g-1} score {got[g-1]}"
