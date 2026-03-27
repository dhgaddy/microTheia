# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2025 Group G Contributors
import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG    = load_config(MODULE)

NUM_CLASSES = CFG["NUM_CLASSES"]
SCORE_BITS  = CFG["SCORE_BITS"]

SCORE_MASK = (1 << SCORE_BITS) - 1

# scores_valid → outputs latency: 4 cycles (capture → pair-reduce → merge → hold → compare)
CLASSIFY_PIPE_CYCLES = 4

# Must match weights/thresholds.mem (all zeros by default).
CLASS_THRESHOLDS = [0, 0, 0, 0]
DIFF_THRESHOLDS  = [0, 0, 0, 0]


def pack_scores(scores):
    packed = 0
    for i, s in enumerate(scores):
        packed |= (s & SCORE_MASK) << (i * SCORE_BITS)
    return packed


class GestureClassifierModel:
    """Cycle-accurate golden model for voxel_gesture_classifier (per-class threshold)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.class_gesture      = 0
        self.class_valid        = 0
        self.class_pass         = 0
        self.gesture            = 0
        self.gesture_valid      = 0
        self.gesture_confidence = 0

    def step(self, scores_flat, scores_valid):
        self.class_valid        = 0
        self.class_pass         = 0
        self.gesture_valid      = 0
        self.gesture_confidence = 0

        if not scores_valid:
            return

        scores = [(scores_flat >> (i * SCORE_BITS)) & SCORE_MASK
                  for i in range(NUM_CLASSES)]

        if scores[0] >= scores[1]:
            p0_max, p0_min, p0_cls = scores[0], scores[1], 0
        else:
            p0_max, p0_min, p0_cls = scores[1], scores[0], 1
        if scores[2] >= scores[3]:
            p1_max, p1_min, p1_cls = scores[2], scores[3], 2
        else:
            p1_max, p1_min, p1_cls = scores[3], scores[2], 3

        if p0_max >= p1_max:
            max_score, max_class = p0_max, p0_cls
            second_a, second_b   = p1_max, p0_min
        else:
            max_score, max_class = p1_max, p1_cls
            second_a, second_b   = p0_max, p1_min

        second_score = second_a if second_a >= second_b else second_b
        diff         = max_score - second_score

        class_thresh = CLASS_THRESHOLDS[max_class]
        diff_thresh  = DIFF_THRESHOLDS[max_class]

        self.class_gesture = max_class
        self.class_valid   = 1
        self.class_pass    = 1 if max_score > class_thresh else 0

        if self.class_pass:
            self.gesture            = max_class
            self.gesture_valid      = 1
            self.gesture_confidence = 1 if diff > diff_thresh else 0


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value              = 1
    dut.scores_flat.value      = 0
    dut.scores_valid.value     = 0
    dut.thresh_data.value      = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive_and_check(dut, model, scores, valid, tag):
    packed = pack_scores(scores)
    dut.scores_flat.value  = packed
    dut.scores_valid.value = valid

    model.step(packed, valid)

    await RisingEdge(dut.clk)
    dut.scores_valid.value = 0

    for stage in range(CLASSIFY_PIPE_CYCLES):
        await RisingEdge(dut.clk)
        if stage != CLASSIFY_PIPE_CYCLES - 1:
            continue
        await ReadOnly()

        assert int(dut.class_gesture.value) == model.class_gesture, \
            f"{tag}: class_gesture DUT={int(dut.class_gesture.value)} model={model.class_gesture}"
        assert int(dut.class_valid.value) == model.class_valid, \
            f"{tag}: class_valid DUT={int(dut.class_valid.value)} model={model.class_valid}"
        assert int(dut.class_pass.value) == model.class_pass, \
            f"{tag}: class_pass DUT={int(dut.class_pass.value)} model={model.class_pass}"
        assert int(dut.gesture.value) == model.gesture, \
            f"{tag}: gesture DUT={int(dut.gesture.value)} model={model.gesture}"
        assert int(dut.gesture_valid.value) == model.gesture_valid, \
            f"{tag}: gesture_valid DUT={int(dut.gesture_valid.value)} model={model.gesture_valid}"
        assert int(dut.gesture_confidence.value) == model.gesture_confidence, \
            f"{tag}: confidence DUT={int(dut.gesture_confidence.value)} model={model.gesture_confidence}"

    await NextTimeStep()


@logged_test()
async def test_reset_defaults(dut):
    """After reset all outputs are deasserted."""
    await setup(dut)
    assert int(dut.class_valid.value) == 0
    assert int(dut.class_pass.value) == 0
    assert int(dut.gesture_valid.value) == 0


@logged_test()
async def test_single_pass_immediate(dut):
    """CLASS_THRESHOLD=0: a single window with max_score>0 immediately fires gesture_valid."""
    await setup(dut)
    model = GestureClassifierModel()

    await drive_and_check(dut, model, [100, 1, 1, 1], 1, "single-pass")
    assert int(dut.class_valid.value) == 1
    assert int(dut.gesture_valid.value) == 1
    assert int(dut.gesture.value) == 0


@logged_test()
async def test_no_pass_zero(dut):
    """Scores ≤ CLASS_THRESHOLD (=0) fail; gesture_valid stays deasserted."""
    await setup(dut)
    model = GestureClassifierModel()

    # max_score = 0, threshold = 0 — strict > means this fails.
    await drive_and_check(dut, model, [0, 0, 0, 0], 1, "zero-max")
    assert int(dut.class_pass.value) == 0
    assert int(dut.gesture_valid.value) == 0

    # All at minimum (0).
    await drive_and_check(dut, model, [0, 0, 0, 0], 1, "all-zero")
    assert int(dut.class_pass.value) == 0
    assert int(dut.gesture_valid.value) == 0


@logged_test()
async def test_confidence_bit(dut):
    """DIFF_THRESHOLD=0: confidence=1 when diff>0, confidence=0 when diff=0."""
    await setup(dut)
    model = GestureClassifierModel()

    # All equal scores: diff=0, not >0 → confidence=0; max_score=100>0 → pass.
    await drive_and_check(dut, model, [100, 100, 100, 100], 1, "equal")
    assert int(dut.class_pass.value) == 1
    assert int(dut.gesture_confidence.value) == 0

    # Clear winner: diff > 0 → confidence=1.
    await drive_and_check(dut, model, [500, 10, 10, 10], 1, "clear")
    assert int(dut.gesture_confidence.value) == 1


@logged_test()
async def test_argmax_all_classes(dut):
    """Each of the four classes can be the winning class."""
    await setup(dut)
    model = GestureClassifierModel()

    for cls in range(NUM_CLASSES):
        scores = [1] * NUM_CLASSES
        scores[cls] = 500
        await drive_and_check(dut, model, scores, 1, f"cls{cls}")
        assert int(dut.gesture_valid.value) == 1
        assert int(dut.gesture.value) == cls, \
            f"Expected gesture class {cls}, got {int(dut.gesture.value)}"


@logged_test()
async def test_invalid_cycles_no_output(dut):
    """scores_valid=0 produces class_valid=0 (pipeline idles)."""
    await setup(dut)
    model = GestureClassifierModel()

    for i in range(5):
        await drive_and_check(dut, model, [500, 0, 0, 0], 0, f"invalid-{i}")
        assert int(dut.class_valid.value) == 0
        assert int(dut.gesture_valid.value) == 0


@logged_test()
async def test_consecutive_passes_each_valid(dut):
    """No persistence: every passing window fires gesture_valid independently."""
    await setup(dut)
    model = GestureClassifierModel()

    for i in range(5):
        await drive_and_check(dut, model, [300, 1, 1, 1], 1, f"pass-{i}")
        assert int(dut.gesture_valid.value) == 1, f"Expected gesture_valid at pass {i}"


@logged_test()
async def test_class_changes_fire_each_time(dut):
    """No streak needed: alternating classes each independently fire gesture_valid."""
    await setup(dut)
    model = GestureClassifierModel()

    for cycle in range(8):
        cls = cycle % NUM_CLASSES
        scores = [1] * NUM_CLASSES
        scores[cls] = 200
        await drive_and_check(dut, model, scores, 1, f"alt-{cycle}")
        assert int(dut.gesture_valid.value) == 1
        assert int(dut.gesture.value) == cls, \
            f"cycle {cycle}: expected cls {cls}, got {int(dut.gesture.value)}"


@logged_test()
async def test_tie_break(dut):
    """Tied scores: RTL pair-reduce favors lower class index."""
    await setup(dut)
    model = GestureClassifierModel()

    # All equal: s0==s1 → pair0 winner=cls0; s2==s3 → pair1 winner=cls2.
    # pair0_max==pair1_max → merge winner=cls0.
    await drive_and_check(dut, model, [100, 100, 100, 100], 1, "full-tie")
    assert int(dut.class_gesture.value) == 0


@logged_test()
async def test_large_score_no_overflow(dut):
    """Near-maximum unsigned values must not overflow the diff computation."""
    await setup(dut)
    model = GestureClassifierModel()

    huge = (1 << 28) - 1
    await drive_and_check(dut, model, [huge, 1, 1, 1], 1, "huge")
    assert int(dut.gesture_valid.value) == 1
    assert int(dut.gesture_confidence.value) == 1


@logged_test()
async def test_fail_then_immediate_pass(dut):
    """After a failing window, the very next passing window fires gesture_valid."""
    await setup(dut)
    model = GestureClassifierModel()

    # Fail (max_score = 0).
    await drive_and_check(dut, model, [0, 0, 0, 0], 1, "fail")
    assert int(dut.gesture_valid.value) == 0

    # Pass immediately after — no streak required.
    await drive_and_check(dut, model, [100, 1, 1, 1], 1, "pass-after-fail")
    assert int(dut.gesture_valid.value) == 1


@logged_test()
async def test_randomized_golden_scoreboard(dut):
    """Random inputs: DUT must match golden model on all outputs for 500 cycles."""
    await setup(dut)
    model = GestureClassifierModel()
    rng   = random.Random(0xBADC0DE)

    for cycle in range(500):
        valid  = rng.choice([0, 1, 1, 1])
        scores = [rng.randint(0, (1 << 20) - 1) for _ in range(NUM_CLASSES)]
        await drive_and_check(dut, model, scores, valid, f"rnd-{cycle}")
