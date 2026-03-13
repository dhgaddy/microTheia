"""Robust cocotb testbench for gesture_classifier using a cycle-accurate golden model."""

import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

NUM_CLASSES       = CFG["NUM_CLASSES"]
SCORE_BITS        = CFG["SCORE_BITS"]
PASS_MARGIN       = CFG["PASS_MARGIN"]
PERSISTENCE_COUNT = CFG["PERSISTENCE_COUNT"]
CONF_BITS         = CFG["CONF_BITS"]
CONF_SHIFT        = CFG["CONF_SHIFT"]

CONF_MAX   = (1 << CONF_BITS) - 1
SCORE_MASK = (1 << SCORE_BITS) - 1
# scores_valid -> class_valid latency through the pipelined classifier.
CLASSIFY_PIPE_CYCLES = 3


def to_signed(val, bits):
    val &= (1 << bits) - 1
    if val & (1 << (bits - 1)):
        return val - (1 << bits)
    return val


def from_signed(val, bits):
    return val & ((1 << bits) - 1)


def pack_scores(scores):
    packed = 0
    for i, s in enumerate(scores):
        packed |= from_signed(s, SCORE_BITS) << (i * SCORE_BITS)
    return packed


class GestureClassifierModel:
    """Cycle-accurate golden model for rtl/gesture_classifier.sv defaults."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.class_gesture = 0
        self.class_valid = 0
        self.class_pass = 0
        self.gesture = 0
        self.gesture_valid = 0
        self.gesture_confidence = 0
        self.last_pass_class = 0
        self.pass_streak = 0
        self.debug_state = 0

    def step(self, scores_flat, scores_valid):
        self.class_valid = 0
        self.class_pass = 0
        self.gesture_valid = 0
        self.debug_state = 0

        if not scores_valid:
            return

        scores = [to_signed((scores_flat >> (i * SCORE_BITS)) & SCORE_MASK, SCORE_BITS)
                  for i in range(NUM_CLASSES)]

        max_score = scores[0]
        second_score = -(1 << (SCORE_BITS - 1))
        max_class = 0

        for i in range(1, NUM_CLASSES):
            if scores[i] > max_score:
                second_score = max_score
                max_score = scores[i]
                max_class = i
            elif scores[i] > second_score:
                second_score = scores[i]

        margin = max_score - second_score
        passed = int(margin > PASS_MARGIN)

        self.class_gesture = max_class
        self.class_valid = 1
        self.class_pass = passed

        if passed:
            if max_class == self.last_pass_class:
                if self.pass_streak < PERSISTENCE_COUNT:
                    next_streak = self.pass_streak + 1
                else:
                    next_streak = self.pass_streak
            else:
                next_streak = 1

            self.last_pass_class = max_class
            self.pass_streak = next_streak

            if next_streak >= PERSISTENCE_COUNT:
                self.gesture = max_class
                self.gesture_valid = 1
                self.debug_state = 2
            else:
                self.debug_state = 1

            conf_shifted = margin >> CONF_SHIFT
            if conf_shifted > CONF_MAX:
                self.gesture_confidence = CONF_MAX
            elif margin <= 0:
                self.gesture_confidence = 0
            else:
                self.gesture_confidence = conf_shifted & CONF_MAX
        else:
            self.pass_streak = 0
            self.debug_state = 0


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.scores_flat.value = 0
    dut.scores_valid.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive_and_check(dut, model, scores, valid, tag):
    packed = pack_scores(scores)
    dut.scores_flat.value = packed
    dut.scores_valid.value = valid

    model.step(packed, valid)

    # Launch input into stage-0.
    await RisingEdge(dut.clk)
    dut.scores_valid.value = 0

    # Wait for the pipelined decision point corresponding to this input.
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
        assert int(dut.debug_state.value) == model.debug_state, \
            f"{tag}: debug_state DUT={int(dut.debug_state.value)} model={model.debug_state}"

    await NextTimeStep()


@logged_test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.class_valid.value) == 0
    assert int(dut.class_pass.value) == 0
    assert int(dut.gesture_valid.value) == 0
    assert int(dut.debug_state.value) == 0


@logged_test()
async def test_tie_break_and_threshold(dut):
    await setup(dut)
    model = GestureClassifierModel()

    # All tied: class 0 should win tie-break.
    await drive_and_check(dut, model, [10, 10, 10, 10], 1, "tie")
    assert int(dut.class_gesture.value) == 0

    # Margin exactly PASS_MARGIN should fail (strict >).
    await drive_and_check(dut, model, [100, 36, 36, 36], 1, "margin-eq")
    assert int(dut.class_pass.value) == 0

    # Margin PASS_MARGIN+1 should pass.
    await drive_and_check(dut, model, [101, 36, 36, 36], 1, "margin-gt")
    assert int(dut.class_pass.value) == 1


@logged_test()
async def test_persistence_and_class_change(dut):
    await setup(dut)
    model = GestureClassifierModel()

    # Two consecutive passes on same class needed to assert gesture_valid.
    await drive_and_check(dut, model, [300, 0, -10, -20], 1, "persist-1")
    assert int(dut.gesture_valid.value) == 0
    await drive_and_check(dut, model, [350, 0, -10, -20], 1, "persist-2")
    assert int(dut.gesture_valid.value) == 1

    # Class change should restart streak.
    await drive_and_check(dut, model, [0, 400, -10, -20], 1, "change-1")
    assert int(dut.gesture_valid.value) == 0
    await drive_and_check(dut, model, [0, 420, -10, -20], 1, "change-2")
    assert int(dut.gesture_valid.value) == 1


@logged_test()
async def test_confidence_edges(dut):
    await setup(dut)
    model = GestureClassifierModel()

    # Build streak first.
    await drive_and_check(dut, model, [300, 0, 0, 0], 1, "warm-1")
    await drive_and_check(dut, model, [301, 0, 0, 0], 1, "warm-2")

    # Saturation case.
    huge = (1 << 20)
    await drive_and_check(dut, model, [huge, -huge, -huge, -huge], 1, "conf-sat")
    assert int(dut.gesture_confidence.value) == CONF_MAX

    # Not-valid cycle keeps confidence stable (model enforces hold behavior).
    prev = int(dut.gesture_confidence.value)
    await drive_and_check(dut, model, [0, 0, 0, 0], 0, "conf-hold")
    assert int(dut.gesture_confidence.value) == prev


@logged_test()
async def test_randomized_golden_scoreboard(dut):
    await setup(dut)
    model = GestureClassifierModel()
    rng = random.Random(0xBADC0DE)

    for cycle in range(1500):
        valid = rng.choice([0, 1, 1, 1])
        scores = [rng.randint(-(1 << 20), (1 << 20) - 1) for _ in range(NUM_CLASSES)]
        await drive_and_check(dut, model, scores, valid, f"rnd-{cycle}")


@logged_test()
async def test_all_negative_scores(dut):
    """All-negative scores: argmax finds the least-negative class correctly."""
    await setup(dut)
    model = GestureClassifierModel()

    # class 2 is least negative -> max; margin = (-50) - (-150) = 100 > PASS_MARGIN.
    scores = [-200, -150, -50, -180]
    for i in range(PERSISTENCE_COUNT + 1):
        await drive_and_check(dut, model, scores, 1, f"neg-{i}")

    assert int(dut.class_gesture.value) == 2, \
        f"Expected class 2 (least negative), got {int(dut.class_gesture.value)}"


@logged_test()
async def test_rapid_class_switch_never_fires_gesture(dut):
    """Alternating classes each cycle must never accumulate a streak."""
    await setup(dut)
    model = GestureClassifierModel()

    for cycle in range(20):
        # Alternate between class 0 and class 1 each cycle.
        scores = [500, 0, 0, 0] if (cycle % 2 == 0) else [0, 500, 0, 0]
        await drive_and_check(dut, model, scores, 1, f"alt-{cycle}")
        assert int(dut.gesture_valid.value) == 0, \
            f"gesture_valid unexpectedly asserted at cycle {cycle}"


@logged_test()
async def test_invalid_cycles_do_not_break_streak(dut):
    """scores_valid=0 cycles between two passing windows must not reset the streak."""
    await setup(dut)
    model = GestureClassifierModel()

    # Build streak=1 on class 3.
    await drive_and_check(dut, model, [0, 0, 0, 500], 1, "streak-1")
    assert int(dut.gesture_valid.value) == 0

    # Insert invalid (scores_valid=0) cycles.
    for _ in range(10):
        await drive_and_check(dut, model, [0, 0, 0, 0], 0, "invalid")

    # Next passing window on the same class completes the streak.
    await drive_and_check(dut, model, [0, 0, 0, 500], 1, "streak-2")
    assert int(dut.gesture_valid.value) == 1, \
        "Streak should have reached PERSISTENCE_COUNT"
    assert int(dut.gesture.value) == 3, "Expected gesture class 3"


@logged_test()
async def test_large_score_no_overflow(dut):
    """Near-maximum score values must not overflow the confidence computation."""
    await setup(dut)
    model = GestureClassifierModel()

    # Scores sized to fit in SCORE_BITS=32 signed; margin >> CONF_SHIFT saturates at CONF_MAX.
    huge = (1 << 28) - 1
    scores = [huge, 0, 0, 0]
    for _ in range(PERSISTENCE_COUNT):
        await drive_and_check(dut, model, scores, 1, "large")

    assert int(dut.gesture_valid.value) == 1
    assert int(dut.gesture_confidence.value) == CONF_MAX, \
        f"Confidence should saturate at {CONF_MAX}, got {int(dut.gesture_confidence.value)}"


@logged_test()
async def test_fail_then_pass_resets_streak(dut):
    """A non-passing window resets the streak; subsequent passes start fresh."""
    await setup(dut)
    model = GestureClassifierModel()

    # First passing window (streak=1).
    await drive_and_check(dut, model, [300, 0, 0, 0], 1, "pass-1")
    assert int(dut.gesture_valid.value) == 0

    # Failing window (margin <= PASS_MARGIN) — streak drops to 0.
    await drive_and_check(dut, model, [100, 64, 64, 64], 1, "fail")
    assert int(dut.class_pass.value) == 0

    # Two more passing windows needed to reach gesture_valid.
    await drive_and_check(dut, model, [300, 0, 0, 0], 1, "pass-2")
    assert int(dut.gesture_valid.value) == 0

    await drive_and_check(dut, model, [300, 0, 0, 0], 1, "pass-3")
    assert int(dut.gesture_valid.value) == 1


@logged_test()
async def test_each_class_can_win(dut):
    """Exercise each of the four classes as the winner; golden model tracks all."""
    await setup(dut)
    model = GestureClassifierModel()

    for cls in range(NUM_CLASSES):
        # Build a score vector where only class `cls` is high.
        scores = [-100] * NUM_CLASSES
        scores[cls] = 500
        for _ in range(PERSISTENCE_COUNT + 1):
            await drive_and_check(dut, model, scores, 1, f"cls{cls}-run")

        assert int(dut.gesture.value) == cls, \
            f"Expected gesture class {cls}, got {int(dut.gesture.value)}"
        assert int(dut.gesture_valid.value) == 1


@logged_test()
async def test_debug_state_transitions(dut):
    """Explicit check of all three debug_state values: 0=fail, 1=accumulating, 2=gesture valid."""
    await setup(dut)
    model = GestureClassifierModel()

    # debug_state=0: margin <= PASS_MARGIN (fails threshold).
    await drive_and_check(dut, model, [100, 64, 64, 64], 1, "fail")
    assert int(dut.debug_state.value) == 0, \
        f"Expected debug_state=0 on fail, got {int(dut.debug_state.value)}"
    assert int(dut.gesture_valid.value) == 0

    # debug_state=1: first passing window — streak=1 < PERSISTENCE_COUNT=2.
    await drive_and_check(dut, model, [300, 0, 0, 0], 1, "pass-1")
    assert int(dut.debug_state.value) == 1, \
        f"Expected debug_state=1 on first pass, got {int(dut.debug_state.value)}"
    assert int(dut.gesture_valid.value) == 0

    # debug_state=2: second consecutive passing window — streak reaches PERSISTENCE_COUNT.
    await drive_and_check(dut, model, [300, 0, 0, 0], 1, "pass-2")
    assert int(dut.debug_state.value) == 2, \
        f"Expected debug_state=2 on gesture valid, got {int(dut.debug_state.value)}"
    assert int(dut.gesture_valid.value) == 1
    assert int(dut.gesture.value) == 0

    # Fail resets streak -> debug_state=0, gesture_valid deasserts.
    await drive_and_check(dut, model, [100, 64, 64, 64], 1, "fail-2")
    assert int(dut.debug_state.value) == 0
    assert int(dut.gesture_valid.value) == 0

    # Class change on first pass: streak restarts at 1 -> debug_state=1.
    await drive_and_check(dut, model, [0, 400, 0, 0], 1, "class-change")
    assert int(dut.debug_state.value) == 1, \
        f"Expected debug_state=1 after class change pass, got {int(dut.debug_state.value)}"
    assert int(dut.class_gesture.value) == 1
    assert int(dut.gesture_valid.value) == 0
