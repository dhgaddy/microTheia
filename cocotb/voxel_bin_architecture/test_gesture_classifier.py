"""Unit testbench for gesture_classifier with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly, NextTimeStep
import random

ACC_SUM_BITS = 18
PERSISTENCE_COUNT = 2


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class VBGestureClassifierModel:
    """Golden model for the persistence-filter gesture classifier."""

    ST_IDLE, ST_TRACKING, ST_CONFIRMED = 0, 1, 2

    def __init__(self, persistence_count=PERSISTENCE_COUNT):
        self.persistence_count = persistence_count
        self.reset()

    def reset(self):
        self.state = self.ST_IDLE
        self.last_gesture = 0
        self.match_count = 0
        self.gesture = 0
        self.gesture_valid = 0
        self.gesture_confidence = 0

    def step(self, class_gesture, class_valid, class_pass, abs_delta_x, abs_delta_y):
        """One clock cycle. Returns (gesture, gesture_valid, gesture_confidence)."""
        self.gesture_valid = 0

        if class_valid:
            if class_pass:
                if class_gesture == self.last_gesture:
                    prev_match_count = self.match_count
                    if self.match_count < self.persistence_count:
                        self.match_count += 1

                    # Match RTL nonblocking semantics: confirmation checks the
                    # pre-increment match_count value from this cycle.
                    if prev_match_count >= self.persistence_count - 1:
                        self.state = self.ST_CONFIRMED
                        self.gesture = class_gesture
                        self.gesture_valid = 1
                        self.match_count = 0

                        if abs_delta_x > abs_delta_y:
                            self.gesture_confidence = 15 if abs_delta_x > 255 \
                                else (abs_delta_x >> 4) & 0xF
                        else:
                            self.gesture_confidence = 15 if abs_delta_y > 255 \
                                else (abs_delta_y >> 4) & 0xF
                    else:
                        self.state = self.ST_TRACKING
                else:
                    self.last_gesture = class_gesture
                    self.match_count = 0
                    self.state = self.ST_TRACKING
            else:
                self.match_count = 0
                self.state = self.ST_IDLE

        return self.gesture, self.gesture_valid, self.gesture_confidence


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.class_gesture.value = 0
    dut.class_valid.value = 0
    dut.class_pass.value = 0
    dut.abs_delta_x.value = 0
    dut.abs_delta_y.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def classify(dut, gesture, pass_flag, dx=100, dy=0):
    dut.class_gesture.value = gesture
    dut.class_valid.value = 1
    dut.class_pass.value = pass_flag
    dut.abs_delta_x.value = dx
    dut.abs_delta_y.value = dy
    await RisingEdge(dut.clk)
    dut.class_valid.value = 0
    dut.class_pass.value = 0


async def step_and_check(dut, model, gesture, class_valid, pass_flag, dx, dy, tag):
    """Drive one cycle and compare DUT outputs against golden model."""
    dut.class_gesture.value = gesture
    dut.class_valid.value = class_valid
    dut.class_pass.value = pass_flag
    dut.abs_delta_x.value = dx
    dut.abs_delta_y.value = dy
    mg, mv, mc = model.step(gesture, class_valid, pass_flag, dx, dy)
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.gesture.value) == mg, \
        f"{tag}: gesture DUT={int(dut.gesture.value)}, model={mg}"
    assert int(dut.gesture_valid.value) == mv, \
        f"{tag}: gesture_valid DUT={int(dut.gesture_valid.value)}, model={mv}"
    assert int(dut.gesture_confidence.value) == mc, \
        f"{tag}: confidence DUT={int(dut.gesture_confidence.value)}, model={mc}"
    assert int(dut.debug_state.value) == model.state, \
        f"{tag}: debug_state DUT={int(dut.debug_state.value)}, model={model.state}"
    await NextTimeStep()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """All outputs zero after reset."""
    await setup(dut)
    assert int(dut.gesture_valid.value) == 0
    assert int(dut.gesture.value) == 0
    assert int(dut.gesture_confidence.value) == 0


@cocotb.test()
async def test_single_pass_not_enough(dut):
    """A single passing classification should NOT trigger gesture_valid."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)
    await step_and_check(dut, model, 0, 1, 1, 64, 8, "single-pass")
    await step_and_check(dut, model, 0, 0, 0, 0, 0, "single-pass-idle")
    if PERSISTENCE_COUNT > 1:
        assert int(dut.gesture_valid.value) == 0, "gesture_valid asserted too early"


@cocotb.test()
async def test_persistence_triggers(dut):
    """PERSISTENCE_COUNT consecutive matching passes should trigger gesture_valid."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)
    seen_valid = False

    for i in range(PERSISTENCE_COUNT):
        await step_and_check(dut, model, 0, 1, 1, 200, 0, f"persist-{i}")
        seen_valid = seen_valid or (int(dut.gesture_valid.value) == 1)

    assert seen_valid, "gesture_valid never asserted after persistence sequence"
    assert int(dut.gesture.value) == 0


@cocotb.test()
async def test_class_change_resets_count(dut):
    """Changing the gesture class should reset the match counter."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)

    await step_and_check(dut, model, 0, 1, 1, 90, 10, "class-change-a")
    await step_and_check(dut, model, 1, 1, 1, 90, 10, "class-change-b")
    assert int(dut.gesture_valid.value) == 0

    seen_valid = False
    for i in range(PERSISTENCE_COUNT):
        await step_and_check(dut, model, 1, 1, 1, 200, 0, f"class-change-c{i}")
        seen_valid = seen_valid or (int(dut.gesture_valid.value) == 1)
    assert seen_valid, "gesture_valid missing after class change reset and re-accumulate"


@cocotb.test()
async def test_no_pass_resets(dut):
    """class_pass=0 should reset match count to 0."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)
    await step_and_check(dut, model, 0, 1, 1, 120, 0, "pass-1")
    await step_and_check(dut, model, 0, 1, 0, 120, 0, "pass-reset")
    await step_and_check(dut, model, 0, 1, 1, 120, 0, "pass-2")
    assert int(dut.gesture_valid.value) == 0


@cocotb.test()
async def test_confidence_calculation(dut):
    """Confidence should be (dominant_magnitude >> 4), capped at 15."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)
    seen_first = False

    for _ in range(PERSISTENCE_COUNT + 1):
        await step_and_check(dut, model, 2, 1, 1, 0, 300, "conf-sat")
        if int(dut.gesture_valid.value) == 1:
            seen_first = True
            assert int(dut.gesture_confidence.value) == 15
    assert seen_first, "Did not observe saturated confidence gesture"

    for _ in range(PERSISTENCE_COUNT + 1):
        await step_and_check(dut, model, 3, 1, 1, 80, 0, "conf-linear")
        if int(dut.gesture_valid.value) == 1:
            expected = (80 >> 4) & 0xF
            assert int(dut.gesture_confidence.value) == expected, \
                f"Expected confidence {expected}, got {int(dut.gesture_confidence.value)}"


@cocotb.test()
async def test_confidence_tie_uses_y_path(dut):
    """When abs_delta_x == abs_delta_y, confidence must use the Y branch."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)
    tie_mag = 47

    seen_valid = False
    for _ in range(PERSISTENCE_COUNT + 1):
        await step_and_check(dut, model, 1, 1, 1, tie_mag, tie_mag, "conf-tie")
        if int(dut.gesture_valid.value):
            seen_valid = True
            expected = (tie_mag >> 4) & 0xF
            assert int(dut.gesture_confidence.value) == expected
    assert seen_valid, "No gesture_valid seen in tie-confidence test"


@cocotb.test()
async def test_golden_model_random(dut):
    """Long randomized cycle-by-cycle scoreboard against golden model."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)
    rng = random.Random(0x91E57A)

    for cycle in range(1200):
        class_valid = rng.choice([0, 1, 1, 1])
        gesture = rng.randint(0, 3)
        pass_flag = rng.choice([0, 1, 1]) if class_valid else 0
        dx = rng.randint(0, 1023)
        dy = rng.randint(0, 1023)
        await step_and_check(
            dut,
            model,
            gesture,
            class_valid,
            pass_flag,
            dx,
            dy,
            f"random-cycle-{cycle}",
        )


