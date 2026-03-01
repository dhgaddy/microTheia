"""Unit testbench for gesture_classifier with golden reference model.

This module instantiates systolic_array and weight_ram internally, so all three
RTL files are compiled together. We test the complete frame-based pipeline:
frame_pulse -> scan gradient_mapping -> systolic MAC -> energy threshold -> output.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_FREQ_HZ = 12_000_000
FRAME_PERIOD_MS = 1
GRID_SIZE = 16
ADDR_BITS = 8
VALUE_BITS = 8
NUM_CELLS = GRID_SIZE * GRID_SIZE
NUM_CLASSES = 4
MIN_MASS_THRESH = 20

FRAME_CYCLES = (CLK_FREQ_HZ // 1000) * FRAME_PERIOD_MS
PIPE_DEPTH = 2
# Use a generous timeout so tests still pass if RTL runs with default 10 ms frames.
FRAME_WAIT_CYCLES = FRAME_CYCLES * 12


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class GMGestureClassifierModel:
    """Transaction-level model of the gesture_classifier.

    Given a surface map (what ts_read_value returns for each address),
    predicts gesture_class, gesture_valid, and gesture_confidence.
    """

    def __init__(self, grid_size=GRID_SIZE, num_classes=NUM_CLASSES,
                 min_mass_thresh=MIN_MASS_THRESH):
        self.grid_size = grid_size
        self.num_cells = grid_size * grid_size
        self.num_classes = num_classes
        self.min_mass_thresh = min_mass_thresh
        self.weights = self._init_weights()

    def _init_weights(self):
        """Replicate the gradient_map weight_ram initialization."""
        weights = [[0] * self.num_classes for _ in range(self.num_cells)]
        centre = self.grid_size // 2

        for cy in range(self.grid_size):
            for cx in range(self.grid_size):
                addr = cy * self.grid_size + cx
                for class_idx in range(self.num_classes):
                    if class_idx == 0:  # UP
                        if cy < centre:
                            raw = (centre - cy) * 6
                        else:
                            raw = -((cy - centre + 1) * 4)
                    elif class_idx == 1:  # DOWN
                        if cy >= centre:
                            raw = (cy - centre + 1) * 6
                        else:
                            raw = -((centre - cy) * 4)
                    elif class_idx == 2:  # LEFT
                        if cx < centre:
                            raw = (centre - cx) * 6
                        else:
                            raw = -((cx - centre + 1) * 4)
                    elif class_idx == 3:  # RIGHT
                        if cx >= centre:
                            raw = (cx - centre + 1) * 6
                        else:
                            raw = -((centre - cx) * 4)
                    else:
                        raw = 0
                    weights[addr][class_idx] = max(-128, min(127, raw))
        return weights

    def to_signed(self, val, bits):
        val = val & ((1 << bits) - 1)
        return val - (1 << bits) if val >= (1 << (bits - 1)) else val

    def classify(self, surface_values):
        """Given NUM_CELLS surface values, compute classification.
        Returns (gesture_class, gesture_valid, gesture_confidence, energy)."""
        energy = sum(surface_values)

        scores = [0] * self.num_classes
        for i in range(self.num_cells):
            feat = surface_values[i] & 0xFF
            for k in range(self.num_classes):
                scores[k] += feat * self.weights[i][k]

        best = 0
        for k in range(1, self.num_classes):
            if scores[k] > scores[best]:
                best = k

        if energy >= self.min_mass_thresh:
            confidence = 255 if energy > 65535 else (energy >> 8) & 0xFF
            return best, True, confidence, energy
        else:
            return 0, False, 0, energy


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.ts_read_value.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def run_one_frame(dut, surface_values):
    """Drive one complete frame and collect classifier outputs.

    Returns (gesture_class, gesture_valid, gesture_confidence, enable_cycles).
    """
    gesture_class = None
    gesture_valid = False
    gesture_confidence = 0
    enable_cycles = 0
    max_cycles = FRAME_WAIT_CYCLES + NUM_CELLS + PIPE_DEPTH + 1000

    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        en = int(dut.ts_read_enable.value)
        if en:
            enable_cycles += 1
            addr = int(dut.ts_read_addr.value)
            dut.ts_read_value.value = surface_values[addr] if addr < NUM_CELLS else 0
        else:
            dut.ts_read_value.value = 0

        if int(dut.gesture_valid.value) == 1:
            gesture_class = int(dut.gesture_class.value)
            gesture_valid = True
            gesture_confidence = int(dut.gesture_confidence.value)

    assert enable_cycles > 0, "Timed out waiting for frame scan activity"

    return gesture_class, gesture_valid, gesture_confidence, enable_cycles


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """gesture_valid=0 after reset."""
    await setup(dut)
    assert int(dut.gesture_valid.value) == 0


@cocotb.test()
async def test_frame_pulse_generation(dut):
    """The module should generate periodic frame pulses (ts_read_enable goes high)."""
    await setup(dut)
    seen_enable = False
    high_run = 0
    max_high_run = 0
    seen_vals = set()
    seen_states = set()
    for _ in range(FRAME_WAIT_CYCLES + 200):
        await RisingEdge(dut.clk)
        en = int(dut.ts_read_enable.value)
        seen_vals.add(str(dut.ts_read_enable.value))
        seen_states.add(int(dut.debug_state.value))
        if en == 1:
            seen_enable = True
            high_run += 1
            if high_run > max_high_run:
                max_high_run = high_run
        else:
            high_run = 0

    assert seen_enable, \
        f"No frame scan start observed; ts_read_enable seen={sorted(seen_vals)}, debug_state seen={sorted(seen_states)}"
    assert max_high_run >= NUM_CELLS - 4, \
        f"Scan active window too short: longest high run={max_high_run}"


@cocotb.test()
async def test_below_threshold_no_gesture(dut):
    """Surface with very low energy should not trigger gesture_valid."""
    await setup(dut)
    surface = [1] * 10 + [0] * (NUM_CELLS - 10)
    total_energy = sum(surface)
    assert total_energy < MIN_MASS_THRESH

    _, gv, _, enable_cycles = await run_one_frame(dut, surface)
    assert enable_cycles >= NUM_CELLS - 2, f"Unexpected short scan window: {enable_cycles} cycles"
    assert not gv, "gesture_valid asserted with sub-threshold energy"


@cocotb.test()
async def test_above_threshold_gesture(dut):
    """Surface with sufficient energy should produce a gesture."""
    await setup(dut)
    surface = [0] * NUM_CELLS
    for cy in range(GRID_SIZE):
        for cx in range(GRID_SIZE):
            if cy < 4:
                surface[cy * GRID_SIZE + cx] = 200

    model = GMGestureClassifierModel()
    _, expected_valid, _, _ = model.classify(surface)
    _, gv, conf, enable_cycles = await run_one_frame(dut, surface)
    assert enable_cycles >= NUM_CELLS - 2, f"Unexpected short scan window: {enable_cycles} cycles"
    assert expected_valid, "Golden model expected this surface to exceed threshold"
    assert gv, "No gesture detected for high-energy surface"
    assert conf > 0, f"Expected positive confidence, got {conf}"


@cocotb.test()
async def test_golden_up_gesture(dut):
    """Events concentrated at the top should classify as UP (class 0)."""
    await setup(dut)
    model = GMGestureClassifierModel()

    surface = [0] * NUM_CELLS
    for cy in range(GRID_SIZE // 2):
        for cx in range(GRID_SIZE):
            surface[cy * GRID_SIZE + cx] = 200

    expected_class, expected_valid, _, _ = model.classify(surface)
    gc, gv, _, _ = await run_one_frame(dut, surface)
    assert expected_valid, "Golden model expected UP surface to be valid"
    assert gv, "Expected gesture_valid for UP pattern, but none observed"
    assert gc == expected_class, f"UP class mismatch: DUT={gc}, model={expected_class}"


@cocotb.test()
async def test_golden_right_gesture(dut):
    """Events concentrated on the right should classify as RIGHT (class 3)."""
    await setup(dut)
    model = GMGestureClassifierModel()

    surface = [0] * NUM_CELLS
    for cy in range(GRID_SIZE):
        for cx in range(GRID_SIZE // 2, GRID_SIZE):
            surface[cy * GRID_SIZE + cx] = 200

    expected_class, expected_valid, _, _ = model.classify(surface)
    gc, gv, _, _ = await run_one_frame(dut, surface)
    assert expected_valid, "Golden model expected RIGHT surface to be valid"
    assert gv, "Expected gesture_valid for RIGHT pattern, but none observed"
    assert gc == expected_class, f"RIGHT class mismatch: DUT={gc}, model={expected_class}"


@cocotb.test()
async def test_golden_all_directions(dut):
    """Test all four gesture directions against golden model."""
    await setup(dut)
    model = GMGestureClassifierModel()

    surfaces = {}
    half = GRID_SIZE // 2

    # UP: top half active
    s = [0] * NUM_CELLS
    for cy in range(half):
        for cx in range(GRID_SIZE):
            s[cy * GRID_SIZE + cx] = 200
    surfaces[0] = s

    # DOWN: bottom half active
    s = [0] * NUM_CELLS
    for cy in range(half, GRID_SIZE):
        for cx in range(GRID_SIZE):
            s[cy * GRID_SIZE + cx] = 200
    surfaces[1] = s

    # LEFT: left half active
    s = [0] * NUM_CELLS
    for cy in range(GRID_SIZE):
        for cx in range(half):
            s[cy * GRID_SIZE + cx] = 200
    surfaces[2] = s

    # RIGHT: right half active
    s = [0] * NUM_CELLS
    for cy in range(GRID_SIZE):
        for cx in range(half, GRID_SIZE):
            s[cy * GRID_SIZE + cx] = 200
    surfaces[3] = s

    for expected_dir in range(4):
        expected_class, expected_valid, _, _ = model.classify(surfaces[expected_dir])
        gc, gv, _, _ = await run_one_frame(dut, surfaces[expected_dir])
        assert expected_valid, f"Golden model marked direction {expected_dir} invalid unexpectedly"
        assert gv, f"Direction {expected_dir}: no gesture detected"
        assert gc == expected_class, \
            f"Direction {expected_dir}: class mismatch DUT={gc}, model={expected_class}"


@cocotb.test()
async def test_confidence_scaling(dut):
    """Confidence should scale with total energy."""
    await setup(dut)
    low_surface = [40] * NUM_CELLS
    high_surface = [200] * NUM_CELLS

    _, gv_low, conf_low, _ = await run_one_frame(dut, low_surface)
    _, gv_high, conf_high, _ = await run_one_frame(dut, high_surface)

    assert gv_low, "Low-energy-above-threshold frame should still produce gesture_valid"
    assert gv_high, "High-energy frame should produce gesture_valid"
    assert conf_low > 0, f"Expected positive low confidence, got {conf_low}"
    assert conf_high > 0, f"Expected positive high confidence, got {conf_high}"
    assert conf_high >= conf_low, \
        f"Confidence not monotonic: low={conf_low}, high={conf_high}"


