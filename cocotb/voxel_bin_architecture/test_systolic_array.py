"""Unit testbench for systolic_array with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly, NextTimeStep
import random

NUM_CLASSES = 4
NUM_CELLS = 16
VALUE_BITS = 6
WEIGHT_BITS = 8
ACC_BITS = 24
PARALLEL_INPUTS = 4
ADDR_BITS = NUM_CELLS.bit_length()  # clog2(16) = 4
CYCLES_NEEDED = (NUM_CELLS + PARALLEL_INPUTS - 1) // PARALLEL_INPUTS  # 4


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class VBSystolicModel:
    """Golden model for the parallel-input systolic array."""

    def __init__(self, num_classes=NUM_CLASSES, num_cells=NUM_CELLS,
                 value_bits=VALUE_BITS, weight_bits=WEIGHT_BITS,
                 acc_bits=ACC_BITS, parallel_inputs=PARALLEL_INPUTS):
        self.num_classes = num_classes
        self.num_cells = num_cells
        self.value_bits = value_bits
        self.weight_bits = weight_bits
        self.acc_bits = acc_bits
        self.parallel_inputs = parallel_inputs
        self.acc_mask = (1 << acc_bits) - 1
        self.acc_sign = 1 << (acc_bits - 1)

    def to_signed(self, val, bits):
        val = val & ((1 << bits) - 1)
        if val >= (1 << (bits - 1)):
            return val - (1 << bits)
        return val

    def compute(self, features, weights):
        """
        Compute scores.
        features: list of NUM_CELLS unsigned values (VALUE_BITS wide)
        weights: 2D list [cell][class] of signed WEIGHT_BITS values
        Returns (best_class, scores[NUM_CLASSES]) as signed ACC_BITS values.
        """
        scores = [0] * self.num_classes
        for i in range(self.num_cells):
            feat = features[i] & ((1 << self.value_bits) - 1)
            for k in range(self.num_classes):
                w = self.to_signed(weights[i][k], self.weight_bits)
                product = feat * w
                scores[k] += product

        for k in range(self.num_classes):
            scores[k] = self.to_signed(scores[k], self.acc_bits)

        best = 0
        for k in range(1, self.num_classes):
            if scores[k] > scores[best]:
                best = k

        return best, scores


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    dut.feature_in.value = 0
    dut.feature_valid.value = 0
    dut.w_data_flat.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


def pack_features(values):
    """Pack PARALLEL_INPUTS feature values into a flat bus."""
    result = 0
    for i, v in enumerate(values):
        result |= (v & ((1 << VALUE_BITS) - 1)) << (i * VALUE_BITS)
    return result


def pack_weights(weight_matrix, addrs):
    """
    Pack PARALLEL_INPUTS * NUM_CLASSES weights into w_data_flat.
    weight_matrix[cell][class] -> signed WEIGHT_BITS.
    """
    result = 0
    for p_idx, addr in enumerate(addrs):
        for k in range(NUM_CLASSES):
            w = weight_matrix[addr][k] & ((1 << WEIGHT_BITS) - 1)
            bit_pos = p_idx * NUM_CLASSES * WEIGHT_BITS + k * WEIGHT_BITS
            result |= w << bit_pos
    return result


def unpack_addrs(flat):
    """Unpack PARALLEL_INPUTS addresses from w_addr_flat."""
    addrs = []
    mask = (1 << ADDR_BITS) - 1
    for p in range(PARALLEL_INPUTS):
        addrs.append((flat >> (p * ADDR_BITS)) & mask)
    return addrs


def unpack_scores(raw):
    """Unpack signed ACC_BITS class scores from scores_flat."""
    vals = []
    for k in range(NUM_CLASSES):
        v = (raw >> (k * ACC_BITS)) & ((1 << ACC_BITS) - 1)
        if v >= (1 << (ACC_BITS - 1)):
            v -= (1 << ACC_BITS)
        vals.append(v)
    return vals


async def run_inference(dut, features, weights):
    """Drive one inference and return (best_class, scores, valid_pulse_count)."""
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    for cycle in range(CYCLES_NEEDED):
        feat_base = cycle * PARALLEL_INPUTS
        feat_vals = [features[feat_base + p] if (feat_base + p) < NUM_CELLS else 0
                     for p in range(PARALLEL_INPUTS)]
        weight_cycle = cycle - 1 if cycle > 0 else 0
        weight_base = weight_cycle * PARALLEL_INPUTS
        weight_addrs = [weight_base + p for p in range(PARALLEL_INPUTS)]
        w_packed = pack_weights(weights, [min(a, NUM_CELLS - 1) for a in weight_addrs])

        dut.feature_in.value = pack_features(feat_vals)
        dut.feature_valid.value = 1
        dut.w_data_flat.value = w_packed
        await RisingEdge(dut.clk)

    dut.feature_valid.value = 0
    last_base = (CYCLES_NEEDED - 1) * PARALLEL_INPUTS
    last_addrs = [last_base + p for p in range(PARALLEL_INPUTS)]
    dut.w_data_flat.value = pack_weights(weights, [min(a, NUM_CELLS - 1) for a in last_addrs])

    result_class = None
    result_scores = None
    valid_pulses = 0

    for _ in range(20):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.result_valid.value) == 1:
            valid_pulses += 1
            if result_class is None:
                result_class = int(dut.best_class.value)
                result_scores = unpack_scores(int(dut.scores_flat.value))
        await NextTimeStep()

    return result_class, result_scores, valid_pulses


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """result_valid=0 after reset."""
    await setup(dut)
    assert int(dut.result_valid.value) == 0
    assert int(dut.best_class.value) == 0


@cocotb.test()
async def test_all_zeros(dut):
    """All-zero features should produce all-zero scores."""
    await setup(dut)

    features = [0] * NUM_CELLS
    weights = [[0] * NUM_CLASSES for _ in range(NUM_CELLS)]

    dut_class, dut_scores, valid_pulses = await run_inference(dut, features, weights)
    assert dut_class == 0, f"Expected class 0 tie-break, got {dut_class}"
    assert dut_scores == [0, 0, 0, 0], f"Expected zero scores, got {dut_scores}"
    assert valid_pulses == 1, f"Expected single result_valid pulse, got {valid_pulses}"


@cocotb.test()
async def test_known_values(dut):
    """Known feature/weight values, verify scores match golden model."""
    await setup(dut)

    features = [i % (1 << VALUE_BITS) for i in range(NUM_CELLS)]
    weights = [[(i * (k + 1)) % 256 for k in range(NUM_CLASSES)]
               for i in range(NUM_CELLS)]
    for i in range(NUM_CELLS):
        for k in range(NUM_CLASSES):
            if weights[i][k] > 127:
                weights[i][k] -= 256

    model = VBSystolicModel()
    w_unsigned = [[(w & 0xFF) for w in row] for row in weights]
    expected_class, expected_scores = model.compute(features, w_unsigned)

    dut_class, dut_scores, valid_pulses = await run_inference(dut, features, w_unsigned)
    assert valid_pulses == 1, f"Expected single result_valid pulse, got {valid_pulses}"
    assert dut_class == expected_class, \
        f"best_class: DUT={dut_class}, model={expected_class}"
    assert dut_scores == expected_scores, \
        f"scores: DUT={dut_scores}, model={expected_scores}"


@cocotb.test()
async def test_golden_random(dut):
    """Randomized inputs with strict class+score match against golden model."""
    await setup(dut)
    model = VBSystolicModel()
    rng = random.Random(0x5A57A11C)

    for trial in range(40):
        features = [rng.randint(0, (1 << VALUE_BITS) - 1) for _ in range(NUM_CELLS)]
        weights = [[rng.randint(0, 255) for _ in range(NUM_CLASSES)]
                    for _ in range(NUM_CELLS)]

        expected_class, expected_scores = model.compute(features, weights)
        dut_class, dut_scores, valid_pulses = await run_inference(dut, features, weights)

        assert dut_class is not None, f"Trial {trial}: no result_valid pulse observed"
        assert valid_pulses == 1, f"Trial {trial}: result_valid pulses={valid_pulses}"
        assert dut_class == expected_class, \
            f"Trial {trial}: class DUT={dut_class}, model={expected_class}"
        assert dut_scores == expected_scores, \
            f"Trial {trial}: scores DUT={dut_scores}, model={expected_scores}"

        await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_result_valid_pulse(dut):
    """result_valid should be a single-cycle pulse."""
    await setup(dut)

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    for cycle in range(CYCLES_NEEDED):
        dut.feature_in.value = 0
        dut.feature_valid.value = 1
        dut.w_data_flat.value = 0
        await RisingEdge(dut.clk)
    dut.feature_valid.value = 0

    valid_count = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.result_valid.value) == 1:
            valid_count += 1

    assert valid_count == 1, f"result_valid asserted {valid_count} times (expected 1)"


@cocotb.test()
async def test_tie_break_prefers_lowest_class(dut):
    """When all scores are tied, best_class must resolve to class 0."""
    await setup(dut)
    features = [1] * NUM_CELLS
    weights = [[0] * NUM_CLASSES for _ in range(NUM_CELLS)]

    dut_class, dut_scores, valid_pulses = await run_inference(dut, features, weights)
    assert valid_pulses == 1, f"Expected single result_valid pulse, got {valid_pulses}"
    assert dut_scores == [0, 0, 0, 0], f"Expected tied zero scores, got {dut_scores}"
    assert dut_class == 0, f"Expected class 0 on tie-break, got {dut_class}"


