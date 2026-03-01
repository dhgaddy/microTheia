"""Unit testbench for voxel_binning with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

CYCLES_PER_BIN = 100
NUM_BINS = 4
READOUT_BINS = 4
GRID_SIZE = 16
COUNTER_BITS = 6
PARALLEL_READS = 4
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE  # 256
TOTAL_CELLS = NUM_BINS * CELLS_PER_BIN  # 1024
MAX_COUNTER = (1 << COUNTER_BITS) - 1


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class VoxelBinningModel:
    """Transaction-level model of the voxel binning engine."""

    def __init__(self):
        self.mem = [0] * TOTAL_CELLS
        self.current_bin_idx = 0
        self.bin_timer = 0
        self.readout_head_snapshot = [0] * CELLS_PER_BIN

    def reset(self):
        self.mem = [0] * TOTAL_CELLS
        self.current_bin_idx = 0
        self.bin_timer = 0
        self.readout_head_snapshot = [0] * CELLS_PER_BIN

    def inject_event(self, event_x_signed, event_y_signed):
        """Increment the counter for this event's cell in the current bin."""
        mapped_x = (event_x_signed + 8) & 0xF
        mapped_y = (event_y_signed + 8) & 0xF
        cell_addr = (mapped_y << 4) | mapped_x
        full_addr = self.current_bin_idx * CELLS_PER_BIN + cell_addr
        if self.mem[full_addr] < MAX_COUNTER:
            self.mem[full_addr] += 1

    def rotate_bin(self):
        """Rotate to next bin and preserve its pre-clear contents for readout."""
        self.current_bin_idx = (self.current_bin_idx + 1) % NUM_BINS
        base = self.current_bin_idx * CELLS_PER_BIN
        # RTL starts readout at the same time it clears this bin. Readout
        # observes pre-clear values, so snapshot before clearing.
        self.readout_head_snapshot = self.mem[base:base + CELLS_PER_BIN].copy()
        for i in range(CELLS_PER_BIN):
            self.mem[base + i] = 0

    def get_readout(self):
        """Return the expected readout data in chronological order."""
        readout = []
        for bin_offset in range(READOUT_BINS):
            idx = (self.current_bin_idx + bin_offset) % NUM_BINS
            base = idx * CELLS_PER_BIN
            if bin_offset == 0:
                readout.extend(self.readout_head_snapshot)
            else:
                readout.extend(self.mem[base:base + CELLS_PER_BIN])
        return readout


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.event_valid.value = 0
    dut.event_x.value = 0
    dut.event_y.value = 0
    dut.event_polarity.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, CELLS_PER_BIN + 10)


async def inject_event(dut, x_signed, y_signed, pol=1):
    x_val = x_signed & 0x1F
    dut.event_x.value = x_val
    y_val = y_signed & 0x1F
    dut.event_y.value = y_val
    dut.event_polarity.value = pol
    dut.event_valid.value = 1
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0
    await RisingEdge(dut.clk)


async def wait_for_readout_start(dut, timeout=CYCLES_PER_BIN + CELLS_PER_BIN + 500):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.readout_start.value) == 1:
            return True
    return False


async def collect_readout(dut, timeout=CYCLES_PER_BIN * READOUT_BINS + 500):
    """Collect all readout_data while readout_valid is asserted."""
    values = []
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.readout_valid.value) == 1:
            packed_bits = str(dut.readout_data.value).replace("x", "0").replace("X", "0").replace("z", "0").replace("Z", "0")
            packed = int(packed_bits, 2)
            for p in range(PARALLEL_READS):
                val = (packed >> (p * COUNTER_BITS)) & MAX_COUNTER
                values.append(val)
            if len(values) >= READOUT_BINS * CELLS_PER_BIN:
                break
        elif len(values) > 0:
            break
    return values[:READOUT_BINS * CELLS_PER_BIN]


def assert_readout_equal(dut_vals, model_vals, tag):
    assert len(dut_vals) == len(model_vals), \
        f"{tag}: readout length DUT={len(dut_vals)}, model={len(model_vals)}"
    mismatches = []
    for i, (dv, mv) in enumerate(zip(dut_vals, model_vals)):
        if dv != mv:
            mismatches.append((i, dv, mv))
            if len(mismatches) >= 8:
                break
    assert not mismatches, f"{tag}: first mismatches {mismatches}"


async def expect_next_readout_matches(dut, model, tag):
    found = await wait_for_readout_start(dut)
    assert found, f"{tag}: no readout_start seen"
    model.rotate_bin()
    expected = model.get_readout()
    readout = await collect_readout(dut)
    assert_readout_equal(readout, expected, tag)
    return readout, expected


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """After reset, readout_start and readout_valid should be 0."""
    await setup(dut)
    assert int(dut.readout_valid.value) == 0


@cocotb.test()
async def test_single_event_increment(dut):
    """Inject one event and verify it increments the correct cell."""
    await setup(dut)
    model = VoxelBinningModel()
    await inject_event(dut, 0, 0)
    model.inject_event(0, 0)
    readout, expected = await expect_next_readout_matches(dut, model, "single-event")

    target_cell = (8 << 4) | 8
    target_idx = 3 * CELLS_PER_BIN + target_cell
    assert readout[target_idx] == 1, \
        f"Expected mapped center cell to be 1, got {readout[target_idx]}"
    assert sum(readout) == sum(expected) == 1


@cocotb.test()
async def test_bin_rotation(dut):
    """Multiple bin rotations should produce readout_start pulses."""
    await setup(dut)
    starts_seen = 0
    for _ in range((CYCLES_PER_BIN + CELLS_PER_BIN) * (NUM_BINS + 1) + 500):
        await RisingEdge(dut.clk)
        if int(dut.readout_start.value) == 1:
            starts_seen += 1
    assert starts_seen >= NUM_BINS, f"Only saw {starts_seen} readout_start pulses"


@cocotb.test()
async def test_counter_saturation(dut):
    """Heavy repeated events must never overflow the counter width."""
    await setup(dut)

    dut.event_x.value = 0 & 0x1F
    dut.event_y.value = 0 & 0x1F
    dut.event_polarity.value = 1
    dut.event_valid.value = 1
    for _ in range(MAX_COUNTER + 5):
        await RisingEdge(dut.clk)
    dut.event_valid.value = 0
    await RisingEdge(dut.clk)

    found = await wait_for_readout_start(dut)
    assert found, "No readout_start seen after heavy event burst"
    readout = await collect_readout(dut)
    assert len(readout) == READOUT_BINS * CELLS_PER_BIN

    target_cell = (8 << 4) | 8
    target_idx = 3 * CELLS_PER_BIN + target_cell
    val = readout[target_idx]
    assert 0 < val <= MAX_COUNTER, f"Counter out of range at hot cell: {val}"
    assert max(readout) <= MAX_COUNTER, "Observed counter overflow beyond COUNTER_BITS"


@cocotb.test()
async def test_golden_model_basic(dut):
    """Inject known events and compare readout against golden model."""
    await setup(dut)
    model = VoxelBinningModel()

    test_events = [
        (-3, -3), (0, 0), (3, 3), (-7, 7), (7, -7),
        (0, 0), (0, 0), (1, 1), (-1, -1), (2, 2),
    ]

    for x, y in test_events:
        await inject_event(dut, x, y)
        model.inject_event(x, y)

    await expect_next_readout_matches(dut, model, "golden-basic")


@cocotb.test()
async def test_parallel_readout_width(dut):
    """Readout data should be PARALLEL_READS * COUNTER_BITS wide."""
    await setup(dut)
    await inject_event(dut, 0, 0)
    found = await wait_for_readout_start(dut)
    assert found
    for _ in range(50):
        await RisingEdge(dut.clk)
        if int(dut.readout_valid.value) == 1:
            packed_bits = str(dut.readout_data.value).replace("x", "0").replace("X", "0").replace("z", "0").replace("Z", "0")
            packed = int(packed_bits, 2)
            assert packed < (1 << (PARALLEL_READS * COUNTER_BITS))
            break


@cocotb.test()
async def test_coordinate_wrap_and_polarity_ignored(dut):
    """Out-of-range 5-bit signed coordinates should wrap; polarity must not affect counts."""
    await setup(dut)
    model = VoxelBinningModel()

    test_events = [
        (8, -8, 0),   # wraps to (-8,-8) in 5-bit signed interpretation
        (-9, 7, 1),   # wraps x to +7
        (15, 15, 0),  # wraps both to -1
        (-16, -16, 1),
    ]
    for x, y, pol in test_events:
        await inject_event(dut, x, y, pol)
        model.inject_event(x, y)

    await expect_next_readout_matches(dut, model, "coord-wrap")


@cocotb.test()
async def test_golden_model_multibin_random(dut):
    """Randomized multi-bin traffic with strict readout equivalence to golden model."""
    await setup(dut)
    model = VoxelBinningModel()
    rng = random.Random(0xB1A5)

    for bin_id in range(8):
        events_this_bin = rng.randint(0, 30)
        for _ in range(events_this_bin):
            x = rng.randint(-16, 15)
            y = rng.randint(-16, 15)
            pol = rng.randint(0, 1)
            await inject_event(dut, x, y, pol)
            model.inject_event(x, y)

        await expect_next_readout_matches(dut, model, f"multibin-{bin_id}")

