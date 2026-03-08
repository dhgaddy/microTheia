"""Robust cocotb testbench for voxel_binning with golden-model scoreboarding."""

import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

CLK_FREQ_HZ = 12_000_000
WINDOW_MS = 1000
NUM_BINS = 8
READOUT_BINS = 8
GRID_SIZE = 16
COUNTER_BITS = 16
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE
TOTAL_CELLS = NUM_BINS * CELLS_PER_BIN
FEATURE_COUNT = READOUT_BINS * CELLS_PER_BIN
MAX_COUNTER = (1 << COUNTER_BITS) - 1

BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS  # default-derived: 1500000

ST_ACCUM = 0
ST_WAIT_RD = 1
ST_READOUT = 2
ST_CLEAR = 3


class VoxelBinningModel:
    """Transaction-level model matching rtl/voxel_binning.sv defaults."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.mem = [0] * TOTAL_CELLS
        self.wr_bin_idx = 0
        self.completed_bins = 0

    def _cell_addr(self, x, y):
        return (y * GRID_SIZE) + x

    def inject_event(self, x, y):
        addr = (self.wr_bin_idx * CELLS_PER_BIN) + self._cell_addr(x, y)
        if self.mem[addr] < MAX_COUNTER:
            self.mem[addr] += 1

    def _readout_snapshot(self):
        start = (self.wr_bin_idx + NUM_BINS - (READOUT_BINS - 1)) % NUM_BINS
        out = []
        for off in range(READOUT_BINS):
            b = (start + off) % NUM_BINS
            base = b * CELLS_PER_BIN
            out.extend(self.mem[base:base + CELLS_PER_BIN])
        return out

    def rotate_bin(self):
        """Apply one timer expiry; return expected readout list or None."""
        next_wr = (self.wr_bin_idx + 1) % NUM_BINS
        completed_next = self.completed_bins + 1
        if completed_next > NUM_BINS:
            completed_next = NUM_BINS

        expected = None
        if completed_next >= READOUT_BINS:
            expected = self._readout_snapshot()

        # Clear next write bin after readout phase.
        base = next_wr * CELLS_PER_BIN
        for i in range(CELLS_PER_BIN):
            self.mem[base + i] = 0

        self.wr_bin_idx = next_wr
        self.completed_bins = completed_next
        return expected


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.event_valid.value = 0
    dut.event_x.value = 0
    dut.event_y.value = 0
    dut.event_polarity.value = 0
    dut.readout_ready.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0

    # Wait for post-reset clear state to return to accumulate.
    await wait_for_state(dut, ST_ACCUM, timeout=5000)


async def wait_for_state(dut, target_state, timeout=10000):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.state.value) == target_state:
            return
    raise AssertionError(f"Timeout waiting for state {target_state}")


async def inject_event(dut, model, x, y, pol=1):
    assert int(dut.event_ready.value) == 1, "Attempted event inject while event_ready=0"
    dut.event_x.value = x & 0xF
    dut.event_y.value = y & 0xF
    dut.event_polarity.value = pol & 1
    dut.event_valid.value = 1
    model.inject_event(x & 0xF, y & 0xF)
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0
    await RisingEdge(dut.clk)


async def force_timer_rollover(dut):
    assert int(dut.state.value) == ST_ACCUM, "Rollover forcing requires ST_ACCUM"
    dut.timer_ctr.value = CYCLES_PER_BIN_SAFE - 1
    await RisingEdge(dut.clk)
    await ReadOnly()


async def collect_readout(dut):
    # Wait for readout_start pulse.
    await ReadOnly()
    start_seen = int(dut.readout_start.value) == 1
    if not start_seen:
        for _ in range(20000):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.readout_start.value) == 1:
                start_seen = True
                break
        else:
            raise AssertionError("Timed out waiting for readout_start")

    values = []
    expected_idx = 0

    def sample_cycle():
        nonlocal expected_idx
        if int(dut.readout_valid.value):
            idx = int(dut.readout_index.value)
            val = int(dut.readout_data.value)
            assert idx == expected_idx, f"readout_index mismatch: DUT={idx}, expected={expected_idx}"
            values.append(val)
            if int(dut.readout_last.value):
                assert expected_idx == FEATURE_COUNT - 1, "readout_last asserted at wrong index"
                return True
            expected_idx += 1
        return False

    # readout_valid can already be high in the same cycle as readout_start.
    if start_seen and sample_cycle():
        assert len(values) == FEATURE_COUNT, f"Readout length {len(values)} != {FEATURE_COUNT}"
        return values

    for _ in range(FEATURE_COUNT + 2000):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if sample_cycle():
            break

    assert len(values) == FEATURE_COUNT, f"Readout length {len(values)} != {FEATURE_COUNT}"
    return values


async def rotate_and_check(dut, model, tag):
    expected = model.rotate_bin()

    if expected is None:
        await force_timer_rollover(dut)
        await wait_for_state(dut, ST_ACCUM)
        return

    # Arm collection before rollover so a same-cycle readout_start pulse is not missed.
    read_task = cocotb.start_soon(collect_readout(dut))
    await NextTimeStep()
    await force_timer_rollover(dut)
    got = await read_task
    assert got == expected, f"{tag}: readout mismatch"
    await wait_for_state(dut, ST_ACCUM)


@logged_test()
async def test_reset_and_event_ready(dut):
    await setup(dut)
    assert int(dut.event_ready.value) == 1
    assert int(dut.readout_valid.value) == 0


@logged_test()
async def test_known_events_then_readout(dut):
    await setup(dut)
    model = VoxelBinningModel()

    events = [(0, 0), (8, 8), (15, 15), (8, 8), (3, 9), (3, 9), (3, 9)]
    for x, y in events:
        await inject_event(dut, model, x, y)

    for i in range(READOUT_BINS):
        await rotate_and_check(dut, model, f"known-{i}")


@logged_test()
async def test_wait_rd_backpressure(dut):
    await setup(dut)
    model = VoxelBinningModel()

    # Advance to first readout-eligible boundary.
    for _ in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, "prefill")

    # Hold readout_ready low at rollover; should park in ST_WAIT_RD.
    dut.readout_ready.value = 0
    expected = model.rotate_bin()
    await force_timer_rollover(dut)
    assert expected is not None
    assert int(dut.state.value) in (ST_WAIT_RD, ST_READOUT)

    saw_start_while_low = False
    for _ in range(50):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.readout_start.value):
            saw_start_while_low = True
            break
    assert not saw_start_while_low, "readout_start asserted despite readout_ready=0"

    await NextTimeStep()
    dut.readout_ready.value = 1
    got = await collect_readout(dut)
    assert got == expected, "Backpressured readout mismatch"
    await wait_for_state(dut, ST_ACCUM)


@logged_test()
async def test_counter_saturation(dut):
    await setup(dut)
    model = VoxelBinningModel()

    hot_x, hot_y = 5, 6
    hot_addr = (model.wr_bin_idx * CELLS_PER_BIN) + (hot_y * GRID_SIZE + hot_x)

    # Seed near max directly, then add events to verify clamp at MAX_COUNTER.
    dut.mem[hot_addr].value = MAX_COUNTER - 1
    model.mem[hot_addr] = MAX_COUNTER - 1

    await inject_event(dut, model, hot_x, hot_y)
    await inject_event(dut, model, hot_x, hot_y)

    for i in range(READOUT_BINS):
        await rotate_and_check(dut, model, f"sat-{i}")


@logged_test()
async def test_events_ignored_when_not_accum(dut):
    await setup(dut)
    model = VoxelBinningModel()

    # Trigger rollover to enter clear/readout path (not accum).
    for _ in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, "pre")

    expected = model.rotate_bin()
    read_task = cocotb.start_soon(collect_readout(dut))
    await NextTimeStep()
    await force_timer_rollover(dut)
    assert expected is not None

    # While not in ST_ACCUM, pulse event_valid; model does not accept this event.
    if int(dut.state.value) != ST_ACCUM:
        await NextTimeStep()
        dut.event_x.value = 9
        dut.event_y.value = 9
        dut.event_polarity.value = 1
        dut.event_valid.value = 1
        await RisingEdge(dut.clk)
        dut.event_valid.value = 0

    got = await read_task
    assert got == expected, "Non-accum event unexpectedly affected readout"
    await wait_for_state(dut, ST_ACCUM)


@logged_test()
async def test_randomized_multibin_scoreboard(dut):
    await setup(dut)
    model = VoxelBinningModel()
    rng = random.Random(0xB1A5)

    for bin_idx in range(10):
        events = rng.randint(0, 40)
        for _ in range(events):
            x = rng.randint(0, GRID_SIZE - 1)
            y = rng.randint(0, GRID_SIZE - 1)
            pol = rng.randint(0, 1)
            await inject_event(dut, model, x, y, pol)

        await rotate_and_check(dut, model, f"rnd-{bin_idx}")


@logged_test()
async def test_corner_coordinates_binned_correctly(dut):
    """Events at all four grid corners and centre are stored in the right cells."""
    await setup(dut)
    model = VoxelBinningModel()

    corners = [
        (0,           0),
        (GRID_SIZE-1, 0),
        (0,           GRID_SIZE-1),
        (GRID_SIZE-1, GRID_SIZE-1),
        (GRID_SIZE//2, GRID_SIZE//2),
    ]
    for x, y in corners:
        await inject_event(dut, model, x, y)
        await inject_event(dut, model, x, y)  # inject twice to distinguish from zero

    for i in range(READOUT_BINS):
        await rotate_and_check(dut, model, f"corner-{i}")


@logged_test()
async def test_readout_index_monotonically_increasing(dut):
    """readout_index must count from 0 to FEATURE_COUNT-1 with no gaps or repeats."""
    await setup(dut)
    model = VoxelBinningModel()

    # Inject a handful of events so the readout is non-trivial.
    for _ in range(8):
        await inject_event(dut, model, 4, 4)

    # Advance to the first readout-eligible rotation.
    for _ in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, "prime")

    # Trigger one readout and capture the index sequence.
    model.rotate_bin()
    await force_timer_rollover(dut)

    indices = []
    if int(dut.readout_valid.value):
        idx = int(dut.readout_index.value)
        indices.append(idx)
        if int(dut.readout_last.value):
            assert indices == list(range(FEATURE_COUNT)), \
                f"readout_index sequence wrong: first={indices[:5]}... last={indices[-5:]}"
            return

    for _ in range(FEATURE_COUNT + 2000):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.readout_valid.value):
            idx = int(dut.readout_index.value)
            indices.append(idx)
            if int(dut.readout_last.value):
                break

    assert indices == list(range(FEATURE_COUNT)), \
        f"readout_index sequence wrong: first={indices[:5]}... last={indices[-5:]}"


@logged_test()
async def test_clear_zeros_old_bin_data(dut):
    """After rotation, the newly cleared bin must read back as zero in the next window."""
    await setup(dut)
    model = VoxelBinningModel()

    hot_x, hot_y = 7, 3

    # Fill bin 0 heavily, then rotate all bins so those counts appear in readout.
    for _ in range(50):
        await inject_event(dut, model, hot_x, hot_y)

    # Rotate enough bins to push those events through the entire readout window.
    for i in range(READOUT_BINS + NUM_BINS):
        await rotate_and_check(dut, model, f"flush-{i}")

    # Now the model has cleared all bins; the hot cell should be zero.
    hot_cell = (model.wr_bin_idx * CELLS_PER_BIN) + (hot_y * GRID_SIZE + hot_x)
    assert model.mem[hot_cell] == 0, \
        f"Model hot cell not zeroed after full rotation: {model.mem[hot_cell]}"


@logged_test()
async def test_high_event_rate_multi_bin_accuracy(dut):
    """Dense event stream across many bins; compare each readout against golden model."""
    await setup(dut)
    model = VoxelBinningModel()
    rng = random.Random(0xE7E7_E7E7)

    for bin_idx in range(20):
        n_events = rng.randint(10, 60)
        for _ in range(n_events):
            x = rng.randint(0, GRID_SIZE - 1)
            y = rng.randint(0, GRID_SIZE - 1)
            await inject_event(dut, model, x, y, rng.randint(0, 1))

        await rotate_and_check(dut, model, f"dense-{bin_idx}")


@logged_test()
async def test_stale_bins_preserved_after_reset(dut):
    """After reset only bin 0 is cleared; bins 1..NUM_BINS-1 retain their pre-reset contents."""
    await setup(dut)

    # Plant a known sentinel value in a sample of cells across bins 1..NUM_BINS-1.
    stale_val = 77
    cells_to_check = 4  # check 4 cells per bin
    for b in range(1, NUM_BINS):
        for cell in range(cells_to_check):
            addr = b * CELLS_PER_BIN + cell
            dut.mem[addr].value = stale_val

    # Assert reset — DUT only clears bin 0 during the post-reset ST_CLEAR.
    dut.rst.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await wait_for_state(dut, ST_ACCUM, timeout=5000)

    # Bin 0 cells must be zeroed by the post-reset clear.
    for cell in range(cells_to_check):
        got = int(dut.mem[cell].value)  # bin 0 base = 0
        assert got == 0, f"Bin 0 cell {cell} should be 0 after reset, got {got}"

    # Bins 1..NUM_BINS-1 must still hold the stale sentinel.
    for b in range(1, NUM_BINS):
        for cell in range(cells_to_check):
            addr = b * CELLS_PER_BIN + cell
            got = int(dut.mem[addr].value)
            assert got == stale_val, \
                f"Bin {b} cell {cell} (addr={addr}) expected stale {stale_val}, got {got}"
