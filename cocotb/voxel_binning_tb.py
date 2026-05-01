# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

CLK_FREQ_HZ    = CFG["CLK_FREQ_HZ"]
WINDOW_MS      = CFG["WINDOW_MS"]
NUM_BINS       = CFG["NUM_BINS"]
READOUT_BINS   = CFG["READOUT_BINS"]
GRID_SIZE      = CFG["GRID_SIZE"]
COUNTER_BITS   = CFG["COUNTER_BITS"]
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE
TOTAL_CELLS   = NUM_BINS * CELLS_PER_BIN
FEATURE_COUNT = READOUT_BINS * CELLS_PER_BIN
MAX_COUNTER   = (1 << COUNTER_BITS) - 1

BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
BIN_DURATION_US = BIN_DURATION_MS * 1000

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
        self.ts_initialized = False
        self.bin_start_ts = 0
        self.next_event_ts = 0

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
        """Apply one bin rollover; return expected readout list or None."""
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

    def force_rollover(self):
        if self.ts_initialized:
            self.bin_start_ts += BIN_DURATION_US
            self.next_event_ts = max(self.next_event_ts, self.bin_start_ts)
        return self.rotate_bin()

    def accept_event(self, x, y, ts):
        """Apply timestamp rollover(s), then accumulate this event in the active bin."""
        readouts = []
        if not self.ts_initialized:
            self.ts_initialized = True
            self.bin_start_ts = ts
            self.next_event_ts = ts
        else:
            while ts - self.bin_start_ts >= BIN_DURATION_US:
                expected = self.rotate_bin()
                if expected is not None:
                    readouts.append(expected)
                self.bin_start_ts += BIN_DURATION_US

        self.inject_event(x, y)
        self.next_event_ts = max(self.next_event_ts, ts + 1)
        return readouts


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    dut.rst.value = 1
    dut.event_valid.value = 0
    dut.event_x.value = 0
    dut.event_y.value = 0
    dut.ts_in.value = 0
    dut.force_rollover_i.value = 0
    dut.readout_ready.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0

    await wait_for_state(dut, ST_ACCUM, timeout=5000)


async def wait_for_state(dut, target_state, timeout=10000):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.state.value) == target_state:
            return
    raise AssertionError(f"Timeout waiting for state {target_state}")


async def wait_for_event_ready(dut, timeout=20000):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.event_ready.value) == 1:
            return
    raise AssertionError("Timeout waiting for event_ready=1")


async def inject_event(dut, model, x, y, pol=1, ts=None):
    # Wait until event_ready is asserted (may be delayed by rmw_pending from prior event).
    for _ in range(20):
        await ReadOnly()
        if int(dut.event_ready.value) == 1:
            break
        await RisingEdge(dut.clk)
    else:
        assert False, "Timed out waiting for event_ready=1"
    if ts is None:
        ts = model.next_event_ts
    await NextTimeStep()
    dut.event_x.value = x & 0xF
    dut.event_y.value = y & 0xF
    dut.ts_in.value = ts
    dut.event_valid.value = 1
    readouts = model.accept_event(x & 0xF, y & 0xF, ts)
    assert not readouts, "inject_event unexpectedly crossed a timestamp bin boundary"
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0
    # Wait for the 2-cycle RMW writeback to complete (event_ready may be 0 during rmw_pending).
    await RisingEdge(dut.clk)


async def force_timer_rollover(dut):
    await wait_for_state(dut, ST_ACCUM)
    await wait_for_event_ready(dut)
    await NextTimeStep()
    dut.force_rollover_i.value = 1
    await RisingEdge(dut.clk)
    dut.force_rollover_i.value = 0
    await ReadOnly()


async def collect_readout(dut):
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
            raise AssertionError(
                "Timed out waiting for readout_start "
                f"(state={int(dut.state.value)} wr_bin={int(dut.wr_bin_idx.value)} "
                f"completed={int(dut.completed_bins.value)} "
                f"pending={int(dut.pending_event_valid.value)} "
                f"bin_start_ts={int(dut.bin_start_ts.value)})"
            )

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
    expected = model.force_rollover()

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


async def collect_readouts(dut, count):
    return [await collect_readout(dut) for _ in range(count)]


async def timestamp_event_and_check(dut, model, x, y, ts, tag):
    expected_readouts = model.accept_event(x, y, ts)

    await ReadOnly()
    assert int(dut.event_ready.value) == 1, "Timestamp event requires event_ready=1"
    await NextTimeStep()
    dut.event_x.value = x
    dut.event_y.value = y
    dut.ts_in.value = ts
    dut.event_valid.value = 1
    read_task = None
    if expected_readouts:
        read_task = cocotb.start_soon(collect_readouts(dut, len(expected_readouts)))
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0

    if read_task is not None:
        got_readouts = await read_task
        assert got_readouts == expected_readouts, f"{tag}: timestamp readout mismatch"

    await wait_for_state(dut, ST_ACCUM)
    await wait_for_event_ready(dut)
    await RisingEdge(dut.clk)  # exit ReadOnly so callers can use inject_event/rotate_and_check


@logged_test()
async def test_reset_and_event_ready(dut):
    await setup(dut)
    assert int(dut.event_ready.value) == 1
    assert int(dut.readout_valid.value) == 0


@logged_test()
async def test_known_events_then_readout(dut):
    await setup(dut)
    model = VoxelBinningModel()

    events = [
        (0, 0),
        (GRID_SIZE // 2, GRID_SIZE // 2),
        (GRID_SIZE - 1, GRID_SIZE - 1),
        (GRID_SIZE // 2, GRID_SIZE // 2),
        (GRID_SIZE // 4, (3 * GRID_SIZE) // 5),
        (GRID_SIZE // 4, (3 * GRID_SIZE) // 5),
        (GRID_SIZE // 4, (3 * GRID_SIZE) // 5),
    ]
    for x, y in events:
        await inject_event(dut, model, x, y)

    for i in range(READOUT_BINS):
        await rotate_and_check(dut, model, f"known-{i}")


@logged_test()
async def test_timestamp_boundary_event_starts_next_bin(dut):
    await setup(dut)
    model = VoxelBinningModel()

    await inject_event(dut, model, 0, 0, ts=10)

    # This event crosses the first timestamp boundary. The golden model rolls
    # first, then accumulates the event in the new active bin.
    await timestamp_event_and_check(
        dut, model, 1, 1, 10 + BIN_DURATION_US, "ts-boundary"
    )

    for i in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, f"ts-boundary-flush-{i}")


@logged_test()
async def test_timestamp_jump_rolls_multiple_bins(dut):
    await setup(dut)
    model = VoxelBinningModel()

    await inject_event(dut, model, 0, 0, ts=0)

    # A large timestamp gap closes every readout bin before this event is
    # counted. This exercises the perfect timestamp model, not a cycle timer.
    hot_x = min(2, GRID_SIZE - 1)
    hot_y = min(2, GRID_SIZE - 1)
    await timestamp_event_and_check(
        dut, model, hot_x, hot_y,
        READOUT_BINS * BIN_DURATION_US + 7,
        "ts-multibin",
    )

    # Verify golden model: post-rollover event landed in the new active bin.
    assert model.mem[
        model.wr_bin_idx * CELLS_PER_BIN + hot_y * GRID_SIZE + hot_x
    ] == 1, "Timestamp jump event was not accumulated in model after catch-up"

    # Verify DUT: flush remaining bins and check the readout that contains
    # hot_x/hot_y in the new bin appears correctly against the golden model.
    for i in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, f"ts-multibin-flush-{i}")


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
    """Counter must clamp at MAX_COUNTER without wrapping.

    With SRAM-backed counters we cannot seed memory directly, so we inject
    MAX_COUNTER + 2 events (or 30 + 2 if MAX_COUNTER > 30 — large counter_bits
    configs verify counting but not the exact saturation boundary).
    """
    await setup(dut)
    model = VoxelBinningModel()

    hot_x, hot_y = 5, 6
    n_inject = min(MAX_COUNTER + 2, 32)

    for _ in range(n_inject):
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

    # force_timer_rollover ends in ReadOnly; state is now ST_READOUT (or ST_CLEAR).
    # The DUT stays outside ST_ACCUM for many cycles — drive event_valid unconditionally.
    assert int(dut.state.value) != ST_ACCUM, \
        "DUT should not be in ST_ACCUM immediately after rollover triggers readout"
    await NextTimeStep()
    dut.event_x.value = GRID_SIZE // 2
    dut.event_y.value = GRID_SIZE // 2
    dut.event_valid.value = 1
    await RisingEdge(dut.clk)
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
async def test_timestamp_driven_multibin_readout_matches_golden(dut):
    """Accumulate events in several bins via timestamp-boundary crossings only.

    Unlike the force-rollover tests, every bin advance here is triggered by
    ts_in crossing a BIN_DURATION_US boundary — exactly as in production with
    the evt2 decoder driving ts_in.  After warm-up via force rollovers the
    complete readout window is compared cell-by-cell against the golden model.
    """
    await setup(dut)
    model = VoxelBinningModel()

    cx = min(3, GRID_SIZE - 1)
    cy = min(7, GRID_SIZE - 1)
    bx = min(5, GRID_SIZE - 1)
    by = min(2, GRID_SIZE - 1)
    dx = min(8, GRID_SIZE - 1)
    dy = min(8, GRID_SIZE - 1)

    # --- Bin 0: inject several events ---
    for _ in range(3):
        await inject_event(dut, model, cx, cy)
    for _ in range(2):
        await inject_event(dut, model, min(1, GRID_SIZE - 1), min(4, GRID_SIZE - 1))

    # --- Timestamp boundary → Bin 1 ---
    ts1 = model.bin_start_ts + BIN_DURATION_US
    await timestamp_event_and_check(dut, model, bx, by, ts1, "ts-cross-0to1")
    for _ in range(2):
        await inject_event(dut, model, bx, by)
    await inject_event(dut, model, 0, GRID_SIZE - 1)

    # --- Timestamp boundary → Bin 2 ---
    ts2 = model.bin_start_ts + BIN_DURATION_US
    await timestamp_event_and_check(dut, model, dx, dy, ts2, "ts-cross-1to2")

    # --- Warm up remaining bins with force rollovers and verify readout ---
    # READOUT_BINS total rotations required; 2 done via timestamps, rest forced.
    for i in range(READOUT_BINS - 2):
        await rotate_and_check(dut, model, f"ts-multibin-flush-{i}")


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
async def test_event_ready_low_during_rmw_pending(dut):
    """event_ready must be 0 during the RMW writeback cycle and 1 the cycle after."""
    await setup(dut)

    await ReadOnly()
    assert int(dut.event_ready.value) == 1, "event_ready must be 1 before first event"

    # Present one event — DUT issues SRAM read and sets rmw_pending.
    await NextTimeStep()
    dut.event_x.value = 1
    dut.event_y.value = 1
    dut.ts_in.value = 0
    dut.event_valid.value = 1
    await RisingEdge(dut.clk)   # cycle N: rmw_pending becomes 1
    dut.event_valid.value = 0

    await ReadOnly()
    assert int(dut.event_ready.value) == 0, \
        "event_ready must be 0 during rmw_pending writeback cycle"

    await RisingEdge(dut.clk)   # cycle N+1: rmw_pending clears
    await ReadOnly()
    assert int(dut.event_ready.value) == 1, \
        "event_ready must return to 1 after RMW writeback completes"


@logged_test()
async def test_readout_start_single_cycle_pulse(dut):
    """readout_start must assert for exactly one clock cycle when readout begins."""
    await setup(dut)
    model = VoxelBinningModel()

    for _ in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, "prime")

    # Arm collection before rollover so the readout_start pulse is not missed.
    model.rotate_bin()
    read_task = cocotb.start_soon(collect_readout(dut))
    await NextTimeStep()
    await force_timer_rollover(dut)
    # force_timer_rollover ends in ReadOnly — readout_start is asserted now.
    assert int(dut.readout_start.value) == 1, \
        "readout_start did not assert when readout began"

    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.readout_start.value) == 0, \
        "readout_start did not deassert after one cycle"

    await read_task
    await wait_for_state(dut, ST_ACCUM)


@logged_test()
async def test_pending_event_on_force_rollover_cycle(dut):
    """An event presented simultaneously with force_rollover must be latched as pending
    and accumulated in the new active bin after the rollover completes."""
    await setup(dut)
    model = VoxelBinningModel()

    # Seed event to initialise ts_initialized in the DUT and model.
    await inject_event(dut, model, 0, 0, ts=0)
    await wait_for_event_ready(dut)

    px = min(3, GRID_SIZE - 1)
    py = min(3, GRID_SIZE - 1)

    # The pending event's timestamp must be >= the new bin_start_ts after the
    # force rollover (bin_start_ts + BIN_DURATION_US), otherwise the unsigned
    # subtraction wraps and triggers a cascade of timestamp-driven rollovers.
    # Setting ts = BIN_DURATION_US places the event exactly at the new boundary:
    # acc_event_ts - new_bin_start_ts = 125000 - 125000 = 0 < BIN_DURATION_TS.
    pending_ts = BIN_DURATION_US

    # Drive event_valid and force_rollover_i simultaneously.
    await NextTimeStep()
    dut.event_x.value = px
    dut.event_y.value = py
    dut.ts_in.value = pending_ts
    dut.event_valid.value = 1
    dut.force_rollover_i.value = 1

    # Golden model: rollover first, then pending event lands in the new bin.
    model.force_rollover()
    model.inject_event(px, py)

    await RisingEdge(dut.clk)
    dut.event_valid.value = 0
    dut.force_rollover_i.value = 0

    # Flush remaining bins; final rotate_and_check verifies the pending event
    # appears in the correct bin of the readout window.
    for i in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, f"pending-flush-{i}")

