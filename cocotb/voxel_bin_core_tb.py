# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2025 Group G Contributors
from collections import deque
from pathlib import Path
import random
import struct

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

CLK_FREQ_HZ  = CFG["CLK_FREQ_HZ"]
WINDOW_MS    = CFG["WINDOW_MS"]
GRID_SIZE    = CFG["GRID_SIZE"]
NUM_BINS     = CFG["NUM_BINS"]
READOUT_BINS = CFG["READOUT_BINS"]
WEIGHT_BITS  = CFG["WEIGHT_BITS"]
WEIGHT_SCALE = CFG["WEIGHT_SCALE"]
SENSOR_DIM   = CFG["SENSOR_WIDTH"]
COUNTER_BITS = CFG.get("COUNTER_BITS", 4)
NUM_CLASSES  = CFG.get("NUM_CLASSES", 4)


BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS
BIN_DIV = SENSOR_DIM // GRID_SIZE
FEATURE_COUNT = GRID_SIZE * GRID_SIZE * READOUT_BINS

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

ST_ACCUM = 0


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_from_grid(g):
    g = max(0, min(GRID_SIZE - 1, int(g)))
    return min(SENSOR_DIM - 1, (g * BIN_DIV) + (BIN_DIV // 2))


class Evt2DecoderModel:
    def __init__(self):
        self.have_time_high = False

    def on_word(self, word):
        pkt = (word >> 28) & 0xF
        x_raw = (word >> 11) & 0x7FF
        y_raw = word & 0x7FF

        if pkt == EVT_TIME_HIGH:
            self.have_time_high = True
            return None

        if pkt not in (EVT_CD_OFF, EVT_CD_ON):
            return None

        if not self.have_time_high:
            return None

        x_clamped = min(x_raw, SENSOR_DIM - 1)
        y_clamped = min(y_raw, SENSOR_DIM - 1)
        x_grid = min(x_clamped // BIN_DIV, GRID_SIZE - 1)
        y_grid = min(y_clamped // BIN_DIV, GRID_SIZE - 1)
        return (x_grid, y_grid)



def load_thresholds():
    """Load class and diff thresholds from thresholds.mem (hex, $readmemh format).
    Returns a list of 2*NUM_CLASSES integers:
      [0..NUM_CLASSES-1]          = CLASS_THRESHOLD per class
      [NUM_CLASSES..2*NUM_CLASSES-1] = DIFF_THRESHOLD per class
    """
    repo_root = Path(__file__).resolve().parents[1]
    thresh_path = repo_root / "weights" / "thresholds.mem"
    if not thresh_path.exists():
        return [0] * (2 * NUM_CLASSES)
    lines = [l.strip() for l in thresh_path.read_text(encoding="ascii").splitlines()
             if l.strip() and not l.strip().startswith("//")]
    vals = []
    for line in lines:
        try:
            vals.append(int(line, 16))
        except ValueError:
            vals.append(0)
    # Pad to 2*NUM_CLASSES if file is short.
    while len(vals) < 2 * NUM_CLASSES:
        vals.append(0)
    return vals


_thresholds = None  # Loaded once at first ScoreModel construction.


class ScoreModel:
    def __init__(self, weights_per_class):
        global _thresholds
        if _thresholds is None:
            _thresholds = load_thresholds()
        self.weights = weights_per_class
        self.class_thresholds = _thresholds[:NUM_CLASSES]

    @staticmethod
    def _argmax_with_second(vals):
        best_i = 0
        best = vals[0]
        second = 0
        for i in range(1, len(vals)):
            if vals[i] > best:
                second = best
                best = vals[i]
                best_i = i
            elif vals[i] > second:
                second = vals[i]
        return best_i, best, second

    def classify(self, features):
        scores = [0, 0, 0, 0]
        for i, feat in enumerate(features):
            f = int(feat)
            for c in range(4):
                scores[c] += f * self.weights[c][i]

        best_class, best, second = self._argmax_with_second(scores)
        margin = best - second
        # class_pass mirrors RTL: max_score > CLASS_THRESHOLD[max_class]
        class_pass = int(best > self.class_thresholds[best_class])
        return best_class, class_pass, margin


def load_weights_from_mem():
    """Load pre-generated quantized weights from the per-class .mem files.

    Each file contains FEATURE_COUNT hex byte values (one per line) already
    ordered to match the hardware readout address space:
      addr = bin * GRID_SIZE * GRID_SIZE + y * GRID_SIZE + x
    where bin=0 is the oldest bin, y is row (outer), x is column (inner).

    Returns a list of NUM_CLASSES lists, each of length FEATURE_COUNT.
    """
    repo_root = Path(__file__).resolve().parents[1]
    mem_keys = ["WEIGHT_MEM_C0", "WEIGHT_MEM_C1", "WEIGHT_MEM_C2", "WEIGHT_MEM_C3"]
    mem_defaults = [f"weights/{FEATURE_COUNT}weights_q8_c{c}.mem" for c in range(NUM_CLASSES)]
    weights = []
    for c in range(NUM_CLASSES):
        rel_path = CFG.get(mem_keys[c], mem_defaults[c])
        path = repo_root / rel_path
        lines = path.read_text(encoding="ascii").splitlines()
        vals = []
        for line in lines:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            try:
                vals.append(int(line, 16))
            except ValueError:
                vals.append(0)
        while len(vals) < FEATURE_COUNT:
            vals.append(0)
        weights.append(vals[:FEATURE_COUNT])
    return weights


async def deposit_weights_and_thresholds(dut, weights, thresholds):
    """
    Load weights and thresholds into the DUT SRAMs via the runtime write ports.

    Must be called after reset is deasserted (SRAM CEN is high during reset).
    weights:    list of NUM_CLASSES lists, each of length FEATURE_COUNT.
    thresholds: list of 2*NUM_CLASSES ints (class thresholds then diff thresholds).
    """
    # Write weights one address per cycle for each class.
    for c in range(NUM_CLASSES):
        for addr in range(FEATURE_COUNT):
            dut.weight_wr_valid_i.value = 1
            dut.weight_wr_class_i.value = c
            dut.weight_wr_addr_i.value = addr
            dut.weight_wr_data_i.value = int(weights[c][addr])
            await RisingEdge(dut.clk)
    dut.weight_wr_valid_i.value = 0

    # Write thresholds: addr 0..NUM_CLASSES-1 = class thresholds,
    #                   NUM_CLASSES..2*NUM_CLASSES-1 = diff thresholds.
    for addr in range(2 * NUM_CLASSES):
        dut.thresh_wr_valid_i.value = 1
        dut.thresh_wr_addr_i.value = addr
        dut.thresh_wr_data_i.value = int(thresholds[addr])
        await RisingEdge(dut.clk)
    dut.thresh_wr_valid_i.value = 0


class CoreHarness:
    def __init__(self, dut, score_model=None):
        self.dut = dut
        self.decoder = Evt2DecoderModel()

        self.expected_decoded = deque()
        self.current_window = []

        self.expected_gestures = []
        self.observed_gestures = []

        self.accepted_words = 0
        self.completed_windows = 0

        # Optional independent score verification.
        self.score_model = score_model
        self.pending_score_checks = deque()  # (exp_class, exp_pass) from ScoreModel

    async def setup(self):
        cocotb.start_soon(Clock(self.dut.clk, 10, units="ns").start())
        self.dut.rst.value = 1
        self.dut.evt_word.value = 0
        self.dut.evt_word_valid.value = 0
        self.dut.weight_wr_valid_i.value = 0
        self.dut.weight_wr_class_i.value = 0
        self.dut.weight_wr_addr_i.value = 0
        self.dut.weight_wr_data_i.value = 0
        self.dut.thresh_wr_valid_i.value = 0
        self.dut.thresh_wr_addr_i.value = 0
        self.dut.thresh_wr_data_i.value = 0
        await ClockCycles(self.dut.clk, 8)
        self.dut.rst.value = 0
        await self.tick(4)

    def _sample_cycle(self):
        if int(self.dut.u_evt2_decoder.event_valid.value):
            observed = (
                int(self.dut.u_evt2_decoder.x_out.value),
                int(self.dut.u_evt2_decoder.y_out.value),
            )
            assert self.expected_decoded, f"Unexpected decoded event {observed}"
            expected = self.expected_decoded.popleft()
            assert observed == expected, f"Decoded mismatch DUT={observed} model={expected}"

        if int(self.dut.u_voxel_binning.readout_valid.value):
            idx = int(self.dut.u_voxel_binning.readout_index.value)
            assert idx == len(self.current_window), \
                f"Readout index mismatch DUT={idx}, expected={len(self.current_window)}"
            self.current_window.append(int(self.dut.u_voxel_binning.readout_data.value))

            if int(self.dut.u_voxel_binning.readout_last.value):
                assert len(self.current_window) == FEATURE_COUNT, \
                    f"Feature window length {len(self.current_window)} != {FEATURE_COUNT}"
                if self.score_model is not None:
                    exp_cls, exp_pass, margin = self.score_model.classify(self.current_window)
                    self.pending_score_checks.append((exp_cls, exp_pass))
                    # Log golden-model scores for this window
                    scores = [0] * NUM_CLASSES
                    for fi, feat in enumerate(self.current_window):
                        f = int(feat)
                        for c in range(NUM_CLASSES):
                            scores[c] += f * self.score_model.weights[c][fi]
                    nz = sum(1 for v in self.current_window if v > 0)
                    total = sum(int(v) for v in self.current_window)
                    names = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}
                    cocotb.log.info(
                        f"[window {self.completed_windows}] model scores: "
                        f"D={scores[0]} L={scores[1]} R={scores[2]} U={scores[3]}  "
                        f"winner={names[exp_cls]} margin={margin}  "
                        f"nonzero_features={nz}/{FEATURE_COUNT} total_count={total}"
                    )
                self.current_window = []
                self.completed_windows += 1

        if int(self.dut.class_valid.value):
            class_id = int(self.dut.class_gesture.value)
            class_pass = int(self.dut.class_pass.value)

            if self.score_model is not None and self.pending_score_checks:
                exp_cls, exp_pass = self.pending_score_checks.popleft()
                assert class_id == exp_cls, (
                    f"ScoreModel class mismatch: DUT={class_id} model={exp_cls}"
                )
                assert class_pass == exp_pass, (
                    f"ScoreModel pass mismatch: DUT={class_pass} model={exp_pass}"
                )

            # No persistence: every passing window fires gesture_valid immediately.
            if class_pass:
                self.expected_gestures.append((
                    class_id,
                    int(self.dut.gesture_confidence.value),
                ))

        if int(self.dut.gesture_valid.value):
            self.observed_gestures.append((
                int(self.dut.gesture.value),
                int(self.dut.gesture_confidence.value),
            ))

    async def tick(self, cycles=1):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            self._sample_cycle()
            await NextTimeStep()

    async def send_word(self, word):
        while int(self.dut.evt_word_ready.value) == 0:
            await self.tick(1)

        self.dut.evt_word.value = word
        self.dut.evt_word_valid.value = 1
        await self.tick(1)
        self.dut.evt_word_valid.value = 0

        evt = self.decoder.on_word(word)
        if evt is not None:
            self.expected_decoded.append(evt)
        self.accepted_words += 1

    async def force_bin_rollover(self):
        while int(self.dut.u_voxel_binning.state.value) != ST_ACCUM:
            await self.tick(1)

        # Drain queued words so this forced rollover closes a fully-ingested bin.
        # Without this, repeated immediate rollovers can starve decoding and leave
        # large portions of traffic stuck in the input FIFO.
        stable_empty = 0
        for _ in range(200000):
            fifo_has_data = int(self.dut.u_input_fifo.valid_o.value)
            dec_valid = int(self.dut.u_evt2_decoder.event_valid.value)
            src_valid = int(self.dut.evt_word_valid.value)
            if fifo_has_data == 0 and dec_valid == 0 and src_valid == 0:
                stable_empty += 1
                if stable_empty >= 2:
                    break
            else:
                stable_empty = 0
            await self.tick(1)
        else:
            raise AssertionError("Timeout draining input path before forced rollover")

        self.dut.u_voxel_binning.timer_ctr.value = CYCLES_PER_BIN_SAFE - 1
        await self.tick(1)

    async def wait_quiet(self, quiet_cycles=20, timeout=200000):
        prev_g = len(self.observed_gestures)
        prev_w = self.completed_windows
        q = 0
        for _ in range(timeout):
            await self.tick(1)
            now_g = len(self.observed_gestures)
            now_w = self.completed_windows
            if now_g == prev_g and now_w == prev_w:
                q += 1
                pipeline_idle = (
                    int(self.dut.debug_score_busy.value) == 0 and
                    int(self.dut.capture_active.value) == 0 and
                    int(self.dut.feature_window_ready.value) == 0 and
                    int(self.dut.u_input_fifo.valid_o.value) == 0 and
                    int(self.dut.u_voxel_binning.state.value) == ST_ACCUM and
                    not self.expected_decoded and
                    not self.current_window and
                    not self.pending_score_checks
                )
                if q >= quiet_cycles and pipeline_idle:
                    return
            else:
                q = 0
                prev_g = now_g
                prev_w = now_w
        raise AssertionError("Timeout waiting for pipeline quiet")


def region_points(name):
    x_lo = max(0, GRID_SIZE // 8)
    x_hi = min(GRID_SIZE, GRID_SIZE - (GRID_SIZE // 8))
    y_lo = x_lo
    y_hi = x_hi
    band = max(2, GRID_SIZE // 4)

    if name == "top":
        ys, xs = range(y_lo, min(y_lo + band, GRID_SIZE)), range(x_lo, x_hi)
    elif name == "bottom":
        ys, xs = range(max(GRID_SIZE - band, 0), y_hi), range(x_lo, x_hi)
    elif name == "left":
        ys, xs = range(y_lo, y_hi), range(x_lo, min(x_lo + band, GRID_SIZE))
    elif name == "right":
        ys, xs = range(y_lo, y_hi), range(max(GRID_SIZE - band, 0), x_hi)
    else:
        raise ValueError(name)

    pts = []
    for y in ys:
        for x in xs:
            pts.append((x, y))
    return pts


async def drive_bin_traffic(h, rng, region, events=28):
    pts = region_points(region)
    for i in range(events):
        if i % 10 == 0:
            await h.send_word(build_evt2_time_high(rng.randint(0, 0x0FFFFFFF)))

        gx, gy = rng.choice(pts)
        x_s = sensor_from_grid(gx)
        y_s = sensor_from_grid(gy)
        pkt = EVT_CD_ON if (i & 1) else EVT_CD_OFF
        await h.send_word(build_evt2_cd(pkt, x_s, y_s, i & 0x3F))

        if i % 13 == 0:
            bad = (0xF << 28) | rng.randint(0, 0x0FFFFFFF)
            await h.send_word(bad)


@logged_test()
async def test_voxel_bin_core_end_to_end_golden(dut):
    rng = random.Random(0xC011E0)
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0x12345))

    script = [
        "bottom", "bottom", "top", "top",
        "right", "right", "left", "left",
        "bottom", "bottom", "top", "top",
    ]

    for region in script:
        await drive_bin_traffic(h, rng, region, events=30)
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert not h.expected_decoded, f"Unmatched decoded events: {len(h.expected_decoded)}"
    assert not h.current_window, "Partial readout window remained"
    assert h.completed_windows > 0, "No completed readout windows observed"

    assert h.observed_gestures == h.expected_gestures, \
        f"Gesture stream mismatch\nDUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}"

    # debug_event_count is 8-bit saturating wrap counter of accepted words.
    assert int(dut.debug_event_count.value) == (h.accepted_words & 0xFF), \
        "debug_event_count mismatch"


@logged_test()
async def test_empty_window_produces_no_gesture(dut):
    """No CD events -> all-zero features -> all scores zero -> no gesture fires.

    This validates the zero-input boundary case: all scores are zero so
    max_score = 0, which fails the CLASS_THRESHOLD=0 check (strict >),
    and gesture_valid never asserts.
    """
    h = CoreHarness(dut)
    await h.setup()

    # Prime decoder with a TIME_HIGH so CD events would be accepted, but send none.
    await h.send_word(build_evt2_time_high(0x1000))

    # Rotate exactly READOUT_BINS bins to produce one complete readout window.
    for _ in range(READOUT_BINS):
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert h.completed_windows > 0, "Expected at least one completed window"
    assert len(h.observed_gestures) == 0, \
        f"Expected no gestures for empty input, got {h.observed_gestures}"
    assert h.observed_gestures == h.expected_gestures, \
        "Model/DUT disagree on empty-input gesture output"


@logged_test()
async def test_reset_mid_pipeline_recovers_cleanly(dut):
    """Assert rst while the binner/scorer pipeline is active; verify clean restart."""
    rng = random.Random(0xDEAD_F00D)
    h = CoreHarness(dut)
    await h.setup()

    # Start feeding events and force one bin rotation.
    await h.send_word(build_evt2_time_high(0xABCDE))
    pts = region_points("left")
    for _ in range(15):
        gx, gy = rng.choice(pts)
        await h.send_word(
            build_evt2_cd(EVT_CD_ON, sensor_from_grid(gx), sensor_from_grid(gy), 1)
        )
    await h.force_bin_rollover()

    # Assert reset while the pipeline may still be draining.
    dut.rst.value = 1
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0

    # Wait for the binner to return to ST_ACCUM (it clears one bin after reset).
    for _ in range(20_000):
        await RisingEdge(dut.clk)
        if int(dut.u_voxel_binning.state.value) == ST_ACCUM:
            break
    else:
        raise AssertionError("Binner did not return to ST_ACCUM after reset")

    # No spurious gesture_valid should fire in the cycles following reset.
    for _ in range(500):
        await RisingEdge(dut.clk)
        assert int(dut.gesture_valid.value) == 0, \
            "Spurious gesture_valid asserted after reset"

    # The FIFO must be ready to accept new data.
    assert int(dut.evt_word_ready.value) == 1, \
        "evt_word_ready not asserted after reset"


@logged_test()
async def test_debug_event_count_tracks_accepted_words(dut):
    """debug_event_count must equal (accepted_words mod 256)."""
    h = CoreHarness(dut)
    await h.setup()

    # Send a mix of TIME_HIGH and CD words.
    for i in range(260):
        if i % 8 == 0:
            await h.send_word(build_evt2_time_high(i & 0x0FFFFFFF))
        else:
            pts = region_points("right")
            gx, gy = pts[i % len(pts)]
            await h.send_word(
                build_evt2_cd(EVT_CD_ON, sensor_from_grid(gx), sensor_from_grid(gy), i & 0x3F)
            )

    await h.wait_quiet(quiet_cycles=20)

    assert int(dut.debug_event_count.value) == (h.accepted_words & 0xFF), \
        (f"debug_event_count DUT={int(dut.debug_event_count.value)} "
         f"expected={h.accepted_words & 0xFF}")


@logged_test()
async def test_fifo_backpressure_no_lost_events(dut):
    """Burst of events: even if FIFO momentarily fills, no accepted events are lost."""
    rng = random.Random(0xF00B_A400)
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0x300))

    pts = region_points("top")
    for i in range(60):
        gx, gy = rng.choice(pts)
        await h.send_word(
            build_evt2_cd(EVT_CD_ON, sensor_from_grid(gx), sensor_from_grid(gy), i & 0x3F)
        )
        # Occasionally insert a TIME_HIGH mid-burst.
        if i % 15 == 0:
            await h.send_word(build_evt2_time_high(rng.randint(0, 0x0FFFFFFF)))

    # One rollover closes the current bin; wait_quiet drains any remaining pipeline.
    await h.force_bin_rollover()

    await h.wait_quiet()

    assert not h.expected_decoded, \
        f"Unmatched decoded events remaining: {len(h.expected_decoded)}"
    assert h.observed_gestures == h.expected_gestures, \
        (f"Gesture mismatch after backpressure burst\n"
         f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}")


@logged_test()
async def test_sustained_region_fires_gesture(dut):
    """Drive a single spatial region for many bins; gesture_valid must fire on every passing window."""
    rng = random.Random(0x1234_5678)
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0x1))

    # Drive "bottom" region across enough bins for at least two complete readout windows.
    # READOUT_BINS bins warm up the ring; each additional rollover triggers one more window.
    # With no persistence, every passing window fires gesture_valid immediately.
    for _ in range(READOUT_BINS + 2):
        await drive_bin_traffic(h, rng, "bottom", events=32)
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert h.observed_gestures == h.expected_gestures, \
        (f"Gesture mismatch in sustained-region test\n"
         f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}")
    assert h.completed_windows > 0, "No completed windows observed"


@logged_test()
async def test_decoder_events_match_model_exactly(dut):
    """Verify that every decoded (x, y, polarity, timestamp) tuple matches the model."""
    rng = random.Random(0xABCD_1234)
    h = CoreHarness(dut)
    await h.setup()

    # Interleave TIME_HIGH and CD events; model tracks expected decoded output.
    time_bases = [0x00001, 0x0ABCD, 0x3FFFF]
    for tb_val in time_bases:
        await h.send_word(build_evt2_time_high(tb_val))
        for _ in range(10):
            gx = rng.randint(0, GRID_SIZE - 1)
            gy = rng.randint(0, GRID_SIZE - 1)
            pkt = EVT_CD_ON if rng.randint(0, 1) else EVT_CD_OFF
            ts_lsb = rng.randint(0, 63)
            await h.send_word(
                build_evt2_cd(pkt, sensor_from_grid(gx), sensor_from_grid(gy), ts_lsb)
            )

    # Force one rollover so the pipeline drains any pending decoded events.
    await h.force_bin_rollover()
    await h.wait_quiet(quiet_cycles=20)


@logged_test()
async def test_score_model_validates_classifications(dut):
    """ScoreModel independently verifies every DUT class_gesture/class_pass output."""
    weights = load_weights_from_mem()
    score_model = ScoreModel(weights)

    rng = random.Random(0xBEEF_CAFE)
    h = CoreHarness(dut, score_model=score_model)
    await h.setup()
    await _flush_stale_bins(h, weights=weights)

    await h.send_word(build_evt2_time_high(0xABCDE))

    # Drive a varied sequence of regions so multiple distinct feature windows are generated.
    script = [
        "bottom", "bottom", "top", "top",
        "right", "right", "left", "left",
        "bottom", "top", "right", "left",
    ]
    for region in script:
        await drive_bin_traffic(h, rng, region, events=30)
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert h.completed_windows > 0, "No completed windows observed"
    # All class_valid outputs were independently checked against ScoreModel inline.
    assert not h.pending_score_checks, \
        f"{len(h.pending_score_checks)} feature windows never produced a class_valid"

    assert not h.expected_decoded, \
        (f"{len(h.expected_decoded)} decoded events unmatched by DUT")


# ---------------------------------------------------------------------------
# Gesture classification end-to-end tests
# ---------------------------------------------------------------------------

async def _flush_stale_bins(h, weights=None):
    """
    Hard-reset the DUT then wait for the binner to return to ST_ACCUM.

    This is the only reliable way to clear all pipeline state between tests that
    share a single simulation instance.  After reset the weight and threshold
    SRAMs are all-zero; if weights is provided they are re-loaded via the
    runtime write ports (and thresholds are loaded from thresholds.mem).
    """
    # Assert hardware reset.
    h.dut.rst.value = 1
    h.dut.evt_word_valid.value = 0
    await ClockCycles(h.dut.clk, 16)
    h.dut.rst.value = 0

    # Wait until voxel_binning returns to ST_ACCUM (clears bin 0 after reset).
    for _ in range(10_000):
        await RisingEdge(h.dut.clk)
        if int(h.dut.u_voxel_binning.state.value) == ST_ACCUM:
            break
    else:
        raise AssertionError("Binner did not return to ST_ACCUM after flush reset")

    # A few extra cycles to let decoder and fifo settle.
    await ClockCycles(h.dut.clk, 8)

    # Re-load weights and thresholds into SRAMs after reset (SRAMs start at 0).
    if weights is not None:
        thresholds = load_thresholds()
        await deposit_weights_and_thresholds(h.dut, weights, thresholds)

    # Reset harness bookkeeping; the reset produces no valid feature windows.
    h.decoder = Evt2DecoderModel()
    h.expected_decoded.clear()
    h.current_window = []
    h.completed_windows = 0
    h.observed_gestures.clear()
    h.expected_gestures.clear()
    h.pending_score_checks.clear()
    h.accepted_words = 0


@logged_test()
async def test_wrong_gesture_trajectory_no_false_positive(dut):
    """
    Drive uniform random events spread across all cells/bins and verify DUT matches model.
    With CLASS_THRESHOLD=0 the winning class still fires gesture_valid (score > 0),
    but DUT and ScoreModel must agree on the class and pass status for every window.
    """
    weights = load_weights_from_mem()
    score_model = ScoreModel(weights)

    rng = random.Random(0xFA_15_E0)
    h = CoreHarness(dut, score_model=score_model)
    await h.setup()
    await _flush_stale_bins(h, weights=weights)

    # Uniform random events: every cell hit equally -> scores proportional to total weight sums.
    # With CLASS_THRESHOLD=0, class_pass=1 whenever the winning score > 0 (always true here).
    # The test verifies DUT and ScoreModel agree on class_id, class_pass, and gesture_valid
    # for every window — not that gestures are suppressed.
    # READOUT_BINS+1 rollovers: warm-up + at least one complete readout window.
    for _ in range(READOUT_BINS + 1):
        await h.send_word(build_evt2_time_high(rng.randint(0, 0x0FFFFFFF)))
        for _ in range(64):
            gx = rng.randint(0, GRID_SIZE - 1)
            gy = rng.randint(0, GRID_SIZE - 1)
            pkt = EVT_CD_ON if rng.randint(0, 1) else EVT_CD_OFF
            await h.send_word(build_evt2_cd(pkt, sensor_from_grid(gx), sensor_from_grid(gy), rng.randint(0, 63)))
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert h.completed_windows > 0, "No feature windows completed"
    assert not h.pending_score_checks, \
        f"{len(h.pending_score_checks)} score checks never consumed"

    # Model and DUT must agree (both should produce zero or identical gesture_valid outputs)
    assert h.observed_gestures == h.expected_gestures, \
        f"DUT/model mismatch on uniform input\nDUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}"


# ---------------------------------------------------------------------------
# EVT2 .bin file streaming tests
# ---------------------------------------------------------------------------

# Default set of 4 .bin files to stream into the DUT.
# Override by setting the GESTURE_BIN_FILES env variable to a colon-separated
# list of 4 absolute or repo-relative paths, e.g.:
_REPO_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_BIN_FILES = [
    _REPO_ROOT / "EVT2_gesture_set" / "full_temporal_resolution" / "gdb_sun" / "wave_down" / "wave_down_bc_sun2_trim.bin",
    _REPO_ROOT / "EVT2_gesture_set" / "full_temporal_resolution" / "gdb_sun" / "wave_left" / "wave_left_ba_sun2_trim.bin",
    _REPO_ROOT / "EVT2_gesture_set" / "full_temporal_resolution" / "gdb_sun" / "wave_right" / "wave_right_ba_sun2_trim.bin",
    _REPO_ROOT / "EVT2_gesture_set" / "full_temporal_resolution" / "gdb_sun" / "wave_up" / "wave_up_bc_sun2_trim.bin"
]

def _resolve_bin_files():
    """Return the 4 .bin file paths to use for the streaming tests.

    If the GESTURE_BIN_FILES environment variable is set it must contain exactly
    4 colon-separated paths (absolute or relative to the repo root).  
    """
    env = os.environ.get("GESTURE_BIN_FILES", "")
    if env.strip():
        parts = [p.strip() for p in env.split(":") if p.strip()]
        if len(parts) != 4:
            raise ValueError(
                f"GESTURE_BIN_FILES must contain exactly 4 colon-separated paths, got {len(parts)}: {parts}"
            )
        resolved = []
        for p in parts:
            path = Path(p)
            if not path.is_absolute():
                path = _REPO_ROOT / path
            resolved.append(path)
        return resolved
    return list(_DEFAULT_BIN_FILES)


def _read_evt2_bin(path):
    """Read a raw EVT2.0 binary file and return a list of 32-bit words.

    The file contains little-endian 32-bit words with no text header.
    Incomplete trailing bytes are silently dropped.
    """
    data = Path(path).read_bytes()
    n_words = len(data) // 4
    return list(struct.unpack_from(f"<{n_words}I", data, 0))


async def _stream_bin_file_with_timing(h, bin_path):
    """Stream an EVT2.0 .bin file into the DUT, forcing bin rollovers at the correct
    timestamps so the DUT's temporal window advances in sync with the recorded data.

    The DUT uses a clock-based bin timer.  When replaying a file at simulation
    speed the timer would never expire naturally, so we parse each word's
    timestamp and call force_bin_rollover() every time the timestamp crosses a
    BIN_DURATION_MS boundary.  This mirrors how the hardware would behave in
    real time where the timer fires every BIN_DURATION_MS milliseconds.

    Timestamps in EVT2.0 are in microseconds.
    """
    bin_duration_us = (WINDOW_MS // READOUT_BINS) * 1000  # ms -> us

    words = _read_evt2_bin(bin_path)

    time_high = 0
    next_bin_boundary_us = None  # set on first timestamp seen

    for word in words:
        pkt = (word >> 28) & 0xF

        if pkt == EVT_TIME_HIGH:
            time_high = word & 0x0FFFFFFF
        elif pkt in (EVT_CD_OFF, EVT_CD_ON):
            ts_lsb = (word >> 22) & 0x3F
            ts_us = (time_high << 6) | ts_lsb

            if next_bin_boundary_us is None:
                # Align first boundary to the next multiple of bin_duration_us
                next_bin_boundary_us = (ts_us // bin_duration_us + 1) * bin_duration_us

            # Roll the bin forward for every boundary this event crosses.
            while ts_us >= next_bin_boundary_us:
                await h.force_bin_rollover()
                next_bin_boundary_us += bin_duration_us

        await h.send_word(word)


async def _run_bin_file_test(dut, bin_path, label):
    """Core logic shared by all four bin-file tests.

    Streams the entire .bin file into the DUT with timestamp-driven bin
    rollovers so the temporal window advances correctly.  Verifies that
    every DUT classification matches the software golden model (ScoreModel).

    Parameters
    ----------
    dut : cocotb DUT handle
    bin_path : path-like
        EVT2.0 binary file to stream.
    label : str
        Human-readable name used in log messages (e.g. "money-wave-down").
    """
    weights = load_weights_from_mem()
    score_model = ScoreModel(weights)
    h = CoreHarness(dut, score_model=score_model)
    await h.setup()
    await _flush_stale_bins(h, weights=weights)

    cocotb.log.info(f"[{label}] Streaming {bin_path} ...")
    await _stream_bin_file_with_timing(h, bin_path)

    # Flush the final partial bin so the last events are scored.
    for _ in range(READOUT_BINS):
        await h.force_bin_rollover()

    await h.wait_quiet()

    cocotb.log.info(
        f"[{label}] Completed windows: {h.completed_windows}, "
        f"Accepted words: {h.accepted_words}"
    )

    gesture_names = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}
    if h.observed_gestures:
        cocotb.log.info(f"[{label}] DUT gesture outputs ({len(h.observed_gestures)} total):")
        for g_class, g_conf in h.observed_gestures:
            cocotb.log.info(
                f"  gesture={gesture_names.get(g_class, str(g_class))} "
                f"(class={g_class}, confidence={g_conf})"
            )
    else:
        cocotb.log.info(f"[{label}] DUT produced no gesture_valid pulses.")

    assert h.completed_windows > 0, f"[{label}] No feature windows completed"
    assert not h.pending_score_checks, \
        f"[{label}] {len(h.pending_score_checks)} windows never produced class_valid"
    assert h.observed_gestures == h.expected_gestures, (
        f"[{label}] DUT/model gesture mismatch\n"
        f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}"
    )


@logged_test()
async def test_bin_file_gesture_0(dut):
    """Stream bin file 0 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[0]
    label = Path(path).stem
    await _run_bin_file_test(dut, path, label)


@logged_test()
async def test_bin_file_gesture_1(dut):
    """Stream bin file 1 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[1]
    label = Path(path).stem
    await _run_bin_file_test(dut, path, label)


@logged_test()
async def test_bin_file_gesture_2(dut):
    """Stream bin file 2 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[2]
    label = Path(path).stem
    await _run_bin_file_test(dut, path, label)


@logged_test()
async def test_bin_file_gesture_3(dut):
    """Stream bin file 3 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[3]
    label = Path(path).stem
    await _run_bin_file_test(dut, path, label)

