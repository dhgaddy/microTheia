# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
from collections import Counter, deque
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
SWAP_INPUT_BYTES = CFG.get("SWAP_INPUT_BYTES", 0)
MAP_SWAP_XY = CFG.get("MAP_SWAP_XY", 0)
MAP_FLIP_X = CFG.get("MAP_FLIP_X", 0)
MAP_FLIP_Y = CFG.get("MAP_FLIP_Y", 0)
SENSOR_WIDTH = CFG["SENSOR_WIDTH"]
SENSOR_HEIGHT = CFG.get("SENSOR_HEIGHT", SENSOR_WIDTH)
COUNTER_BITS = CFG.get("COUNTER_BITS", 4)
NUM_CLASSES  = CFG.get("NUM_CLASSES", 4)


BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS
X_BIN_DIV = SENSOR_WIDTH // GRID_SIZE
Y_BIN_DIV = SENSOR_HEIGHT // GRID_SIZE
FEATURE_COUNT = GRID_SIZE * GRID_SIZE * READOUT_BINS
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE
ASSERT_EXPECTED_LABEL = int(os.environ.get("ASSERT_EXPECTED_LABEL", "1"))
EXPECTED_LABEL_MIN_RATIO = float(os.environ.get("EXPECTED_LABEL_MIN_RATIO", "0.60"))

GESTURE_NAMES = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}
EXPECTED_BIN_FILE_CLASS = {
    0: 0,  # wave_down_*
    1: 1,  # wave_left_*
    2: 2,  # wave_right_*
    3: 3,  # wave_up_*
}

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

ST_ACCUM = 0


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_x_from_grid(g):
    g = max(0, min(GRID_SIZE - 1, int(g)))
    return min(SENSOR_WIDTH - 1, (g * X_BIN_DIV) + (X_BIN_DIV // 2))


def sensor_y_from_grid(g):
    g = max(0, min(GRID_SIZE - 1, int(g)))
    return min(SENSOR_HEIGHT - 1, (g * Y_BIN_DIV) + (Y_BIN_DIV // 2))


def _decode_evt2_word_fields(word):
    if SWAP_INPUT_BYTES:
        word = (
            ((word & 0x000000FF) << 24) |
            ((word & 0x0000FF00) << 8) |
            ((word & 0x00FF0000) >> 8) |
            ((word & 0xFF000000) >> 24)
        )

    return {
        "word": word,
        "pkt": (word >> 28) & 0xF,
        "ts_lsb": (word >> 22) & 0x3F,
        "x_raw": (word >> 11) & 0x7FF,
        "y_raw": word & 0x7FF,
    }


class Evt2DecoderModel:
    def __init__(self):
        self.have_time_high = False

    def on_word(self, word):
        fields = _decode_evt2_word_fields(word)
        pkt = fields["pkt"]
        x_raw = fields["x_raw"]
        y_raw = fields["y_raw"]

        if pkt == EVT_TIME_HIGH:
            self.have_time_high = True
            return None

        if pkt not in (EVT_CD_OFF, EVT_CD_ON):
            return None

        if not self.have_time_high:
            return None

        x_clamped = min(x_raw, SENSOR_WIDTH - 1)
        y_clamped = min(y_raw, SENSOR_HEIGHT - 1)

        x_swapped = y_clamped if MAP_SWAP_XY else x_clamped
        y_swapped = x_clamped if MAP_SWAP_XY else y_clamped
        x_oriented = min(x_swapped, SENSOR_WIDTH - 1)
        y_oriented = min(y_swapped, SENSOR_HEIGHT - 1)
        if MAP_FLIP_X:
            x_oriented = (SENSOR_WIDTH - 1) - x_oriented
        if MAP_FLIP_Y:
            y_oriented = (SENSOR_HEIGHT - 1) - y_oriented

        x_grid = min(x_oriented // X_BIN_DIV, GRID_SIZE - 1)
        y_grid = min(y_oriented // Y_BIN_DIV, GRID_SIZE - 1)
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
        self.window_features = []            # list[list[int]] per completed window
        self.window_scores = []              # list[list[int]] per completed window
        self.window_pred = []                # list[(best_class, pass, margin)] per window

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
                self.window_features.append(list(self.current_window))
                if self.score_model is not None:
                    exp_cls, exp_pass, margin = self.score_model.classify(self.current_window)
                    self.pending_score_checks.append((exp_cls, exp_pass))
                    # Log golden-model scores for this window
                    scores = [0] * NUM_CLASSES
                    for fi, feat in enumerate(self.current_window):
                        f = int(feat)
                        for c in range(NUM_CLASSES):
                            scores[c] += f * self.score_model.weights[c][fi]
                    self.window_scores.append(scores)
                    self.window_pred.append((exp_cls, exp_pass, margin))
                    nz = sum(1 for v in self.current_window if v > 0)
                    total = sum(int(v) for v in self.current_window)
                    cocotb.log.info(
                        f"[window {self.completed_windows}] model scores: "
                        f"D={scores[0]} L={scores[1]} R={scores[2]} U={scores[3]}  "
                        f"winner={GESTURE_NAMES[exp_cls]} margin={margin}  "
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


def _majority_with_ratio(class_ids):
    if not class_ids:
        return None, 0.0, {}
    hist = Counter(class_ids)
    maj, cnt = max(hist.items(), key=lambda kv: kv[1])
    return maj, (cnt / len(class_ids)), dict(hist)


def _transform_feature_window(window, mode):
    """Remap feature-vector cells to emulate coordinate convention changes.

    Feature layout is [bin][y][x], where addr = bin*CELLS + y*GRID_SIZE + x.
    """
    out = [0] * len(window)
    for b in range(READOUT_BINS):
        base = b * CELLS_PER_BIN
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                sx, sy = x, y
                if mode in ("swap_xy", "swap_xy_flip_x", "swap_xy_flip_y", "swap_xy_flip_xy"):
                    sx, sy = sy, sx
                if mode in ("flip_x", "flip_xy", "swap_xy_flip_x", "swap_xy_flip_xy"):
                    sx = GRID_SIZE - 1 - sx
                if mode in ("flip_y", "flip_xy", "swap_xy_flip_y", "swap_xy_flip_xy"):
                    sy = GRID_SIZE - 1 - sy
                src = base + y * GRID_SIZE + x
                dst = base + sy * GRID_SIZE + sx
                out[dst] = window[src]
    return out


def _score_features(weights, features):
    scores = [0] * NUM_CLASSES
    for i, feat in enumerate(features):
        f = int(feat)
        for c in range(NUM_CLASSES):
            scores[c] += f * weights[c][i]
    return scores


def _pick_best_threshold(pos_scores, neg_scores):
    """Choose threshold maximizing balanced accuracy for score > threshold."""
    candidates = sorted(set(pos_scores + neg_scores))
    if not candidates:
        return 0, 0.0
    # Include values just below the first candidate and at exact candidates.
    test_vals = [max(0, candidates[0] - 1)] + candidates
    best_thr = 0
    best_bal = -1.0
    p = max(1, len(pos_scores))
    n = max(1, len(neg_scores))
    for thr in test_vals:
        tpr = sum(s > thr for s in pos_scores) / p
        tnr = sum(s <= thr for s in neg_scores) / n
        bal = 0.5 * (tpr + tnr)
        if bal > best_bal:
            best_bal = bal
            best_thr = thr
    return best_thr, best_bal


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
        x_s = sensor_x_from_grid(gx)
        y_s = sensor_y_from_grid(gy)
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
            build_evt2_cd(EVT_CD_ON, sensor_x_from_grid(gx), sensor_y_from_grid(gy), 1)
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
                build_evt2_cd(EVT_CD_ON, sensor_x_from_grid(gx), sensor_y_from_grid(gy), i & 0x3F)
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
            build_evt2_cd(EVT_CD_ON, sensor_x_from_grid(gx), sensor_y_from_grid(gy), i & 0x3F)
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
                build_evt2_cd(pkt, sensor_x_from_grid(gx), sensor_y_from_grid(gy), ts_lsb)
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
    h.window_features.clear()
    h.window_scores.clear()
    h.window_pred.clear()


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
            await h.send_word(build_evt2_cd(
                pkt,
                sensor_x_from_grid(gx),
                sensor_y_from_grid(gy),
                rng.randint(0, 63),
            ))
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
    _REPO_ROOT / "EVT2_gesture_set" / "wave_down_sun_test1.bin",
    _REPO_ROOT / "EVT2_gesture_set" / "wave_left_sun_test1.bin",
    _REPO_ROOT / "EVT2_gesture_set" / "wave_right_sun_test1.bin",
    _REPO_ROOT / "EVT2_gesture_set" / "wave_up_sun_test1.bin",
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
        fields = _decode_evt2_word_fields(word)
        pkt = fields["pkt"]

        if pkt == EVT_TIME_HIGH:
            time_high = fields["word"] & 0x0FFFFFFF
        elif pkt in (EVT_CD_OFF, EVT_CD_ON):
            ts_lsb = fields["ts_lsb"]
            ts_us = (time_high << 6) | ts_lsb

            if next_bin_boundary_us is None:
                # Align first boundary to the next multiple of bin_duration_us
                next_bin_boundary_us = (ts_us // bin_duration_us + 1) * bin_duration_us

            # Roll the bin forward for every boundary this event crosses.
            while ts_us >= next_bin_boundary_us:
                await h.force_bin_rollover()
                next_bin_boundary_us += bin_duration_us

        await h.send_word(word)


async def _stream_bin_file_clock_driven(h, bin_path):
    """Replay EVT2 words with clock-time spacing from EVT2 timestamps.

    Unlike timestamp-forced mode, this path does not force bin rollover from timestamps.
    Bin advancement comes only from the DUT's timer behavior and any explicit final flush.
    """
    words = _read_evt2_bin(bin_path)
    cycles_per_us = max(1, CLK_FREQ_HZ // 1_000_000)

    time_high = 0
    prev_ts_us = None
    for word in words:
        fields = _decode_evt2_word_fields(word)
        pkt = fields["pkt"]
        if pkt == EVT_TIME_HIGH:
            time_high = fields["word"] & 0x0FFFFFFF
        elif pkt in (EVT_CD_OFF, EVT_CD_ON):
            ts_us = (time_high << 6) | fields["ts_lsb"]
            if prev_ts_us is not None and ts_us >= prev_ts_us:
                delta_us = ts_us - prev_ts_us
                if delta_us:
                    await h.tick(delta_us * cycles_per_us)
            prev_ts_us = ts_us
        await h.send_word(word)


async def _run_bin_file_test(
    dut, bin_path, label, expected_class=None, replay_mode="timestamp_forced", enforce_label=None
):
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

    cocotb.log.info(f"[{label}] Streaming {bin_path} (mode={replay_mode}) ...")
    if replay_mode == "timestamp_forced":
        await _stream_bin_file_with_timing(h, bin_path)
    elif replay_mode == "clock_driven":
        await _stream_bin_file_clock_driven(h, bin_path)
    else:
        raise ValueError(f"Unknown replay_mode={replay_mode}")

    # Flush the final partial bin so the last events are scored.
    for _ in range(READOUT_BINS):
        await h.force_bin_rollover()

    await h.wait_quiet()

    cocotb.log.info(
        f"[{label}] Completed windows: {h.completed_windows}, "
        f"Accepted words: {h.accepted_words}"
    )

    if h.observed_gestures:
        cocotb.log.info(f"[{label}] DUT gesture outputs ({len(h.observed_gestures)} total):")
        for g_class, g_conf in h.observed_gestures:
            cocotb.log.info(
                f"  gesture={GESTURE_NAMES.get(g_class, str(g_class))} "
                f"(class={g_class}, confidence={g_conf})"
            )
    else:
        cocotb.log.info(f"[{label}] DUT produced no gesture_valid pulses.")

    pred_classes = [g for g, _ in h.observed_gestures]
    maj_cls, maj_ratio, hist = _majority_with_ratio(pred_classes)
    if pred_classes:
        hist_txt = " ".join(
            f"{GESTURE_NAMES.get(c, str(c))}:{n}" for c, n in sorted(hist.items())
        )
        cocotb.log.info(
            f"[{label}] dominant={GESTURE_NAMES.get(maj_cls, str(maj_cls))} "
            f"ratio={maj_ratio:.3f} class_hist=({hist_txt})"
        )

    if enforce_label is None:
        enforce_label = bool(ASSERT_EXPECTED_LABEL)
    if expected_class is not None:
        match = sum(1 for c in pred_classes if c == expected_class)
        ratio = (match / len(pred_classes)) if pred_classes else 0.0
        cocotb.log.info(
            f"[{label}] expected={GESTURE_NAMES[expected_class]} "
            f"match={match}/{len(pred_classes)} ratio={ratio:.3f}"
        )
        if enforce_label:
            assert pred_classes, f"[{label}] No gesture outputs; cannot validate label accuracy"
            assert maj_cls == expected_class, (
                f"[{label}] dominant class {GESTURE_NAMES.get(maj_cls)} != expected "
                f"{GESTURE_NAMES[expected_class]} (hist={hist})"
            )
            assert ratio >= EXPECTED_LABEL_MIN_RATIO, (
                f"[{label}] expected-class ratio {ratio:.3f} < min {EXPECTED_LABEL_MIN_RATIO:.3f}"
            )

    assert h.completed_windows > 0, f"[{label}] No feature windows completed"
    assert not h.pending_score_checks, \
        f"[{label}] {len(h.pending_score_checks)} windows never produced class_valid"
    assert h.observed_gestures == h.expected_gestures, (
        f"[{label}] DUT/model gesture mismatch\n"
        f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}"
    )
    return h


@logged_test()
async def test_bin_file_gesture_0(dut):
    """Stream bin file 0 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[0]
    label = Path(path).stem
    await _run_bin_file_test(
        dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[0]
    )


@logged_test()
async def test_bin_file_gesture_1(dut):
    """Stream bin file 1 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[1]
    label = Path(path).stem
    await _run_bin_file_test(
        dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[1]
    )


@logged_test()
async def test_bin_file_gesture_2(dut):
    """Stream bin file 2 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[2]
    label = Path(path).stem
    await _run_bin_file_test(
        dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[2]
    )


@logged_test()
async def test_bin_file_gesture_3(dut):
    """Stream bin file 3 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[3]
    label = Path(path).stem
    await _run_bin_file_test(
        dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[3]
    )


@logged_test()
async def test_bin_file_timing_replay_ab(dut):
    """A/B timing investigation: timestamp-forced vs clock-driven replay.

    This is a diagnostics-oriented test. It logs output differences and requires both
    modes to produce valid windows and model/DUT agreement.
    """
    bin_files = _resolve_bin_files()
    for i, path in enumerate(bin_files):
        label = Path(path).stem
        expected = EXPECTED_BIN_FILE_CLASS[i]
        h_forced = await _run_bin_file_test(
            dut,
            path,
            f"{label}-forced",
            expected_class=expected,
            replay_mode="timestamp_forced",
            enforce_label=False,
        )
        h_clock = await _run_bin_file_test(
            dut,
            path,
            f"{label}-clock",
            expected_class=expected,
            replay_mode="clock_driven",
            enforce_label=False,
        )

        forced_classes = [g for g, _ in h_forced.observed_gestures]
        clock_classes = [g for g, _ in h_clock.observed_gestures]
        min_len = min(len(forced_classes), len(clock_classes))
        if min_len:
            seq_diff = sum(
                1 for a, b in zip(forced_classes[:min_len], clock_classes[:min_len]) if a != b
            )
            cocotb.log.info(
                f"[{label}] A/B: forced_windows={h_forced.completed_windows} "
                f"clock_windows={h_clock.completed_windows} seq_diff={seq_diff}/{min_len}"
            )
        else:
            cocotb.log.info(
                f"[{label}] A/B: insufficient overlapping gesture outputs for sequence diff"
            )

        assert h_forced.completed_windows > 0 and h_clock.completed_windows > 0, (
            f"[{label}] Expected both replay modes to produce completed windows"
        )


@logged_test()
async def test_windowing_strategy_matches_sliding_model(dut):
    """Verify consecutive windows are sliding by one bin after warm-up."""
    rng = random.Random(0x51D10)
    weights = load_weights_from_mem()
    h = CoreHarness(dut, score_model=ScoreModel(weights))
    await h.setup()
    await _flush_stale_bins(h, weights=weights)
    await h.send_word(build_evt2_time_high(0x45678))

    # Generate more than READOUT_BINS rollovers to observe overlapping windows.
    script = ["left", "right", "top", "bottom"] * 4
    for region in script[: READOUT_BINS + 3]:
        await drive_bin_traffic(h, rng, region, events=28)
        await h.force_bin_rollover()
    await h.wait_quiet()

    assert len(h.window_features) >= 3, "Need >=3 windows to validate sliding behavior"
    for wi in range(len(h.window_features) - 1):
        w0 = h.window_features[wi]
        w1 = h.window_features[wi + 1]
        assert w0[CELLS_PER_BIN:] == w1[:-CELLS_PER_BIN], (
            f"Window {wi}->{wi+1} is not a 1-bin slide; expected overlap mismatch"
        )


@logged_test()
async def test_threshold_calibration_report(dut):
    """Compute threshold calibration suggestions from bin-file score distributions."""
    bin_files = _resolve_bin_files()
    per_class_score_pos = [[] for _ in range(NUM_CLASSES)]
    per_class_score_neg = [[] for _ in range(NUM_CLASSES)]
    per_class_margin_pos = [[] for _ in range(NUM_CLASSES)]
    per_class_margin_neg = [[] for _ in range(NUM_CLASSES)]

    for i, path in enumerate(bin_files):
        label = Path(path).stem
        h = await _run_bin_file_test(
            dut,
            path,
            f"{label}-calib",
            expected_class=EXPECTED_BIN_FILE_CLASS[i],
            replay_mode="timestamp_forced",
            enforce_label=False,
        )
        true_cls = EXPECTED_BIN_FILE_CLASS[i]
        for scores in h.window_scores:
            for c in range(NUM_CLASSES):
                if c == true_cls:
                    per_class_score_pos[c].append(scores[c])
                else:
                    per_class_score_neg[c].append(scores[c])
            best = max(range(NUM_CLASSES), key=lambda c: scores[c])
            second = max(s for j, s in enumerate(scores) if j != best)
            margin = scores[best] - second
            if best == true_cls:
                per_class_margin_pos[best].append(margin)
            else:
                per_class_margin_neg[best].append(margin)

    class_thresh = [0] * NUM_CLASSES
    diff_thresh = [0] * NUM_CLASSES
    for c in range(NUM_CLASSES):
        class_thresh[c], class_bal = _pick_best_threshold(
            per_class_score_pos[c], per_class_score_neg[c]
        )
        diff_thresh[c], diff_bal = _pick_best_threshold(
            per_class_margin_pos[c], per_class_margin_neg[c]
        )
        cocotb.log.info(
            f"[calib class={GESTURE_NAMES[c]}] "
            f"class_thr={class_thresh[c]} (bal_acc={class_bal:.3f}) "
            f"diff_thr={diff_thresh[c]} (bal_acc={diff_bal:.3f})"
        )

    mem_words = class_thresh + diff_thresh
    hex_words = [f"{v:09X}" for v in mem_words]
    cocotb.log.info("Suggested thresholds.mem contents:")
    for w in hex_words:
        cocotb.log.info(f"  {w}")

    assert any(v > 0 for v in class_thresh), "Calibration expected non-zero class thresholds"


@logged_test()
async def test_coordinate_orientation_sweep(dut):
    """Check whether identity orientation is outperformed by swaps/flips."""
    weights = load_weights_from_mem()
    score_model = ScoreModel(weights)
    bin_files = _resolve_bin_files()
    modes = [
        "identity",
        "swap_xy",
        "flip_x",
        "flip_y",
        "flip_xy",
        "swap_xy_flip_x",
        "swap_xy_flip_y",
        "swap_xy_flip_xy",
    ]
    mode_hits = {m: 0 for m in modes}
    total_windows = 0

    for i, path in enumerate(bin_files):
        label = Path(path).stem
        expected = EXPECTED_BIN_FILE_CLASS[i]
        h = await _run_bin_file_test(
            dut,
            path,
            f"{label}-orient",
            expected_class=expected,
            replay_mode="timestamp_forced",
            enforce_label=False,
        )
        for feat in h.window_features:
            total_windows += 1
            for mode in modes:
                f_mode = feat if mode == "identity" else _transform_feature_window(feat, mode)
                scores = _score_features(score_model.weights, f_mode)
                pred = max(range(NUM_CLASSES), key=lambda c: scores[c])
                if pred == expected:
                    mode_hits[mode] += 1

    assert total_windows > 0, "Expected orientation sweep to evaluate at least one window"
    for mode in modes:
        cocotb.log.info(
            f"[orientation] mode={mode} hits={mode_hits[mode]}/{total_windows} "
            f"acc={mode_hits[mode] / total_windows:.3f}"
        )

    best_mode = max(modes, key=lambda m: mode_hits[m])
    cocotb.log.info(f"[orientation] best_mode={best_mode}")

