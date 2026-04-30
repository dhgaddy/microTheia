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

CLK_FREQ_HZ      = CFG["CLK_FREQ_HZ"]
WINDOW_MS        = CFG["WINDOW_MS"]
GRID_SIZE        = CFG["GRID_SIZE"]
NUM_BINS         = CFG["NUM_BINS"]
READOUT_BINS     = CFG["READOUT_BINS"]
WEIGHT_BITS      = CFG["WEIGHT_BITS"]
WEIGHT_SCALE     = CFG["WEIGHT_SCALE"]
SWAP_INPUT_BYTES = CFG.get("SWAP_INPUT_BYTES", 0)
MAP_SWAP_XY      = CFG.get("MAP_SWAP_XY", 0)
MAP_FLIP_X       = CFG.get("MAP_FLIP_X", 0)
MAP_FLIP_Y       = CFG.get("MAP_FLIP_Y", 0)
SENSOR_WIDTH     = CFG["SENSOR_WIDTH"]
SENSOR_HEIGHT    = CFG.get("SENSOR_HEIGHT", SENSOR_WIDTH)
COUNTER_BITS     = CFG.get("COUNTER_BITS", 4)
NUM_CLASSES      = CFG.get("NUM_CLASSES", 4)

BIN_DURATION_MS          = WINDOW_MS // READOUT_BINS
BIN_DURATION_US          = BIN_DURATION_MS * 1000
X_BIN_DIV                = SENSOR_WIDTH // GRID_SIZE
Y_BIN_DIV                = SENSOR_HEIGHT // GRID_SIZE
DIV_K                    = 12
X_GRID_M                 = (1 << DIV_K) // X_BIN_DIV + 1
Y_GRID_M                 = (1 << DIV_K) // Y_BIN_DIV + 1
FEATURE_COUNT            = GRID_SIZE * GRID_SIZE * READOUT_BINS
CELLS_PER_BIN            = GRID_SIZE * GRID_SIZE
TOTAL_CELLS              = NUM_BINS * CELLS_PER_BIN
MAX_COUNTER              = (1 << COUNTER_BITS) - 1
ASSERT_EXPECTED_LABEL    = int(os.environ.get("ASSERT_EXPECTED_LABEL", "1"))
EXPECTED_LABEL_MIN_RATIO = float(os.environ.get("EXPECTED_LABEL_MIN_RATIO", "0.60"))

GESTURE_NAMES = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}
EXPECTED_BIN_FILE_CLASS = {
    0: 0,  # wave_down_*
    1: 1,  # wave_left_*
    2: 2,  # wave_right_*
    3: 3,  # wave_up_*
}

EVT_CD_OFF    = 0x0
EVT_CD_ON     = 0x1
EVT_TIME_HIGH = 0x8

ST_ACCUM = 0

# FSM state encodings (mirror RTL typedef)
ST_BOOT  = 0
ST_LOAD  = 1
ST_RUN   = 2
ST_DEBUG = 3

# Load sub-state encodings
LD_IDLE     = 0
LD_WAIT_PWR = 1
LD_OPEN     = 2
LD_WAIT     = 3
LD_DONE     = 4
LD_FAIL     = 5

# Must match control_fsm parameter PWR_WAIT_CYCLES
PWR_WAIT_CYCLES = 1024

# Special control words (pkt_type in bits [31:28])
BOOT_REQ_WORD       = 0xC0000000
RELOAD_REQ_WORD     = 0xB0000000
DEBUG_REQ_WORD      = 0xA0000000
EVT_READS_DONE_WORD = 0xF0000000


# ---------------------------------------------------------------------------
# EVT2 word builders
# ---------------------------------------------------------------------------

def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def build_weight_word(weight_data, weight_addr, sram_addr):
    """EVT_WEIGHT = 0x2: [31:28]=type [27:20]=data [19:9]=addr [8:7]=sram_sel"""
    return (0x2 << 28) | ((weight_data & 0xFF) << 20) | \
           ((weight_addr & 0x7FF) << 9) | ((sram_addr & 0x3) << 7)


def build_thresh_upper_word(upper18):
    """EVT_THRESH_U = 0x3: [31:28]=type [27:10]=upper 18 bits of threshold"""
    return (0x3 << 28) | ((upper18 & 0x3FFFF) << 10)


def build_thresh_lower_word(lower18, addr):
    """EVT_THRESH_L = 0x4: [31:28]=type [27:10]=lower 18 bits [9:7]=thresh_addr"""
    return (0x4 << 28) | ((lower18 & 0x3FFFF) << 10) | ((addr & 0x7) << 7)


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


# ---------------------------------------------------------------------------
# FSM boot helpers (bypass harness bookkeeping — send directly to DUT ports)
# ---------------------------------------------------------------------------

async def _send_raw_word(dut, word):
    """Push one word into the FIFO, respecting backpressure."""
    while int(dut.evt_word_ready.value) == 0:
        await RisingEdge(dut.clk)
    dut.evt_word.value = word
    dut.evt_word_valid.value = 1
    await RisingEdge(dut.clk)
    dut.evt_word_valid.value = 0


async def _wait_for_evt_ld_en(dut, timeout=PWR_WAIT_CYCLES + 50):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.evt_ld_en.value) == 1:
            return
    raise AssertionError("evt_ld_en never asserted — FSM did not reach LD_OPEN")


async def _wait_for_st_run(dut, timeout=512):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.core_rst_o.value) == 0:
            return
    raise AssertionError("core_rst_o never deasserted — FSM did not reach ST_RUN")


async def _minimal_boot(dut):
    """Send BOOT_REQ + EVT_READS_DONE to move FSM to ST_RUN with zero weights."""
    await _send_raw_word(dut, BOOT_REQ_WORD)
    await _wait_for_evt_ld_en(dut)
    await _send_raw_word(dut, EVT_READS_DONE_WORD)
    await _wait_for_st_run(dut)


async def deposit_weights_and_thresholds(dut, weights, thresholds):
    """Load weights and thresholds via EVT2 event stream (BOOT_REQ path).

    Sends BOOT_REQ, waits for evt_ld_en, streams all weight and threshold
    event words, then sends EVT_READS_DONE and waits for ST_RUN.

    weights:    list of NUM_CLASSES lists, each of length FEATURE_COUNT.
    thresholds: list of 2*NUM_CLASSES ints.
    """
    await _send_raw_word(dut, BOOT_REQ_WORD)
    await _wait_for_evt_ld_en(dut)

    for c in range(NUM_CLASSES):
        for addr in range(FEATURE_COUNT):
            await _send_raw_word(dut, build_weight_word(int(weights[c][addr]), addr, c))

    for addr in range(2 * NUM_CLASSES):
        val = int(thresholds[addr])
        upper18 = (val >> 18) & 0x3FFFF
        lower18 = val & 0x3FFFF
        await _send_raw_word(dut, build_thresh_upper_word(upper18))
        await _send_raw_word(dut, build_thresh_lower_word(lower18, addr))

    await _send_raw_word(dut, EVT_READS_DONE_WORD)

    # After the push loop the FIFO has at most 256 words pending; all drain
    # before the FSM sees EVT_READS_DONE and transitions to ST_RUN.
    await _wait_for_st_run(dut, timeout=512)


# ---------------------------------------------------------------------------
# Software models
# ---------------------------------------------------------------------------

class Evt2DecoderModel:
    def __init__(self):
        self.have_time_high = False
        self.time_high = 0

    def on_word(self, word):
        fields = _decode_evt2_word_fields(word)
        pkt = fields["pkt"]
        x_raw = fields["x_raw"]
        y_raw = fields["y_raw"]

        if pkt == EVT_TIME_HIGH:
            self.have_time_high = True
            self.time_high = fields["word"] & 0x0FFFFFFF
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

        x_grid = min((x_oriented * X_GRID_M) >> DIV_K, GRID_SIZE - 1)
        y_grid = min((y_oriented * Y_GRID_M) >> DIV_K, GRID_SIZE - 1)
        ts = ((self.time_high & 0x0FFFFFFF) << 6) | fields["ts_lsb"]
        return (x_grid, y_grid, ts)


class TimestampVoxelModel:
    """Perfect timestamp-driven model for voxel_binning feature windows."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.mem = [0] * TOTAL_CELLS
        self.wr_bin_idx = 0
        self.completed_bins = 0
        self.ts_initialized = False
        self.bin_start_ts = 0

    @staticmethod
    def _cell_addr(x, y):
        return (y * GRID_SIZE) + x

    def _readout_snapshot(self):
        start = (self.wr_bin_idx + NUM_BINS - (READOUT_BINS - 1)) % NUM_BINS
        out = []
        for off in range(READOUT_BINS):
            b = (start + off) % NUM_BINS
            base = b * CELLS_PER_BIN
            out.extend(self.mem[base:base + CELLS_PER_BIN])
        return out

    def _rotate_bin(self):
        next_wr = (self.wr_bin_idx + 1) % NUM_BINS
        self.completed_bins = min(self.completed_bins + 1, NUM_BINS)
        expected = self._readout_snapshot() if self.completed_bins >= READOUT_BINS else None

        base = next_wr * CELLS_PER_BIN
        for i in range(CELLS_PER_BIN):
            self.mem[base + i] = 0

        self.wr_bin_idx = next_wr
        return expected

    def force_rollover(self):
        if self.ts_initialized:
            self.bin_start_ts += BIN_DURATION_US
        expected = self._rotate_bin()
        return [] if expected is None else [expected]

    def accept_event(self, x, y, ts):
        readouts = []
        if not self.ts_initialized:
            self.ts_initialized = True
            self.bin_start_ts = ts
        else:
            while ts - self.bin_start_ts >= BIN_DURATION_US:
                expected = self._rotate_bin()
                if expected is not None:
                    readouts.append(expected)
                self.bin_start_ts += BIN_DURATION_US

        addr = (self.wr_bin_idx * CELLS_PER_BIN) + self._cell_addr(x, y)
        if self.mem[addr] < MAX_COUNTER:
            self.mem[addr] += 1
        return readouts


def load_thresholds():
    """Load class and diff thresholds from thresholds.mem (hex, $readmemh format).
    Returns a list of 2*NUM_CLASSES integers.
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
    while len(vals) < 2 * NUM_CLASSES:
        vals.append(0)
    return vals


_thresholds = None


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
        class_pass = int(best > self.class_thresholds[best_class])
        return best_class, class_pass, margin


def load_weights_from_mem():
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


# ---------------------------------------------------------------------------
# CoreHarness
# ---------------------------------------------------------------------------

class CoreHarness:
    def __init__(self, dut, score_model=None, check_feature_windows=True):
        self.dut = dut
        self.decoder = Evt2DecoderModel()
        self.bin_model = TimestampVoxelModel()
        self.check_feature_windows = check_feature_windows

        self.expected_decoded = deque()
        self.expected_feature_windows = deque()
        self.current_window = []

        self.expected_gestures = []
        self.observed_gestures = []

        self.accepted_words = 0
        self.completed_windows = 0
        self.next_event_ts = 0
        self.last_time_high = None

        self.score_model = score_model
        self.pending_score_checks = deque()
        self.window_features = []
        self.window_scores = []
        self.window_pred = []

    async def setup(self, start_clock=True):
        if start_clock:
            cocotb.start_soon(Clock(self.dut.clk, 10, units="ns").start())
        self.dut.rst.value = 1
        self.dut.evt_word.value = 0
        self.dut.evt_word_valid.value = 0
        self.dut.force_rollover_i.value = 0
        await ClockCycles(self.dut.clk, 8)
        self.dut.rst.value = 0
        await self.tick(4)
        # Minimal boot: move FSM from ST_BOOT → ST_RUN with zero weights so
        # the MAC engine is ungated and tests can observe class_valid/gesture_valid.
        await _minimal_boot(self.dut)
        # The 2 boot words (BOOT_REQ + EVT_READS_DONE) were accepted by the FIFO
        # and counted by debug_event_count. Reset the harness word counter to match.
        self.accepted_words = int(self.dut.debug_event_count.value)

    def _sample_cycle(self):
        if int(self.dut.u_evt2_decoder.event_valid.value):
            observed = (
                int(self.dut.u_evt2_decoder.x_out.value),
                int(self.dut.u_evt2_decoder.y_out.value),
                int(self.dut.u_evt2_decoder.ts_out.value),
            )
            assert self.expected_decoded, f"Unexpected decoded event {observed}"
            expected = self.expected_decoded.popleft()
            assert observed == expected, f"Decoded mismatch DUT={observed} model={expected}"
            x, y, ts = observed
            self.expected_feature_windows.extend(self.bin_model.accept_event(x, y, ts))

        if int(self.dut.u_voxel_binning.readout_valid.value):
            idx = int(self.dut.u_voxel_binning.readout_index.value)
            assert idx == len(self.current_window), \
                f"Readout index mismatch DUT={idx}, expected={len(self.current_window)}"
            self.current_window.append(int(self.dut.u_voxel_binning.readout_data.value))

            if int(self.dut.u_voxel_binning.readout_last.value):
                assert len(self.current_window) == FEATURE_COUNT, \
                    f"Feature window length {len(self.current_window)} != {FEATURE_COUNT}"
                if self.check_feature_windows:
                    assert self.expected_feature_windows, "DUT emitted an unexpected feature window"
                    expected_window = self.expected_feature_windows.popleft()
                    assert self.current_window == expected_window, (
                        "Feature window mismatch against timestamp golden model\n"
                        f"DUT:   {self.current_window}\nMODEL: {expected_window}"
                    )
                else:
                    expected_window = list(self.current_window)
                    if self.expected_feature_windows:
                        self.expected_feature_windows.popleft()
                self.window_features.append(list(expected_window))
                if self.score_model is not None:
                    exp_cls, exp_pass, margin = self.score_model.classify(expected_window)
                    self.pending_score_checks.append((exp_cls, exp_pass))
                    scores = [0] * NUM_CLASSES
                    for fi, feat in enumerate(expected_window):
                        f = int(feat)
                        for c in range(NUM_CLASSES):
                            scores[c] += f * self.score_model.weights[c][fi]
                    self.window_scores.append(scores)
                    self.window_pred.append((exp_cls, exp_pass, margin))
                    nz = sum(1 for v in expected_window if v > 0)
                    total = sum(int(v) for v in expected_window)
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

    async def send_grid_event(self, gx, gy, pkt=EVT_CD_ON, ts=None):
        if ts is None:
            ts = self.next_event_ts
        time_high = (int(ts) >> 6) & 0x0FFFFFFF
        if self.last_time_high != time_high:
            await self.send_word(build_evt2_time_high(time_high))
            self.last_time_high = time_high
        await self.send_word(build_evt2_cd(
            pkt,
            sensor_x_from_grid(gx),
            sensor_y_from_grid(gy),
            int(ts) & 0x3F,
        ))
        self.next_event_ts = max(self.next_event_ts, int(ts) + 1)

    async def force_bin_rollover(self):
        while int(self.dut.u_voxel_binning.state.value) != ST_ACCUM:
            await self.tick(1)

        stable_empty = 0
        for _ in range(200000):
            fifo_has_data = int(self.dut.u_input_fifo.valid_o.value)
            dec_valid = int(self.dut.u_evt2_decoder.event_valid.value)
            src_valid = int(self.dut.evt_word_valid.value)
            binner_ready = int(self.dut.u_voxel_binning.event_ready.value)
            if fifo_has_data == 0 and dec_valid == 0 and src_valid == 0 and binner_ready == 1:
                stable_empty += 1
                if stable_empty >= 2:
                    break
            else:
                stable_empty = 0
            await self.tick(1)
        else:
            raise AssertionError("Timeout draining input path before forced rollover")

        self.expected_feature_windows.extend(self.bin_model.force_rollover())
        if self.bin_model.ts_initialized:
            self.next_event_ts = max(self.next_event_ts, self.bin_model.bin_start_ts)
        self.dut.force_rollover_i.value = 1
        await self.tick(1)
        self.dut.force_rollover_i.value = 0

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
                    (not self.check_feature_windows or not self.expected_feature_windows) and
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
    candidates = sorted(set(pos_scores + neg_scores))
    if not candidates:
        return 0, 0.0
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
        gx, gy = rng.choice(pts)
        pkt = EVT_CD_ON if (i & 1) else EVT_CD_OFF
        await h.send_grid_event(gx, gy, pkt=pkt)

        if i % 13 == 0:
            bad = (0xF << 28) | rng.randint(0, 0x0FFFFFFF)
            await h.send_word(bad)


# ---------------------------------------------------------------------------
# Flush helper (reset + reload weights)
# ---------------------------------------------------------------------------

async def _flush_stale_bins(h, weights=None):
    """Hard-reset the DUT, boot via event stream, optionally reload weights."""
    h.dut.rst.value = 1
    h.dut.evt_word_valid.value = 0
    await ClockCycles(h.dut.clk, 16)
    h.dut.rst.value = 0

    for _ in range(10_000):
        await RisingEdge(h.dut.clk)
        if int(h.dut.u_voxel_binning.state.value) == ST_ACCUM:
            break
    else:
        raise AssertionError("Binner did not return to ST_ACCUM after flush reset")

    await ClockCycles(h.dut.clk, 8)

    if weights is not None:
        thresholds = load_thresholds()
        await deposit_weights_and_thresholds(h.dut, weights, thresholds)
    else:
        await _minimal_boot(h.dut)

    h.decoder = Evt2DecoderModel()
    h.bin_model = TimestampVoxelModel()
    h.expected_decoded.clear()
    h.expected_feature_windows.clear()
    h.current_window = []
    h.completed_windows = 0
    h.observed_gestures.clear()
    h.expected_gestures.clear()
    h.pending_score_checks.clear()
    h.accepted_words = 0
    h.next_event_ts = 0
    h.last_time_high = None
    h.window_features.clear()
    h.window_scores.clear()
    h.window_pred.clear()


# ---------------------------------------------------------------------------
# Existing core pipeline tests
# ---------------------------------------------------------------------------

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

    # debug_event_count wraps at 8 bits; baseline was set after setup() boot words.
    expected_count = (h.accepted_words) & 0xFF
    assert int(dut.debug_event_count.value) == expected_count, \
        "debug_event_count mismatch"


@logged_test()
async def test_empty_window_produces_no_gesture(dut):
    """No CD events -> all-zero features -> all scores zero -> no gesture fires."""
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0x1000))

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
    """Assert rst while pipeline is active; verify clean restart."""
    rng = random.Random(0xDEAD_F00D)
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0xABCDE))
    pts = region_points("left")
    for _ in range(15):
        gx, gy = rng.choice(pts)
        await h.send_word(
            build_evt2_cd(EVT_CD_ON, sensor_x_from_grid(gx), sensor_y_from_grid(gy), 1)
        )
    await h.force_bin_rollover()

    dut.rst.value = 1
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0

    # After reset the FSM is in ST_BOOT; core_rst_o=1 holds MAC in reset.
    # No boot sequence needed here since this test only checks for absence of
    # spurious gestures and that the FIFO is ready to accept new data.
    for _ in range(20_000):
        await RisingEdge(dut.clk)
        if int(dut.u_voxel_binning.state.value) == ST_ACCUM:
            break
    else:
        raise AssertionError("Binner did not return to ST_ACCUM after reset")

    for _ in range(500):
        await RisingEdge(dut.clk)
        assert int(dut.gesture_valid.value) == 0, \
            "Spurious gesture_valid asserted after reset"

    assert int(dut.evt_word_ready.value) == 1, \
        "evt_word_ready not asserted after reset"


@logged_test()
async def test_debug_event_count_tracks_accepted_words(dut):
    """debug_event_count must track accepted words relative to setup baseline."""
    h = CoreHarness(dut)
    await h.setup()

    # accepted_words was seeded from debug_event_count after setup boot words.
    baseline = h.accepted_words

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

    # h.accepted_words already includes the boot-word baseline set in setup(),
    # so the total DUT count equals h.accepted_words (mod 256).
    expected = h.accepted_words & 0xFF
    assert int(dut.debug_event_count.value) == expected, (
        f"debug_event_count DUT={int(dut.debug_event_count.value)} "
        f"expected={expected} (accepted_words={h.accepted_words})"
    )


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
        await h.send_grid_event(gx, gy, pkt=EVT_CD_ON)
        if i % 15 == 0:
            await h.send_word(build_evt2_time_high(h.last_time_high or 0))

    await h.force_bin_rollover()
    await h.wait_quiet()

    assert not h.expected_decoded, \
        f"Unmatched decoded events remaining: {len(h.expected_decoded)}"
    assert h.observed_gestures == h.expected_gestures, \
        (f"Gesture mismatch after backpressure burst\n"
         f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}")


@logged_test()
async def test_core_timestamp_boundary_binning_matches_golden(dut):
    """Decoded event timestamps, not host-forced cycles, define bin boundaries."""
    h = CoreHarness(dut)
    await h.setup()

    await h.send_grid_event(0, 0, pkt=EVT_CD_ON, ts=5)
    await h.send_grid_event(1, 1, pkt=EVT_CD_ON, ts=5 + BIN_DURATION_US)

    for _ in range(READOUT_BINS - 1):
        await h.force_bin_rollover()

    await h.wait_quiet()
    assert h.completed_windows > 0, "Expected a timestamp-driven feature window"


@logged_test()
async def test_sustained_region_fires_gesture(dut):
    """Drive a single spatial region for many bins; gesture_valid must fire on every passing window."""
    rng = random.Random(0x1234_5678)
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0x1))

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

    time_bases = [0x00001, 0x00002, 0x00003]
    for tb_val in time_bases:
        await h.send_word(build_evt2_time_high(tb_val))
        for i in range(10):
            gx = rng.randint(0, GRID_SIZE - 1)
            gy = rng.randint(0, GRID_SIZE - 1)
            pkt = EVT_CD_ON if rng.randint(0, 1) else EVT_CD_OFF
            ts_lsb = i
            await h.send_word(
                build_evt2_cd(pkt, sensor_x_from_grid(gx), sensor_y_from_grid(gy), ts_lsb)
            )

    await h.force_bin_rollover()
    await h.wait_quiet(quiet_cycles=20)


@logged_test()
async def test_score_model_validates_classifications(dut):
    """ScoreModel independently verifies every DUT class_gesture/class_pass output."""
    weights = load_weights_from_mem()
    score_model = ScoreModel(weights)

    rng = random.Random(0xBEEF_CAFE)
    h = CoreHarness(dut, score_model=score_model)
    await h.setup(start_clock=True)
    await _flush_stale_bins(h, weights=weights)

    await h.send_word(build_evt2_time_high(0xABCDE))

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
    assert not h.pending_score_checks, \
        f"{len(h.pending_score_checks)} feature windows never produced a class_valid"
    assert not h.expected_decoded, \
        (f"{len(h.expected_decoded)} decoded events unmatched by DUT")


@logged_test()
async def test_wrong_gesture_trajectory_no_false_positive(dut):
    """Uniform random events: DUT and ScoreModel must agree on every classification."""
    weights = load_weights_from_mem()
    score_model = ScoreModel(weights)

    rng = random.Random(0xFA_15_E0)
    h = CoreHarness(dut, score_model=score_model, check_feature_windows=False)
    await h.setup()
    await _flush_stale_bins(h, weights=weights)

    for _ in range(READOUT_BINS + 1):
        for _ in range(32):
            gx = rng.randint(0, GRID_SIZE - 1)
            gy = rng.randint(0, GRID_SIZE - 1)
            pkt = EVT_CD_ON if rng.randint(0, 1) else EVT_CD_OFF
            await h.send_grid_event(gx, gy, pkt=pkt)
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert h.completed_windows > 0, "No feature windows completed"
    assert not h.pending_score_checks, \
        f"{len(h.pending_score_checks)} score checks never consumed"
    assert h.observed_gestures == h.expected_gestures, \
        f"DUT/model mismatch on uniform input\nDUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}"


# ---------------------------------------------------------------------------
# EVT2 .bin file streaming tests
# ---------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).resolve().parents[1]


def _default_bin_paths():
    test_set = _REPO_ROOT / "EVT2_gesture_set" / "test_set"
    if test_set.exists():
        return [
            test_set / "wave_down_sun_test1.bin",
            test_set / "wave_left_sun_test1.bin",
            test_set / "wave_right_sun_test1.bin",
            test_set / "wave_up_sun_test1.bin",
        ]
    legacy_root = _REPO_ROOT / "EVT2_gesture_set"
    return [
        legacy_root / "wave_down_sun_test1.bin",
        legacy_root / "wave_left_sun_test1.bin",
        legacy_root / "wave_right_sun_test1.bin",
        legacy_root / "wave_up_sun_test1.bin",
    ]


_DEFAULT_BIN_FILES = _default_bin_paths()


def _resolve_bin_files():
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
    data = Path(path).read_bytes()
    n_words = len(data) // 4
    return list(struct.unpack_from(f"<{n_words}I", data, 0))


async def _stream_bin_file_with_timing(h, bin_path):
    words = _read_evt2_bin(bin_path)
    for word in words:
        await h.send_word(word)


async def _stream_bin_file_clock_driven(h, bin_path):
    words = _read_evt2_bin(bin_path)
    for word in words:
        await h.send_word(word)


async def _run_bin_file_test(
    dut, bin_path, label, expected_class=None, replay_mode="timestamp_forced", enforce_label=None, start_clock=True
):
    weights = load_weights_from_mem()
    score_model = ScoreModel(weights)
    h = CoreHarness(dut, score_model=score_model, check_feature_windows=False)
    await h.setup(start_clock=start_clock)
    await _flush_stale_bins(h, weights=weights)

    cocotb.log.info(f"[{label}] Streaming {bin_path} (mode={replay_mode}) ...")
    if replay_mode == "timestamp_forced":
        await _stream_bin_file_with_timing(h, bin_path)
    elif replay_mode == "clock_driven":
        await _stream_bin_file_clock_driven(h, bin_path)
    else:
        raise ValueError(f"Unknown replay_mode={replay_mode}")

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
    await _run_bin_file_test(dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[0])


@logged_test()
async def test_bin_file_gesture_1(dut):
    """Stream bin file 1 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[1]
    label = Path(path).stem
    await _run_bin_file_test(dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[1])


@logged_test()
async def test_bin_file_gesture_2(dut):
    """Stream bin file 2 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[2]
    label = Path(path).stem
    await _run_bin_file_test(dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[2])


@logged_test()
async def test_bin_file_gesture_3(dut):
    """Stream bin file 3 into the DUT and report classifications."""
    bin_files = _resolve_bin_files()
    path = bin_files[3]
    label = Path(path).stem
    await _run_bin_file_test(dut, path, label, expected_class=EXPECTED_BIN_FILE_CLASS[3])


@logged_test()
async def test_bin_file_timing_replay_ab(dut):
    """A/B timing investigation: timestamp-forced vs clock-driven replay."""
    bin_files = _resolve_bin_files()
    for i, path in enumerate(bin_files):
        label = Path(path).stem
        expected = EXPECTED_BIN_FILE_CLASS[i]
        h_forced = await _run_bin_file_test(
            dut, path, f"{label}-forced", expected_class=expected,
            replay_mode="timestamp_forced", enforce_label=False, start_clock=(i == 0),
        )
        h_clock = await _run_bin_file_test(
            dut, path, f"{label}-clock", expected_class=expected,
            replay_mode="clock_driven", enforce_label=False, start_clock=False,
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
async def test_mac_backpressure_holds_binner(dut):
    """When the MAC is busy the binner must park in ST_WAIT_RD and complete both windows."""
    weights = load_weights_from_mem()
    h = CoreHarness(dut, score_model=ScoreModel(weights))
    await h.setup()
    await _flush_stale_bins(h, weights=weights)

    rng = random.Random(0xBEEF_0001)
    await h.send_word(build_evt2_time_high(0x1))

    for _ in range(READOUT_BINS):
        await drive_bin_traffic(h, rng, "right", events=8)
        await h.force_bin_rollover()

    for _ in range(10_000):
        await h.tick(1)
        if int(dut.debug_score_busy.value):
            break
    else:
        raise AssertionError("MAC never went busy after first feature window")

    await drive_bin_traffic(h, rng, "right", events=4)
    await h.force_bin_rollover()

    await h.wait_quiet()
    assert h.completed_windows >= 2, \
        f"Expected >=2 completed windows after backpressure, got {h.completed_windows}"
    assert not h.pending_score_checks, \
        f"{len(h.pending_score_checks)} feature windows never produced class_valid"
    assert h.observed_gestures == h.expected_gestures, \
        f"Gesture mismatch after MAC backpressure\nDUT: {h.observed_gestures}\nMODEL: {h.expected_gestures}"


# ---------------------------------------------------------------------------
# control_fsm integration tests
# ---------------------------------------------------------------------------

@logged_test()
async def test_fsm_boots_from_st_boot_to_st_run(dut):
    """After reset FSM is in ST_BOOT with core_rst_o=1; after BOOT_REQ+done it reaches ST_RUN."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # FSM should be in ST_BOOT immediately after reset
    await ReadOnly()
    assert int(dut.controller_fsm.main_state.value) == ST_BOOT, \
        f"Expected ST_BOOT after reset, got {int(dut.controller_fsm.main_state.value)}"
    assert int(dut.core_rst_o.value) == 1, "core_rst_o must be 1 in ST_BOOT"
    assert int(dut.evt_ld_en.value) == 0, "evt_ld_en must be 0 in ST_BOOT"
    assert int(dut.boot_done_o.value) == 0, "boot_done_o must be 0 in ST_BOOT"

    # Exit ReadOnly phase before driving any signals.
    await NextTimeStep()

    # Boot sequence: BOOT_REQ → wait for LD_OPEN → EVT_READS_DONE → ST_RUN
    await _send_raw_word(dut, BOOT_REQ_WORD)

    # FSM should move to ST_LOAD within a few cycles (decoder pipeline + 1 FSM cycle)
    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.controller_fsm.main_state.value) == ST_LOAD:
            break
    else:
        raise AssertionError("FSM did not enter ST_LOAD after BOOT_REQ")

    assert int(dut.core_rst_o.value) == 1, "core_rst_o must remain 1 during ST_LOAD"

    # Wait for evt_ld_en to assert (after PWR_WAIT_CYCLES in LD_WAIT_PWR)
    await _wait_for_evt_ld_en(dut)
    assert int(dut.controller_fsm.load_state.value) in (LD_OPEN, LD_WAIT), \
        f"Expected load_state LD_OPEN/LD_WAIT when evt_ld_en=1, got {int(dut.controller_fsm.load_state.value)}"

    # Close boot window with EVT_READS_DONE
    await _send_raw_word(dut, EVT_READS_DONE_WORD)
    await _wait_for_st_run(dut)

    await ReadOnly()
    assert int(dut.controller_fsm.main_state.value) == ST_RUN, \
        f"Expected ST_RUN after boot, got {int(dut.controller_fsm.main_state.value)}"
    assert int(dut.core_rst_o.value) == 0, "core_rst_o must be 0 in ST_RUN"
    assert int(dut.evt_ld_en.value) == 0, "evt_ld_en must be 0 in ST_RUN"
    assert int(dut.boot_done_o.value) == 1, "boot_done_o must be 1 in ST_RUN"


@logged_test()
async def test_evt_ld_en_only_asserts_during_load_window(dut):
    """evt_ld_en is 0 in ST_BOOT, 1 during LD_OPEN/LD_WAIT, and 0 again in ST_RUN."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # Must be 0 before any boot
    assert int(dut.evt_ld_en.value) == 0, "evt_ld_en must be 0 before boot"

    await _send_raw_word(dut, BOOT_REQ_WORD)

    # Must remain 0 during LD_WAIT_PWR
    for _ in range(PWR_WAIT_CYCLES - 10):
        await RisingEdge(dut.clk)
        assert int(dut.evt_ld_en.value) == 0, \
            "evt_ld_en must not assert before LD_OPEN"

    # Must assert at LD_OPEN
    await _wait_for_evt_ld_en(dut, timeout=20)
    assert int(dut.evt_ld_en.value) == 1, "evt_ld_en must be 1 at LD_OPEN"

    # Send EVT_READS_DONE to close the window
    await _send_raw_word(dut, EVT_READS_DONE_WORD)
    await _wait_for_st_run(dut)

    # Must be 0 again in ST_RUN
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.evt_ld_en.value) == 0, "evt_ld_en must be 0 in ST_RUN"


@logged_test()
async def test_weight_writes_blocked_outside_load_window(dut):
    """Weight event words are silently discarded when evt_ld_en=0 (ST_BOOT and ST_RUN)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # ST_BOOT: weight word should be blocked (evt_ld_en=0).
    # Drive valid for one cycle then deassert; give decoder 2 cycles to process.
    w = build_weight_word(0xFF, 0, 0)
    dut.evt_word.value = w
    dut.evt_word_valid.value = 1
    await RisingEdge(dut.clk)
    dut.evt_word_valid.value = 0
    await ClockCycles(dut.clk, 2)
    await ReadOnly()
    assert int(dut.weight_wr_valid_gated.value) == 0, \
        "weight_wr_valid_gated must be 0 in ST_BOOT"
    await NextTimeStep()

    # Minimal boot to reach ST_RUN
    await _minimal_boot(dut)
    await ClockCycles(dut.clk, 4)

    # ST_RUN: weight word should also be blocked.
    dut.evt_word.value = w
    dut.evt_word_valid.value = 1
    await RisingEdge(dut.clk)
    dut.evt_word_valid.value = 0
    # Give FIFO → decoder pipeline 2 cycles to process the word.
    await ClockCycles(dut.clk, 2)
    await ReadOnly()
    assert int(dut.weight_wr_valid_gated.value) == 0, \
        "weight_wr_valid_gated must be 0 in ST_RUN (evt_ld_en=0)"
    await NextTimeStep()


@logged_test()
async def test_weight_writes_accepted_during_load_window(dut):
    """Weight event words decoded while evt_ld_en=1 must assert weight_wr_valid_gated."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # Enter load window
    await _send_raw_word(dut, BOOT_REQ_WORD)
    await _wait_for_evt_ld_en(dut)

    # Send one weight word and verify gated signal asserts
    w = build_weight_word(0xAB, 0, 0)
    await _send_raw_word(dut, w)

    # The word must propagate through FIFO → decoder in a few cycles
    gated_seen = False
    for _ in range(20):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.weight_wr_valid_gated.value) == 1:
            gated_seen = True
            break
    assert gated_seen, "weight_wr_valid_gated never asserted during load window"

    # Exit ReadOnly phase (loop may have broken out of await ReadOnly()) before driving.
    await NextTimeStep()

    # Close the window cleanly
    await _send_raw_word(dut, EVT_READS_DONE_WORD)
    await _wait_for_st_run(dut)


@logged_test()
async def test_no_gesture_before_boot_completes(dut):
    """Events and rollovers fed before boot must not produce gesture_valid (MAC in reset)."""
    rng = random.Random(0xABCD_EF01)
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # Send event data while FSM is still in ST_BOOT (no boot sequence sent)
    th_word = build_evt2_time_high(0x100)
    await _send_raw_word(dut, th_word)

    pts = region_points("bottom")
    for _ in range(20):
        gx, gy = rng.choice(pts)
        w = build_evt2_cd(EVT_CD_ON, sensor_x_from_grid(gx), sensor_y_from_grid(gy), 0)
        await _send_raw_word(dut, w)

    # Force several rollovers to produce feature windows while in ST_BOOT
    for _ in range(READOUT_BINS + 2):
        dut.force_rollover_i.value = 1
        await RisingEdge(dut.clk)
        dut.force_rollover_i.value = 0
        await ClockCycles(dut.clk, 5)

    # Wait for pipeline to settle
    await ClockCycles(dut.clk, 200)

    # No gesture should have fired — core_rst_o=1 holds the MAC in reset
    assert int(dut.gesture_valid.value) == 0, \
        "gesture_valid fired before boot completed — core_rst_o did not gate MAC"
    assert int(dut.core_rst_o.value) == 1, \
        "core_rst_o must remain 1 before boot completes"


@logged_test()
async def test_debug_req_during_load_clears_evt_ld_en(dut):
    """If debug_req fires while in LD_OPEN/LD_WAIT, evt_ld_en must deassert immediately.

    This is a regression test for the bug where ST_LOAD's debug_req branch did not
    clear evt_ld_en, allowing SRAM writes to leak into ST_DEBUG.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # Open the load window
    await _send_raw_word(dut, BOOT_REQ_WORD)
    await _wait_for_evt_ld_en(dut)
    assert int(dut.evt_ld_en.value) == 1, "Precondition: evt_ld_en must be 1"

    # Inject DEBUG_REQ — should interrupt ST_LOAD and clear evt_ld_en
    await _send_raw_word(dut, DEBUG_REQ_WORD)

    # evt_ld_en must deassert within a few cycles of the FSM seeing debug_req_i
    ld_en_cleared = False
    for _ in range(20):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.evt_ld_en.value) == 0:
            ld_en_cleared = True
            break
    assert ld_en_cleared, "evt_ld_en not cleared after debug_req interrupted load"

    # FSM should be in ST_DEBUG
    assert int(dut.controller_fsm.main_state.value) == ST_DEBUG, \
        f"Expected ST_DEBUG after debug_req, got {int(dut.controller_fsm.main_state.value)}"
    assert int(dut.core_rst_o.value) == 1, "core_rst_o must be 1 in ST_DEBUG"

    # Exit ReadOnly phase before driving signals.
    await NextTimeStep()

    # Weight write words while in ST_DEBUG with evt_ld_en=0 must be blocked
    w = build_weight_word(0xFF, 0, 0)
    await _send_raw_word(dut, w)
    for _ in range(10):
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.weight_wr_valid_gated.value) == 0, \
            "weight_wr_valid_gated must be 0 in ST_DEBUG (evt_ld_en cleared)"


@logged_test()
async def test_debug_req_from_boot_then_reload(dut):
    """DEBUG_REQ from ST_BOOT → ST_DEBUG → boot_req restarts load → reaches ST_RUN."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # Go directly to ST_DEBUG from ST_BOOT
    await _send_raw_word(dut, DEBUG_REQ_WORD)

    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.controller_fsm.main_state.value) == ST_DEBUG:
            break
    else:
        raise AssertionError("FSM did not enter ST_DEBUG")

    assert int(dut.core_rst_o.value) == 1, "core_rst_o must be 1 in ST_DEBUG"
    assert int(dut.evt_ld_en.value) == 0, "evt_ld_en must be 0 in ST_DEBUG"

    # Exit DEBUG with BOOT_REQ (debug_req_i already deasserted — it was a one-cycle pulse)
    await _send_raw_word(dut, BOOT_REQ_WORD)

    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.controller_fsm.main_state.value) == ST_LOAD:
            break
    else:
        raise AssertionError("FSM did not return to ST_LOAD from ST_DEBUG")

    # Complete boot and verify ST_RUN reached
    await _wait_for_evt_ld_en(dut)
    await _send_raw_word(dut, EVT_READS_DONE_WORD)
    await _wait_for_st_run(dut)

    assert int(dut.controller_fsm.main_state.value) == ST_RUN, \
        "FSM did not reach ST_RUN after debug→boot sequence"
    assert int(dut.boot_done_o.value) == 1


@logged_test()
async def test_reload_req_from_run_state(dut):
    """RELOAD_REQ while in ST_RUN must re-enter ST_LOAD and eventually reach ST_RUN."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)

    # Boot to ST_RUN
    await _minimal_boot(dut)
    assert int(dut.controller_fsm.main_state.value) == ST_RUN, "Precondition: must be in ST_RUN"

    # Reload
    await _send_raw_word(dut, RELOAD_REQ_WORD)

    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.controller_fsm.main_state.value) == ST_LOAD:
            break
    else:
        raise AssertionError("FSM did not enter ST_LOAD on RELOAD_REQ")

    # core_rst_o is registered: ST_RUN sets it to 0 at the same posedge that transitions
    # main_state to ST_LOAD. ST_LOAD's body sets it to 1 only at the *next* posedge.
    await RisingEdge(dut.clk)
    assert int(dut.core_rst_o.value) == 1, "core_rst_o must be 1 during reload"
    assert int(dut.boot_done_o.value) == 0, "boot_done_o must deassert during reload"
    assert int(dut.evt_ld_en.value) == 0, "evt_ld_en must be 0 at start of reload"

    # Complete reload
    await _wait_for_evt_ld_en(dut)
    await _send_raw_word(dut, EVT_READS_DONE_WORD)
    await _wait_for_st_run(dut)

    assert int(dut.controller_fsm.main_state.value) == ST_RUN, \
        "FSM must return to ST_RUN after reload"
    assert int(dut.boot_done_o.value) == 1
    assert int(dut.evt_ld_en.value) == 0


@logged_test()
async def test_fsm_state_debug_signals_through_boot(dut):
    """main_state_dbg_o and load_state_dbg_o must reflect the correct encoding at each stage."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 4)
    await ReadOnly()

    assert int(dut.main_state_dbg_o.value) == ST_BOOT, \
        f"main_state_dbg_o should be ST_BOOT(0), got {int(dut.main_state_dbg_o.value)}"
    assert int(dut.load_state_dbg_o.value) == LD_IDLE, \
        f"load_state_dbg_o should be LD_IDLE(0), got {int(dut.load_state_dbg_o.value)}"

    # Exit ReadOnly phase before driving signals.
    await NextTimeStep()

    await _send_raw_word(dut, BOOT_REQ_WORD)

    # Confirm ST_LOAD is reflected in the debug output
    for _ in range(10):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.main_state_dbg_o.value) == ST_LOAD:
            break
    else:
        raise AssertionError("main_state_dbg_o never showed ST_LOAD")

    # Exit ReadOnly before the next loop's first iteration reads a signal.
    await NextTimeStep()

    # Wait for LD_WAIT_PWR to appear in load_state_dbg_o
    found_wait_pwr = False
    for _ in range(PWR_WAIT_CYCLES + 5):
        val = int(dut.load_state_dbg_o.value)
        if val == LD_WAIT_PWR:
            found_wait_pwr = True
        if val == LD_OPEN:
            break
        await RisingEdge(dut.clk)
    assert found_wait_pwr, "load_state_dbg_o never showed LD_WAIT_PWR"

    # evt_ld_en asserts → load_state_dbg_o should be LD_OPEN then LD_WAIT
    await _wait_for_evt_ld_en(dut, timeout=5)

    await _send_raw_word(dut, EVT_READS_DONE_WORD)
    await _wait_for_st_run(dut)

    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.main_state_dbg_o.value) == ST_RUN, \
        f"main_state_dbg_o should be ST_RUN(2), got {int(dut.main_state_dbg_o.value)}"
    assert int(dut.load_state_dbg_o.value) == LD_IDLE, \
        f"load_state_dbg_o should be LD_IDLE(0) in ST_RUN, got {int(dut.load_state_dbg_o.value)}"


@logged_test()
async def test_boot_with_weights_enables_correct_classification(dut):
    """Full event-stream weight load → classification produces expected class per ScoreModel."""
    weights = load_weights_from_mem()
    thresholds = load_thresholds()
    score_model = ScoreModel(weights)

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.evt_word.value = 0
    dut.evt_word_valid.value = 0
    dut.force_rollover_i.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 8)

    # Load weights via boot sequence through the event stream
    await deposit_weights_and_thresholds(dut, weights, thresholds)

    assert int(dut.core_rst_o.value) == 0, "core_rst_o must be 0 after boot with weights"
    assert int(dut.controller_fsm.main_state.value) == ST_RUN

    # Now exercise the full pipeline with the harness (clock already running)
    rng = random.Random(0x1357_9BDF)
    h = CoreHarness(dut, score_model=score_model, check_feature_windows=False)
    # Sync harness accepted_words with current debug_event_count (boot words already counted)
    h.accepted_words = int(dut.debug_event_count.value)

    await h.send_word(build_evt2_time_high(0x500))
    for _ in range(READOUT_BINS + 2):
        await drive_bin_traffic(h, rng, "bottom", events=20)
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert h.completed_windows > 0, "No feature windows completed"
    assert not h.pending_score_checks, \
        f"{len(h.pending_score_checks)} windows never produced class_valid"
    assert h.observed_gestures == h.expected_gestures, (
        f"DUT/model mismatch after event-stream weight load\n"
        f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}"
    )
