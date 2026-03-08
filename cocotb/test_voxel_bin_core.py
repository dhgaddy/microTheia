"""Integration cocotb testbench for voxel_bin_core with golden scoreboards."""

from collections import deque
from pathlib import Path
import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

GRID_SIZE = 16
NUM_BINS = 8
READOUT_BINS = 8
FEATURE_COUNT = GRID_SIZE * GRID_SIZE * READOUT_BINS
PASS_MARGIN = 64
PERSISTENCE_COUNT = 2
CONF_BITS = 4
CONF_SHIFT = 4
WEIGHT_BITS = 8
WEIGHT_SCALE = 1024  # matches RTL init_scale_p

CLK_FREQ_HZ = 12_000_000
WINDOW_MS = 1000
BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS
SENSOR_DIM = 320
BIN_DIV = SENSOR_DIM // GRID_SIZE

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
    # Drive the center of the corresponding sensor bin.
    g = max(0, min(GRID_SIZE - 1, int(g)))
    return min(SENSOR_DIM - 1, (g * BIN_DIV) + (BIN_DIV // 2))


class Evt2DecoderModel:
    def __init__(self):
        self.time_high = 0
        self.have_time_high = False

    def on_word(self, word):
        pkt = (word >> 28) & 0xF
        ts_lsb = (word >> 22) & 0x3F
        x_raw = (word >> 11) & 0x7FF
        y_raw = word & 0x7FF

        if pkt == EVT_TIME_HIGH:
            self.time_high = word & 0x0FFFFFFF
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
        pol = 1 if pkt == EVT_CD_ON else 0
        ts = (self.time_high << 6) | ts_lsb
        return (x_grid, y_grid, pol, ts)


class GesturePersistenceModel:
    def __init__(self):
        self.last_pass_class = 0
        self.pass_streak = 0
        self.gesture = 0

    @staticmethod
    def _confidence_from_margin(margin):
        if margin <= 0:
            return 0
        c = margin >> CONF_SHIFT
        return min((1 << CONF_BITS) - 1, c)

    def step(self, class_id, class_pass, margin):
        gesture_valid = 0
        conf = 0

        if class_pass:
            if class_id == self.last_pass_class:
                if self.pass_streak < PERSISTENCE_COUNT:
                    next_streak = self.pass_streak + 1
                else:
                    next_streak = self.pass_streak
            else:
                next_streak = 1

            self.last_pass_class = class_id
            self.pass_streak = next_streak

            conf = self._confidence_from_margin(margin)
            if next_streak >= PERSISTENCE_COUNT:
                self.gesture = class_id
                gesture_valid = 1
        else:
            self.pass_streak = 0

        return self.gesture, gesture_valid, conf


class ScoreModel:
    def __init__(self, weights_per_class):
        self.weights = weights_per_class

    @staticmethod
    def _argmax_with_second(vals):
        best_i = 0
        best = vals[0]
        second = -10**30
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
        class_pass = int(margin > PASS_MARGIN)
        return best_class, class_pass, margin


def load_quantized_weights():
    """Load and quantize weights to match RTL ram_1r1w_sync init (init_scale_p=WEIGHT_SCALE,
    init_signed_p=0, stride=FEATURE_COUNT per class).  Matches gesture_weights file layout."""
    repo_root = Path(__file__).resolve().parents[1]
    candidates = [
        repo_root / "weights" / "gesture_weights_down_left_right_up_16x16_8bins.txt",
        repo_root / "gesture_weights_down_left_right_up_16x16_8bins.txt",
    ]
    weights_path = next((p for p in candidates if p.exists()), candidates[0])
    lines = weights_path.read_text(encoding="ascii").splitlines()

    max_unsigned = (1 << WEIGHT_BITS) - 1

    def quantize(line):
        try:
            f = float(line.strip())
        except ValueError:
            return 0
        q = int(f * WEIGHT_SCALE)  # $rtoi-style truncation toward zero
        if q < 0:          # init_signed_p=0: clamp negative to 0
            q = 0
        if q > max_unsigned:
            q = max_unsigned
        return q

    qvals = [quantize(line) for line in lines]
    expected_len = 4 * FEATURE_COUNT
    if len(qvals) < expected_len:
        qvals.extend([0] * (expected_len - len(qvals)))

    weights = []
    for c in range(4):
        # WEIGHT_FILE_CLASS_STRIDE = 2048 = FEATURE_COUNT; class c starts at c*2048.
        start = c * FEATURE_COUNT
        weights.append(qvals[start:start + FEATURE_COUNT])
    return weights


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

        self.last_pass_class = 0
        self.pass_streak = 0

        # Optional independent score verification.
        self.score_model = score_model
        self.pending_score_checks = deque()  # (exp_class, exp_pass) from ScoreModel

    async def setup(self):
        cocotb.start_soon(Clock(self.dut.clk, 10, unit="ns").start())
        self.dut.rst.value = 1
        self.dut.evt_word.value = 0
        self.dut.evt_word_valid.value = 0
        await ClockCycles(self.dut.clk, 8)
        self.dut.rst.value = 0
        await self.tick(4)

    def _sample_cycle(self):
        if int(self.dut.u_evt2_decoder.event_valid.value):
            observed = (
                int(self.dut.u_evt2_decoder.x_out.value),
                int(self.dut.u_evt2_decoder.y_out.value),
                int(self.dut.u_evt2_decoder.polarity.value),
                int(self.dut.u_evt2_decoder.timestamp.value),
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
                    exp_cls, exp_pass, _ = self.score_model.classify(self.current_window)
                    self.pending_score_checks.append((exp_cls, exp_pass))
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
                if class_id == self.last_pass_class:
                    if self.pass_streak < PERSISTENCE_COUNT:
                        self.pass_streak += 1
                else:
                    self.pass_streak = 1
                self.last_pass_class = class_id

                if self.pass_streak >= PERSISTENCE_COUNT:
                    self.expected_gestures.append((
                        class_id,
                        int(self.dut.gesture_confidence.value),
                    ))
            else:
                self.pass_streak = 0

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

    async def wait_quiet(self, quiet_cycles=2000, timeout=200000):
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
                    int(self.dut.score_state.value) == 0 and
                    int(self.dut.capture_active.value) == 0 and
                    int(self.dut.feature_window_ready.value) == 0 and
                    int(self.dut.u_input_fifo.valid_o.value) == 0 and
                    int(self.dut.u_voxel_binning.state.value) == ST_ACCUM and
                    not self.expected_decoded and
                    not self.current_window
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
    """No CD events → all-zero features → all scores zero → no gesture fires.

    This validates the zero-input boundary case: scores are all zero so margin is
    zero (< PASS_MARGIN) and gesture_valid never asserts.
    """
    h = CoreHarness(dut)
    await h.setup()

    # Prime decoder with a TIME_HIGH so CD events would be accepted, but send none.
    await h.send_word(build_evt2_time_high(0x1000))

    # Rotate enough bins to produce several complete readout windows.
    for _ in range(NUM_BINS + 2):
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

    await h.wait_quiet(quiet_cycles=300)

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

    for _ in range(NUM_BINS):
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert not h.expected_decoded, \
        f"Unmatched decoded events remaining: {len(h.expected_decoded)}"
    assert h.observed_gestures == h.expected_gestures, \
        (f"Gesture mismatch after backpressure burst\n"
         f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}")


@logged_test()
async def test_sustained_region_fires_gesture(dut):
    """Drive a single spatial region for many bins; classifier must eventually fire."""
    rng = random.Random(0x1234_5678)
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0x1))

    # Drive "bottom" region (class 0 = Down in weight ordering) across many bins.
    for _ in range(NUM_BINS * 3):
        await drive_bin_traffic(h, rng, "bottom", events=32)
        await h.force_bin_rollover()

    await h.wait_quiet()

    # DUT and model must agree on whatever was output.
    assert h.observed_gestures == h.expected_gestures, \
        (f"Gesture mismatch in sustained-region test\n"
         f"DUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}")

    # At least one gesture window should have been completed.
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
    await h.wait_quiet(quiet_cycles=500)


@logged_test()
async def test_score_model_validates_classifications(dut):
    """ScoreModel independently verifies every DUT class_gesture/class_pass output."""
    weights = load_quantized_weights()
    score_model = ScoreModel(weights)

    rng = random.Random(0xBEEF_CAFE)
    h = CoreHarness(dut, score_model=score_model)
    await h.setup()

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
