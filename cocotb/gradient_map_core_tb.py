"""End-to-end golden-model testbench for gradient_map_core."""

from collections import deque
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

GRID_SIZE = 16
NUM_CELLS = GRID_SIZE * GRID_SIZE
NUM_CLASSES = 4
VALUE_BITS = 8
MAX_VALUE = 255
TS_MASK = 0xFFFF

DECAY_SHIFT = 15
MIN_MASS_THRESH = 20

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_from_grid(g, jitter=0):
    return ((g & 0xF) << 4) | (jitter & 0xF)


class Evt2DecoderModel:
    def __init__(self):
        self.time_high_reg = 0

    def step(self, word):
        pkt_type = (word >> 28) & 0xF
        ts_lsb = (word >> 22) & 0x3F
        x_raw = (word >> 11) & 0x7FF
        y_raw = word & 0x7FF

        if pkt_type == EVT_TIME_HIGH:
            self.time_high_reg = word & 0x0FFFFFFF
            return None

        if pkt_type not in (EVT_CD_OFF, EVT_CD_ON):
            return None

        x_grid = min((x_raw >> 4) & 0x1F, 15)
        y_grid = min((y_raw >> 4) & 0x1F, 15)
        pol = 1 if pkt_type == EVT_CD_ON else 0
        ts = ((self.time_high_reg & 0x3FF) << 6) | ts_lsb
        return (x_grid, y_grid, pol, ts)


class GradientSurfaceModel:
    def __init__(self, decay_shift=DECAY_SHIFT):
        self.decay_shift = decay_shift
        self.mem_ts = [0] * NUM_CELLS
        self.valid = [False] * NUM_CELLS

    def write_event(self, x_grid, y_grid, t_event):
        addr = (y_grid * GRID_SIZE + x_grid) & 0xFF
        self.mem_ts[addr] = t_event & TS_MASK
        self.valid[addr] = True

    def cell_value(self, addr, t_now):
        if not self.valid[addr]:
            return 0
        delta = (t_now - self.mem_ts[addr]) & TS_MASK
        decay_steps = delta >> self.decay_shift
        if decay_steps >= VALUE_BITS:
            return 0
        return MAX_VALUE >> decay_steps

    def snapshot(self, t_now):
        return [self.cell_value(addr, t_now) for addr in range(NUM_CELLS)]


class GMClassifierModel:
    def __init__(self, min_mass_thresh=MIN_MASS_THRESH):
        self.min_mass_thresh = min_mass_thresh
        self.weights = self._init_weights()

    def _init_weights(self):
        weights = [[0] * NUM_CLASSES for _ in range(NUM_CELLS)]
        center = GRID_SIZE // 2
        for cy in range(GRID_SIZE):
            for cx in range(GRID_SIZE):
                addr = cy * GRID_SIZE + cx
                for class_idx in range(NUM_CLASSES):
                    raw = 0
                    if class_idx == 0:  # UP
                        raw = (center - cy) * 6 if cy < center else -((cy - center + 1) * 4)
                    elif class_idx == 1:  # DOWN
                        raw = (cy - center + 1) * 6 if cy >= center else -((center - cy) * 4)
                    elif class_idx == 2:  # LEFT
                        raw = (center - cx) * 6 if cx < center else -((cx - center + 1) * 4)
                    elif class_idx == 3:  # RIGHT
                        raw = (cx - center + 1) * 6 if cx >= center else -((center - cx) * 4)
                    weights[addr][class_idx] = max(-128, min(127, raw))
        return weights

    def classify(self, surface_values):
        energy = sum(surface_values)
        scores = [0] * NUM_CLASSES
        for i in range(NUM_CELLS):
            feat = surface_values[i] & 0xFF
            for k in range(NUM_CLASSES):
                scores[k] += feat * self.weights[i][k]

        best = 0
        for k in range(1, NUM_CLASSES):
            if scores[k] > scores[best]:
                best = k

        if energy >= self.min_mass_thresh:
            confidence = 255 if energy > 65535 else ((energy >> 8) & 0xFF)
            return best, True, confidence, energy
        return 0, False, 0, energy


class GradientCoreGolden:
    def __init__(self):
        self.decoder = Evt2DecoderModel()
        self.surface = GradientSurfaceModel()
        self.classifier = GMClassifierModel()
        self.expected_decoded = deque()

    def on_word_accepted(self, word):
        evt = self.decoder.step(word)
        if evt is not None:
            self.expected_decoded.append(evt)


class GradientCoreHarness:
    def __init__(self, dut):
        self.dut = dut
        self.model = GradientCoreGolden()
        self.observed_gestures = []

    async def setup(self):
        cocotb.start_soon(Clock(self.dut.clk, 10, unit="ns").start())
        await self.reset()

    async def reset(self):
        self.model = GradientCoreGolden()
        self.observed_gestures = []
        self.dut.rst.value = 1
        self.dut.evt_word.value = 0
        self.dut.evt_word_valid.value = 0
        await ClockCycles(self.dut.clk, 8)
        self.dut.rst.value = 0
        await self.tick(4)

    def _sample_cycle(self):
        if int(self.dut.decoded_event_valid.value):
            obs = (
                int(self.dut.u_evt2_decoder.x_out.value),
                int(self.dut.u_evt2_decoder.y_out.value),
                int(self.dut.u_evt2_decoder.polarity.value),
                int(self.dut.u_evt2_decoder.timestamp.value),
            )
            assert self.model.expected_decoded, \
                f"Observed decoded event {obs} with empty expected queue"
            exp = self.model.expected_decoded.popleft()
            assert obs == exp, f"Decoded mismatch: DUT={obs}, model={exp}"
            t_now = int(self.dut.global_timestamp.value)
            # Drive the golden surface from expected decoded events derived from input words.
            self.model.surface.write_event(exp[0], exp[1], t_now)

        if int(self.dut.gesture_valid.value):
            t_now = int(self.dut.global_timestamp.value)
            values = self.model.surface.snapshot(t_now)
            exp_class, exp_valid, exp_conf, exp_energy = self.model.classifier.classify(values)
            assert exp_valid, "DUT asserted gesture_valid while golden model is invalid"
            dut_class = int(self.dut.gesture_class.value)
            dut_conf = int(self.dut.gesture_confidence.value)
            dut_energy = int(self.dut.debug_m00.value)
            conf_from_energy = 255 if dut_energy > 65535 else ((dut_energy >> 8) & 0xFF)
            assert dut_class == exp_class, f"Class mismatch: DUT={dut_class}, model={exp_class}"
            assert dut_conf == conf_from_energy, \
                f"Confidence formula mismatch: DUT={dut_conf}, expected={conf_from_energy} from debug_m00={dut_energy}"
            assert abs(dut_conf - exp_conf) <= 1, \
                f"Confidence mismatch: DUT={dut_conf}, model={exp_conf}"
            assert abs(dut_energy - exp_energy) <= 255, \
                f"Energy mismatch too large: DUT={dut_energy}, model={exp_energy}"
            self.observed_gestures.append((dut_class, dut_conf))

    async def tick(self, cycles=1):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk)
            self._sample_cycle()

    async def send_word(self, word):
        while int(self.dut.evt_word_ready.value) == 0:
            await self.tick()
        self.dut.evt_word.value = word
        self.dut.evt_word_valid.value = 1
        await self.tick()
        self.dut.evt_word_valid.value = 0
        self.model.on_word_accepted(word)

    async def wait_for_gesture(self, timeout_cycles=6000):
        start = len(self.observed_gestures)
        for _ in range(timeout_cycles):
            await self.tick()
            if len(self.observed_gestures) > start:
                return self.observed_gestures[-1]
        raise AssertionError("Timed out waiting for gesture_valid")


def region_points(name):
    pts = []
    if name == "top":
        ys, xs = range(1, 6), range(3, 13)
    elif name == "bottom":
        ys, xs = range(10, 15), range(3, 13)
    elif name == "left":
        ys, xs = range(3, 13), range(1, 6)
    elif name == "right":
        ys, xs = range(3, 13), range(10, 15)
    else:
        raise ValueError(f"Unknown region {name}")

    for y in ys:
        for x in xs:
            pts.append((x, y))
    return pts


async def drive_direction_burst(h, rng, region_name, num_events=260):
    pts = region_points(region_name)
    await h.send_word(build_evt2_time_high(rng.randint(0, 0x3FF)))
    for i in range(num_events):
        gx, gy = rng.choice(pts)
        x_sensor = sensor_from_grid(gx, rng.randint(0, 15))
        y_sensor = sensor_from_grid(gy, rng.randint(0, 15))
        pkt = EVT_CD_ON if (i & 1) else EVT_CD_OFF
        await h.send_word(build_evt2_cd(pkt, x_sensor, y_sensor, i & 0x3F))
        if i % 41 == 0:
            bad_word = (0xF << 28) | rng.randint(0, 0x0FFFFFFF)
            await h.send_word(bad_word)


@cocotb.test()
async def test_gradient_map_core_golden_directions(dut):
    """Directional EVT2 bursts should classify to the expected gestures."""
    rng = random.Random(0x7A11D1)
    h = GradientCoreHarness(dut)
    await h.setup()

    direction_cases = [
        (0, "top"),
        (1, "bottom"),
        (2, "left"),
        (3, "right"),
    ]

    for expected_class, region in direction_cases:
        await h.reset()
        await drive_direction_burst(h, rng, region_name=region, num_events=280)
        g, _ = await h.wait_for_gesture()
        assert g == expected_class, \
            f"Direction {region}: expected class {expected_class}, got {g}"
        assert not h.model.expected_decoded, \
            f"Direction {region}: unconsumed decoded queue {len(h.model.expected_decoded)}"


@cocotb.test()
async def test_gradient_map_core_golden_random_and_edges(dut):
    """Randomized frame-batched EVT2 stream with edge packets and scoreboard checks."""
    rng = random.Random(0xBAD5EED)
    h = GradientCoreHarness(dut)
    await h.setup()

    gestures_seen = 0
    for round_idx in range(6):
        await h.reset()

        await h.send_word(build_evt2_time_high(rng.randint(0, 0x0FFFFFFF)))
        await h.send_word(build_evt2_cd(EVT_CD_ON, 0x7FF, 0x7FF, 0x2A))  # clamp edge
        await h.send_word((0x2 << 28) | 0x1234567)  # ignored packet

        for i in range(420):
            pick = rng.random()
            if pick < 0.10:
                word = build_evt2_time_high(rng.randint(0, 0x0FFFFFFF))
            elif pick < 0.86:
                pkt = EVT_CD_ON if rng.randint(0, 1) else EVT_CD_OFF
                x_sensor = rng.randint(0, 0x7FF)
                y_sensor = rng.randint(0, 0x7FF)
                ts_lsb = rng.randint(0, 63)
                word = build_evt2_cd(pkt, x_sensor, y_sensor, ts_lsb)
            else:
                word = ((rng.choice([0x2, 0x3, 0x4, 0x5, 0x9, 0xF]) & 0xF) << 28) | \
                    rng.randint(0, 0x0FFFFFFF)
            await h.send_word(word)

            if i % 23 == 0:
                await h.tick(rng.randint(0, 2))

        await h.wait_for_gesture(timeout_cycles=9000)
        gestures_seen += len(h.observed_gestures)
        assert not h.model.expected_decoded, \
            f"Round {round_idx}: unconsumed decoded expectations {len(h.model.expected_decoded)}"

    assert gestures_seen > 0, "No gestures observed in randomized edge stress test"
