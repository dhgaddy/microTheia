"""End-to-end EVT2.0 core testbench for voxel_bin_core with golden scoreboards."""

from collections import deque
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

GRID_SIZE = 16
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE
CLEAR_CYCLES = CELLS_PER_BIN
PERSISTENCE_COUNT = 2

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_from_grid(g):
    return (g & 0xF) << 4


def logic_to_int(val):
    bits = str(val).replace("x", "0").replace("X", "0").replace("z", "0").replace("Z", "0")
    return int(bits, 2)


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


class PersistenceModel:
    def __init__(self, persistence_count=PERSISTENCE_COUNT):
        self.persistence_count = persistence_count
        self.last_gesture = 0
        self.match_count = 0

    def step(self, class_gesture, class_valid, class_pass, abs_delta_x, abs_delta_y):
        gesture = 0
        gesture_valid = 0
        confidence = 0

        if class_valid:
            if class_pass:
                if class_gesture == self.last_gesture:
                    prev_match_count = self.match_count
                    if self.match_count < self.persistence_count:
                        self.match_count += 1

                    if prev_match_count >= self.persistence_count - 1:
                        gesture = class_gesture
                        gesture_valid = 1
                        self.match_count = 0
                        dominant = abs_delta_x if abs_delta_x > abs_delta_y else abs_delta_y
                        confidence = 15 if dominant > 255 else ((dominant >> 4) & 0xF)
                else:
                    self.last_gesture = class_gesture
                    self.match_count = 0
            else:
                self.match_count = 0

        return gesture, gesture_valid, confidence


class VoxelCoreGolden:
    """Golden path:
    1) EVT2 decode stream from accepted words.
    2) Gesture-persistence behavior from classifier-side outputs.
    """

    def __init__(self):
        self.decoder = Evt2DecoderModel()
        self.persistence = PersistenceModel()
        self.expected_decoded = deque()
        self.expected_gestures = []

    def on_word_accepted(self, word):
        evt = self.decoder.step(word)
        if evt is not None:
            self.expected_decoded.append(evt)

    def on_classifier_result(self, best_class, class_pass, pseudo_mag_x):
        g, gv, gc = self.persistence.step(best_class, 1, class_pass, pseudo_mag_x, 0)
        if gv:
            self.expected_gestures.append((g, gc))


class VoxelCoreHarness:
    def __init__(self, dut):
        self.dut = dut
        self.model = VoxelCoreGolden()
        self.observed_gestures = []
        self.readout_starts = 0

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

        if int(self.dut.u_voxel_binning.readout_start.value):
            self.readout_starts += 1

        if int(self.dut.sys_result_valid.value):
            self.model.on_classifier_result(
                logic_to_int(self.dut.sys_best_class.value),
                logic_to_int(self.dut.score_above_thresh.value),
                logic_to_int(self.dut.pseudo_mag_x.value),
            )

        if int(self.dut.gesture_valid.value):
            self.observed_gestures.append((
                int(self.dut.gesture.value),
                int(self.dut.gesture_confidence.value),
            ))

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

    async def wait_next_readout_start(self, timeout=2500):
        target = self.readout_starts + 1
        for _ in range(timeout):
            await self.tick()
            if self.readout_starts >= target:
                return
        raise AssertionError("Timed out waiting for readout_start")


def region_points(name):
    pts = []
    if name == "top":
        ys, xs = range(1, 5), range(4, 12)
    elif name == "bottom":
        ys, xs = range(11, 15), range(4, 12)
    elif name == "left":
        ys, xs = range(4, 12), range(1, 5)
    elif name == "right":
        ys, xs = range(4, 12), range(11, 15)
    else:
        raise ValueError(f"Unknown region {name}")

    for y in ys:
        for x in xs:
            pts.append((x, y))
    return pts


async def drive_bin_traffic(h, rng, region_name, event_count=24):
    await h.tick(CLEAR_CYCLES + 4)
    pts = region_points(region_name)
    for i in range(event_count):
        if i % 11 == 0:
            await h.send_word(build_evt2_time_high(rng.randint(0, 0x3FF)))

        gx, gy = rng.choice(pts)
        x_sensor = sensor_from_grid(gx)
        y_sensor = sensor_from_grid(gy)
        pkt_type = EVT_CD_ON if (i & 1) else EVT_CD_OFF
        await h.send_word(build_evt2_cd(pkt_type, x_sensor, y_sensor, i & 0x3F))

        if i % 13 == 0:
            bad_word = (0xF << 28) | rng.randint(0, 0x0FFFFFFF)
            await h.send_word(bad_word)


@cocotb.test()
async def test_voxel_bin_core_golden_directional(dut):
    """Scripted directional traffic with decode + classifier scoreboards."""
    rng = random.Random(0xB16B00B5)
    h = VoxelCoreHarness(dut)
    await h.setup()
    await h.send_word(build_evt2_time_high(0x0123))
    await h.wait_next_readout_start()

    scripts = [
        ["bottom", "bottom", "top", "top", "bottom", "bottom", "top", "top"],
        ["top", "top", "bottom", "bottom", "top", "top", "bottom", "bottom"],
        ["right", "right", "left", "left", "right", "right", "left", "left"],
        ["left", "left", "right", "right", "left", "left", "right", "right"],
    ]

    for regions in scripts:
        for region in regions:
            await drive_bin_traffic(h, rng, region_name=region, event_count=26)
            await h.wait_next_readout_start()

    for _ in range(4):
        await h.wait_next_readout_start()

    assert not h.model.expected_decoded, \
        f"Unconsumed decoded expectations: {len(h.model.expected_decoded)}"
    assert h.observed_gestures == h.model.expected_gestures, \
        f"Gesture mismatch.\nDUT:   {h.observed_gestures}\nMODEL: {h.model.expected_gestures}"
    assert len(h.observed_gestures) > 0, "No gestures observed in directional run"


@cocotb.test()
async def test_voxel_bin_core_golden_random(dut):
    """Randomized EVT2 traffic with strict decoder + gesture stream checks."""
    rng = random.Random(0xC011E0)
    h = VoxelCoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(rng.randint(0, 0x3FF)))
    await h.wait_next_readout_start()

    for _ in range(28):
        await h.tick(CLEAR_CYCLES + 4)
        events_this_bin = rng.randint(0, 32)
        for _ in range(events_this_bin):
            choose = rng.random()
            if choose < 0.12:
                word = build_evt2_time_high(rng.randint(0, 0x0FFFFFFF))
            elif choose < 0.85:
                x_sensor = rng.randint(0, 0x7FF)
                y_sensor = rng.randint(0, 0x7FF)
                pkt_type = EVT_CD_ON if rng.randint(0, 1) else EVT_CD_OFF
                word = build_evt2_cd(pkt_type, x_sensor, y_sensor, rng.randint(0, 63))
            else:
                word = ((rng.choice([0x2, 0x3, 0x5, 0x9, 0xF]) & 0xF) << 28) | \
                    rng.randint(0, 0x0FFFFFFF)
            await h.send_word(word)

        await h.wait_next_readout_start()

    for _ in range(4):
        await h.wait_next_readout_start()

    assert not h.model.expected_decoded, \
        f"Unconsumed decoded expectations: {len(h.model.expected_decoded)}"
    assert h.observed_gestures == h.model.expected_gestures, \
        f"Random gesture mismatch.\nDUT:   {h.observed_gestures}\nMODEL: {h.model.expected_gestures}"

