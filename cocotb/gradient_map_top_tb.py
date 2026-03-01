"""Top-level EVT2 UART testbench for gradient_map_top with golden scoreboards."""

from collections import deque
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Event, RisingEdge

CLK_FREQ_HZ = 1_000_000
BAUD_RATE = 250_000
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE

GRID_SIZE = 16
NUM_CELLS = GRID_SIZE * GRID_SIZE
NUM_CLASSES = 4
VALUE_BITS = 8
MAX_VALUE = 255
TS_MASK = 0xFFFF
DECAY_SHIFT = 6
MIN_MASS_THRESH = 20

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

GESTURE_TO_MSG = {
    0: b"UP\r\n",
    1: b"DOWN\r\n",
    2: b"LEFT\r\n",
    3: b"RIGHT\r\n",
}


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_from_grid(g, jitter):
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
                    if class_idx == 0:
                        raw = (center - cy) * 6 if cy < center else -((cy - center + 1) * 4)
                    elif class_idx == 1:
                        raw = (cy - center + 1) * 6 if cy >= center else -((center - cy) * 4)
                    elif class_idx == 2:
                        raw = (center - cx) * 6 if cx < center else -((cx - center + 1) * 4)
                    elif class_idx == 3:
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
            return best, True, confidence, energy, scores
        return 0, False, 0, energy, scores


class GradientTopHarness:
    def __init__(self, dut):
        self.dut = dut
        self.decoder = Evt2DecoderModel()
        self.surface = GradientSurfaceModel()
        self.classifier = GMClassifierModel()
        self.expected_decoded = deque()
        self.expected_uart_bytes = deque()
        self.observed_uart_bytes = []
        self.observed_gestures = []
        self.cycle = 0
        self._uart_stop = Event()
        self._uart_task = None

    async def setup(self):
        cocotb.start_soon(Clock(self.dut.clk, 10, unit="ns").start())
        self.dut.uart_rx.value = 1
        await ClockCycles(self.dut.clk, 16)
        await self._wait_por_release()
        self._uart_task = cocotb.start_soon(self._uart_tx_monitor())
        await self.tick(8)

    async def shutdown(self):
        self._uart_stop.set()
        await self.tick(CLKS_PER_BIT * 12)

    async def _wait_por_release(self):
        stable = 0
        for _ in range(800):
            await RisingEdge(self.dut.clk)
            self.cycle += 1
            if int(self.dut.rst.value) == 0:
                stable += 1
                if stable >= 6:
                    return
            else:
                stable = 0
        raise AssertionError("Timed out waiting for internal reset deassertion")

    def _on_word_streamed(self, word):
        evt = self.decoder.step(word)
        if evt is not None:
            self.expected_decoded.append(evt)

    def _enqueue_uart_message(self, gesture_class):
        msg = GESTURE_TO_MSG[gesture_class]
        for b in msg:
            self.expected_uart_bytes.append(int(b))

    def _sample_cycle(self):
        if int(self.dut.rst.value):
            return

        if int(self.dut.u_core.decoded_event_valid.value):
            obs = (
                int(self.dut.u_core.u_evt2_decoder.x_out.value),
                int(self.dut.u_core.u_evt2_decoder.y_out.value),
                int(self.dut.u_core.u_evt2_decoder.polarity.value),
                int(self.dut.u_core.u_evt2_decoder.timestamp.value),
            )
            assert self.expected_decoded, \
                f"Decoded event observed with empty model queue: {obs}"
            exp = self.expected_decoded.popleft()
            assert obs == exp, f"Decoded mismatch: DUT={obs}, model={exp}"
            t_now = int(self.dut.u_core.global_timestamp.value)
            self.surface.write_event(exp[0], exp[1], t_now)

        if int(self.dut.u_core.gesture_valid.value):
            dut_class = int(self.dut.u_core.gesture_class.value)
            dut_conf = int(self.dut.u_core.gesture_confidence.value)
            dut_energy = int(self.dut.u_core.debug_m00.value)
            conf_from_energy = 255 if dut_energy > 65535 else ((dut_energy >> 8) & 0xFF)
            t_now = int(self.dut.u_core.global_timestamp.value)
            snapshot = self.surface.snapshot(t_now)
            exp_class, exp_valid, exp_conf, _, exp_scores = self.classifier.classify(snapshot)
            assert exp_valid, "DUT asserted gesture_valid while model is invalid"
            assert dut_conf == conf_from_energy, \
                f"Confidence formula mismatch: DUT={dut_conf}, expected={conf_from_energy} from debug_m00={dut_energy}"
            assert abs(dut_conf - exp_conf) <= 4, \
                f"Confidence mismatch too large: DUT={dut_conf}, model={exp_conf}"

            self.observed_gestures.append((dut_class, dut_conf))
            self._enqueue_uart_message(dut_class)

    async def tick(self, cycles=1):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk)
            self.cycle += 1
            self._sample_cycle()

    async def _drive_bit(self, bit, cycles):
        self.dut.uart_rx.value = bit
        await self.tick(cycles)

    async def send_uart_byte(self, byte_val):
        await self._drive_bit(0, CLKS_PER_BIT)
        for i in range(8):
            await self._drive_bit((byte_val >> i) & 1, CLKS_PER_BIT)
        await self._drive_bit(1, CLKS_PER_BIT)

    async def send_evt2_word(self, word):
        for shift in (24, 16, 8, 0):
            await self.send_uart_byte((word >> shift) & 0xFF)
        self._on_word_streamed(word)

    async def wait_for_new_gesture(self, baseline_count, timeout_cycles=100000):
        for _ in range(timeout_cycles):
            await self.tick()
            if len(self.observed_gestures) > baseline_count:
                return self.observed_gestures[-1]
        raise AssertionError("Timed out waiting for gesture_valid")

    async def wait_uart_drain(self, timeout_cycles=140000):
        quiet = 0
        last_len = len(self.observed_uart_bytes)
        for _ in range(timeout_cycles):
            await self.tick()
            now_len = len(self.observed_uart_bytes)
            if now_len == last_len and not self.expected_uart_bytes:
                quiet += 1
                if quiet >= CLKS_PER_BIT * 12:
                    return
            else:
                quiet = 0
                last_len = now_len
        raise AssertionError("Timed out waiting for UART TX queue to drain")

    async def _uart_tx_monitor(self):
        while not self._uart_stop.is_set():
            await RisingEdge(self.dut.clk)
            if self._uart_stop.is_set():
                return

            if int(self.dut.uart_tx.value) != 0:
                continue

            await ClockCycles(self.dut.clk, CLKS_PER_BIT // 2)
            if int(self.dut.uart_tx.value) != 0:
                continue

            byte_val = 0
            for i in range(8):
                await ClockCycles(self.dut.clk, CLKS_PER_BIT)
                if int(self.dut.uart_tx.value):
                    byte_val |= 1 << i

            await ClockCycles(self.dut.clk, CLKS_PER_BIT)
            assert int(self.dut.uart_tx.value) == 1, "UART TX framing error: stop bit low"

            self.observed_uart_bytes.append(byte_val)
            assert self.expected_uart_bytes, \
                f"Unexpected UART byte 0x{byte_val:02X} with empty expected queue"
            exp = self.expected_uart_bytes.popleft()
            assert byte_val == exp, f"UART byte mismatch: DUT=0x{byte_val:02X}, model=0x{exp:02X}"


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
        raise ValueError(f"Unknown region: {name}")

    for y in ys:
        for x in xs:
            pts.append((x, y))
    return pts


async def drive_direction_burst(h, rng, region_name, num_events=96):
    pts = region_points(region_name)
    await h.send_evt2_word(build_evt2_time_high(rng.randint(0, 0x3FF)))

    for i in range(num_events):
        gx, gy = rng.choice(pts)
        x_sensor = sensor_from_grid(gx, rng.randint(0, 15))
        y_sensor = sensor_from_grid(gy, rng.randint(0, 15))
        pkt = EVT_CD_ON if (i & 1) else EVT_CD_OFF
        await h.send_evt2_word(build_evt2_cd(pkt, x_sensor, y_sensor, i & 0x3F))

        if i % 31 == 0:
            bad_type = rng.choice([0x2, 0x3, 0x4, 0x5, 0x9, 0xA])
            bad_word = (bad_type << 28) | rng.randint(0, 0x0FFFFFFF)
            await h.send_evt2_word(bad_word)

        if i % 13 == 0:
            await h.tick(rng.randint(0, 4))


@cocotb.test()
async def test_gradient_map_top_uart_evt2_golden(dut):
    """End-to-end top-level scoreboard: UART EVT2 in -> gesture UART text out."""
    rng = random.Random(0x51A0C7)
    h = GradientTopHarness(dut)
    await h.setup()

    direction_cases = ["top"]
    for region_name in direction_cases:
        baseline = len(h.observed_gestures)
        await drive_direction_burst(h, rng, region_name=region_name, num_events=120)
        await h.wait_for_new_gesture(baseline, timeout_cycles=120000)
        # Let old timestamps decay to reduce cross-direction contamination.
        await h.tick(6000)

    await h.wait_uart_drain(timeout_cycles=200000)

    assert not h.expected_decoded, \
        f"Unconsumed decoded expectations: {len(h.expected_decoded)}"
    assert not h.expected_uart_bytes, \
        f"Unsent expected UART bytes: {len(h.expected_uart_bytes)}"
    assert len(h.observed_gestures) >= len(direction_cases), \
        "Did not observe enough gestures for directional burst"

    await h.shutdown()
