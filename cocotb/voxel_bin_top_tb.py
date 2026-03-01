"""Top-level EVT2 UART testbench for voxel_bin_top with ideal golden models."""

from collections import deque
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Event, RisingEdge

CLK_FREQ_HZ = 1_000_000
BAUD_RATE = 250_000
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE

GRID_SIZE = 16
NUM_BINS = 4
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE
TOTAL_CELLS = NUM_BINS * CELLS_PER_BIN
COUNTER_BITS = 6
MAX_COUNTER = (1 << COUNTER_BITS) - 1
PARALLEL_READS = 4
NUM_CLASSES = 4
PERSISTENCE_COUNT = 2
MIN_SCORE_THRESH = 30

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8


def logic_to_int(val):
    bits = str(val).replace("x", "0").replace("X", "0").replace("z", "0").replace("Z", "0")
    return int(bits, 2)


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


class VoxelBinningModel:
    """Idealized model of voxel_binning ring-buffer accumulation."""

    def __init__(self):
        self.mem = [0] * TOTAL_CELLS
        self.readout_head_snapshot = [0] * CELLS_PER_BIN

    def inject_grid_event(self, bin_idx, x_grid, y_grid):
        cell_addr = ((y_grid & 0xF) << 4) | (x_grid & 0xF)
        full_addr = (bin_idx % NUM_BINS) * CELLS_PER_BIN + cell_addr
        if self.mem[full_addr] < MAX_COUNTER:
            self.mem[full_addr] += 1

    def on_readout_start(self, current_bin_idx):
        idx = current_bin_idx % NUM_BINS
        base = idx * CELLS_PER_BIN
        self.readout_head_snapshot = self.mem[base:base + CELLS_PER_BIN].copy()
        for i in range(CELLS_PER_BIN):
            self.mem[base + i] = 0

    def get_readout(self, current_bin_idx):
        idx0 = current_bin_idx % NUM_BINS
        readout = []
        for bin_offset in range(NUM_BINS):
            idx = (idx0 + bin_offset) % NUM_BINS
            base = idx * CELLS_PER_BIN
            if bin_offset == 0:
                readout.extend(self.readout_head_snapshot)
            else:
                readout.extend(self.mem[base:base + CELLS_PER_BIN])
        return readout


class VoxelWeightClassifierModel:
    """Idealized model of weight_ram + systolic_array + score thresholding."""

    def __init__(self):
        self.weights = self._init_weights()

    def _init_weights(self):
        weights = [[0] * NUM_CLASSES for _ in range(TOTAL_CELLS)]
        half = GRID_SIZE // 2
        for i in range(TOTAL_CELLS):
            spatial_addr = i % CELLS_PER_BIN
            bin_idx = i // CELLS_PER_BIN
            cy = spatial_addr // GRID_SIZE
            cx = spatial_addr % GRID_SIZE
            temp_phase = 1 if bin_idx >= 2 else -1

            for class_idx in range(NUM_CLASSES):
                if class_idx == 0:
                    raw = temp_phase * (half - cy) if cy < half else -(temp_phase * (cy - half + 1))
                elif class_idx == 1:
                    raw = temp_phase * (cy - half + 1) if cy >= half else -(temp_phase * (half - cy))
                elif class_idx == 2:
                    raw = temp_phase * (half - cx) if cx < half else -(temp_phase * (cx - half + 1))
                else:
                    raw = temp_phase * (cx - half + 1) if cx >= half else -(temp_phase * (half - cx))
                weights[i][class_idx] = max(-128, min(127, raw))
        return weights

    def classify(self, features):
        scores = [0] * NUM_CLASSES
        for i, feat in enumerate(features):
            f = feat & MAX_COUNTER
            for k in range(NUM_CLASSES):
                scores[k] += f * self.weights[i][k]

        best = 0
        for k in range(1, NUM_CLASSES):
            if scores[k] > scores[best]:
                best = k

        abs_best = abs(scores[best])
        class_pass = abs_best >= MIN_SCORE_THRESH
        pseudo_mag_x = abs_best & 0xFFFF
        conf = 15 if pseudo_mag_x > 255 else ((pseudo_mag_x >> 4) & 0xF)
        return best, int(class_pass), pseudo_mag_x, conf


class PersistenceModel:
    """Idealized model of voxel gesture_classifier persistence filter."""

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


class VoxelTopHarness:
    def __init__(self, dut):
        self.dut = dut
        self.decoder = Evt2DecoderModel()
        self.binning = VoxelBinningModel()
        self.classifier = VoxelWeightClassifierModel()
        self.persistence_dut = PersistenceModel(PERSISTENCE_COUNT)

        self.expected_decoded = deque()
        self.expected_sys = deque()
        self.expected_gesture = deque()
        self.expected_uart = deque()

        self.observed_uart_bytes = []
        self.readout_start_count = 0
        self.cycle = 0
        self.model_sys_samples = 0
        self.model_sys_mismatches = 0

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

    def _enqueue_uart_byte(self, val, mask=0xFF, tag=""):
        self.expected_uart.append((val & 0xFF, mask & 0xFF, tag))

    def _sample_cycle(self):
        if int(self.dut.rst.value):
            return

        if int(self.dut.u_core.decoded_valid.value):
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
            dut_state = logic_to_int(self.dut.u_core.u_voxel_binning.state.value)
            dut_bin_idx = logic_to_int(self.dut.u_core.u_voxel_binning.current_bin_idx.value)
            # Voxel-binning gives clear writes priority; ignore events while clearing.
            if dut_state != 2:
                self.binning.inject_grid_event(dut_bin_idx, exp[0], exp[1])

        if int(self.dut.u_core.u_voxel_binning.readout_start.value):
            self.readout_start_count += 1
            dut_bin_idx = logic_to_int(self.dut.u_core.u_voxel_binning.current_bin_idx.value)
            self.binning.on_readout_start(dut_bin_idx)
            features = self.binning.get_readout(dut_bin_idx)
            best, class_pass, pseudo_mag_x, conf = self.classifier.classify(features)
            self.expected_sys.append((best, class_pass, pseudo_mag_x))

        if int(self.dut.u_core.sys_result_valid.value):
            assert self.expected_sys, "sys_result_valid observed with empty expected queue"
            exp_best, exp_pass, exp_mag = self.expected_sys.popleft()

            dut_best = logic_to_int(self.dut.u_core.sys_best_class.value)
            dut_pass = logic_to_int(self.dut.u_core.score_above_thresh.value)
            dut_mag = logic_to_int(self.dut.u_core.pseudo_mag_x.value) & 0xFFFF

            self.model_sys_samples += 1
            if (dut_best != exp_best) or (dut_pass != exp_pass) or (dut_mag != (exp_mag & 0xFFFF)):
                self.model_sys_mismatches += 1

            mg, mv, mc = self.persistence_dut.step(dut_best, 1, dut_pass, dut_mag, 0)
            if mv:
                self.expected_gesture.append((mg, mc))
                self._enqueue_uart_byte(0xA0 | mg, 0xFF, "gesture-header")
                self._enqueue_uart_byte((mc & 0xF) << 4, 0xF0, "confidence-high-nibble")

        if int(self.dut.u_core.gesture_valid.value):
            assert self.expected_gesture, "gesture_valid observed with empty expected queue"
            exp_g, exp_c = self.expected_gesture.popleft()
            dut_g = int(self.dut.u_core.gesture.value)
            dut_c = int(self.dut.u_core.gesture_confidence.value)

            dut_mag_x = logic_to_int(self.dut.u_core.pseudo_mag_x.value)
            conf_from_mag = 15 if dut_mag_x > 255 else ((dut_mag_x >> 4) & 0xF)
            assert dut_c == conf_from_mag, \
                f"Confidence formula mismatch: DUT={dut_c}, expected={conf_from_mag} from pseudo_mag_x={dut_mag_x}"

            assert dut_g == exp_g, f"gesture mismatch: DUT={dut_g}, model={exp_g}"
            assert dut_c == exp_c, f"gesture_confidence mismatch: DUT={dut_c}, model={exp_c}"

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

    async def send_command_echo(self):
        self._enqueue_uart_byte(0x55, 0xFF, "echo")
        await self.send_uart_byte(0xFF)

    async def send_evt2_word(self, word):
        first = (word >> 24) & 0xFF
        assert first not in (0xFC, 0xFD, 0xFE, 0xFF), \
            f"Word 0x{word:08X} collides with command-byte parser in top"
        for shift in (24, 16, 8, 0):
            await self.send_uart_byte((word >> shift) & 0xFF)
        self._on_word_streamed(word)

    async def wait_next_readout_start(self, timeout_cycles=300000):
        target = self.readout_start_count + 1
        for _ in range(timeout_cycles):
            await self.tick()
            if self.readout_start_count >= target:
                return
        raise AssertionError("Timed out waiting for readout_start")

    async def wait_uart_drain(self, timeout_cycles=320000):
        quiet = 0
        last_len = len(self.observed_uart_bytes)
        for _ in range(timeout_cycles):
            await self.tick()
            now_len = len(self.observed_uart_bytes)
            if now_len == last_len and not self.expected_uart and not self.expected_gesture and not self.expected_sys:
                quiet += 1
                if quiet >= CLKS_PER_BIT * 14:
                    return
            else:
                quiet = 0
                last_len = now_len
        raise AssertionError("Timed out waiting for UART/model queues to drain")

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
            assert self.expected_uart, \
                f"Unexpected UART byte 0x{byte_val:02X} with empty expected queue"
            exp_val, exp_mask, tag = self.expected_uart.popleft()
            assert (byte_val & exp_mask) == (exp_val & exp_mask), \
                f"{tag} mismatch: DUT=0x{byte_val:02X}, model=0x{exp_val:02X}, mask=0x{exp_mask:02X}"


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
        raise ValueError(f"Unknown region: {name}")

    for y in ys:
        for x in xs:
            pts.append((x, y))
    return pts


async def drive_region_words(h, rng, region_name, words=10):
    pts = region_points(region_name)
    for i in range(words):
        gx, gy = rng.choice(pts)
        x_sensor = sensor_from_grid(gx, rng.randint(0, 15))
        y_sensor = sensor_from_grid(gy, rng.randint(0, 15))
        pkt = EVT_CD_ON if (i & 1) else EVT_CD_OFF
        await h.send_evt2_word(build_evt2_cd(pkt, x_sensor, y_sensor, i & 0x3F))


@cocotb.test()
async def test_voxel_bin_top_uart_evt2_golden(dut):
    """End-to-end ideal-model scoreboard: EVT2 UART -> binning/classifier -> UART packets."""
    rng = random.Random(0xA91B57)
    h = VoxelTopHarness(dut)
    await h.setup()

    await h.send_command_echo()
    await h.send_evt2_word(build_evt2_time_high(0x0123))

    await h.wait_next_readout_start()

    # Consecutive-region script to exercise persistence and directional response.
    script = [
        "bottom", "bottom", "top", "top", "bottom", "bottom", "top", "top",
        "right", "right", "left", "left", "right", "right", "left", "left",
    ]

    for region in script:
        # Keep events away from clear boundaries so accumulation semantics are deterministic.
        await h.tick(300)
        await drive_region_words(h, rng, region_name=region, words=10)
        await h.wait_next_readout_start()

    # Flush final pipeline outputs.
    for _ in range(4):
        await h.wait_next_readout_start()

    await h.wait_uart_drain()

    assert not h.expected_decoded, \
        f"Unconsumed decoded expectations: {len(h.expected_decoded)}"
    assert not h.expected_sys, \
        f"Unconsumed classifier expectations: {len(h.expected_sys)}"
    assert not h.expected_gesture, \
        f"Unconsumed gesture expectations: {len(h.expected_gesture)}"
    assert not h.expected_uart, \
        f"Unsent expected UART bytes: {len(h.expected_uart)}"
    assert h.model_sys_samples > 0, "No classifier model comparison samples were collected"

    await h.shutdown()
