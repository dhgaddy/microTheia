"""Robust cocotb testbench for evt2_decoder with cycle-accurate golden model."""

import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
import math
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

GRID_SIZE = CFG["GRID_SIZE"]
SENSOR_WIDTH = CFG["SENSOR_WIDTH"]
SENSOR_HEIGHT = CFG["SENSOR_HEIGHT"]

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

GRID_BITS = math.sqrt(GRID_SIZE)
REQUIRE_TIME_HIGH = 1


def build_evt2_cd(pkt_type, x, y, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | ((x & 0x7FF) << 11) | (y & 0x7FF)


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


class Evt2DecoderModel:
    def __init__(self):
        self.reset()

    def reset(self):
        self.time_high_reg = 0
        self.have_time_high = 0
        self.x_out = 0
        self.y_out = 0
        self.polarity = 0
        self.timestamp = 0
        self.event_valid = 0

    @staticmethod
    def _grid_map(x_raw, y_raw):
        x_clamped = x_raw if x_raw < SENSOR_WIDTH else SENSOR_WIDTH - 1
        y_clamped = y_raw if y_raw < SENSOR_HEIGHT else SENSOR_HEIGHT - 1
        x_grid = x_clamped // (SENSOR_WIDTH // GRID_SIZE)
        y_grid = y_clamped // (SENSOR_HEIGHT // GRID_SIZE)
        x_grid = min(x_grid, GRID_SIZE - 1)
        y_grid = min(y_grid, GRID_SIZE - 1)
        return x_grid, y_grid

    def step(self, rst, data_in, data_valid, event_ready_i):
        pkt_type = (data_in >> 28) & 0xF
        ts_lsb = (data_in >> 22) & 0x3F
        x_raw = (data_in >> 11) & 0x7FF
        y_raw = data_in & 0x7FF
        time_high_payload = data_in & 0x0FFFFFFF

        is_cd = pkt_type in (EVT_CD_OFF, EVT_CD_ON)
        data_ready = int((not is_cd) or event_ready_i)

        if rst:
            self.reset()
            return data_ready

        self.event_valid = 0
        if data_valid and data_ready:
            if pkt_type == EVT_TIME_HIGH:
                self.time_high_reg = time_high_payload
                self.have_time_high = 1
            elif pkt_type in (EVT_CD_OFF, EVT_CD_ON):
                if (not REQUIRE_TIME_HIGH) or self.have_time_high:
                    self.x_out, self.y_out = self._grid_map(x_raw, y_raw)
                    self.polarity = 1 if pkt_type == EVT_CD_ON else 0
                    self.timestamp = ((self.time_high_reg & 0x0FFFFFFF) << 6) | ts_lsb
                    self.event_valid = 1

        return data_ready


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.data_in.value = 0
    dut.data_valid.value = 0
    dut.event_ready_i.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive_and_check(dut, model, rst, data_in, data_valid, event_ready_i, tag):
    dut.rst.value = rst
    dut.data_in.value = data_in
    dut.data_valid.value = data_valid
    dut.event_ready_i.value = event_ready_i

    exp_data_ready = model.step(rst, data_in, data_valid, event_ready_i)

    await RisingEdge(dut.clk)
    await ReadOnly()

    assert int(dut.data_ready.value) == exp_data_ready, \
        f"{tag}: data_ready DUT={int(dut.data_ready.value)} model={exp_data_ready}"
    assert int(dut.x_out.value) == model.x_out, \
        f"{tag}: x_out DUT={int(dut.x_out.value)} model={model.x_out}"
    assert int(dut.y_out.value) == model.y_out, \
        f"{tag}: y_out DUT={int(dut.y_out.value)} model={model.y_out}"
    assert int(dut.polarity.value) == model.polarity, \
        f"{tag}: polarity DUT={int(dut.polarity.value)} model={model.polarity}"
    assert int(dut.timestamp.value) == model.timestamp, \
        f"{tag}: timestamp DUT={int(dut.timestamp.value)} model={model.timestamp}"
    assert int(dut.event_valid.value) == model.event_valid, \
        f"{tag}: event_valid DUT={int(dut.event_valid.value)} model={model.event_valid}"

    await NextTimeStep()


@logged_test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.event_valid.value) == 0
    assert int(dut.x_out.value) == 0
    assert int(dut.y_out.value) == 0


@logged_test()
async def test_cd_requires_time_high(dut):
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    cd_word = build_evt2_cd(EVT_CD_ON, 100, 120, 7)
    await drive_and_check(dut, model, 0, cd_word, 1, 1, "cd-before-th")
    assert int(dut.event_valid.value) == 0

    th = build_evt2_time_high(0x12345)
    await drive_and_check(dut, model, 0, th, 1, 1, "th")
    await drive_and_check(dut, model, 0, cd_word, 1, 1, "cd-after-th")
    assert int(dut.event_valid.value) == 1


@logged_test()
async def test_backpressure_on_cd_only(dut):
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    th = build_evt2_time_high(0x77)
    await drive_and_check(dut, model, 0, th, 1, 0, "th-ready-low")
    assert int(dut.data_ready.value) == 1, "TIME_HIGH should ignore event_ready_i"

    cd = build_evt2_cd(EVT_CD_OFF, 50, 60, 1)
    await drive_and_check(dut, model, 0, cd, 1, 0, "cd-stall")
    assert int(dut.data_ready.value) == 0
    assert int(dut.event_valid.value) == 0

    await drive_and_check(dut, model, 0, cd, 1, 1, "cd-accept")
    assert int(dut.event_valid.value) == 1


@logged_test()
async def test_coordinate_clamp_and_timestamp(dut):
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0x0FFFFFFF), 1, 1, "th")
    word = build_evt2_cd(EVT_CD_ON, 0x7FF, 0x7FF, 0x2A)
    await drive_and_check(dut, model, 0, word, 1, 1, "clamp")

    assert int(dut.x_out.value) == GRID_SIZE - 1
    assert int(dut.y_out.value) == GRID_SIZE - 1
    exp_ts = ((0x0FFFFFFF << 6) | 0x2A)
    assert int(dut.timestamp.value) == exp_ts


@logged_test()
async def test_randomized_golden_scoreboard(dut):
    await setup(dut)
    model = Evt2DecoderModel()
    rng = random.Random(0xE172D2)

    # Mirror setup timing in model.
    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    for cycle in range(1800):
        pkt_type = rng.choice([0x0, 0x1, 0x2, 0x5, 0x8, 0xF])
        if pkt_type in (EVT_CD_OFF, EVT_CD_ON):
            word = build_evt2_cd(pkt_type, rng.randint(0, 0x7FF), rng.randint(0, 0x7FF), rng.randint(0, 63))
        elif pkt_type == EVT_TIME_HIGH:
            word = build_evt2_time_high(rng.randint(0, 0x0FFFFFFF))
        else:
            word = (pkt_type << 28) | rng.randint(0, 0x0FFFFFFF)

        dv = rng.choice([0, 1, 1, 1])
        ready = rng.choice([0, 1, 1])
        await drive_and_check(dut, model, 0, word, dv, ready, f"rnd-{cycle}")


@logged_test()
async def test_polarity_off_vs_on(dut):
    """CD_OFF → polarity=0; CD_ON → polarity=1."""
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0x100), 1, 1, "th")

    cd_off = build_evt2_cd(EVT_CD_OFF, 10, 10, 5)
    await drive_and_check(dut, model, 0, cd_off, 1, 1, "cd-off")
    assert int(dut.event_valid.value) == 1, "Expected event_valid for CD_OFF"
    assert int(dut.polarity.value) == 0, "CD_OFF should set polarity=0"

    cd_on = build_evt2_cd(EVT_CD_ON, 20, 20, 6)
    await drive_and_check(dut, model, 0, cd_on, 1, 1, "cd-on")
    assert int(dut.event_valid.value) == 1, "Expected event_valid for CD_ON"
    assert int(dut.polarity.value) == 1, "CD_ON should set polarity=1"


@logged_test()
async def test_grid_coordinate_lower_boundaries(dut):
    """Sensor coordinate at exact lower boundary of each grid cell maps to correct cell."""
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0x1), 1, 1, "th")

    step = SENSOR_WIDTH // GRID_SIZE  # = 20
    for g in range(GRID_SIZE):
        x_sensor = g * step  # lower boundary of each cell
        word = build_evt2_cd(EVT_CD_ON, x_sensor, 0, g & 0x3F)
        await drive_and_check(dut, model, 0, word, 1, 1, f"x-lb-{g}")
        assert int(dut.event_valid.value) == 1
        assert int(dut.x_out.value) == g, \
            f"x_sensor={x_sensor} should map to grid {g}, got {int(dut.x_out.value)}"


@logged_test()
async def test_unknown_packet_types_no_event(dut):
    """Packet types other than 0x0, 0x1, 0x8 must not emit event_valid."""
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0xAB), 1, 1, "th")

    for pkt_type in [0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x9, 0xA, 0xF]:
        word = (pkt_type << 28) | 0x00ABCDEF
        await drive_and_check(dut, model, 0, word, 1, 1, f"unk-0x{pkt_type:X}")
        assert int(dut.event_valid.value) == 0, \
            f"Packet type 0x{pkt_type:X} should not emit event_valid"


@logged_test()
async def test_consecutive_cd_events_all_valid(dut):
    """Back-to-back CD events (no stall) all produce event_valid on successive cycles."""
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0x7777), 1, 1, "th")

    step = SENSOR_WIDTH // GRID_SIZE
    coords = [(0, 0), (15, 15), (8, 8), (3, 12), (10, 2)]
    for i, (gx, gy) in enumerate(coords):
        word = build_evt2_cd(EVT_CD_ON, gx * step, gy * step, i & 0x3F)
        await drive_and_check(dut, model, 0, word, 1, 1, f"cd-{i}")
        assert int(dut.event_valid.value) == 1, f"No event_valid for CD event {i}"
        assert int(dut.x_out.value) == gx, f"x_out mismatch at event {i}"
        assert int(dut.y_out.value) == gy, f"y_out mismatch at event {i}"


@logged_test()
async def test_time_high_updates_timestamp_correctly(dut):
    """Multiple TIME_HIGH words update the timestamp base; CD event picks up the latest."""
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    th_values = [0x000001, 0x00ABCD, 0x3FFFFF, 0x0FFFFFF]
    for th_val in th_values:
        await drive_and_check(dut, model, 0, build_evt2_time_high(th_val), 1, 1, f"th-{th_val}")

    ts_lsb = 0x2A
    cd = build_evt2_cd(EVT_CD_ON, 0, 0, ts_lsb)
    await drive_and_check(dut, model, 0, cd, 1, 1, "cd-final")
    assert int(dut.event_valid.value) == 1

    exp_ts = ((th_values[-1] & 0x0FFFFFFF) << 6) | ts_lsb
    assert int(dut.timestamp.value) == exp_ts, \
        f"Timestamp mismatch: DUT=0x{int(dut.timestamp.value):X} expected=0x{exp_ts:X}"


@logged_test()
async def test_reset_clears_time_high_requirement(dut):
    """After reset, REQUIRE_TIME_HIGH blocks events again until a new TIME_HIGH arrives."""
    await setup(dut)
    model = Evt2DecoderModel()

    # Bring model through reset.
    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    # Prime with TIME_HIGH, emit one event.
    await drive_and_check(dut, model, 0, build_evt2_time_high(0x55), 1, 1, "th1")
    cd = build_evt2_cd(EVT_CD_ON, 40, 40, 1)
    await drive_and_check(dut, model, 0, cd, 1, 1, "cd1")
    assert int(dut.event_valid.value) == 1

    # Reset.
    for _ in range(3):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst2")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "post-rst")

    # Without a new TIME_HIGH, a CD event should be suppressed.
    await drive_and_check(dut, model, 0, cd, 1, 1, "cd-no-th")
    assert int(dut.event_valid.value) == 0, \
        "event_valid should be suppressed when no TIME_HIGH since reset"

    # After a fresh TIME_HIGH it should work again.
    await drive_and_check(dut, model, 0, build_evt2_time_high(0x66), 1, 1, "th2")
    await drive_and_check(dut, model, 0, cd, 1, 1, "cd2")
    assert int(dut.event_valid.value) == 1


@logged_test()
async def test_grid_coordinate_upper_boundaries(dut):
    """Sensor coordinate at the upper boundary of each grid cell maps to that cell, not the next."""
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0x2), 1, 1, "th")

    step = SENSOR_WIDTH // GRID_SIZE  # = 20
    for g in range(GRID_SIZE - 1):
        # Upper boundary of cell g is one pixel below the start of cell g+1.
        x_upper = g * step + step - 1
        word = build_evt2_cd(EVT_CD_ON, x_upper, 0, g & 0x3F)
        await drive_and_check(dut, model, 0, word, 1, 1, f"x-ub-{g}")
        assert int(dut.event_valid.value) == 1
        got = int(dut.x_out.value)
        assert got == g, \
            f"x_sensor={x_upper} (upper bound of cell {g}) mapped to {got}, expected {g}"

        # First pixel of the next cell must map to g+1.
        x_next = (g + 1) * step
        word = build_evt2_cd(EVT_CD_ON, x_next, 0, g & 0x3F)
        await drive_and_check(dut, model, 0, word, 1, 1, f"x-next-{g}")
        assert int(dut.event_valid.value) == 1
        got_next = int(dut.x_out.value)
        assert got_next == g + 1, \
            f"x_sensor={x_next} (start of cell {g+1}) mapped to {got_next}, expected {g+1}"
