# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import math
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

GRID_SIZE     = CFG["GRID_SIZE"]
SENSOR_WIDTH  = CFG["SENSOR_WIDTH"]
SENSOR_HEIGHT = CFG["SENSOR_HEIGHT"]
MAP_SWAP_XY = CFG.get("MAP_SWAP_XY", 0)
MAP_FLIP_X = CFG.get("MAP_FLIP_X", 0)
MAP_FLIP_Y = CFG.get("MAP_FLIP_Y", 0)

REQUIRE_TIME_HIGH = 1

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

def build_evt2_cd(pkt_type, x, y, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | ((x & 0x7FF) << 11) | (y & 0x7FF)


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


class Evt2DecoderModel:
    def __init__(self):
        self.reset()

    def reset(self):
        self.have_time_high = 0
        self.x_out = 0
        self.y_out = 0
        self.event_valid = 0

    @staticmethod
    def _grid_map(x_raw, y_raw):
        x_clamped = x_raw if x_raw < SENSOR_WIDTH else SENSOR_WIDTH - 1
        y_clamped = y_raw if y_raw < SENSOR_HEIGHT else SENSOR_HEIGHT - 1

        x_swapped = y_clamped if MAP_SWAP_XY else x_clamped
        y_swapped = x_clamped if MAP_SWAP_XY else y_clamped
        x_oriented = x_swapped if x_swapped < SENSOR_WIDTH else SENSOR_WIDTH - 1
        y_oriented = y_swapped if y_swapped < SENSOR_HEIGHT else SENSOR_HEIGHT - 1
        if MAP_FLIP_X:
            x_oriented = (SENSOR_WIDTH - 1) - x_oriented
        if MAP_FLIP_Y:
            y_oriented = (SENSOR_HEIGHT - 1) - y_oriented

        x_grid = x_oriented // (SENSOR_WIDTH // GRID_SIZE)
        y_grid = y_oriented // (SENSOR_HEIGHT // GRID_SIZE)
        x_grid = min(x_grid, GRID_SIZE - 1)
        y_grid = min(y_grid, GRID_SIZE - 1)
        return x_grid, y_grid

    def step(self, rst, data_in, data_valid, event_ready_i):
        pkt_type = (data_in >> 28) & 0xF
        x_raw = (data_in >> 11) & 0x7FF
        y_raw = data_in & 0x7FF

        is_cd = pkt_type in (EVT_CD_OFF, EVT_CD_ON)
        data_ready = int((not is_cd) or event_ready_i)

        if rst:
            self.reset()
            return data_ready

        self.event_valid = 0
        if data_valid and data_ready:
            if pkt_type == EVT_TIME_HIGH:
                self.have_time_high = 1
            elif pkt_type in (EVT_CD_OFF, EVT_CD_ON):
                if (not REQUIRE_TIME_HIGH) or self.have_time_high:
                    self.x_out, self.y_out = self._grid_map(x_raw, y_raw)
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

    exp_x, exp_y = model._grid_map(0x7FF, 0x7FF)
    assert int(dut.x_out.value) == exp_x, \
        f"clamp x_out mismatch: got {int(dut.x_out.value)} exp {exp_x}"
    assert int(dut.y_out.value) == exp_y, \
        f"clamp y_out mismatch: got {int(dut.y_out.value)} exp {exp_y}"


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
async def test_grid_coordinate_lower_boundaries(dut):
    """Sensor lower boundaries map correctly after configured orientation transform."""
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
        exp_x, exp_y = model._grid_map(x_sensor, 0)
        got_x = int(dut.x_out.value)
        got_y = int(dut.y_out.value)
        assert got_x == exp_x, \
            f"x_sensor={x_sensor} should map to x={exp_x}, got {got_x}"
        assert got_y == exp_y, \
            f"x_sensor={x_sensor} should map to y={exp_y}, got {got_y}"


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
    """Back-to-back CD events (no stall) produce valid transformed coordinates."""
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0x7777), 1, 1, "th")

    step = SENSOR_WIDTH // GRID_SIZE
    coords = [
        (0, 0),
        (GRID_SIZE - 1, GRID_SIZE - 1),
        (GRID_SIZE // 2, GRID_SIZE // 2),
        (GRID_SIZE // 4, (3 * GRID_SIZE) // 4),
        ((5 * GRID_SIZE) // 8, GRID_SIZE // 8),
    ]
    for i, (gx, gy) in enumerate(coords):
        x_sensor = gx * step
        y_sensor = gy * step
        word = build_evt2_cd(EVT_CD_ON, x_sensor, y_sensor, i & 0x3F)
        await drive_and_check(dut, model, 0, word, 1, 1, f"cd-{i}")
        assert int(dut.event_valid.value) == 1, f"No event_valid for CD event {i}"
        exp_x, exp_y = model._grid_map(x_sensor, y_sensor)
        assert int(dut.x_out.value) == exp_x, \
            f"x_out mismatch at event {i}: got {int(dut.x_out.value)} exp {exp_x}"
        assert int(dut.y_out.value) == exp_y, \
            f"y_out mismatch at event {i}: got {int(dut.y_out.value)} exp {exp_y}"


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

    step = SENSOR_WIDTH // GRID_SIZE

    for g in range(GRID_SIZE - 1):
        # Upper boundary of cell g
        x_upper = g * step + step - 1
        word = build_evt2_cd(EVT_CD_ON, x_upper, 0, g & 0x3F)

        await drive_and_check(dut, model, 0, word, 1, 1, f"x-ub-{g}")
        assert int(dut.event_valid.value) == 1

        got = int(dut.x_out.value)
        exp = model._grid_map(x_upper, 0)[0]

        assert got == exp, \
            f"x_sensor={x_upper} mapped to {got}, expected {exp}"

        # First pixel of next cell
        x_next = (g + 1) * step
        word = build_evt2_cd(EVT_CD_ON, x_next, 0, g & 0x3F)

        await drive_and_check(dut, model, 0, word, 1, 1, f"x-next-{g}")
        assert int(dut.event_valid.value) == 1

        got_next = int(dut.x_out.value)
        exp_next = model._grid_map(x_next, 0)[0]

        assert got_next == exp_next, \
            f"x_sensor={x_next} mapped to {got_next}, expected {exp_next}"
