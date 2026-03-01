"""Unit testbench for evt2_decoder with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly, NextTimeStep
import random

GRID_BITS = 4


class Evt2DecoderModel:
    EVT_CD_OFF = 0x0
    EVT_CD_ON = 0x1
    EVT_TIME_HIGH = 0x8

    def __init__(self):
        self.reset()

    def reset(self):
        self.time_high_reg = 0
        self.x_out = 0
        self.y_out = 0
        self.polarity = 0
        self.timestamp = 0
        self.event_valid = 0

    def step(self, data_in, data_valid):
        self.event_valid = 0
        if data_valid:
            pkt_type = (data_in >> 28) & 0xF
            ts_lsb = (data_in >> 22) & 0x3F
            x_raw = (data_in >> 11) & 0x7FF
            y_raw = data_in & 0x7FF

            x_grid = min((x_raw >> 4) & 0x1F, 15)
            y_grid = min((y_raw >> 4) & 0x1F, 15)
            full_ts = ((self.time_high_reg & 0x3FF) << 6) | ts_lsb

            if pkt_type == self.EVT_TIME_HIGH:
                self.time_high_reg = data_in & 0x0FFFFFFF
            elif pkt_type in (self.EVT_CD_OFF, self.EVT_CD_ON):
                self.x_out = x_grid
                self.y_out = y_grid
                self.polarity = 1 if pkt_type == self.EVT_CD_ON else 0
                self.timestamp = full_ts
                self.event_valid = 1
        return self.x_out, self.y_out, self.polarity, self.timestamp, self.event_valid


def build_cd(pkt_type, x, y, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
           ((x & 0x7FF) << 11) | (y & 0x7FF)


def build_time_high(payload):
    return (0x8 << 28) | (payload & 0x0FFFFFFF)


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.data_in.value = 0
    dut.data_valid.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive(dut, word):
    dut.data_in.value = word
    dut.data_valid.value = 1
    await RisingEdge(dut.clk)
    dut.data_valid.value = 0


@cocotb.test()
async def test_reset(dut):
    await setup(dut)
    assert int(dut.event_valid.value) == 0


@cocotb.test()
async def test_cd_on(dut):
    await setup(dut)
    word = build_cd(0x1, 160, 240, 10)
    await drive(dut, word)
    await RisingEdge(dut.clk)
    assert int(dut.event_valid.value) == 1
    assert int(dut.polarity.value) == 1
    assert int(dut.x_out.value) == min((160 >> 4) & 0x1F, 15)
    assert int(dut.y_out.value) == min((240 >> 4) & 0x1F, 15)


@cocotb.test()
async def test_cd_off(dut):
    await setup(dut)
    word = build_cd(0x0, 80, 80, 5)
    await drive(dut, word)
    await RisingEdge(dut.clk)
    assert int(dut.event_valid.value) == 1
    assert int(dut.polarity.value) == 0


@cocotb.test()
async def test_time_high(dut):
    await setup(dut)
    model = Evt2DecoderModel()
    th = build_time_high(0xAB)
    await drive(dut, th)
    model.step(th, 1)
    await RisingEdge(dut.clk)
    model.step(0, 0)

    cd = build_cd(0x1, 128, 128, 0x15)
    await drive(dut, cd)
    _, _, _, mts, _ = model.step(cd, 1)
    await RisingEdge(dut.clk)
    assert int(dut.timestamp.value) == mts


@cocotb.test()
async def test_clamping(dut):
    await setup(dut)
    word = build_cd(0x1, 0x7FF, 0x7FF, 0)
    await drive(dut, word)
    await RisingEdge(dut.clk)
    assert int(dut.x_out.value) == 15
    assert int(dut.y_out.value) == 15


@cocotb.test()
async def test_unknown_types(dut):
    await setup(dut)
    for pt in [0x2, 0x3, 0x5, 0x9, 0xF]:
        word = (pt << 28) | random.randint(0, 0x0FFFFFFF)
        await drive(dut, word)
        await RisingEdge(dut.clk)
        assert int(dut.event_valid.value) == 0


@cocotb.test()
async def test_time_high_rollover_low10bits(dut):
    """Timestamp must use only low 10 bits of TIME_HIGH payload."""
    await setup(dut)
    model = Evt2DecoderModel()

    th = build_time_high(0x3ABCD)
    dut.data_in.value = th
    dut.data_valid.value = 1
    _, _, _, _, mv = model.step(th, 1)
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.event_valid.value) == mv
    await NextTimeStep()

    cd = build_cd(0x1, 64, 96, 0x2A)
    dut.data_in.value = cd
    dut.data_valid.value = 1
    _, _, _, mts, mv = model.step(cd, 1)
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.event_valid.value) == mv
    assert int(dut.timestamp.value) == mts, \
        f"Timestamp rollover mismatch: DUT={int(dut.timestamp.value)}, model={mts}"
    await NextTimeStep()


@cocotb.test()
async def test_golden_random(dut):
    """Cycle-by-cycle random scoreboarding against the golden model."""
    await setup(dut)
    model = Evt2DecoderModel()
    rng = random.Random(0xE172D2)

    for cycle in range(1200):
        pt = rng.choice([0x0, 0x1, 0x8, 0x2, 0x4, 0xF])
        if pt in (0x0, 0x1):
            word = build_cd(pt, rng.randint(0, 0x7FF), rng.randint(0, 0x7FF),
                            rng.randint(0, 63))
        elif pt == 0x8:
            word = build_time_high(rng.randint(0, 0x0FFFFFFF))
        else:
            word = (pt << 28) | rng.randint(0, 0x0FFFFFFF)

        dv = rng.choice([0, 1, 1, 1])
        dut.data_in.value = word
        dut.data_valid.value = dv
        mx, my, mp, mts, mv = model.step(word, dv)
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.event_valid.value) == mv, \
            f"Cycle {cycle}: event_valid DUT={int(dut.event_valid.value)}, model={mv}"
        assert int(dut.x_out.value) == mx, \
            f"Cycle {cycle}: x_out DUT={int(dut.x_out.value)}, model={mx}"
        assert int(dut.y_out.value) == my, \
            f"Cycle {cycle}: y_out DUT={int(dut.y_out.value)}, model={my}"
        assert int(dut.polarity.value) == mp, \
            f"Cycle {cycle}: polarity DUT={int(dut.polarity.value)}, model={mp}"
        assert int(dut.timestamp.value) == mts, \
            f"Cycle {cycle}: timestamp DUT={int(dut.timestamp.value)}, model={mts}"
        assert int(dut.data_ready.value) == dv, \
            f"Cycle {cycle}: data_ready DUT={int(dut.data_ready.value)}, expected={dv}"
        await NextTimeStep()

