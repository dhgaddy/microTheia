"""Unit testbench for evt2_decoder with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly, NextTimeStep
import random

GRID_BITS = 4


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class Evt2DecoderModel:
    """Cycle-accurate model of the EVT2 decoder."""

    EVT_CD_OFF = 0x0
    EVT_CD_ON = 0x1
    EVT_TIME_HIGH = 0x8

    def __init__(self, grid_bits=GRID_BITS):
        self.grid_bits = grid_bits
        self.reset()

    def reset(self):
        self.time_high_reg = 0
        self.x_out = 0
        self.y_out = 0
        self.polarity = 0
        self.timestamp = 0
        self.event_valid = 0

    def step(self, data_in, data_valid):
        """One clock cycle. Returns (x, y, pol, ts, event_valid)."""
        self.event_valid = 0

        if data_valid:
            pkt_type = (data_in >> 28) & 0xF
            ts_lsb = (data_in >> 22) & 0x3F
            x_raw = (data_in >> 11) & 0x7FF
            y_raw = data_in & 0x7FF
            time_high_payload = data_in & 0x0FFFFFFF

            x_grid_raw5 = (x_raw >> 4) & 0x1F
            y_grid_raw5 = (y_raw >> 4) & 0x1F
            x_grid = min(x_grid_raw5, 15)
            y_grid = min(y_grid_raw5, 15)

            full_ts = ((self.time_high_reg & 0x3FF) << 6) | ts_lsb

            if pkt_type == self.EVT_TIME_HIGH:
                self.time_high_reg = time_high_payload

            elif pkt_type == self.EVT_CD_OFF:
                self.x_out = x_grid
                self.y_out = y_grid
                self.polarity = 0
                self.timestamp = full_ts
                self.event_valid = 1

            elif pkt_type == self.EVT_CD_ON:
                self.x_out = x_grid
                self.y_out = y_grid
                self.polarity = 1
                self.timestamp = full_ts
                self.event_valid = 1

        return self.x_out, self.y_out, self.polarity, self.timestamp, self.event_valid


def build_evt2_cd(pkt_type, x, y, ts_lsb):
    """Build a 32-bit EVT2 CD_ON or CD_OFF word."""
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
           ((x & 0x7FF) << 11) | (y & 0x7FF)


def build_evt2_time_high(payload):
    return (0x8 << 28) | (payload & 0x0FFFFFFF)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.data_in.value = 0
    dut.data_valid.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive_word(dut, word):
    dut.data_in.value = word
    dut.data_valid.value = 1
    await RisingEdge(dut.clk)
    dut.data_valid.value = 0


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """All outputs zero after reset."""
    await setup(dut)
    assert int(dut.event_valid.value) == 0
    assert int(dut.x_out.value) == 0
    assert int(dut.y_out.value) == 0


@cocotb.test()
async def test_cd_on_event(dut):
    """CD_ON event should decode correctly."""
    await setup(dut)
    x_sensor, y_sensor = 160, 240
    ts_lsb = 10
    word = build_evt2_cd(0x1, x_sensor, y_sensor, ts_lsb)
    await drive_word(dut, word)
    await RisingEdge(dut.clk)

    assert int(dut.event_valid.value) == 1
    assert int(dut.polarity.value) == 1
    assert int(dut.x_out.value) == min((x_sensor >> 4) & 0x1F, 15)
    assert int(dut.y_out.value) == min((y_sensor >> 4) & 0x1F, 15)


@cocotb.test()
async def test_cd_off_event(dut):
    """CD_OFF event should decode with polarity=0."""
    await setup(dut)
    word = build_evt2_cd(0x0, 80, 80, 5)
    await drive_word(dut, word)
    await RisingEdge(dut.clk)

    assert int(dut.event_valid.value) == 1
    assert int(dut.polarity.value) == 0


@cocotb.test()
async def test_time_high_update(dut):
    """TIME_HIGH should update internal register and affect subsequent timestamps."""
    await setup(dut)
    model = Evt2DecoderModel()

    time_high_val = 0x00000AB
    th_word = build_evt2_time_high(time_high_val)
    await drive_word(dut, th_word)
    model.step(th_word, 1)
    await RisingEdge(dut.clk)
    model.step(0, 0)

    ts_lsb = 0x15
    cd_word = build_evt2_cd(0x1, 128, 128, ts_lsb)
    await drive_word(dut, cd_word)
    mx, my, mp, mts, mv = model.step(cd_word, 1)
    await RisingEdge(dut.clk)

    assert int(dut.event_valid.value) == 1
    dut_ts = int(dut.timestamp.value)
    assert dut_ts == mts, f"Timestamp: DUT={dut_ts}, model={mts}"


@cocotb.test()
async def test_coordinate_clamping(dut):
    """Coordinates above 15 should be clamped to 15."""
    await setup(dut)
    x_large = 0x7FF  # max 11-bit: grid_raw5 = (0x7FF >> 4) & 0x1F = 31
    y_large = 0x7FF
    word = build_evt2_cd(0x1, x_large, y_large, 0)
    await drive_word(dut, word)
    await RisingEdge(dut.clk)

    assert int(dut.x_out.value) == 15
    assert int(dut.y_out.value) == 15


@cocotb.test()
async def test_unknown_type_ignored(dut):
    """Packet types other than CD_ON/CD_OFF/TIME_HIGH should not produce events."""
    await setup(dut)
    for pkt_type in [0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x9, 0xA, 0xF]:
        word = (pkt_type << 28) | random.randint(0, 0x0FFFFFFF)
        await drive_word(dut, word)
        await RisingEdge(dut.clk)
        assert int(dut.event_valid.value) == 0, f"Spurious event for type 0x{pkt_type:X}"


@cocotb.test()
async def test_time_high_rollover_low10bits(dut):
    """Timestamp must use only the lower 10 bits of TIME_HIGH payload."""
    await setup(dut)
    model = Evt2DecoderModel()

    # Payload exceeds 10 bits; timestamp should keep only low 10 in high field.
    th_word = build_evt2_time_high(0x3ABCD)
    dut.data_in.value = th_word
    dut.data_valid.value = 1
    _, _, _, _, mv = model.step(th_word, 1)
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.event_valid.value) == mv
    await NextTimeStep()

    cd_word = build_evt2_cd(0x0, 64, 96, 0x2A)
    dut.data_in.value = cd_word
    dut.data_valid.value = 1
    mx, my, mp, mts, mv = model.step(cd_word, 1)
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.event_valid.value) == mv
    assert int(dut.timestamp.value) == mts, \
        f"Timestamp rollover mismatch: DUT={int(dut.timestamp.value)}, model={mts}"
    await NextTimeStep()


@cocotb.test()
async def test_golden_model_random(dut):
    """Cycle-by-cycle randomized scoreboarding against golden model."""
    await setup(dut)
    model = Evt2DecoderModel()
    rng = random.Random(0xE172D2)

    for cycle in range(1200):
        pkt_type = rng.choice([0x0, 0x1, 0x8, 0x2, 0x3, 0xF])
        if pkt_type in (0x0, 0x1):
            word = build_evt2_cd(pkt_type, rng.randint(0, 0x7FF),
                                 rng.randint(0, 0x7FF), rng.randint(0, 63))
        elif pkt_type == 0x8:
            word = build_evt2_time_high(rng.randint(0, 0x0FFFFFFF))
        else:
            word = (pkt_type << 28) | rng.randint(0, 0x0FFFFFFF)

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

