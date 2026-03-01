"""Unit testbench for gradient_mapping with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

GRID_SIZE = 16
ADDR_BITS = 8
TS_BITS = 16
VALUE_BITS = 8
MAX_VALUE = 255
DECAY_SHIFT = 6
NUM_CELLS = GRID_SIZE * GRID_SIZE


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class GradientMappingModel:
    """Golden model of the gradient_mapping time-surface encoder."""

    def __init__(self, grid_size=GRID_SIZE, decay_shift=DECAY_SHIFT,
                 max_value=MAX_VALUE, value_bits=VALUE_BITS, ts_bits=TS_BITS):
        self.grid_size = grid_size
        self.decay_shift = decay_shift
        self.max_value = max_value
        self.value_bits = value_bits
        self.ts_bits = ts_bits
        self.ts_mask = (1 << ts_bits) - 1
        self.num_cells = grid_size * grid_size
        self.reset()

    def reset(self):
        self.mem = [0] * self.num_cells
        self.cell_valid = [False] * self.num_cells

    def write_event(self, event_x, event_y, event_ts):
        addr = (event_y * self.grid_size + event_x) & 0xFF
        self.mem[addr] = event_ts & self.ts_mask
        self.cell_valid[addr] = True

    def read_value(self, addr, t_now):
        """Compute decayed value with 3-stage pipeline latency in mind.
        Returns the expected read_value after the pipeline settles."""
        if not self.cell_valid[addr]:
            return 0

        ts_stored = self.mem[addr]
        delta_t = (t_now - ts_stored) & self.ts_mask
        decay_steps = delta_t >> self.decay_shift

        if decay_steps >= self.value_bits:
            return 0
        return self.max_value >> decay_steps


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.event_valid.value = 0
    dut.event_x.value = 0
    dut.event_y.value = 0
    dut.event_ts.value = 0
    dut.t_now.value = 0
    dut.read_enable.value = 0
    dut.read_addr.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def write_event(dut, x, y, ts, t_now):
    dut.event_x.value = x & 0xF
    dut.event_y.value = y & 0xF
    dut.event_ts.value = ts & 0xFFFF
    dut.t_now.value = t_now & 0xFFFF
    dut.event_valid.value = 1
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0


async def read_cell(dut, addr, t_now, pipeline_cycles=4):
    """Read a cell and wait for the pipeline to produce the result."""
    dut.read_addr.value = addr & 0xFF
    dut.read_enable.value = 1
    dut.t_now.value = t_now & 0xFFFF
    for _ in range(pipeline_cycles):
        await RisingEdge(dut.clk)
    dut.read_enable.value = 0
    return int(dut.read_value.value)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """read_value should be 0 after reset."""
    await setup(dut)
    val = await read_cell(dut, 0, 0)
    assert val == 0


@cocotb.test()
async def test_write_then_immediate_read(dut):
    """Writing an event and reading the same cell immediately should give MAX_VALUE."""
    await setup(dut)
    t = 100
    await write_event(dut, 5, 5, t, t)
    await ClockCycles(dut.clk, 2)

    addr = 5 * GRID_SIZE + 5
    val = await read_cell(dut, addr, t)
    assert val == MAX_VALUE, f"Expected {MAX_VALUE}, got {val}"


@cocotb.test()
async def test_exponential_decay(dut):
    """Value should halve every 2^DECAY_SHIFT ticks."""
    await setup(dut)
    model = GradientMappingModel()

    t_write = 100
    await write_event(dut, 8, 8, t_write, t_write)
    model.write_event(8, 8, t_write)
    await ClockCycles(dut.clk, 2)

    addr = 8 * GRID_SIZE + 8
    half_life = 1 << DECAY_SHIFT

    for step in range(VALUE_BITS):
        t_read = t_write + step * half_life
        dut_val = await read_cell(dut, addr, t_read)
        model_val = model.read_value(addr, t_read)

        if model_val == 0:
            break
        assert dut_val == model_val, \
            f"Step {step}: DUT={dut_val}, model={model_val} (t_read={t_read})"


@cocotb.test()
async def test_full_decay_to_zero(dut):
    """After VALUE_BITS half-lives, value should be 0."""
    await setup(dut)
    t_write = 100
    await write_event(dut, 3, 3, t_write, t_write)
    await ClockCycles(dut.clk, 2)

    addr = 3 * GRID_SIZE + 3
    t_read = t_write + VALUE_BITS * (1 << DECAY_SHIFT)
    val = await read_cell(dut, addr, t_read)
    assert val == 0, f"Expected 0 after full decay, got {val}"


@cocotb.test()
async def test_multiple_cells(dut):
    """Write to multiple cells, verify each decays independently."""
    await setup(dut)
    model = GradientMappingModel()

    cells = [(0, 0), (7, 7), (15, 15), (3, 12)]
    base_t = 200

    for i, (cx, cy) in enumerate(cells):
        t = base_t + i * 10
        await write_event(dut, cx, cy, t, t)
        model.write_event(cx, cy, t)
        await ClockCycles(dut.clk, 2)

    t_read = base_t + 100
    for cx, cy in cells:
        addr = cy * GRID_SIZE + cx
        dut_val = await read_cell(dut, addr, t_read)
        model_val = model.read_value(addr, t_read)
        assert dut_val == model_val, \
            f"Cell ({cx},{cy}): DUT={dut_val}, model={model_val}"


@cocotb.test()
async def test_unwritten_cell_zero(dut):
    """Cells that were never written should read as 0."""
    await setup(dut)
    for addr in [0, 100, 200, 255]:
        val = await read_cell(dut, addr, 5000)
        assert val == 0, f"Unwritten cell {addr} returned {val}"


@cocotb.test()
async def test_overwrite_updates_timestamp(dut):
    """Overwriting a cell should refresh its timestamp."""
    await setup(dut)
    model = GradientMappingModel()
    addr = 4 * GRID_SIZE + 4

    await write_event(dut, 4, 4, 100, 100)
    model.write_event(4, 4, 100)
    await ClockCycles(dut.clk, 2)

    val1 = await read_cell(dut, addr, 200)
    model_val1 = model.read_value(addr, 200)

    await write_event(dut, 4, 4, 200, 200)
    model.write_event(4, 4, 200)
    await ClockCycles(dut.clk, 2)

    val2 = await read_cell(dut, addr, 200)
    assert val2 == MAX_VALUE, f"After overwrite at t=200, expected {MAX_VALUE}, got {val2}"


@cocotb.test()
async def test_golden_random(dut):
    """Random writes and reads, compare DUT vs golden model."""
    await setup(dut)
    model = GradientMappingModel()

    t = 1000
    for _ in range(30):
        cx = random.randint(0, 15)
        cy = random.randint(0, 15)
        await write_event(dut, cx, cy, t, t)
        model.write_event(cx, cy, t)
        t += random.randint(1, 20)
        await ClockCycles(dut.clk, 2)

    mismatches = 0
    t_read = t + random.randint(0, 200)
    for addr in random.sample(range(NUM_CELLS), 30):
        dut_val = await read_cell(dut, addr, t_read)
        model_val = model.read_value(addr, t_read)
        if dut_val != model_val:
            mismatches += 1

    assert mismatches == 0, f"{mismatches} value mismatches in random test"

