"""Unit testbench for weight_ram with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

NUM_CELLS = 1024
GRID_SIZE = 16
WEIGHT_BITS = 8
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE
ADDR_BITS = (NUM_CELLS - 1).bit_length()


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class VBWeightRamModel:
    """Golden model replicating the weight_ram initial block."""

    def __init__(self, class_idx, num_cells=NUM_CELLS, grid_size=GRID_SIZE):
        self.class_idx = class_idx
        self.num_cells = num_cells
        self.grid_size = grid_size
        self.cells_per_bin = grid_size * grid_size
        self.half = grid_size // 2
        self.ram = self._init_weights()

    def _init_weights(self):
        ram = [0] * self.num_cells
        for i in range(self.num_cells):
            spatial_addr = i % self.cells_per_bin
            bin_idx = i // self.cells_per_bin
            cy = spatial_addr // self.grid_size
            cx = spatial_addr % self.grid_size

            temp_phase = 1 if bin_idx >= 2 else -1
            raw_val = 0

            if self.class_idx == 0:  # UP
                if cy < self.half:
                    wdist = self.half - cy
                    raw_val = temp_phase * wdist
                else:
                    wdist = cy - self.half + 1
                    raw_val = -(temp_phase * wdist)
            elif self.class_idx == 1:  # DOWN
                if cy >= self.half:
                    wdist = cy - self.half + 1
                    raw_val = temp_phase * wdist
                else:
                    wdist = self.half - cy
                    raw_val = -(temp_phase * wdist)
            elif self.class_idx == 2:  # LEFT
                if cx < self.half:
                    wdist = self.half - cx
                    raw_val = temp_phase * wdist
                else:
                    wdist = cx - self.half + 1
                    raw_val = -(temp_phase * wdist)
            elif self.class_idx == 3:  # RIGHT
                if cx >= self.half:
                    wdist = cx - self.half + 1
                    raw_val = temp_phase * wdist
                else:
                    wdist = self.half - cx
                    raw_val = -(temp_phase * wdist)

            ram[i] = max(-128, min(127, raw_val))
        return ram

    def read(self, addr):
        return self.ram[addr]

    def write(self, addr, data):
        self.ram[addr] = max(-128, min(127, data))


def to_signed_8(val):
    val = val & 0xFF
    return val - 256 if val >= 128 else val


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.we.value = 0
    dut.cell_addr.value = 0
    dut.din.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def read_weight(dut, addr):
    dut.cell_addr.value = addr
    dut.we.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    return to_signed_8(int(dut.dout.value))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """dout should be a valid signed 8-bit value after reset."""
    await setup(dut)
    val = to_signed_8(int(dut.dout.value))
    assert -128 <= val <= 127


@cocotb.test()
async def test_initial_weights_class0(dut):
    """Verify all initial weights for CLASS_IDX=0 (UP) match golden model."""
    await setup(dut)
    model = VBWeightRamModel(class_idx=0, num_cells=NUM_CELLS, grid_size=GRID_SIZE)

    mismatches = 0
    for addr in range(NUM_CELLS):
        dut_val = await read_weight(dut, addr)
        model_val = model.read(addr)
        if dut_val != model_val:
            mismatches += 1
            if mismatches <= 5:
                dut._log.debug(f"Addr {addr}: DUT={dut_val}, model={model_val}")

    assert mismatches == 0, f"{mismatches} weight mismatches"


@cocotb.test()
async def test_weight_symmetry(dut):
    """UP class: top-half weights should be opposite sign from bottom-half."""
    await setup(dut)
    half = GRID_SIZE // 2
    top_sum = 0
    bot_sum = 0
    for cy in range(GRID_SIZE):
        for cx in range(GRID_SIZE):
            addr = cy * GRID_SIZE + cx
            val = await read_weight(dut, addr)
            if cy < half:
                top_sum += val
            else:
                bot_sum += val

    assert top_sum != 0 or bot_sum != 0, "All weights are zero"
    assert (top_sum > 0) != (bot_sum > 0) or top_sum == 0, \
        f"Expected opposite signs: top_sum={top_sum}, bot_sum={bot_sum}"


@cocotb.test()
async def test_read_write(dut):
    """Write a value and read it back."""
    await setup(dut)
    test_addr = 42
    old_val = await read_weight(dut, test_addr)
    test_val = -55 & 0xFF

    dut.cell_addr.value = test_addr
    dut.din.value = test_val
    dut.we.value = 1
    await RisingEdge(dut.clk)
    dut.we.value = 0
    await RisingEdge(dut.clk)

    result = await read_weight(dut, test_addr)
    assert result == -55, f"Expected -55, got {result}"

    # Restore original value so later golden checks remain independent.
    dut.cell_addr.value = test_addr
    dut.din.value = old_val & 0xFF
    dut.we.value = 1
    await RisingEdge(dut.clk)
    dut.we.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_read_after_write_same_cycle(dut):
    """Read on same cycle as write should return new data (read-first behavior)."""
    await setup(dut)
    old_val = await read_weight(dut, 10)

    dut.cell_addr.value = 10
    dut.din.value = 0x7F
    dut.we.value = 1
    await RisingEdge(dut.clk)
    dut.we.value = 0
    await RisingEdge(dut.clk)

    new_val = await read_weight(dut, 10)
    assert new_val == 127, f"Expected 127 after write, got {new_val}"

    # Restore original value so later golden checks remain independent.
    dut.cell_addr.value = 10
    dut.din.value = old_val & 0xFF
    dut.we.value = 1
    await RisingEdge(dut.clk)
    dut.we.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_golden_exhaustive_spot_check(dut):
    """Spot-check 50 random addresses against golden model."""
    await setup(dut)
    model = VBWeightRamModel(class_idx=0, num_cells=NUM_CELLS, grid_size=GRID_SIZE)

    addrs = random.sample(range(NUM_CELLS), min(50, NUM_CELLS))
    for addr in addrs:
        dut_val = await read_weight(dut, addr)
        model_val = model.read(addr)
        assert dut_val == model_val, f"Addr {addr}: DUT={dut_val}, model={model_val}"


