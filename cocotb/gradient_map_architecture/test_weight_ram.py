"""Unit testbench for weight_ram with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

NUM_CELLS = 256
GRID_SIZE = 16
WEIGHT_BITS = 8
NUM_CLASSES = 4
ADDR_BITS = 8


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class GMWeightRamModel:
    """Golden model replicating the gradient_map weight_ram initial block."""

    def __init__(self, class_idx, num_cells=NUM_CELLS, grid_size=GRID_SIZE):
        self.class_idx = class_idx
        self.num_cells = num_cells
        self.grid_size = grid_size
        self.ram = self._init_weights()

    def _init_weights(self):
        ram = [0] * self.num_cells
        centre = self.grid_size // 2

        for cy in range(self.grid_size):
            for cx in range(self.grid_size):
                addr = cy * self.grid_size + cx
                raw_val = 0

                if self.class_idx == 0:  # UP
                    if cy < centre:
                        raw_val = (centre - cy) * 6
                    else:
                        raw_val = -((cy - centre + 1) * 4)
                elif self.class_idx == 1:  # DOWN
                    if cy >= centre:
                        raw_val = (cy - centre + 1) * 6
                    else:
                        raw_val = -((centre - cy) * 4)
                elif self.class_idx == 2:  # LEFT
                    if cx < centre:
                        raw_val = (centre - cx) * 6
                    else:
                        raw_val = -((cx - centre + 1) * 4)
                elif self.class_idx == 3:  # RIGHT
                    if cx >= centre:
                        raw_val = (cx - centre + 1) * 6
                    else:
                        raw_val = -((centre - cx) * 4)

                ram[addr] = max(-128, min(127, raw_val))
        return ram

    def read(self, addr):
        return self.ram[addr]


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
    await setup(dut)
    val = to_signed_8(int(dut.dout.value))
    assert -128 <= val <= 127


@cocotb.test()
async def test_initial_weights_class0(dut):
    """Verify all initial weights for CLASS_IDX=0 (UP) match golden model."""
    await setup(dut)
    model = GMWeightRamModel(class_idx=0)

    mismatches = 0
    for addr in range(NUM_CELLS):
        dut_val = await read_weight(dut, addr)
        model_val = model.read(addr)
        if dut_val != model_val:
            mismatches += 1
            if mismatches <= 5:
                cy, cx = addr // GRID_SIZE, addr % GRID_SIZE
                dut._log.debug(f"Addr {addr} ({cx},{cy}): DUT={dut_val}, model={model_val}")

    assert mismatches == 0, f"{mismatches} weight mismatches"


@cocotb.test()
async def test_up_weight_pattern(dut):
    """UP class: top rows should have positive weights, bottom rows negative."""
    await setup(dut)
    centre = GRID_SIZE // 2
    cx_test = GRID_SIZE // 2

    top_val = await read_weight(dut, 0 * GRID_SIZE + cx_test)
    bot_val = await read_weight(dut, (GRID_SIZE - 1) * GRID_SIZE + cx_test)

    assert top_val > 0, f"Top-row weight should be positive, got {top_val}"
    assert bot_val < 0, f"Bottom-row weight should be negative, got {bot_val}"


@cocotb.test()
async def test_read_write(dut):
    """Write a custom value and read it back."""
    await setup(dut)
    addr = 42
    model = GMWeightRamModel(class_idx=0)
    original = model.read(addr)
    observed_before = await read_weight(dut, addr)
    assert observed_before == original, \
        f"Pre-write mismatch at addr {addr}: DUT={observed_before}, model={original}"

    write_val = -100 & 0xFF

    dut.cell_addr.value = addr
    dut.din.value = write_val
    dut.we.value = 1
    await RisingEdge(dut.clk)
    dut.we.value = 0
    await RisingEdge(dut.clk)

    result = await read_weight(dut, addr)
    assert result == -100, f"Expected -100, got {result}"

    # Restore the original initialized value so later golden checks stay independent.
    dut.cell_addr.value = addr
    dut.din.value = original & 0xFF
    dut.we.value = 1
    await RisingEdge(dut.clk)
    dut.we.value = 0
    await RisingEdge(dut.clk)
    restored = await read_weight(dut, addr)
    assert restored == original, f"Restore failed at addr {addr}: got {restored}, expected {original}"


@cocotb.test()
async def test_weight_range(dut):
    """All initial weights should be in [-128, 127]."""
    await setup(dut)
    for addr in range(NUM_CELLS):
        val = await read_weight(dut, addr)
        assert -128 <= val <= 127, f"Addr {addr}: weight {val} out of range"


@cocotb.test()
async def test_golden_spot_check(dut):
    """Spot-check 40 random addresses against golden model."""
    await setup(dut)
    model = GMWeightRamModel(class_idx=0)
    addrs = random.sample(range(NUM_CELLS), 40)
    for addr in addrs:
        dut_val = await read_weight(dut, addr)
        model_val = model.read(addr)
        assert dut_val == model_val, f"Addr {addr}: DUT={dut_val}, model={model_val}"


@cocotb.test()
async def test_gradient_magnitude(dut):
    """Weights should increase in magnitude toward edges (UP class)."""
    await setup(dut)
    centre = GRID_SIZE // 2
    cx_test = GRID_SIZE // 2

    prev_mag = 0
    for cy in range(centre - 1, -1, -1):
        val = await read_weight(dut, cy * GRID_SIZE + cx_test)
        mag = abs(val)
        assert mag >= prev_mag, \
            f"Magnitude should increase toward edge: cy={cy}, mag={mag}, prev={prev_mag}"
        prev_mag = mag


