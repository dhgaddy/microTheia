"""Unit testbench for input_fifo with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, ReadOnly, NextTimeStep
import random

DEPTH = 8
PTR_BITS = 3
DATA_WIDTH = 32


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class FifoModel:
    """Cycle-accurate model of the synchronous ring-buffer FIFO."""

    def __init__(self, depth=DEPTH, ptr_bits=PTR_BITS):
        self.depth = depth
        self.ptr_bits = ptr_bits
        self.reset()

    def reset(self):
        self.mem = [0] * self.depth
        self.wr_ptr = 0
        self.rd_ptr = 0
        self.rd_data = 0

    @property
    def full(self):
        msb_mask = 1 << self.ptr_bits
        low_mask = msb_mask - 1
        return ((self.wr_ptr ^ self.rd_ptr) == msb_mask) and \
               ((self.wr_ptr & low_mask) == (self.rd_ptr & low_mask))

    @property
    def empty(self):
        return self.wr_ptr == self.rd_ptr

    @property
    def count(self):
        return (self.wr_ptr - self.rd_ptr) & ((1 << (self.ptr_bits + 1)) - 1)

    def step(self, wr_en, wr_data, rd_en):
        """One clock cycle. Returns (rd_data, empty, full, count) after the edge."""
        do_write = wr_en and not self.full
        do_read = rd_en and not self.empty

        # read port samples old memory contents at the active edge
        rd_addr = self.rd_ptr & ((1 << self.ptr_bits) - 1)
        self.rd_data = self.mem[rd_addr]

        if do_write:
            wr_addr = self.wr_ptr & ((1 << self.ptr_bits) - 1)
            self.mem[wr_addr] = wr_data
            self.wr_ptr = (self.wr_ptr + 1) & ((1 << (self.ptr_bits + 1)) - 1)

        if do_read:
            self.rd_ptr = (self.rd_ptr + 1) & ((1 << (self.ptr_bits + 1)) - 1)

        return self.rd_data, self.empty, self.full, self.count


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.wr_en.value = 0
    dut.wr_data.value = 0
    dut.rd_en.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """FIFO should be empty after reset."""
    await setup(dut)
    assert int(dut.empty.value) == 1
    assert int(dut.full.value) == 0
    assert int(dut.count.value) == 0


@cocotb.test()
async def test_single_write_read(dut):
    """Write one word, read it back."""
    await setup(dut)
    dut.wr_en.value = 1
    dut.wr_data.value = 0xDEADBEEF
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.empty.value) == 0
    assert int(dut.count.value) == 1

    dut.rd_en.value = 1
    await RisingEdge(dut.clk)
    dut.rd_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.rd_data.value) == 0xDEADBEEF
    assert int(dut.empty.value) == 1


@cocotb.test()
async def test_fill_to_capacity(dut):
    """Fill FIFO to DEPTH, verify full flag, then drain."""
    await setup(dut)

    for i in range(DEPTH):
        dut.wr_en.value = 1
        dut.wr_data.value = i + 1
        await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.full.value) == 1
    assert int(dut.count.value) == DEPTH

    read_values = []
    for i in range(DEPTH):
        dut.rd_en.value = 1
        await RisingEdge(dut.clk)
        dut.rd_en.value = 0
        await RisingEdge(dut.clk)
        read_values.append(int(dut.rd_data.value))

    assert read_values == list(range(1, DEPTH + 1)), f"Read back: {read_values}"


@cocotb.test()
async def test_overflow_protection(dut):
    """Writes to a full FIFO should be silently dropped."""
    await setup(dut)

    for i in range(DEPTH):
        dut.wr_en.value = 1
        dut.wr_data.value = 0xAA
        await RisingEdge(dut.clk)

    dut.wr_data.value = 0xBAD
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.full.value) == 1
    assert int(dut.count.value) == DEPTH


@cocotb.test()
async def test_simultaneous_read_write(dut):
    """Read and write in the same cycle when FIFO has data."""
    await setup(dut)

    dut.wr_en.value = 1
    dut.wr_data.value = 0x11
    await RisingEdge(dut.clk)
    dut.wr_data.value = 0x22
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    dut.wr_en.value = 1
    dut.rd_en.value = 1
    dut.wr_data.value = 0x33
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    dut.rd_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.count.value) == 2


@cocotb.test()
async def test_underflow_does_not_advance(dut):
    """Reads on an empty FIFO must not change pointers or count."""
    await setup(dut)
    model = FifoModel(DEPTH, PTR_BITS)

    for _ in range(6):
        dut.wr_en.value = 0
        dut.rd_en.value = 1
        dut.wr_data.value = 0
        _, m_empty, m_full, m_count = model.step(0, 0, 1)
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.empty.value) == (1 if m_empty else 0)
        assert int(dut.full.value) == (1 if m_full else 0)
        assert int(dut.count.value) == m_count
        await NextTimeStep()


@cocotb.test()
async def test_wraparound_ordering(dut):
    """Pointer wrap-around must preserve FIFO ordering."""
    await setup(dut)

    # Fill FIFO, pop half, then push half again to force pointer wrap.
    for i in range(DEPTH):
        dut.wr_en.value = 1
        dut.wr_data.value = i + 1
        dut.rd_en.value = 0
        await RisingEdge(dut.clk)

    popped = []
    for _ in range(DEPTH // 2):
        dut.wr_en.value = 0
        dut.rd_en.value = 1
        await RisingEdge(dut.clk)
        dut.rd_en.value = 0
        await RisingEdge(dut.clk)
        popped.append(int(dut.rd_data.value))

    for i in range(DEPTH // 2):
        dut.wr_en.value = 1
        dut.wr_data.value = 100 + i
        dut.rd_en.value = 0
        await RisingEdge(dut.clk)

    drained = []
    while int(dut.empty.value) == 0:
        dut.wr_en.value = 0
        dut.rd_en.value = 1
        await RisingEdge(dut.clk)
        dut.rd_en.value = 0
        await RisingEdge(dut.clk)
        drained.append(int(dut.rd_data.value))

    assert popped == [1, 2, 3, 4], f"Unexpected popped values: {popped}"
    assert drained == [5, 6, 7, 8, 100, 101, 102, 103], \
        f"Unexpected drained ordering: {drained}"


@cocotb.test()
async def test_golden_model_random(dut):
    """Cycle-by-cycle randomized scoreboarding against the golden model."""
    await setup(dut)
    model = FifoModel(DEPTH, PTR_BITS)
    rng = random.Random(0xC0C0F1F0)

    for cycle in range(1000):
        wr_en = rng.randint(0, 1)
        rd_en = rng.randint(0, 1)
        wr_data = rng.randint(0, (1 << DATA_WIDTH) - 1)

        dut.wr_en.value = wr_en
        dut.wr_data.value = wr_data
        dut.rd_en.value = rd_en
        _, m_empty, m_full, m_count = model.step(wr_en, wr_data, rd_en)
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.empty.value) == (1 if m_empty else 0), \
            f"Cycle {cycle}: empty DUT={int(dut.empty.value)}, model={int(m_empty)}"
        assert int(dut.full.value) == (1 if m_full else 0), \
            f"Cycle {cycle}: full DUT={int(dut.full.value)}, model={int(m_full)}"
        assert int(dut.count.value) == m_count, \
            f"Cycle {cycle}: count DUT={int(dut.count.value)}, model={m_count}"
        await NextTimeStep()

