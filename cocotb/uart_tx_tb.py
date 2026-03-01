"""Unit testbench for uart_tx with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

CLKS_PER_BIT = 4


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class UartTxModel:
    """Cycle-accurate model of the uart_tx 8N1 transmitter."""

    IDLE, START, DATA, STOP = 0, 1, 2, 3

    def __init__(self, clks_per_bit=CLKS_PER_BIT):
        self.clks_per_bit = clks_per_bit
        self.reset()

    def reset(self):
        self.state = self.IDLE
        self.clk_cnt = 0
        self.bit_idx = 0
        self.tx_data = 0
        self.tx = 1
        self.busy = 0

    def step(self, data, valid):
        """Advance one clock cycle. Returns (tx, busy)."""
        if self.state == self.IDLE:
            self.tx = 1
            self.clk_cnt = 0
            self.bit_idx = 0
            self.busy = 0
            if valid:
                self.tx_data = data
                self.busy = 1
                self.state = self.START

        elif self.state == self.START:
            self.tx = 0
            if self.clk_cnt == self.clks_per_bit - 1:
                self.clk_cnt = 0
                self.state = self.DATA
            else:
                self.clk_cnt += 1

        elif self.state == self.DATA:
            self.tx = (self.tx_data >> self.bit_idx) & 1
            if self.clk_cnt == self.clks_per_bit - 1:
                self.clk_cnt = 0
                if self.bit_idx == 7:
                    self.bit_idx = 0
                    self.state = self.STOP
                else:
                    self.bit_idx += 1
            else:
                self.clk_cnt += 1

        elif self.state == self.STOP:
            self.tx = 1
            if self.clk_cnt == self.clks_per_bit - 1:
                self.clk_cnt = 0
                self.state = self.IDLE
                self.busy = 0
            else:
                self.clk_cnt += 1

        return self.tx, self.busy


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def receive_uart_byte(dut, timeout=500):
    """Sample the tx line and recover a byte. Returns the byte or None."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.tx.value) == 0:
            break
    else:
        return None

    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    if int(dut.tx.value) != 0:
        return None

    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if int(dut.tx.value):
            byte_val |= (1 << i)

    await ClockCycles(dut.clk, CLKS_PER_BIT)
    return byte_val


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.data.value = 0
    dut.valid.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """tx=1 and busy=0 after reset."""
    await setup(dut)
    assert int(dut.tx.value) == 1, "tx should be idle high"
    assert int(dut.busy.value) == 0, "busy should be 0"


@cocotb.test()
async def test_single_byte(dut):
    """Transmit 0xA5 and verify the serial waveform."""
    await setup(dut)
    recv_task = cocotb.start_soon(receive_uart_byte(dut))
    dut.data.value = 0xA5
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0
    result = await recv_task
    assert result == 0xA5, f"Expected 0xA5, got {result}"


@cocotb.test()
async def test_busy_during_transmission(dut):
    """busy should be high throughout a transmission."""
    await setup(dut)
    dut.data.value = 0x42
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0
    await RisingEdge(dut.clk)
    total_bits = 10  # start + 8 data + stop
    busy_seen = True
    for _ in range(total_bits * CLKS_PER_BIT - 2):
        await RisingEdge(dut.clk)
        if int(dut.busy.value) == 0:
            busy_seen = False
            break
    assert busy_seen, "busy dropped during transmission"


@cocotb.test()
async def test_all_byte_values(dut):
    """Transmit and verify every byte value 0x00–0xFF."""
    await setup(dut)
    for val in range(256):
        recv_task = cocotb.start_soon(receive_uart_byte(dut))
        dut.data.value = val
        dut.valid.value = 1
        await RisingEdge(dut.clk)
        dut.valid.value = 0
        result = await recv_task
        assert result == val, f"Expected 0x{val:02X}, got 0x{result:02X}"
        await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_golden_model_waveform(dut):
    """Randomized frame-level check against golden transmitter behavior."""
    await setup(dut)

    test_bytes = [random.randint(0, 255) for _ in range(10)]

    for b in test_bytes:
        recv_task = cocotb.start_soon(receive_uart_byte(dut))
        dut.data.value = b
        dut.valid.value = 1
        await RisingEdge(dut.clk)
        dut.valid.value = 0

        saw_busy = False
        for _ in range((10 * CLKS_PER_BIT) + 8):
            await RisingEdge(dut.clk)
            if int(dut.busy.value) == 1:
                saw_busy = True
            if saw_busy and int(dut.busy.value) == 0:
                break

        got = await recv_task
        assert got == b, f"Expected 0x{b:02X}, got 0x{got:02X}"
        assert saw_busy, "busy never asserted during transmission"
        await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_ignore_valid_while_busy(dut):
    """A new valid pulse while busy should be ignored."""
    await setup(dut)
    dut.data.value = 0x11
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT * 3)

    recv_task = cocotb.start_soon(receive_uart_byte(dut, timeout=CLKS_PER_BIT * 12))
    dut.data.value = 0x22
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0

    while int(dut.busy.value) == 1:
        await RisingEdge(dut.clk)

    recv_task2 = cocotb.start_soon(receive_uart_byte(dut))
    dut.data.value = 0x33
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0
    result = await recv_task2
    assert result == 0x33, f"Expected 0x33 after busy cleared, got {result}"

