"""Unit testbench for uart_rx with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

CLKS_PER_BIT = 4


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class UartRxModel:
    """Cycle-accurate model of the uart_rx 8N1 receiver."""

    IDLE, START, DATA, STOP = 0, 1, 2, 3

    def __init__(self, clks_per_bit=CLKS_PER_BIT):
        self.clks_per_bit = clks_per_bit
        self.reset()

    def reset(self):
        self.state = self.IDLE
        self.clk_cnt = 0
        self.bit_idx = 0
        self.rx_data = 0
        self.data = 0
        self.valid = 0
        self.rx_sync = 1
        self.rx_d = 1

    def step(self, rx_pin):
        """Advance one clock cycle. Returns (data, valid)."""
        prev_rx_sync = self.rx_sync
        self.rx_sync = rx_pin
        self.rx_d = prev_rx_sync

        self.valid = 0

        if self.state == self.IDLE:
            self.clk_cnt = 0
            self.bit_idx = 0
            if self.rx_d == 0:
                self.state = self.START

        elif self.state == self.START:
            if self.clk_cnt == (self.clks_per_bit - 1) // 2:
                if self.rx_d == 0:
                    self.clk_cnt = 0
                    self.state = self.DATA
                else:
                    self.state = self.IDLE
            else:
                self.clk_cnt += 1

        elif self.state == self.DATA:
            if self.clk_cnt == self.clks_per_bit - 1:
                self.clk_cnt = 0
                if self.rx_d:
                    self.rx_data |= (1 << self.bit_idx)
                else:
                    self.rx_data &= ~(1 << self.bit_idx)
                if self.bit_idx == 7:
                    self.bit_idx = 0
                    self.state = self.STOP
                else:
                    self.bit_idx += 1
            else:
                self.clk_cnt += 1

        elif self.state == self.STOP:
            if self.clk_cnt == self.clks_per_bit - 1:
                self.clk_cnt = 0
                self.state = self.IDLE
                if self.rx_d == 1:
                    self.data = self.rx_data
                    self.valid = 1
            else:
                self.clk_cnt += 1

        return self.data, self.valid


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def send_uart_byte(dut, byte_val):
    """Drive a full 8N1 frame on dut.rx."""
    dut.rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.rx.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """Outputs must be zero after reset."""
    await setup(dut)
    assert int(dut.data.value) == 0
    assert int(dut.valid.value) == 0


@cocotb.test()
async def test_single_byte(dut):
    """Send 0xA5 and verify data/valid."""
    await setup(dut)
    await send_uart_byte(dut, 0xA5)
    found = False
    for _ in range(CLKS_PER_BIT * 2):
        await RisingEdge(dut.clk)
        if int(dut.valid.value) == 1:
            assert int(dut.data.value) == 0xA5
            found = True
            break
    assert found, "valid never asserted"


@cocotb.test()
async def test_all_byte_values(dut):
    """Send every possible byte value and verify correct reception."""
    await setup(dut)
    for val in range(256):
        await send_uart_byte(dut, val)
        found = False
        for _ in range(CLKS_PER_BIT * 4):
            await RisingEdge(dut.clk)
            if int(dut.valid.value) == 1:
                assert int(dut.data.value) == val, f"Expected 0x{val:02X}, got 0x{int(dut.data.value):02X}"
                found = True
                break
        assert found, f"valid not asserted for byte 0x{val:02X}"
        await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_false_start_rejection(dut):
    """A brief low pulse shorter than half a bit period should not trigger reception."""
    await setup(dut)
    dut.rx.value = 0
    half_start = max(1, (CLKS_PER_BIT - 1) // 2 - 1)
    await ClockCycles(dut.clk, half_start)
    dut.rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT * 12)
    for _ in range(20):
        await RisingEdge(dut.clk)
        assert int(dut.valid.value) == 0, "Spurious valid from false start"


@cocotb.test()
async def test_framing_error(dut):
    """Bad stop bit (0) should not produce valid output."""
    await setup(dut)
    dut.rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.rx.value = (0x55 >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 0  # bad stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 1
    for _ in range(CLKS_PER_BIT * 4):
        await RisingEdge(dut.clk)
        assert int(dut.valid.value) == 0, "valid asserted despite bad stop bit"


@cocotb.test()
async def test_golden_model_random(dut):
    """Drive random rx waveform and compare DUT vs golden model cycle-by-cycle."""
    await setup(dut)
    model = UartRxModel(CLKS_PER_BIT)

    bytes_to_send = [random.randint(0, 255) for _ in range(20)]
    rx_waveform = [1] * (CLKS_PER_BIT * 10)  # idle
    for b in bytes_to_send:
        rx_waveform.extend([0] * CLKS_PER_BIT)  # start
        for i in range(8):
            bit = (b >> i) & 1
            rx_waveform.extend([bit] * CLKS_PER_BIT)
        rx_waveform.extend([1] * CLKS_PER_BIT)  # stop
        rx_waveform.extend([1] * (CLKS_PER_BIT * 2))  # inter-byte gap

    received_dut = []
    received_model = []

    for cycle_idx, rx_bit in enumerate(rx_waveform):
        dut.rx.value = rx_bit
        await RisingEdge(dut.clk)
        m_data, m_valid = model.step(rx_bit)

        dut_valid = int(dut.valid.value)
        dut_data = int(dut.data.value)

        if m_valid:
            received_model.append(m_data)
        if dut_valid:
            received_dut.append(dut_data)

    assert received_model == bytes_to_send, f"Model mismatch: {received_model}"
    assert received_dut == bytes_to_send, f"DUT mismatch: {received_dut}"


@cocotb.test()
async def test_back_to_back_bytes(dut):
    """Receive bytes with minimal idle gap between them."""
    await setup(dut)
    test_bytes = [0x00, 0xFF, 0x55, 0xAA, 0x42]
    received = []
    for b in test_bytes:
        await send_uart_byte(dut, b)
        for _ in range(CLKS_PER_BIT * 4):
            await RisingEdge(dut.clk)
            if int(dut.valid.value) == 1:
                received.append(int(dut.data.value))
                break
    assert received == test_bytes, f"Expected {test_bytes}, got {received}"

