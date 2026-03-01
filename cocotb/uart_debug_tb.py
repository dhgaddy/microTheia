"""Unit testbench for uart_debug with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 3_000_000
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE  # = 4


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class UartDebugModel:
    """Predicts the byte sequence uart_debug sends for each gesture."""

    MESSAGES = {
        0: [ord("U"), ord("P"), 0x0D, 0x0A],
        1: [ord("D"), ord("O"), ord("W"), ord("N"), 0x0D, 0x0A],
        2: [ord("L"), ord("E"), ord("F"), ord("T"), 0x0D, 0x0A],
        3: [ord("R"), ord("I"), ord("G"), ord("H"), ord("T"), 0x0D, 0x0A],
    }

    def expected_bytes(self, gesture_class):
        return self.MESSAGES[gesture_class]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def receive_uart_byte(dut, timeout=2000):
    # Synchronize to an actual start edge to avoid locking onto a low data bit
    # when bytes are transmitted back-to-back.
    prev = int(dut.uart_tx.value)
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        curr = int(dut.uart_tx.value)
        if prev == 1 and curr == 0:
            break
        prev = curr
    else:
        return None

    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    if int(dut.uart_tx.value) != 0:
        return None

    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if int(dut.uart_tx.value):
            byte_val |= (1 << i)

    await ClockCycles(dut.clk, CLKS_PER_BIT)
    return byte_val


async def receive_message(dut, max_bytes=10, timeout_per_byte=2000):
    """Receive bytes until timeout, return list."""
    result = []
    for _ in range(max_bytes):
        b = await receive_uart_byte(dut, timeout=timeout_per_byte)
        if b is None:
            break
        result.append(b)
    return result


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.gesture_class.value = 0
    dut.gesture_valid.value = 0
    dut.gesture_confidence.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def trigger_gesture(dut, gesture_class, confidence=128):
    dut.gesture_class.value = gesture_class
    dut.gesture_confidence.value = confidence
    dut.gesture_valid.value = 1
    await RisingEdge(dut.clk)
    dut.gesture_valid.value = 0


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """tx should be idle high after reset."""
    await setup(dut)
    assert int(dut.uart_tx.value) == 1


@cocotb.test()
async def test_gesture_up(dut):
    """Gesture 0 should send 'UP\\r\\n'."""
    await setup(dut)
    model = UartDebugModel()
    await trigger_gesture(dut, 0)
    received = await receive_message(dut, max_bytes=6)
    expected = model.expected_bytes(0)
    assert received == expected, f"Expected {expected}, got {received}"


@cocotb.test()
async def test_gesture_down(dut):
    """Gesture 1 should send 'DOWN\\r\\n'."""
    await setup(dut)
    model = UartDebugModel()
    await trigger_gesture(dut, 1)
    received = await receive_message(dut, max_bytes=8)
    expected = model.expected_bytes(1)
    assert received == expected, f"Expected {expected}, got {received}"


@cocotb.test()
async def test_gesture_left(dut):
    """Gesture 2 should send 'LEFT\\r\\n'."""
    await setup(dut)
    model = UartDebugModel()
    await trigger_gesture(dut, 2)
    received = await receive_message(dut, max_bytes=8)
    expected = model.expected_bytes(2)
    assert received == expected, f"Expected {expected}, got {received}"


@cocotb.test()
async def test_gesture_right(dut):
    """Gesture 3 should send 'RIGHT\\r\\n'."""
    await setup(dut)
    model = UartDebugModel()
    await trigger_gesture(dut, 3)
    received = await receive_message(dut, max_bytes=10)
    expected = model.expected_bytes(3)
    assert received == expected, f"Expected {expected}, got {received}"


@cocotb.test()
async def test_all_gestures_golden(dut):
    """Send all four gestures sequentially, verify against golden model."""
    await setup(dut)
    model = UartDebugModel()
    for g in range(4):
        await trigger_gesture(dut, g)
        expected = model.expected_bytes(g)
        received = await receive_message(dut, max_bytes=10)
        assert received == expected, f"Gesture {g}: expected {expected}, got {received}"
        await ClockCycles(dut.clk, 20)


@cocotb.test()
async def test_busy_rejection(dut):
    """A gesture_valid pulse during transmission should be ignored."""
    await setup(dut)
    model = UartDebugModel()
    await trigger_gesture(dut, 0)  # UP
    await ClockCycles(dut.clk, CLKS_PER_BIT * 3)
    await trigger_gesture(dut, 1)  # DOWN (should be ignored)
    received = await receive_message(dut, max_bytes=10)
    down_msg = model.expected_bytes(1)
    assert received != down_msg, f"Busy-path should not emit DOWN message, got {received}"

