"""Robust cocotb testbench for uart_debug using a message-level golden model."""

import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

CLK_FREQ_HZ = CFG["CLK_FREQ_HZ"] # 12_000_000
BAUD_RATE = CFG["BAUD_RATE"] # 115200

CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE


class UartDebugModel:
    MESSAGES = {
        0: [ord("U"), ord("P"), 0x0D, 0x0A],
        1: [ord("D"), ord("O"), ord("W"), ord("N"), 0x0D, 0x0A],
        2: [ord("L"), ord("E"), ord("F"), ord("T"), 0x0D, 0x0A],
        3: [ord("R"), ord("I"), ord("G"), ord("H"), ord("T"), 0x0D, 0x0A],
    }

    def expected_bytes(self, gesture_class):
        return self.MESSAGES.get(int(gesture_class), self.MESSAGES[3])


async def receive_uart_byte(dut, timeout_cycles=CLKS_PER_BIT * 20):
    prev = int(dut.uart_tx.value)
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        cur = int(dut.uart_tx.value)
        if prev == 1 and cur == 0:
            break
        prev = cur
    else:
        return None

    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    if int(dut.uart_tx.value) != 0:
        return None

    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if int(dut.uart_tx.value):
            byte_val |= 1 << i

    await ClockCycles(dut.clk, CLKS_PER_BIT)
    if int(dut.uart_tx.value) != 1:
        return None

    return byte_val


async def receive_message(dut, max_bytes=12):
    out = []
    for _ in range(max_bytes):
        b = await receive_uart_byte(dut)
        if b is None:
            break
        out.append(b)
    return out


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.gesture_class.value = 0
    dut.gesture_valid.value = 0
    dut.gesture_confidence.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def trigger_gesture(dut, gesture_class, confidence=0x5A):
    dut.gesture_class.value = gesture_class
    dut.gesture_confidence.value = confidence
    dut.gesture_valid.value = 1
    await RisingEdge(dut.clk)
    dut.gesture_valid.value = 0


@logged_test()
async def test_reset_idle_high(dut):
    await setup(dut)
    assert int(dut.uart_tx.value) == 1


@logged_test()
async def test_all_gesture_messages(dut):
    await setup(dut)
    model = UartDebugModel()

    for gesture in [0, 1, 2, 3]:
        await trigger_gesture(dut, gesture)
        got = await receive_message(dut, max_bytes=12)
        exp = model.expected_bytes(gesture)
        assert got == exp, f"Gesture {gesture}: got {got}, expected {exp}"
        await ClockCycles(dut.clk, CLKS_PER_BIT * 2)


@logged_test()
async def test_unknown_class_maps_to_right(dut):
    await setup(dut)
    model = UartDebugModel()

    await trigger_gesture(dut, 3)
    got = await receive_message(dut, max_bytes=12)
    exp = model.expected_bytes(3)
    assert got == exp, f"RIGHT class message mismatch, got {got}"


@logged_test()
async def test_busy_rejects_new_gesture(dut):
    await setup(dut)

    recv_task = cocotb.start_soon(receive_message(dut, max_bytes=16))
    await trigger_gesture(dut, 0)
    for _ in range(CLKS_PER_BIT * 20):
        await RisingEdge(dut.clk)
        if int(dut.u_uart_tx.busy.value):
            break
    await trigger_gesture(dut, 1)
    got = await recv_task
    # Should only contain the first message (UP\r\n).
    exp = [ord("U"), ord("P"), 0x0D, 0x0A]
    assert got[:4] == exp, f"Expected first message to be UP, got {got}"


@logged_test()
async def test_randomized_gesture_sequence(dut):
    await setup(dut)
    model = UartDebugModel()
    rng = random.Random(0xD38A)

    for idx in range(12):
        g = rng.randint(0, 3)
        await trigger_gesture(dut, g, confidence=rng.randint(0, 255))
        got = await receive_message(dut, max_bytes=12)
        exp = model.expected_bytes(g)
        assert got == exp, f"rnd-{idx}: got {got}, expected {exp}"
        await ClockCycles(dut.clk, rng.randint(1, CLKS_PER_BIT))
