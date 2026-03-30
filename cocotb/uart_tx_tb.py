# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2025 Group G Contributors
import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

CLK_FREQ_HZ = CFG["CLK_FREQ_HZ"] # 12_000_000
BAUD_RATE = CFG["BAUD_RATE"] # 115200

CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE


class UartTxModel:
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

    def step(self, rst, data, valid):
        if rst:
            self.reset()
            return self.tx, self.busy

        if self.state == self.IDLE:
            self.tx = 1
            self.clk_cnt = 0
            self.bit_idx = 0
            self.busy = 0
            if valid:
                self.tx_data = data & 0xFF
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


async def receive_uart_byte(dut, timeout_cycles=200000):
    prev = int(dut.tx.value)
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        cur = int(dut.tx.value)
        if prev == 1 and cur == 0:
            break
        prev = cur
    else:
        return None

    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    if int(dut.tx.value) != 0:
        return None

    val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if int(dut.tx.value):
            val |= 1 << i

    await ClockCycles(dut.clk, CLKS_PER_BIT)
    if int(dut.tx.value) != 1:
        return None
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)  # consume remaining stop bit → caller returns at IDLE
    return val


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.data.value = 0
    dut.valid.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


@logged_test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.tx.value) == 1
    assert int(dut.busy.value) == 0


@logged_test()
async def test_single_byte_transmit(dut):
    await setup(dut)

    rx_task = cocotb.start_soon(receive_uart_byte(dut))
    dut.data.value = 0xA5
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0

    got = await rx_task
    assert got == 0xA5, f"Expected 0xA5, got {got}"


@logged_test()
async def test_ignore_valid_while_busy(dut):
    await setup(dut)

    dut.data.value = 0x11
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0

    await ClockCycles(dut.clk, CLKS_PER_BIT * 2)

    # This should be ignored because busy is high.
    dut.data.value = 0x22
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0

    # Wait for first frame to finish then send a valid one.
    while int(dut.busy.value):
        await RisingEdge(dut.clk)

    rx_task = cocotb.start_soon(receive_uart_byte(dut))
    dut.data.value = 0x33
    dut.valid.value = 1
    await RisingEdge(dut.clk)
    dut.valid.value = 0

    got = await rx_task
    assert got == 0x33, f"Expected 0x33 after busy clear, got {got}"


@logged_test()
async def test_all_byte_values(dut):
    """Transmit every byte 0x00..0xFF and verify each is received correctly."""
    await setup(dut)
    for val in range(256):
        rx_task = cocotb.start_soon(receive_uart_byte(dut))
        dut.data.value = val
        dut.valid.value = 1
        await RisingEdge(dut.clk)
        dut.valid.value = 0
        got = await rx_task
        assert got == val, f"Byte 0x{val:02X}: expected 0x{val:02X}, got {got}"


@logged_test()
async def test_back_to_back_transmission(dut):
    """Start a new transmission immediately after the previous one completes."""
    await setup(dut)
    payload = [0x11, 0x55, 0xAA, 0xFF]
    received = []
    for byte_val in payload:
        rx_task = cocotb.start_soon(receive_uart_byte(dut))
        dut.data.value = byte_val
        dut.valid.value = 1
        await RisingEdge(dut.clk)
        dut.valid.value = 0
        got = await rx_task
        assert got is not None, f"Timed out receiving byte 0x{byte_val:02X}"
        received.append(got)
    assert received == payload, f"Back-to-back mismatch: got {received}, expected {payload}"


@logged_test()
async def test_randomized_cycle_scoreboard(dut):
    await setup(dut)
    model = UartTxModel(CLKS_PER_BIT)
    rng = random.Random(0x7EA11ED)

    for _ in range(5):
        model.step(1, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0)

    for cycle in range(6000):
        data = rng.randint(0, 255)
        # Bias toward no valid to avoid back-to-back perpetual traffic.
        valid = rng.choice([0, 0, 1])

        dut.data.value = data
        dut.valid.value = valid

        exp_tx, exp_busy = model.step(0, data, valid)

        await RisingEdge(dut.clk)
        await ReadOnly()

        got_tx = int(dut.tx.value)
        got_busy = int(dut.busy.value)

        assert got_tx == exp_tx, f"Cycle {cycle}: tx DUT={got_tx} model={exp_tx}"
        assert got_busy == exp_busy, f"Cycle {cycle}: busy DUT={got_busy} model={exp_busy}"

        await NextTimeStep()
