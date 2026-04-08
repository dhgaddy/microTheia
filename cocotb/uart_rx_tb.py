# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
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


class UartRxModel:
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

    def step(self, rst, rx_pin):
        if rst:
            self.reset()
            return self.data, self.valid

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


async def send_uart_byte(dut, byte_val):
    dut.rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)


async def expect_valid_byte(dut, expected, timeout_cycles):
    # Catch the case where valid pulses on the cycle we return from send.
    await ReadOnly()
    if int(dut.valid.value):
        assert int(dut.data.value) == expected, f"Expected 0x{expected:02X}, got 0x{int(dut.data.value):02X}"
        return

    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.valid.value):
            assert int(dut.data.value) == expected, f"Expected 0x{expected:02X}, got 0x{int(dut.data.value):02X}"
            return

    raise AssertionError(f"No valid pulse observed for byte 0x{expected:02X}")


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.rx.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


@logged_test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.data.value) == 0
    assert int(dut.valid.value) == 0


@logged_test()
async def test_single_byte(dut):
    await setup(dut)
    wait_task = cocotb.start_soon(expect_valid_byte(dut, 0xA5, CLKS_PER_BIT * 14))
    await send_uart_byte(dut, 0xA5)
    await wait_task


@logged_test()
async def test_framing_error_rejected(dut):
    await setup(dut)

    dut.rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.rx.value = (0x5A >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 0  # bad stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 1

    for _ in range(CLKS_PER_BIT * 4):
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.valid.value) == 0, "Framing-error byte should be dropped"


@logged_test()
async def test_all_byte_values(dut):
    await setup(dut)
    for val in range(256):
        wait_task = cocotb.start_soon(expect_valid_byte(dut, val, CLKS_PER_BIT * 14))
        await send_uart_byte(dut, val)
        await wait_task


@logged_test()
async def test_randomized_golden_waveform(dut):
    await setup(dut)
    model = UartRxModel(CLKS_PER_BIT)
    rng = random.Random(0x9127)

    waveform = [1] * (CLKS_PER_BIT * 6)
    expected_bytes = []

    for _ in range(40):
        mode = rng.choice(["byte", "byte", "false_start", "idle"])
        if mode == "byte":
            b = rng.randint(0, 255)
            expected_bytes.append(b)
            waveform += [0] * CLKS_PER_BIT
            for i in range(8):
                bit = (b >> i) & 1
                waveform += [bit] * CLKS_PER_BIT
            waveform += [1] * CLKS_PER_BIT
            waveform += [1] * rng.randint(0, CLKS_PER_BIT * 3)
        elif mode == "false_start":
            waveform += [0] * rng.randint(1, max(1, (CLKS_PER_BIT // 2) - 2))
            waveform += [1] * rng.randint(1, CLKS_PER_BIT)
        else:
            waveform += [1] * rng.randint(1, CLKS_PER_BIT * 4)

    observed = []
    model_observed = []

    for bit in waveform:
        dut.rx.value = bit
        await RisingEdge(dut.clk)
        await ReadOnly()
        m_data, m_valid = model.step(int(dut.rst.value), bit)

        if m_valid:
            model_observed.append(m_data)
        if int(dut.valid.value):
            observed.append(int(dut.data.value))
        await NextTimeStep()

    assert observed == model_observed, f"DUT bytes {observed} != model {model_observed}"
    assert observed == expected_bytes, f"Decoded bytes {observed} != expected {expected_bytes}"
