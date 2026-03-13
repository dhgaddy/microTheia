"""Robust cocotb testbench for systolic_array with unsigned matmul golden model."""

import math
import random

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge
import os
from util.config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

N             = CFG["N"]
DATA_BIT_SIZE = CFG["DATA_BIT_SIZE"]

ACC_BITS     = (2 * DATA_BIT_SIZE) + (N - 1).bit_length() + 1
TOTAL_CYCLES = (3 * N) - 1
DATA_MASK    = (1 << DATA_BIT_SIZE) - 1
ACC_MASK     = (1 << ACC_BITS) - 1


def configure_from_dut(dut):
    global N, ACC_BITS, TOTAL_CYCLES, DATA_MASK, ACC_MASK

    n_sq = len(dut.A_matrix_flat) // DATA_BIT_SIZE
    n = math.isqrt(n_sq)
    assert n * n == n_sq, f"Non-square matrix flattening: bits={len(dut.A_matrix_flat)}"
    N = n

    acc_bits = len(dut.Out_matrix_flat) // (N * N)
    ACC_BITS = acc_bits
    TOTAL_CYCLES = (3 * N) - 1
    DATA_MASK = (1 << DATA_BIT_SIZE) - 1
    ACC_MASK = (1 << ACC_BITS) - 1


def to_unsigned(v, bits):
    return v & ((1 << bits) - 1)


def pack_matrix(mat, bits):
    packed = 0
    idx = 0
    for i in range(N):
        for j in range(N):
            packed |= to_unsigned(mat[i][j], bits) << (idx * bits)
            idx += 1
    return packed


def unpack_matrix(packed, bits):
    mat = [[0 for _ in range(N)] for _ in range(N)]
    idx = 0
    for i in range(N):
        for j in range(N):
            raw = (packed >> (idx * bits)) & ((1 << bits) - 1)
            mat[i][j] = raw
            idx += 1
    return mat


def golden_matmul(a, b):
    out = [[0 for _ in range(N)] for _ in range(N)]
    for i in range(N):
        for j in range(N):
            acc = 0
            for k in range(N):
                acc += int(a[i][k]) * int(b[k][j])
            out[i][j] = acc & ACC_MASK
    return out


async def setup(dut):
    configure_from_dut(dut)
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.reset.value = 1
    dut.start.value = 0
    dut.A_matrix_flat.value = 0
    dut.B_matrix_flat.value = 0
    await ClockCycles(dut.clk, 5)
    dut.reset.value = 0
    await ClockCycles(dut.clk, 2)


async def run_mul(dut, a, b, tag):
    dut.A_matrix_flat.value = pack_matrix(a, DATA_BIT_SIZE)
    dut.B_matrix_flat.value = pack_matrix(b, DATA_BIT_SIZE)

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # busy should assert after start acceptance.
    await ReadOnly()
    assert int(dut.busy.value) == 1, f"{tag}: busy did not assert after start"

    saw_done = False
    for cyc in range(TOTAL_CYCLES + 20):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.done.value):
            saw_done = True
            assert int(dut.busy.value) == 0, f"{tag}: busy should drop when done=1"
            break
        else:
            assert int(dut.busy.value) == 1, f"{tag}: busy dropped early at cycle {cyc}"

    assert saw_done, f"{tag}: timed out waiting for done"

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    exp = golden_matmul(a, b)
    assert got == exp, f"{tag}: output matrix mismatch"

    # done is one-cycle pulse.
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(dut.done.value) == 0, f"{tag}: done not a single-cycle pulse"

    await NextTimeStep()


@logged_test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.busy.value) == 0
    assert int(dut.done.value) == 0


@logged_test()
async def test_zero_matrices(dut):
    await setup(dut)
    a = [[0 for _ in range(N)] for _ in range(N)]
    b = [[0 for _ in range(N)] for _ in range(N)]
    await run_mul(dut, a, b, "zeros")


@logged_test()
async def test_identity_times_random(dut):
    await setup(dut)
    rng = random.Random(0x1D31F17)

    a = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    b = [[rng.randint(0, 200) for _ in range(N)] for _ in range(N)]

    await run_mul(dut, a, b, "identity")


@logged_test()
async def test_start_ignored_while_running(dut):
    await setup(dut)

    a = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    b = [[2 if i == j else 0 for j in range(N)] for i in range(N)]

    dut.A_matrix_flat.value = pack_matrix(a, DATA_BIT_SIZE)
    dut.B_matrix_flat.value = pack_matrix(b, DATA_BIT_SIZE)

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # Pulse start again while busy; DUT should ignore it.
    await ClockCycles(dut.clk, 5)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    saw_done = False
    for _ in range(TOTAL_CYCLES + 20):
        await RisingEdge(dut.clk)
        if int(dut.done.value):
            saw_done = True
            break

    assert saw_done, "Did not complete first run"

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    exp = golden_matmul(a, b)
    assert got == exp, "Unexpected restart/perturbation from mid-run start pulse"


@logged_test()
async def test_randomized_golden(dut):
    await setup(dut)
    rng = random.Random(0x5A57A11C)

    for trial in range(10):
        a = [[rng.randint(0, 300) for _ in range(N)] for _ in range(N)]
        b = [[rng.randint(0, 300) for _ in range(N)] for _ in range(N)]
        await run_mul(dut, a, b, f"rnd-{trial}")
        await ClockCycles(dut.clk, 2)


@logged_test()
async def test_back_to_back_runs(dut):
    """Start a new multiply immediately after done pulses; no idle gap between runs."""
    await setup(dut)
    rng = random.Random(0xB2B2B2B2)

    for trial in range(5):
        await NextTimeStep()
        a = [[rng.randint(0, 100) for _ in range(N)] for _ in range(N)]
        b = [[rng.randint(0, 100) for _ in range(N)] for _ in range(N)]

        dut.A_matrix_flat.value = pack_matrix(a, DATA_BIT_SIZE)
        dut.B_matrix_flat.value = pack_matrix(b, DATA_BIT_SIZE)

        dut.start.value = 1
        await RisingEdge(dut.clk)
        dut.start.value = 0

        saw_done = False
        for _ in range(TOTAL_CYCLES + 20):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.done.value):
                saw_done = True
                break
        assert saw_done, f"trial {trial}: timed out waiting for done"

        got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
        exp = golden_matmul(a, b)
        assert got == exp, f"trial {trial}: output mismatch"
        # No gap: next trial's start fires next cycle (loop continues immediately)


@logged_test()
async def test_identity_times_identity(dut):
    """I × I = I: verify every element explicitly."""
    await setup(dut)

    a = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    b = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    await run_mul(dut, a, b, "I*I")

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    for i in range(N):
        for j in range(N):
            exp_val = 1 if i == j else 0
            assert got[i][j] == exp_val, \
                f"I*I mismatch at [{i}][{j}]: DUT={got[i][j]} expected={exp_val}"


@logged_test()
async def test_single_nonzero_element(dut):
    """A[0][0]=k, B[0][0]=k, all others 0 -> Out[0][0]=k*k, rest 0."""
    await setup(dut)

    k = 127
    a = [[0 for _ in range(N)] for _ in range(N)]
    b = [[0 for _ in range(N)] for _ in range(N)]
    a[0][0] = k
    b[0][0] = k
    await run_mul(dut, a, b, "single-elem")

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    assert got[0][0] == k * k, f"Expected {k*k} at [0][0], got {got[0][0]}"
    for i in range(N):
        for j in range(N):
            if (i, j) != (0, 0):
                assert got[i][j] == 0, f"Expected 0 at [{i}][{j}], got {got[i][j]}"


@logged_test()
async def test_all_max_values(dut):
    """Max unsigned values on both inputs should match golden accumulation."""
    await setup(dut)
    max_val = (1 << DATA_BIT_SIZE) - 1
    a = [[max_val for _ in range(N)] for _ in range(N)]
    b = [[max_val for _ in range(N)] for _ in range(N)]
    await run_mul(dut, a, b, "max*max")

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    exp = golden_matmul(a, b)
    assert got == exp, "max*max matrix mismatch"
    for i in range(N):
        for j in range(N):
            assert got[i][j] > 0, \
                f"Expected positive at [{i}][{j}], got {got[i][j]}"


@logged_test()
async def test_accumulator_does_not_persist_across_runs(dut):
    """Accumulators must zero at the start of each run; no bleed from a prior result."""
    await setup(dut)

    # First run fills accumulators with large positive values.
    a1 = [[1 for _ in range(N)] for _ in range(N)]
    b1 = [[1 for _ in range(N)] for _ in range(N)]
    await run_mul(dut, a1, b1, "ones-run")

    # Second run: all-zeros -> result must be exactly zero.
    a2 = [[0 for _ in range(N)] for _ in range(N)]
    b2 = [[0 for _ in range(N)] for _ in range(N)]
    await run_mul(dut, a2, b2, "zeros-run")

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    for i in range(N):
        for j in range(N):
            assert got[i][j] == 0, \
                f"Accumulator bleed at [{i}][{j}]: {got[i][j]} (expected 0)"


@logged_test()
async def test_high_range_stress(dut):
    """High-range unsigned values; golden comparison over 15 trials."""
    await setup(dut)
    rng = random.Random(0xFACEFEED)

    for trial in range(15):
        a = [[rng.randint(0, 500) for _ in range(N)] for _ in range(N)]
        b = [[rng.randint(0, 500) for _ in range(N)] for _ in range(N)]
        await run_mul(dut, a, b, f"hi-{trial}")


@logged_test()
async def test_done_fires_at_exact_cycle(dut):
    """done must assert at exactly cycle TOTAL_CYCLES-1 (0-indexed) after the start edge."""
    await setup(dut)
    configure_from_dut(dut)

    # Use identity matrices so the result is predictable.
    a = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    b = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    dut.A_matrix_flat.value = pack_matrix(a, DATA_BIT_SIZE)
    dut.B_matrix_flat.value = pack_matrix(b, DATA_BIT_SIZE)

    dut.start.value = 1
    await RisingEdge(dut.clk)  # start sampled on this edge
    dut.start.value = 0

    done_cycle = None
    for cyc in range(TOTAL_CYCLES + 10):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.done.value):
            done_cycle = cyc
            break

    assert done_cycle is not None, "done never fired within timeout"
    assert done_cycle == TOTAL_CYCLES - 1, (
        f"done fired at cycle {done_cycle} (0-indexed after start), "
        f"expected TOTAL_CYCLES-1={TOTAL_CYCLES - 1}"
    )
