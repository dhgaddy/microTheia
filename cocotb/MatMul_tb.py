import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, ReadOnly
import os
from config_parser import load_config

MODULE = os.environ.get("TOPLEVEL")
CFG = load_config(MODULE)

N = CFG["N"] # Size of the square matrices
DATA_BIT_SIZE = CFG["DATA_BIT_SIZE"]
PRODUCT_BIT_SIZE = 2 * DATA_BIT_SIZE
ACC_BIT_SIZE = PRODUCT_BIT_SIZE + (N - 1).bit_length()


def matmul_expected(A, B):
    C = [[0 for _ in range(N)] for _ in range(N)]
    for i in range(N):
        for j in range(N):
            total = 0
            for k in range(N):
                total += A[i][k] * B[k][j]
            C[i][j] = total
    return C


def print_matrix(mat):
    for row in mat:
        print(" ".join(f"{val:8}" for val in row))


def check_matrix(expected, actual):
    for i in range(N):
        for j in range(N):
            assert expected[i][j] == actual[i][j], (
                f"Mismatch at ({i}, {j}): expected {expected[i][j]}, got {actual[i][j]}"
            )


def pack_matrix(mat, elem_bits):
    packed = 0
    mask = (1 << elem_bits) - 1
    for i in range(N):
        for j in range(N):
            idx = i * N + j
            packed |= (mat[i][j] & mask) << (idx * elem_bits)
    return packed


def unpack_matrix(packed, elem_bits):
    mat = [[0 for _ in range(N)] for _ in range(N)]
    mask = (1 << elem_bits) - 1
    for i in range(N):
        for j in range(N):
            idx = i * N + j
            mat[i][j] = (packed >> (idx * elem_bits)) & mask
    return mat


async def setup_dut(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    dut.reset.value = 1
    dut.start.value = 0
    dut.A_matrix_flat.value = 0
    dut.B_matrix_flat.value = 0

    await ClockCycles(dut.clk, 3)
    dut.reset.value = 0
    await RisingEdge(dut.clk)


async def run_matmul(dut, A, B, verbose_busy=False):
    dut.A_matrix_flat.value = pack_matrix(A, DATA_BIT_SIZE)
    dut.B_matrix_flat.value = pack_matrix(B, DATA_BIT_SIZE)

    # Pulse start
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # Optional: print while busy
    seen_busy = False
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()

        if int(dut.busy.value):
            seen_busy = True
            if verbose_busy:
                print("busy=1")

        if seen_busy and int(dut.done.value):
            break

    return unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BIT_SIZE)


@cocotb.test()
async def simple_matrix_test(dut):
    await setup_dut(dut)

    print(f"\n--------- {N}x{N} Matrix Multiply Test ---------")

    A = [
        [1 + i for i in range(16)],
        [17 + i for i in range(16)],
        [33 + i for i in range(16)],
        [49 + i for i in range(16)],
        [65 + i for i in range(16)],
        [81 + i for i in range(16)],
        [97 + i for i in range(16)],
        [113 + i for i in range(16)],
        [129 + i for i in range(16)],
        [145 + i for i in range(16)],
        [161 + i for i in range(16)],
        [177 + i for i in range(16)],
        [193 + i for i in range(16)],
        [209 + i for i in range(16)],
        [225 + i for i in range(16)],
        [241 + i for i in range(16)]
    ]

    B = [
        [257 + i for i in range(16)],
        [273 + i for i in range(16)],
        [289 + i for i in range(16)],
        [305 + i for i in range(16)],
        [321 + i for i in range(16)],
        [337 + i for i in range(16)],
        [353 + i for i in range(16)],
        [369 + i for i in range(16)],
        [385 + i for i in range(16)],
        [401 + i for i in range(16)],
        [417 + i for i in range(16)],
        [433 + i for i in range(16)],
        [449 + i for i in range(16)],
        [465 + i for i in range(16)],
        [481 + i for i in range(16)],
        [497 + i for i in range(16)]
    ]

    expected = matmul_expected(A, B)
    actual = await run_matmul(dut, A, B, verbose_busy=False)

    print("\nExpected:")
    print_matrix(expected)

    print("\nActual:")
    print_matrix(actual)

    check_matrix(expected, actual)

    print("\nSimple matrix test passed.")


@cocotb.test()
async def maxed_matrix_test(dut):
    await setup_dut(dut)

    max_val = (1 << DATA_BIT_SIZE) - 1

    A = [[max_val for _ in range(N)] for _ in range(N)]
    B = [[max_val for _ in range(N)] for _ in range(N)]

    expected = matmul_expected(A, B)
    actual = await run_matmul(dut, A, B, verbose_busy=False)

    print("\nExpected (maxed):")
    print_matrix(expected)

    print("\nActual (maxed):")
    print_matrix(actual)

    check_matrix(expected, actual)

    print("\nMaxed matrix test passed.")

@cocotb.test()
async def zeroes_matrix_test(dut):
    await setup_dut(dut)

    print(f"\n--------- {N}x{N} Matrix Multiply Test ---------")

    A = [
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)]
    ]

    B = [
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)]
    ]

    expected = matmul_expected(A, B)
    actual = await run_matmul(dut, A, B, verbose_busy=False)

    print("\nExpected:")
    print_matrix(expected)

    print("\nActual:")
    print_matrix(actual)

    check_matrix(expected, actual)

    print("\nSimple matrix test passed.")

@cocotb.test()
async def identity_matrix_test(dut):
    await setup_dut(dut)

    print(f"\n--------- {N}x{N} Matrix Multiply Test ---------")

    A = [
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    ]

    B = [
        [1 + i for i in range(16)],
        [17 + i for i in range(16)],
        [33 + i for i in range(16)],
        [49 + i for i in range(16)],
        [65 + i for i in range(16)],
        [81 + i for i in range(16)],
        [97 + i for i in range(16)],
        [113 + i for i in range(16)],
        [129 + i for i in range(16)],
        [145 + i for i in range(16)],
        [161 + i for i in range(16)],
        [177 + i for i in range(16)],
        [193 + i for i in range(16)],
        [209 + i for i in range(16)],
        [225 + i for i in range(16)],
        [241 + i for i in range(16)]
    ]

    expected = matmul_expected(A, B)
    actual = await run_matmul(dut, A, B, verbose_busy=False)

    print("\nExpected:")
    print_matrix(expected)

    print("\nActual:")
    print_matrix(actual)

    check_matrix(expected, actual)

    print("\nSimple matrix test passed.")

@cocotb.test()
async def reset_matrix_test(dut):
    await setup_dut(dut)

    print(f"\n--------- {N}x{N} Matrix Multiply Test ---------")

    A = [
        [1 + i for i in range(16)],
        [17 + i for i in range(16)],
        [33 + i for i in range(16)],
        [49 + i for i in range(16)],
        [65 + i for i in range(16)],
        [81 + i for i in range(16)],
        [97 + i for i in range(16)],
        [113 + i for i in range(16)],
        [129 + i for i in range(16)],
        [145 + i for i in range(16)],
        [161 + i for i in range(16)],
        [177 + i for i in range(16)],
        [193 + i for i in range(16)],
        [209 + i for i in range(16)],
        [225 + i for i in range(16)],
        [241 + i for i in range(16)]
    ]

    B = [
        [257 + i for i in range(16)],
        [273 + i for i in range(16)],
        [289 + i for i in range(16)],
        [305 + i for i in range(16)],
        [321 + i for i in range(16)],
        [337 + i for i in range(16)],
        [353 + i for i in range(16)],
        [369 + i for i in range(16)],
        [385 + i for i in range(16)],
        [401 + i for i in range(16)],
        [417 + i for i in range(16)],
        [433 + i for i in range(16)],
        [449 + i for i in range(16)],
        [465 + i for i in range(16)],
        [481 + i for i in range(16)],
        [497 + i for i in range(16)]
    ]

    expected = matmul_expected(A, B)
    actual = await run_matmul(dut, A, B, verbose_busy=False)

    await RisingEdge(dut.clk)   # leave ReadOnly phase safely

    dut.reset.value = 1
    dut.start.value = 0
    dut.A_matrix_flat.value = 0
    dut.B_matrix_flat.value = 0

    await ClockCycles(dut.clk, 3)
    dut.reset.value = 0
    await RisingEdge(dut.clk)

    actual = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BIT_SIZE)

    expected = [
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)],
        [0*i for i in range(16)]
    ]

    print("\nExpected:")
    print_matrix(expected)

    print("\nActual:")
    print_matrix(actual)

    check_matrix(expected, actual)

    print("\nSimple matrix test passed.")