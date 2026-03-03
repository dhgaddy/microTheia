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


# @cocotb.test()
# async def simple_matrix_test(dut):
#     await setup_dut(dut)

#     print(f"\n--------- {N}x{N} Matrix Multiply Test ---------")

#     A = [
#         [1, 2, 3, 4, 5, 6, 7, 8],
#         [9, 10, 11, 12, 13, 14, 15, 16],
#         [17, 18, 19, 20, 21, 22, 23, 24],
#         [25, 26, 27, 28, 29, 30, 31, 32],
#         [33, 34, 35, 36, 37, 38, 39, 40],
#         [41, 42, 43, 44, 45, 46, 47, 48],
#         [49, 50, 51, 52, 53, 54, 55, 56],
#         [57, 58, 59, 60, 61, 62, 63, 64],
#     ]

#     B = [
#         [65, 66, 67, 68, 69, 70, 71, 72],
#         [73, 74, 75, 76, 77, 78, 79, 80],
#         [81, 82, 83, 84, 85, 86, 87, 88],
#         [89, 90, 91, 92, 93, 94, 95, 96],
#         [97, 98, 99, 100, 101, 102, 103, 104],
#         [105, 106, 107, 108, 109, 110, 111, 112],
#         [113, 114, 115, 116, 117, 118, 119, 120],
#         [121, 122, 123, 124, 125, 126, 127, 128],
#     ]

#     expected = matmul_expected(A, B)
#     actual = await run_matmul(dut, A, B, verbose_busy=True)

#     print("\nExpected:")
#     print_matrix(expected)

#     print("\nActual:")
#     print_matrix(actual)

#     check_matrix(expected, actual)

#     print("\nSimple matrix test passed.")


@cocotb.test()
async def maxed_matrix_test(dut):
    await setup_dut(dut)

    max_val = (1 << DATA_BIT_SIZE) - 1

    A = [[max_val for _ in range(N)] for _ in range(N)]
    B = [[max_val for _ in range(N)] for _ in range(N)]

    expected = matmul_expected(A, B)
    actual = await run_matmul(dut, A, B, verbose_busy=True)

    print("\nExpected (maxed):")
    print_matrix(expected)

    print("\nActual (maxed):")
    print_matrix(actual)

    check_matrix(expected, actual)

    print("\nMaxed matrix test passed.")