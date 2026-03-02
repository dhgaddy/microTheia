import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, ReadOnly

N = 8 # Size of the square matrices

def matmul_expected(A, B):
    C = [[0 for _ in range(N)] for _ in range(N)]
    for i in range(N):
        for j in range(N):
            total = 0
            for k in range(N):
                total += A[i][k] * B[k][j]
            C[i][j] = total & 0xFFFF  # 16-bit truncation
    return C

def print_matrix(mat):
    for row in mat:
        print(" ".join(f"{val:6}" for val in row))

def get_a_ports(dut):
    return [
        dut.A_cell_1, dut.A_cell_2, dut.A_cell_3, dut.A_cell_4,
        dut.A_cell_5, dut.A_cell_6, dut.A_cell_7, dut.A_cell_8
    ]

def get_b_ports(dut):
    return [
        dut.B_cell_1, dut.B_cell_2, dut.B_cell_3, dut.B_cell_4,
        dut.B_cell_5, dut.B_cell_6, dut.B_cell_7, dut.B_cell_8
    ]

def get_out_ports(dut):
    return [
        dut.out1, dut.out2, dut.out3, dut.out4, dut.out5, dut.out6, dut.out7, dut.out8,
        dut.out9, dut.out10, dut.out11, dut.out12, dut.out13, dut.out14, dut.out15, dut.out16,
        dut.out17, dut.out18, dut.out19, dut.out20, dut.out21, dut.out22, dut.out23, dut.out24,
        dut.out25, dut.out26, dut.out27, dut.out28, dut.out29, dut.out30, dut.out31, dut.out32,
        dut.out33, dut.out34, dut.out35, dut.out36, dut.out37, dut.out38, dut.out39, dut.out40,
        dut.out41, dut.out42, dut.out43, dut.out44, dut.out45, dut.out46, dut.out47, dut.out48,
        dut.out49, dut.out50, dut.out51, dut.out52, dut.out53, dut.out54, dut.out55, dut.out56,
        dut.out57, dut.out58, dut.out59, dut.out60, dut.out61, dut.out62, dut.out63, dut.out64
    ]

def read_output_matrix(dut):
    outs = get_out_ports(dut)
    vals = [int(port.value) for port in outs]
    return [vals[r * N:(r + 1) * N] for r in range(N)]

def check_matrix(expected, actual):
    for i in range(N):
        for j in range(N):
            assert expected[i][j] == actual[i][j], (
                f"Mismatch at ({i}, {j}): expected {expected[i][j]}, got {actual[i][j]}"
            )

async def setup_dut(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    for p in get_a_ports(dut):
        p.value = 0
    for p in get_b_ports(dut):
        p.value = 0

    dut.start.value = 0
    dut.reset.value = 1

    await ClockCycles(dut.clk, 3)
    dut.reset.value = 0
    await RisingEdge(dut.clk)

def build_diagonal_inputs(A, B, t):
    """
    At cycle t, each input lane gets one value from the active diagonal.
    A lane r gets A[r][t-r] if valid.
    B lane c gets B[t-c][c] if valid.
    Otherwise input is 0.
    """
    a_vals = [0] * N
    b_vals = [0] * N

    for r in range(N):
        k = t - r
        if 0 <= k < N:
            a_vals[r] = A[r][k]

    for c in range(N):
        k = t - c
        if 0 <= k < N:
            b_vals[c] = B[k][c]

    return a_vals, b_vals

async def drive_diagonal_wave(dut, A, B, verbose=True):
    a_ports = get_a_ports(dut)
    b_ports = get_b_ports(dut)

    # Total active diagonal-feed cycles: 2N-1
    for t in range(2 * N - 1):
        a_vals, b_vals = build_diagonal_inputs(A, B, t)

        await FallingEdge(dut.clk)
        for i in range(N):
            a_ports[i].value = a_vals[i]
            b_ports[i].value = b_vals[i]

        await RisingEdge(dut.clk)
        await ReadOnly()

        if verbose:
            print(f"\nCycle t={t}")
            print(f"A inputs: {a_vals}")
            print(f"B inputs: {b_vals}")
            print_matrix(read_output_matrix(dut))

    # Clear inputs after the last active wave
    await FallingEdge(dut.clk)
    for p in a_ports:
        p.value = 0
    for p in b_ports:
        p.value = 0

@cocotb.test()
async def simple_matrix_test(dut):
    await setup_dut(dut)

    print("\n--------- 8x8 Matrix Multiply Test ---------")

    A = [
        [1, 2, 3, 4, 5, 6, 7, 8],
        [9, 10, 11, 12, 13, 14, 15, 16],
        [17, 18, 19, 20, 21, 22, 23, 24],
        [25, 26, 27, 28, 29, 30, 31, 32],
        [33, 34, 35, 36, 37, 38, 39, 40],
        [41, 42, 43, 44, 45, 46, 47, 48],
        [49, 50, 51, 52, 53, 54, 55, 56],
        [57, 58, 59, 60, 61, 62, 63, 64],
    ]

    B = [
        [65, 66, 67, 68, 69, 70, 71, 72],
        [73, 74, 75, 76, 77, 78, 79, 80],
        [81, 82, 83, 84, 85, 86, 87, 88],
        [89, 90, 91, 92, 93, 94, 95, 96],
        [97, 98, 99, 100, 101, 102, 103, 104],
        [105, 106, 107, 108, 109, 110, 111, 112],
        [113, 114, 115, 116, 117, 118, 119, 120],
        [121, 122, 123, 124, 125, 126, 127, 128],
    ]

    expected = matmul_expected(A, B)

    # Start pulse
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # Feed the same diagonal-wave pattern
    await drive_diagonal_wave(dut, A, B, verbose=True)

    # Flush pipeline
    # Keep this as a tunable number depending on your internal latency
    await ClockCycles(dut.clk, 8)

    await RisingEdge(dut.clk)
    await ReadOnly()

    actual = read_output_matrix(dut)

    print("\nExpected:")
    print_matrix(expected)

    print("\nActual:")
    print_matrix(actual)

    check_matrix(expected, actual)

    print("\nSimple matrix test passed.")