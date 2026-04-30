import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

PWR_WAIT = 1024

async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.boot_req_i.value = 0
    dut.reload_req_i.value = 0
    dut.debug_req_i.value = 0
    dut.evt_reads_done.value = 0
    dut.evt_ld_bypass.value = 0

    await Timer(10, units="ns")
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def wait_cycles(dut, n):
    for _ in range(n):
        await RisingEdge(dut.clk)


@cocotb.test()
async def test_full_boot_sequence(dut):
    """Normal boot → load → wait → done → run"""

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await RisingEdge(dut.clk)
    dut.boot_req_i.value = 0

    await wait_cycles(dut, 2)
    assert dut.core_rst_o.value == 1

    await wait_cycles(dut, PWR_WAIT)

    await RisingEdge(dut.clk)
    assert dut.evt_ld_en.value == 1, "evt_ld_en not asserted"

    dut.evt_reads_done.value = 1
    await RisingEdge(dut.clk)
    dut.evt_reads_done.value = 0

    await wait_cycles(dut, 2)

    assert dut.core_rst_o.value == 0, "core not released"
    assert dut.boot_done_o.value == 1, "boot_done not asserted"
    assert dut.evt_ld_en.value == 0, "evt_ld_en not cleared"


@cocotb.test()
async def test_bypass_mode(dut):
    """Test evt_ld_bypass skips wait"""

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut.boot_req_i.value = 1
    dut.evt_ld_bypass.value = 1
    await RisingEdge(dut.clk)
    dut.boot_req_i.value = 0

    await wait_cycles(dut, PWR_WAIT)
    await wait_cycles(dut, 3)

    assert dut.boot_done_o.value == 1
    assert dut.core_rst_o.value == 0


@cocotb.test()
async def test_reload_from_run(dut):
    """Test reload path from RUN"""

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await RisingEdge(dut.clk)
    dut.boot_req_i.value = 0

    await wait_cycles(dut, PWR_WAIT + 5)
    dut.evt_reads_done.value = 1
    await RisingEdge(dut.clk)
    dut.evt_reads_done.value = 0

    await wait_cycles(dut, 2)

    assert dut.core_rst_o.value == 0

    dut.reload_req_i.value = 1
    await RisingEdge(dut.clk)
    dut.reload_req_i.value = 0

    await wait_cycles(dut, 2)
    assert dut.core_rst_o.value == 1
    assert dut.boot_done_o.value == 0


@cocotb.test()
async def test_debug_interrupt(dut):
    """Debug request should override everything"""

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await RisingEdge(dut.clk)
    dut.boot_req_i.value = 0

    await wait_cycles(dut, 10)

    dut.debug_req_i.value = 1
    await RisingEdge(dut.clk)

    assert dut.core_rst_o.value == 1
    assert dut.boot_done_o.value == 0

    dut.debug_req_i.value = 0
    dut.boot_req_i.value = 1
    await RisingEdge(dut.clk)
    dut.boot_req_i.value = 0

    await wait_cycles(dut, 2)
    assert dut.core_rst_o.value == 1