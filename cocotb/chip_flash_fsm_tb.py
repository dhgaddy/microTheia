import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, First, Timer
from cocotb.triggers import with_timeout, SimTimeoutError
from cocotb.utils import get_sim_time


MAIN_STATE_NAMES = {
    0: "ST_BOOT",
    1: "ST_LOAD",
    2: "ST_RUN",
    3: "ST_DEBUG",
}

LOAD_STATE_NAMES = {
    0:  "LD_IDLE",
    1:  "LD_WAIT_PWR",
    2:  "LD_SEND_RSTEN",
    3:  "LD_SEND_RST",
    4:  "LD_WAIT_RESET_GAP",
    5:  "LD_SEND_RDID",
    6:  "LD_RDID_BYTES",
    7:  "LD_CHECK_ID",
    8:  "LD_W_OPEN",
    9:  "LD_W_ADDR",
    10: "LD_W_DATA",
    11: "LD_W_WRITE",
    12: "LD_W_NEXT",
    13: "LD_T_DATA",
    14: "LD_T_WRITE",
    15: "LD_T_NEXT",
    16: "LD_DONE",
    17: "LD_FAIL",
    18: "LD_W_CAPTURE",
}

SPI_EDGE_TIMEOUT_US = 200

FEATURE_COUNT = 8 * 16 * 16
NUM_CLASSES = 4
TOTAL_WEIGHT_BYTES = FEATURE_COUNT * NUM_CLASSES

FLASH_WEIGHT_BASE = 0x00000000
FLASH_THRESH_BASE = 0x00002000

THRESH_COUNT = 8
THRESH_BYTES = 5


def main_state_name(v):
    return MAIN_STATE_NAMES.get(v, f"UNKNOWN_MAIN_{v}")


def load_state_name(v):
    return LOAD_STATE_NAMES.get(v, f"UNKNOWN_LOAD_{v}")


def sig_int(sig, default=0):
    try:
        return int(sig.value)
    except Exception:
        return default


def signal_width(sig):
    return len(sig)


def mask_for_width(width):
    return (1 << width) - 1


def weight_byte(global_idx: int) -> int:
    # Deterministic but not just 0,1,2,3...
    return ((global_idx * 73 + 41) ^ 0xA5) & 0xFF


def thresh_byte(global_idx: int) -> int:
    # Separate pattern for threshold bytes
    return ((global_idx * 91 + 17) ^ 0x5C) & 0xFF


def threshold_word(entry_idx: int) -> int:
    value = 0
    for byte_idx in range(THRESH_BYTES):
        value = (value << 8) | thresh_byte(entry_idx * THRESH_BYTES + byte_idx)
    return value


async def reset_dut(dut):
    dut.boot_req_i.value = 0
    dut.reload_req_i.value = 0
    dut.debug_req_i.value = 0
    dut.spi_miso_i.value = 0

    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def wait_until_high(signal, clk, timeout_cycles=5000):
    for _ in range(timeout_cycles):
        await RisingEdge(clk)
        if int(signal.value) == 1:
            return
    raise SimTimeoutError("Timed out waiting for signal to go high")


async def wait_for_main_state(dut, expected_main, timeout_cycles=20000):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if sig_int(dut.main_state_dbg_o) == expected_main:
            return
    raise SimTimeoutError(
        f"Timed out waiting for main state {main_state_name(expected_main)}"
    )


async def wait_for_load_state(dut, expected_load, timeout_cycles=20000):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if sig_int(dut.load_state_dbg_o) == expected_load:
            return
    raise SimTimeoutError(
        f"Timed out waiting for load state {load_state_name(expected_load)}"
    )


class SimpleSpiFlash:
    """
    Small SPI flash model for:
      0x66 : reset enable
      0x99 : reset
      0x9F : read ID
      0x03 : 3-byte read
      0x13 : 4-byte read
    """

    def __init__(self, dut, id_bytes=(0x01, 0x60, 0x19), memory=None):
        self.dut = dut
        self.id_bytes = list(id_bytes)
        self.memory = memory or {}
        self.cmd_log = []
        self.read_log = []

    async def recv_byte(self):
        value = 0

        for _ in range(8):
            rise = RisingEdge(self.dut.spi_sck_o)
            cs_rise = RisingEdge(self.dut.spi_cs_n_o)
            timeout = Timer(SPI_EDGE_TIMEOUT_US, units="us")

            trig = await First(rise, cs_rise, timeout)

            if trig is cs_rise:
                return None

            if trig is timeout:
                self.dut._log.warning("Flash model: timeout waiting for SCK rise in recv_byte()")
                return None

            value = ((value << 1) | sig_int(self.dut.spi_mosi_o)) & 0xFF

        return value

    async def send_byte(self, value, first_bit_already_loaded=False):
        if not first_bit_already_loaded:
            self.dut.spi_miso_i.value = (value >> 7) & 1

        for bit_idx in range(7, -1, -1):
            rise = RisingEdge(self.dut.spi_sck_o)
            cs_rise = RisingEdge(self.dut.spi_cs_n_o)
            timeout = Timer(SPI_EDGE_TIMEOUT_US, units="us")

            trig = await First(rise, cs_rise, timeout)

            if trig is cs_rise:
                return False

            if trig is timeout:
                self.dut._log.warning("Flash model: timeout waiting for SCK rise in send_byte()")
                return False

            if bit_idx > 0:
                fall = FallingEdge(self.dut.spi_sck_o)
                cs_rise2 = RisingEdge(self.dut.spi_cs_n_o)
                timeout2 = Timer(SPI_EDGE_TIMEOUT_US, units="us")

                trig2 = await First(fall, cs_rise2, timeout2)

                if trig2 is cs_rise2:
                    return False

                if trig2 is timeout2:
                    self.dut._log.warning("Flash model: timeout waiting for SCK fall in send_byte()")
                    return False

                self.dut.spi_miso_i.value = (value >> (bit_idx - 1)) & 1

        return True

    async def wait_for_cs_high(self):
        cs_rise = RisingEdge(self.dut.spi_cs_n_o)
        timeout = Timer(SPI_EDGE_TIMEOUT_US, units="us")

        trig = await First(cs_rise, timeout)
        if trig is timeout:
            self.dut._log.warning("Flash model: timeout waiting for CS# high")
            return False
        return True

    async def run(self):
        while True:
            self.dut.spi_miso_i.value = 0

            await FallingEdge(self.dut.spi_cs_n_o)

            cmd = await self.recv_byte()
            if cmd is None:
                self.dut.spi_miso_i.value = 0
                continue

            self.cmd_log.append(cmd)
            self.dut._log.info(
                f"[{get_sim_time('ns'):>10} ns] FLASH CMD 0x{cmd:02X}"
            )

            if cmd in (0x66, 0x99):
                await self.wait_for_cs_high()
                self.dut.spi_miso_i.value = 0

            elif cmd == 0x9F:
                for b in self.id_bytes:
                    ok = await self.send_byte(b)
                    if not ok:
                        break

                if sig_int(self.dut.spi_cs_n_o) == 0:
                    await self.wait_for_cs_high()

                self.dut.spi_miso_i.value = 0

            elif cmd in (0x03, 0x13):
                addr_len = 4 if cmd == 0x13 else 3
                addr = 0

                for _ in range(addr_len):
                    b = await self.recv_byte()
                    if b is None:
                        addr = None
                        break
                    addr = (addr << 8) | b

                if addr is None:
                    self.dut.spi_miso_i.value = 0
                    continue

                self.dut._log.info(
                    f"[{get_sim_time('ns'):>10} ns] FLASH READ START cmd=0x{cmd:02X} addr=0x{addr:08X}"
                )

                first_data = self.memory.get(addr, 0x00)
                self.dut.spi_miso_i.value = (first_data >> 7) & 1

                first_byte = True
                while sig_int(self.dut.spi_cs_n_o) == 0:
                    data_byte = self.memory.get(addr, 0x00)

                    if len(self.read_log) < 64:
                        self.read_log.append((addr, data_byte))

                    ok = await self.send_byte(
                        data_byte,
                        first_bit_already_loaded=first_byte,
                    )
                    if not ok:
                        break

                    first_byte = False
                    addr += 1

                self.dut.spi_miso_i.value = 0

            else:
                self.dut._log.warning(f"Flash model: unknown command 0x{cmd:02X}")
                await self.wait_for_cs_high()
                self.dut.spi_miso_i.value = 0


async def capture_weight_writes(dut, expected_count, timeout_cycles=3_000_000):
    writes = []

    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if sig_int(dut.weight_wr_valid_o) == 1:
            writes.append(
                (
                    sig_int(dut.weight_wr_class_o),
                    sig_int(dut.weight_wr_addr_o),
                    sig_int(dut.weight_wr_data_o),
                )
            )
            if len(writes) >= expected_count:
                return writes

    raise SimTimeoutError(
        f"Timed out capturing weight writes. Got {len(writes)} / {expected_count}"
    )


async def capture_thresh_writes(dut, expected_count, timeout_cycles=1_500_000):
    writes = []
    width = signal_width(dut.thresh_wr_data_o)
    mask = mask_for_width(width)

    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if sig_int(dut.thresh_wr_valid_o) == 1:
            writes.append(
                (
                    sig_int(dut.thresh_wr_addr_o),
                    sig_int(dut.thresh_wr_data_o) & mask,
                )
            )
            if len(writes) >= expected_count:
                return writes

    raise SimTimeoutError(
        f"Timed out capturing threshold writes. Got {len(writes)} / {expected_count}"
    )


def build_flash_memory():
    flash_mem = {}

    for global_idx in range(TOTAL_WEIGHT_BYTES):
        flash_mem[FLASH_WEIGHT_BASE + global_idx] = weight_byte(global_idx)

    for entry_idx in range(THRESH_COUNT):
        for byte_idx in range(THRESH_BYTES):
            flash_addr = FLASH_THRESH_BASE + (entry_idx * THRESH_BYTES) + byte_idx
            flash_mem[flash_addr] = thresh_byte(entry_idx * THRESH_BYTES + byte_idx)

    return flash_mem


@cocotb.test()
async def test_reset_holds_core_and_flash_idle(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    assert sig_int(dut.spi_cs_n_o) == 1, "CS# should be high after reset"
    assert sig_int(dut.core_rst_o) == 1, "Core should be held in reset after reset"
    assert sig_int(dut.boot_done_o) == 0, "boot_done should start low"
    assert sig_int(dut.boot_fail_o) == 0, "boot_fail should start low"


@cocotb.test()
async def test_boot_starts_with_flash_reset_then_rdid(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    flash = SimpleSpiFlash(dut, id_bytes=(0x01, 0x60, 0x19), memory={})
    cocotb.start_soon(flash.run())

    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await ClockCycles(dut.clk, 1)
    dut.boot_req_i.value = 0

    for _ in range(15000):
        await RisingEdge(dut.clk)
        if len(flash.cmd_log) >= 4:
            break
    else:
        raise SimTimeoutError(f"Did not observe enough commands. cmd_log={flash.cmd_log}")

    dut._log.info(f"Observed command log: {[hex(x) for x in flash.cmd_log]}")

    assert flash.cmd_log[0] == 0x66, f"Expected first command 0x66, got {flash.cmd_log}"
    assert flash.cmd_log[1] == 0x99, f"Expected second command 0x99, got {flash.cmd_log}"
    assert flash.cmd_log[2] == 0x9F, f"Expected third command 0x9F, got {flash.cmd_log}"
    assert flash.cmd_log[3] in (0x03, 0x13), (
        f"Expected fourth command to be a read command, got {flash.cmd_log}"
    )


@cocotb.test()
async def test_zero_mfr_id_goes_to_fail_then_debug(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    flash = SimpleSpiFlash(dut, id_bytes=(0x00, 0x60, 0x19), memory={})
    cocotb.start_soon(flash.run())

    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await ClockCycles(dut.clk, 1)
    dut.boot_req_i.value = 0

    await with_timeout(wait_for_main_state(dut, 3, timeout_cycles=20000), 20, "ms")

    assert sig_int(dut.boot_fail_o) == 1, "boot_fail should go high on bad ID"
    assert sig_int(dut.main_state_dbg_o) == 3, "Main state should end in ST_DEBUG"
    assert len(flash.cmd_log) >= 3, "Expected at least reset/reset/read-id sequence"


@cocotb.test()
async def test_debug_request_interrupts_load_and_holds_idle(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    flash = SimpleSpiFlash(dut, id_bytes=(0x01, 0x60, 0x19), memory={})
    cocotb.start_soon(flash.run())

    await reset_dut(dut)

    dut.debug_req_i.value = 1
    await ClockCycles(dut.clk, 2)

    assert sig_int(dut.main_state_dbg_o) == 3, "Should jump to ST_DEBUG when debug_req_i is high"
    assert sig_int(dut.spi_cs_n_o) == 1, "CS# should stay high in debug hold"
    assert sig_int(dut.core_rst_o) == 1, "Core should stay in reset in debug"
    assert len(flash.cmd_log) == 0, "Flash should not be touched during direct debug entry"

    dut.debug_req_i.value = 0


@cocotb.test()
async def test_debug_first_weight_bytes(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    flash_mem = {}
    for i in range(64):
        flash_mem[FLASH_WEIGHT_BASE + i] = weight_byte(i)

    flash = SimpleSpiFlash(dut, id_bytes=(0x01, 0x60, 0x19), memory=flash_mem)
    cocotb.start_soon(flash.run())

    first_writes_task = cocotb.start_soon(capture_weight_writes(dut, expected_count=16))

    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await ClockCycles(dut.clk, 1)
    dut.boot_req_i.value = 0

    observed = await with_timeout(first_writes_task, 20, "ms")

    expected = []
    for i in range(16):
        expected.append((0, i, weight_byte(i)))

    for idx, (cls, addr, data) in enumerate(observed):
        dut._log.info(
            f"[FIRST_WEIGHT_WRITES] idx={idx} class={cls} addr={addr} data=0x{data:02X}"
        )

    dut._log.info("========== DEBUG SUMMARY ==========")
    dut._log.info(
        "First flash bytes sent back on read: " +
        ", ".join(f"(addr=0x{addr:08X}, data=0x{data:02X})" for addr, data in flash.read_log[:16])
    )
    dut._log.info(
        "First observed weight writes: " +
        ", ".join(f"(class={c}, addr={a}, data=0x{d:02X})" for c, a, d in observed)
    )

    for idx in range(16):
        dut._log.info(
            f"[COMPARE] idx={idx} expected=(class={expected[idx][0]}, addr={expected[idx][1]}, data=0x{expected[idx][2]:02X}) "
            f"observed=(class={observed[idx][0]}, addr={observed[idx][1]}, data=0x{observed[idx][2]:02X})"
        )

    assert observed == expected, (
        "First 16 weight writes do not match the non-sequential pattern.\n"
        f"Observed: {observed}\nExpected: {expected}"
    )


@cocotb.test()
async def test_full_boot_loads_weights_thresholds_and_reload_restarts(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    flash_mem = build_flash_memory()
    flash = SimpleSpiFlash(dut, id_bytes=(0x01, 0x60, 0x19), memory=flash_mem)
    cocotb.start_soon(flash.run())

    weight_task = cocotb.start_soon(
        capture_weight_writes(dut, expected_count=TOTAL_WEIGHT_BYTES)
    )
    thresh_task = cocotb.start_soon(
        capture_thresh_writes(dut, expected_count=THRESH_COUNT)
    )

    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await ClockCycles(dut.clk, 1)
    dut.boot_req_i.value = 0

    await with_timeout(
        wait_until_high(dut.boot_done_o, dut.clk, timeout_cycles=2_500_000),
        80,
        "ms",
    )

    weight_writes = await with_timeout(weight_task, 80, "ms")
    thresh_writes = await with_timeout(thresh_task, 80, "ms")

    thresh_width = signal_width(dut.thresh_wr_data_o)
    thresh_mask = mask_for_width(thresh_width)
    expected_thresh_values = [
        threshold_word(i) & thresh_mask for i in range(THRESH_COUNT)
    ]

    first_16_writes = weight_writes[:16]
    first_16_flash = flash.read_log[:16]

    dut._log.info(
        "[FLASH_FIRST_BYTES] " +
        ", ".join(f"(addr=0x{addr:08X}, data=0x{data:02X})" for addr, data in first_16_flash)
    )
    dut._log.info(
        "First 16 weight writes from full boot: " +
        ", ".join(f"(class={c}, addr={a}, data=0x{d:02X})" for c, a, d in first_16_writes)
    )
    dut._log.info(
        "First 16 flash bytes seen by model: " +
        ", ".join(f"(addr=0x{addr:08X}, data=0x{data:02X})" for addr, data in first_16_flash)
    )

    assert len(weight_writes) == TOTAL_WEIGHT_BYTES, (
        f"Expected {TOTAL_WEIGHT_BYTES} weight writes, got {len(weight_writes)}"
    )
    assert len(thresh_writes) == THRESH_COUNT, (
        f"Expected {THRESH_COUNT} threshold writes, got {len(thresh_writes)}"
    )

    for idx, (cls, addr, data) in enumerate(weight_writes):
        exp_cls = idx // FEATURE_COUNT
        exp_addr = idx % FEATURE_COUNT
        exp_data = weight_byte(idx)

        assert (cls, addr) == (exp_cls, exp_addr), (
            f"Weight write {idx} wrong location: "
            f"got (class={cls}, addr={addr}), "
            f"expected (class={exp_cls}, addr={exp_addr})"
        )
        assert data == exp_data, (
            f"Weight write {idx} wrong data: got 0x{data:02X}, expected 0x{exp_data:02X}"
        )

    for idx, (addr, value) in enumerate(thresh_writes):
        assert addr == idx, f"Threshold write {idx} wrong address: got {addr}, expected {idx}"
        assert value == expected_thresh_values[idx], (
            f"Threshold write {idx} wrong data: got 0x{value:X}, expected 0x{expected_thresh_values[idx]:X}"
        )

    assert sig_int(dut.core_rst_o) == 0, "Core reset should be released after boot"
    assert sig_int(dut.boot_done_o) == 1, "boot_done should go high after loading"
    assert sig_int(dut.boot_fail_o) == 0, "boot_fail should stay low on good boot"

    assert len(flash.cmd_log) >= 4, f"Expected at least 4 flash commands, got {flash.cmd_log}"
    assert flash.cmd_log[0] == 0x66, f"Expected first command 0x66, got {flash.cmd_log}"
    assert flash.cmd_log[1] == 0x99, f"Expected second command 0x99, got {flash.cmd_log}"
    assert flash.cmd_log[2] == 0x9F, f"Expected third command 0x9F, got {flash.cmd_log}"
    assert flash.cmd_log[3] in (0x03, 0x13), f"Expected read command, got {flash.cmd_log}"

    old_cmd_count = len(flash.cmd_log)

    dut.reload_req_i.value = 1
    await ClockCycles(dut.clk, 1)
    dut.reload_req_i.value = 0

    for _ in range(30000):
        await RisingEdge(dut.clk)
        if len(flash.cmd_log) >= old_cmd_count + 4:
            break
    else:
        raise SimTimeoutError(
            f"Reload did not restart flash command sequence. cmd_log={flash.cmd_log}"
        )

    assert flash.cmd_log[old_cmd_count + 0] == 0x66, "Reload should start with 0x66"
    assert flash.cmd_log[old_cmd_count + 1] == 0x99, "Reload should then send 0x99"
    assert flash.cmd_log[old_cmd_count + 2] == 0x9F, "Reload should then send 0x9F"
    assert flash.cmd_log[old_cmd_count + 3] in (0x03, 0x13), "Reload should restart read command"
