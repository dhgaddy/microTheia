
# Simple cocotb tests for flash_boot_controller
#
# This version uses the ACTUAL default parameter values from the controller/core
# draft instead of a toy-sized build.
#
# Default values used here:
#   PWR_WAIT_CYCLES   = 1024
#   RST_WAIT_CYCLES   = 1024
#   SPI_DIV           = 4
#   NUM_CLASSES       = 4
#   GRID_SIZE         = 16
#   READOUT_BINS      = 8
#   WEIGHT_BITS       = 8
#   SCORE_BITS        = 36
#   FLASH_WEIGHT_BASE = 0x00000000
#   FLASH_THRESH_BASE = 0x00100000
#
# Derived values:
#   FEATURE_COUNT     = 8 * 16 * 16 = 2048 weights per class
#   Total weight bytes= 4 * 2048 = 8192 bytes
#   THRESH_COUNT      = 2 * 4 = 8 threshold entries
#   THRESH_BYTES      = ceil(36/8) = 5 bytes per threshold entry
#
# So the full-boot test below now checks a realistic-size load:
#   - 8192 weight writes
#   - 8 threshold writes
#
# To keep the test easy to read, the flash memory is generated with simple
# patterns instead of listing thousands of bytes by hand.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, Timer
from cocotb.triggers import SimTimeoutError
from cocotb.triggers import with_timeout


async def reset_dut(dut):
    dut.boot_req_i.value = 0
    dut.reload_req_i.value = 0
    dut.debug_req_i.value = 0
    dut.spi_miso_i.value = 0

    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def pulse(signal, clocks=1):
    signal.value = 1
    await ClockCycles(signal._path.parent.clk, clocks)
    signal.value = 0


async def wait_until_high(signal, clk, timeout_cycles=5000):
    for _ in range(timeout_cycles):
        await RisingEdge(clk)
        if int(signal.value) == 1:
            return
    raise SimTimeoutError(f"Timed out waiting for {signal._path}")


class SimpleSpiFlash:
    """
    Very small behavioral SPI flash model for bring-up tests.

    What this model supports:
    - 66h  : reset enable
    - 99h  : reset
    - 9Fh  : read ID
    - 03h  : 3-byte read
    - 13h  : 4-byte read

    This model only cares about the behaviors that matter for the boot test:
    - commands start when CS# goes low
    - commands end when CS# goes high
    - RDID returns three bytes
    - READ/4READ returns bytes from a memory dictionary
    """

    def __init__(self, dut, id_bytes=(0x01, 0x60, 0x19), memory=None):
        self.dut = dut
        self.id_bytes = list(id_bytes)
        self.memory = memory or {}
        self.cmd_log = []

    async def recv_byte(self):
        """Receive one byte from MOSI, MSB first, sampled on SCK rising edges."""
        value = 0
        for _ in range(8):
            await RisingEdge(self.dut.spi_sck_o)
            bit = int(self.dut.spi_mosi_o.value)
            value = (value << 1) | bit
        return value

    async def send_byte(self, value):
        """
        Send one byte on MISO.

        The controller samples MISO on SCK rising edges, so this model updates
        MISO on each falling edge before the next rising edge happens.
        """
        for bit_idx in range(7, -1, -1):
            await FallingEdge(self.dut.spi_sck_o)
            self.dut.spi_miso_i.value = (value >> bit_idx) & 1
            await RisingEdge(self.dut.spi_sck_o)

    async def run(self):
        while True:
            # Wait for a new SPI transaction.
            await FallingEdge(self.dut.spi_cs_n_o)

            # First byte in every transaction is the command opcode.
            cmd = await self.recv_byte()
            self.cmd_log.append(cmd)

            if cmd in (0x66, 0x99):
                # These commands are just recorded; no return data needed.
                await RisingEdge(self.dut.spi_cs_n_o)

            elif cmd == 0x9F:
                # RDID: return exactly three bytes.
                for b in self.id_bytes:
                    await self.send_byte(b)
                await RisingEdge(self.dut.spi_cs_n_o)

            elif cmd in (0x03, 0x13):
                # READ / 4READ: first receive the address bytes.
                addr_len = 4 if cmd == 0x13 else 3
                addr = 0
                for _ in range(addr_len):
                    next_byte = await self.recv_byte()
                    addr = (addr << 8) | next_byte

                # Now stream data bytes until CS# goes high.
                while int(self.dut.spi_cs_n_o.value) == 0:
                    data_byte = self.memory.get(addr, 0x00)
                    addr += 1
                    await self.send_byte(data_byte)

            else:
                # Unknown command. Just wait until the transaction ends.
                await RisingEdge(self.dut.spi_cs_n_o)


async def capture_weight_writes(dut, expected_count):
    writes = []
    while len(writes) < expected_count:
        await RisingEdge(dut.clk)
        if int(dut.weight_wr_valid_o.value) == 1:
            writes.append(
                (
                    int(dut.weight_wr_class_o.value),
                    int(dut.weight_wr_addr_o.value),
                    int(dut.weight_wr_data_o.value),
                )
            )
    return writes


def expected_weight_byte(class_idx, addr_idx):
    """
    Simple deterministic pattern for the realistic-size test.

    This makes it easy to predict what each write SHOULD be without manually
    listing 8192 bytes.
    """
    return ((class_idx * 37) + addr_idx) & 0xFF


def encode_threshold_be(value, num_bytes=5):
    """Encode one threshold value as big-endian bytes for flash memory."""
    out = []
    for shift in range((num_bytes - 1) * 8, -1, -8):
        out.append((value >> shift) & 0xFF)
    return out


async def capture_thresh_writes(dut, expected_count):
    writes = []
    while len(writes) < expected_count:
        await RisingEdge(dut.clk)
        if int(dut.thresh_wr_valid_o.value) == 1:
            writes.append(
                (
                    int(dut.thresh_wr_addr_o.value),
                    int(dut.thresh_wr_data_o.value),
                )
            )
    return writes


@cocotb.test()
async def test_reset_holds_core_and_flash_idle(dut):
    """
    Very first sanity check.

    After reset:
    - CS# should be high
    - the core should still be in reset
    - boot_done should be low
    - boot_fail should be low
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    assert int(dut.spi_cs_n_o.value) == 1, "CS# should be high after reset"
    assert int(dut.core_rst_o.value) == 1, "Core should be held in reset after reset"
    assert int(dut.boot_done_o.value) == 0, "boot_done should start low"
    assert int(dut.boot_fail_o.value) == 0, "boot_fail should start low"


@cocotb.test()
async def test_boot_starts_with_flash_reset_then_rdid(dut):
    """
    This checks the very beginning of the boot sequence.

    We expect the controller to send:
    - 0x66 (reset enable)
    - 0x99 (reset)
    - 0x9F (read ID)

    The important idea from the datasheet is that each command is framed by CS#.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    flash = SimpleSpiFlash(dut, id_bytes=(0x01, 0x60, 0x19), memory={})
    cocotb.start_soon(flash.run())

    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await ClockCycles(dut.clk, 1)
    dut.boot_req_i.value = 0

    # Wait until we have seen at least 3 commands.
    for _ in range(5000):
        await RisingEdge(dut.clk)
        if len(flash.cmd_log) >= 3:
            break
    else:
        raise SimTimeoutError("Did not observe enough flash commands")

    assert flash.cmd_log[0] == 0x66, "First command should be RSTEN (0x66)"
    assert flash.cmd_log[1] == 0x99, "Second command should be RST (0x99)"
    assert flash.cmd_log[2] == 0x9F, "Third command should be RDID (0x9F)"


@cocotb.test()
async def test_full_boot_loads_weights_and_thresholds(dut):
    """
    Full boot test using the ACTUAL default parameter sizes.

    What this test checks:
    - the flash reset sequence happens
    - RDID happens
    - 8192 weight bytes are written into the expected class/address slots
    - 8 threshold values are packed and written
    - boot_done goes high
    - core reset is released at the end
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # -----------------------------------------------------------------
    # Build a realistic-size flash image.
    #
    # Weight layout in flash:
    #   class 0 weights first   (2048 bytes)
    #   class 1 weights next    (2048 bytes)
    #   class 2 weights next    (2048 bytes)
    #   class 3 weights last    (2048 bytes)
    #
    # Threshold layout in flash:
    #   8 threshold entries, 5 bytes each, big-endian
    # -----------------------------------------------------------------
    flash_mem = {}

    feature_count = 8 * 16 * 16
    num_classes = 4
    weight_base = 0x00000000
    thresh_base = 0x00100000
    thresh_count = 2 * num_classes
    thresh_bytes = 5

    # Fill weight region with a simple predictable pattern.
    for cls in range(num_classes):
        for addr in range(feature_count):
            flash_addr = weight_base + (cls * feature_count) + addr
            flash_mem[flash_addr] = expected_weight_byte(cls, addr)

    # Choose threshold values that fit inside 36 bits and are easy to recognize.
    expected_threshold_values = [
        0x000000011,
        0x000000122,
        0x000000233,
        0x000000344,
        0x000000455,
        0x000000566,
        0x000000677,
        0x000000788,
    ]

    for idx, value in enumerate(expected_threshold_values):
        encoded = encode_threshold_be(value, thresh_bytes)
        for byte_offset, b in enumerate(encoded):
            flash_mem[thresh_base + (idx * thresh_bytes) + byte_offset] = b

    flash = SimpleSpiFlash(
        dut,
        id_bytes=(0x01, 0x60, 0x19),
        memory=flash_mem,
    )
    cocotb.start_soon(flash.run())

    weight_task = cocotb.start_soon(capture_weight_writes(dut, expected_count=8192))
    thresh_task = cocotb.start_soon(capture_thresh_writes(dut, expected_count=8))

    await reset_dut(dut)

    dut.boot_req_i.value = 1
    await ClockCycles(dut.clk, 1)
    dut.boot_req_i.value = 0

    # Realistic-size load needs a much larger timeout than the toy test.
    await with_timeout(wait_until_high(dut.boot_done_o, dut.clk, timeout_cycles=2000000), 50, "ms")

    weight_writes = await weight_task
    thresh_writes = await thresh_task

    # -----------------------------------------------------------------
    # Weight checks
    # -----------------------------------------------------------------
    assert len(weight_writes) == 8192, f"Expected 8192 weight writes, got {len(weight_writes)}"

    # Check the first few writes.
    assert weight_writes[0] == (0, 0, expected_weight_byte(0, 0))
    assert weight_writes[1] == (0, 1, expected_weight_byte(0, 1))
    assert weight_writes[2] == (0, 2, expected_weight_byte(0, 2))

    # Check boundary between class 0 and class 1.
    assert weight_writes[2047] == (0, 2047, expected_weight_byte(0, 2047))
    assert weight_writes[2048] == (1, 0, expected_weight_byte(1, 0))

    # Check boundary between class 1 and class 2.
    assert weight_writes[4095] == (1, 2047, expected_weight_byte(1, 2047))
    assert weight_writes[4096] == (2, 0, expected_weight_byte(2, 0))

    # Check boundary between class 2 and class 3.
    assert weight_writes[6143] == (2, 2047, expected_weight_byte(2, 2047))
    assert weight_writes[6144] == (3, 0, expected_weight_byte(3, 0))

    # Check the very last write.
    assert weight_writes[8191] == (3, 2047, expected_weight_byte(3, 2047))

    # -----------------------------------------------------------------
    # Threshold checks
    # -----------------------------------------------------------------
    assert len(thresh_writes) == 8, f"Expected 8 threshold writes, got {len(thresh_writes)}"

    for idx, (addr, value) in enumerate(thresh_writes):
        assert addr == idx, f"Threshold write {idx} went to wrong address: {addr}"
        assert value == expected_threshold_values[idx], (
            f"Threshold write {idx} mismatch: got 0x{value:x}, "
            f"expected 0x{expected_threshold_values[idx]:x}"
        )

    # -----------------------------------------------------------------
    # Final status checks
    # -----------------------------------------------------------------
    assert int(dut.core_rst_o.value) == 0, "Core reset should be released after boot"
    assert int(dut.boot_done_o.value) == 1, "boot_done should go high when loading finishes"
    assert int(dut.boot_fail_o.value) == 0, "boot_fail should stay low on a good boot"

    # Command-order sanity check.
    assert flash.cmd_log[0] == 0x66
    assert flash.cmd_log[1] == 0x99
    assert flash.cmd_log[2] == 0x9F
    assert sum(1 for cmd in flash.cmd_log if cmd in (0x03, 0x13)) >= 2, "Expected weight read and threshold read commands"