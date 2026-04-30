# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors

from collections import Counter
from pathlib import Path
import os
import struct

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, NextTimeStep

try:
    from util.config_parser import load_config
except Exception:
    load_config = None


# ---------------------------------------------------------------------------
# Basic config
# ---------------------------------------------------------------------------

MODULE = os.environ.get("TOPLEVEL", "system_package")

if load_config is not None:
    try:
        CFG = load_config(MODULE)
    except Exception:
        CFG = {}
else:
    CFG = {}


# ---------------------------------------------------------------------------
# Clock config
# ---------------------------------------------------------------------------
#
# Force the integration testbench to run at the real ASIC clock:
#
#   32 MHz = 31.25 ns = 31250 ps
#
# Using ps avoids cocotb/Icarus precision problems with fractional ns periods.
# ---------------------------------------------------------------------------

CLK_FREQ_HZ = int(os.environ.get("CLK_FREQ_HZ", "32000000"))
CHIP_PERIOD_PS = int(round(1_000_000_000_000 / CLK_FREQ_HZ))
CHIP_PERIOD_NS = CHIP_PERIOD_PS / 1000.0

DATA_WIDTH = 32

GRID_SIZE = int(CFG.get("GRID_SIZE", os.environ.get("GRID_SIZE", 16)))
READOUT_BINS = int(CFG.get("READOUT_BINS", os.environ.get("READOUT_BINS", 8)))
NUM_CLASSES = int(CFG.get("NUM_CLASSES", os.environ.get("NUM_CLASSES", 4)))
SENSOR_WIDTH = int(CFG.get("SENSOR_WIDTH", os.environ.get("SENSOR_WIDTH", 320)))
SENSOR_HEIGHT = int(CFG.get("SENSOR_HEIGHT", os.environ.get("SENSOR_HEIGHT", SENSOR_WIDTH)))

FEATURE_COUNT = GRID_SIZE * GRID_SIZE * READOUT_BINS

# half_cycles=2 means each SCLK half-period is 2 chip clk cycles.
# Full SCLK period is 4 chip clk cycles.
# At 32 MHz chip clk, this gives 8 MHz SCLK.
SPI_HALF_CHIP_CYCLES_DEFAULT = int(os.environ.get("SPI_HALF_CHIP_CYCLES", "2"))

# MAX_BIN_WORDS=0 means stream the whole file.
MAX_BIN_WORDS = int(os.environ.get("MAX_BIN_WORDS", "0"))

# Set to 0 if you only want to check that classification happens,
# not that the label matches the expected gesture.
ASSERT_EXPECTED_LABEL = int(os.environ.get("ASSERT_EXPECTED_LABEL", "1"))

# Log progress every N SPI words.
LOG_EVERY_WORDS = int(os.environ.get("LOG_EVERY_WORDS", "1000"))

GESTURE_NAMES = {
    0: "Down",
    1: "Left",
    2: "Right",
    3: "Up",
}

EXPECTED_BIN_FILE_CLASS = {
    0: 0,  # wave_down
    1: 1,  # wave_left
    2: 2,  # wave_right
    3: 3,  # wave_up
}


# ---------------------------------------------------------------------------
# EVT2 packet type encodings
# ---------------------------------------------------------------------------

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1

# Weight:
# [4 bit type], [8 bit weight], [11 bit feature_addr], [2 bit class_id], [7 don't care]
EVT_WEIGHT = 0x2

# Threshold:
# [4 bit type], [18 bit upper/lower threshold data], [3 bit threshold addr], [7 don't care]
EVT_THRESH_U = 0x3
EVT_THRESH_L = 0x4

EVT_TIME_HIGH = 0x8

# Debug:
# [4 bit type], [4 bit page select], [24 don't care]
DEBUG_PAGE = 0xE

EVT_BOOT_REQ   = 0xC
EVT_READS_DONE = 0xF


# ---------------------------------------------------------------------------
# EVT2 boot/program word builders
# ---------------------------------------------------------------------------

def build_evt2_weight(weight, feature_addr, class_id):
    """
    Build one EVT_WEIGHT command word.

    Layout (matches evt2_decoder.sv):
        [31:28] type         = 0x2
        [27:20] weight       = 8-bit weight value
        [19:9]  feature_addr = 11-bit address within class SRAM (0..2047)
        [8:7]   class_id     = 2-bit SRAM selector (0..3)
        [6:0]   don't care   = 0
    """
    return (
        ((EVT_WEIGHT & 0xF) << 28)
        | ((int(weight) & 0xFF) << 20)
        | ((int(feature_addr) & 0x7FF) << 9)
        | ((int(class_id) & 0x3) << 7)
    )


def build_evt2_thresh_upper(threshold_value, threshold_addr):
    """
    Build EVT_THRESH_U.

    Layout:
        [31:28] type             = 0x3
        [27:10] threshold upper  = upper 18 bits
        [9:7]   threshold addr   = 3 bits
        [6:0]   don't care       = 0
    """
    threshold_value = int(threshold_value) & ((1 << 36) - 1)
    upper = (threshold_value >> 18) & 0x3FFFF

    return (
        ((EVT_THRESH_U & 0xF) << 28)
        | ((upper & 0x3FFFF) << 10)
        | ((int(threshold_addr) & 0x7) << 7)
    )


def build_evt2_thresh_lower(threshold_value, threshold_addr):
    """
    Build EVT_THRESH_L.

    Layout:
        [31:28] type             = 0x4
        [27:10] threshold lower  = lower 18 bits
        [9:7]   threshold addr   = 3 bits
        [6:0]   don't care       = 0
    """
    threshold_value = int(threshold_value) & ((1 << 36) - 1)
    lower = threshold_value & 0x3FFFF

    return (
        ((EVT_THRESH_L & 0xF) << 28)
        | ((lower & 0x3FFFF) << 10)
        | ((int(threshold_addr) & 0x7) << 7)
    )


def build_evt2_reads_done():
    """
    End-of-boot/programming marker.
    """
    return (EVT_READS_DONE & 0xF) << 28


def build_evt2_debug_page(page):
    """
    Build DEBUG_PAGE word.

    Layout:
        [31:28] type = 0xE
        [27:24] page
        [23:0]  don't care
    """
    return ((DEBUG_PAGE & 0xF) << 28) | ((int(page) & 0xF) << 24)


def build_evt2_boot_req():
    """
    Build BOOT_REQ command word (type=0xC).
    Triggers chip_flash_fsm: ST_BOOT -> ST_LOAD so weights/thresholds can be written.
    """
    return (EVT_BOOT_REQ & 0xF) << 28


def build_evt2_time_high(payload):
    """
    Build EVT_TIME_HIGH word.

    Included in case synthetic EVT2 events are needed later.
    """
    return ((EVT_TIME_HIGH & 0xF) << 28) | (int(payload) & 0x0FFFFFFF)


# ---------------------------------------------------------------------------
# File loading helpers
# ---------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).resolve().parents[1]


def load_weights_from_mem():
    """
    Load per-class weight files.

    Returns:
        weights[class_id][feature_addr]
    """
    mem_keys = ["WEIGHT_MEM_C0", "WEIGHT_MEM_C1", "WEIGHT_MEM_C2", "WEIGHT_MEM_C3"]

    mem_defaults = [
        f"weights/{FEATURE_COUNT}weights_q8_c{c}.mem"
        for c in range(NUM_CLASSES)
    ]

    weights = []

    for c in range(NUM_CLASSES):
        rel_path = CFG.get(mem_keys[c], mem_defaults[c])
        path = _REPO_ROOT / rel_path

        if not path.exists():
            raise FileNotFoundError(f"Missing weight file for class {c}: {path}")

        vals = []

        for line in path.read_text(encoding="ascii").splitlines():
            line = line.strip()

            if not line or line.startswith("//"):
                continue

            try:
                vals.append(int(line, 16))
            except ValueError:
                vals.append(0)

        while len(vals) < FEATURE_COUNT:
            vals.append(0)

        weights.append(vals[:FEATURE_COUNT])

    return weights


def load_thresholds():
    """
    Load thresholds from weights/thresholds.mem.

    Expected order:
        addr 0..NUM_CLASSES-1              = class thresholds
        addr NUM_CLASSES..2*NUM_CLASSES-1 = diff thresholds
    """
    path = _REPO_ROOT / "weights" / "thresholds.mem"

    if not path.exists():
        cocotb.log.warning(f"No thresholds.mem found at {path}; using zeros")
        return [0] * (2 * NUM_CLASSES)

    vals = []

    for line in path.read_text(encoding="ascii").splitlines():
        line = line.strip()

        if not line or line.startswith("//"):
            continue

        try:
            vals.append(int(line, 16))
        except ValueError:
            vals.append(0)

    while len(vals) < 2 * NUM_CLASSES:
        vals.append(0)

    return vals[: 2 * NUM_CLASSES]


def build_boot_stream_words(weights, thresholds):
    """
    Build the full SPI boot/programming stream.

    Order:
        1. BOOT_REQ  — triggers chip_flash_fsm ST_BOOT -> ST_LOAD
        2. all weight words
        3. all threshold upper/lower words
        4. EVT_READS_DONE
    """
    words = [build_evt2_boot_req()]

    for class_id in range(NUM_CLASSES):
        for feature_addr in range(FEATURE_COUNT):
            words.append(
                build_evt2_weight(
                    weight=weights[class_id][feature_addr],
                    feature_addr=feature_addr,
                    class_id=class_id,
                )
            )

    for threshold_addr, threshold_value in enumerate(thresholds):
        words.append(build_evt2_thresh_upper(threshold_value, threshold_addr))
        words.append(build_evt2_thresh_lower(threshold_value, threshold_addr))

    words.append(build_evt2_reads_done())

    return words


def _default_bin_paths():
    test_set = _REPO_ROOT / "EVT2_gesture_set" / "test_set"

    if test_set.exists():
        return [
            test_set / "wave_down_sun_test1.bin",
            test_set / "wave_left_sun_test1.bin",
            test_set / "wave_right_sun_test1.bin",
            test_set / "wave_up_sun_test1.bin",
        ]

    legacy_root = _REPO_ROOT / "EVT2_gesture_set"

    return [
        legacy_root / "wave_down_sun_test1.bin",
        legacy_root / "wave_left_sun_test1.bin",
        legacy_root / "wave_right_sun_test1.bin",
        legacy_root / "wave_up_sun_test1.bin",
    ]


def _resolve_bin_files():
    """
    Override with:
        GESTURE_BIN_FILES="path0:path1:path2:path3"
    """
    env = os.environ.get("GESTURE_BIN_FILES", "")

    if env.strip():
        parts = [p.strip() for p in env.split(":") if p.strip()]

        if len(parts) != 4:
            raise ValueError(
                f"GESTURE_BIN_FILES must contain exactly 4 colon-separated paths, "
                f"got {len(parts)}: {parts}"
            )

        resolved = []

        for p in parts:
            path = Path(p)

            if not path.is_absolute():
                path = _REPO_ROOT / path

            resolved.append(path)

        return resolved

    return list(_default_bin_paths())


def _read_evt2_bin(path):
    """
    Read raw EVT2.0 binary file as little-endian 32-bit words.

    This also detects Git LFS pointer files, which are text placeholders
    and not real EVT2 recordings.
    """
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Missing EVT2 .bin file: {path}")

    data = path.read_bytes()

    lfs_header = b"version https://git-lfs.github.com/spec/v1"

    if data.startswith(lfs_header):
        text = data.decode("utf-8", errors="replace")

        raise RuntimeError(
            f"{path} is a Git LFS pointer file, not the real .bin recording.\n\n"
            f"File contents begin with:\n{text}\n\n"
            f"Fix this by running from the repo root:\n"
            f"    git lfs install\n"
            f"    git lfs pull\n\n"
            f"Or manually replace the file with the real binary recording.\n"
        )

    if len(data) % 4 != 0:
        raise RuntimeError(
            f"{path} size is {len(data)} bytes, which is not divisible by 4. "
            f"Expected raw 32-bit EVT2 words."
        )

    n_words = len(data) // 4

    words = list(struct.unpack_from(f"<{n_words}I", data, 0))

    if MAX_BIN_WORDS > 0:
        words = words[:MAX_BIN_WORDS]

    return words


# ---------------------------------------------------------------------------
# SPI mode 0 helpers
# ---------------------------------------------------------------------------

async def setup_system(dut):
    """
    Start chip clock and apply reset.

    Current system_package top-level uses active-high rst.
    """
    dut._log.info(
        f"Starting chip clock: CLK_FREQ_HZ={CLK_FREQ_HZ}, "
        f"period={CHIP_PERIOD_PS} ps ({CHIP_PERIOD_NS:.3f} ns)"
    )

    cocotb.start_soon(Clock(dut.clk, CHIP_PERIOD_PS, units="ps").start())

    await NextTimeStep()

    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0

    # system_package uses active-high rst.
    dut.rst.value = 1

    await ClockCycles(dut.clk, 16)
    await NextTimeStep()

    dut.rst.value = 0

    await ClockCycles(dut.clk, 50)


async def reset_system_no_new_clock(dut):
    """
    Reset system without starting another clock.
    Used by multi-recording test.

    Current system_package top-level uses active-high rst.
    """
    await NextTimeStep()

    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0

    # system_package uses active-high rst.
    dut.rst.value = 1

    await ClockCycles(dut.clk, 16)
    await NextTimeStep()

    dut.rst.value = 0

    await ClockCycles(dut.clk, 50)


async def wait_for_spi_ready(dut, max_cycles=5000):
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)

        if int(dut.spi_ready.value):
            dut._log.info(f"spi_ready asserted after {cycle} clk cycles")
            return

    raise AssertionError("spi_ready never asserted")


async def spi_mode0_stream_words(
    dut,
    mosi_words,
    width=DATA_WIDTH,
    half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT,
    pre_start_cycles=4,
    inter_word_low_cycles=4,
    post_finish_cycles=4,
    cs_high_gap_cycles=4,
    tag="spi_stream",
    capture_miso=False,
):
    """
    Manual SPI mode 0 streaming transfer.

    CS stays low across the whole stream:
        CS low -> word0 -> word1 -> ... -> wordN -> CS high

    SPI mode 0:
        - SCLK idles low.
        - MOSI is set while SCLK is low.
        - Slave samples MOSI on rising edge.
        - Master samples MISO while SCLK is high.
    """
    assert len(mosi_words) > 0, "mosi_words must not be empty"

    dut._log.info(
        f"{tag}: streaming {len(mosi_words)} words over SPI mode 0, "
        f"half_cycles={half_cycles}, capture_miso={capture_miso}"
    )

    miso_words = [] if capture_miso else None

    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0

    await ClockCycles(dut.clk, cs_high_gap_cycles)

    first_word = int(mosi_words[0]) & 0xFFFFFFFF

    dut.CS.value = 0
    dut.MOSI.value = (first_word >> (width - 1)) & 1

    await ClockCycles(dut.clk, pre_start_cycles)

    for word_idx, mosi_word in enumerate(mosi_words):
        mosi_word = int(mosi_word) & 0xFFFFFFFF
        miso_word = 0

        if word_idx % LOG_EVERY_WORDS == 0:
            dut._log.info(
                f"{tag}: word {word_idx}/{len(mosi_words)} "
                f"MOSI=0x{mosi_word:08X}"
            )

        for bit_idx in range(width):
            # Rising SCLK edge.
            dut.SCLK.value = 1

            await ClockCycles(dut.clk, half_cycles)

            # Sample MISO while SCLK is high.
            if capture_miso:
                miso_bit = int(dut.MISO.value)
                miso_word = (miso_word << 1) | miso_bit

            # Falling SCLK edge.
            dut.SCLK.value = 0

            await ClockCycles(dut.clk, half_cycles)

            # Present next MOSI bit while SCLK is low.
            if bit_idx != width - 1:
                dut.MOSI.value = (mosi_word >> (width - 2 - bit_idx)) & 1

        if capture_miso:
            miso_words.append(miso_word)

        # Keep CS low and SCLK low between words so wrapper can re-arm.
        await ClockCycles(dut.clk, inter_word_low_cycles)

        if word_idx != len(mosi_words) - 1:
            next_word = int(mosi_words[word_idx + 1]) & 0xFFFFFFFF
            dut.MOSI.value = (next_word >> (width - 1)) & 1

    await ClockCycles(dut.clk, post_finish_cycles)

    dut.CS.value = 1
    dut.MOSI.value = 0
    dut.SCLK.value = 0

    await ClockCycles(dut.clk, 8)

    dut._log.info(f"{tag}: finished streaming {len(mosi_words)} words")

    return miso_words if capture_miso else []


async def spi_mode0_transfer_word(dut, mosi_word, tag="spi_single"):
    """
    Single 32-bit SPI transfer with MISO capture.
    """
    miso_words = await spi_mode0_stream_words(
        dut,
        [mosi_word],
        tag=tag,
        capture_miso=True,
    )

    return miso_words[0]


def expected_miso_from_classification(gesture, confidence):
    """
    spi_wrapper returns:
        classification = {confidence, gesture[1:0]}
    shifted into bits [31:29].
    """
    classification = ((int(confidence) & 1) << 2) | (int(gesture) & 0b11)
    return classification << (DATA_WIDTH - 3)


# ---------------------------------------------------------------------------
# Internal gesture monitor
# ---------------------------------------------------------------------------

class GestureMonitor:
    """
    Tracks every gesture_valid pulse.

    The gesture signals are internal signals inside system_package.
    cocotb can usually see them because they are declared in the top module.
    """

    def __init__(self):
        self.count = 0
        self.latest_gesture = None
        self.latest_confidence = None

    async def run(self, dut):
        while True:
            await RisingEdge(dut.clk)

            if int(dut.gesture_valid.value):
                self.count += 1
                self.latest_gesture = int(dut.gesture.value)
                self.latest_confidence = int(dut.gesture_confidence.value)

                dut._log.info(
                    f"gesture_valid pulse #{self.count}: "
                    f"gesture={self.latest_gesture}, "
                    f"confidence={self.latest_confidence}"
                )


async def wait_for_monitor_count(dut, monitor, min_count=1, max_cycles=2_000_000):
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)

        if monitor.count >= min_count:
            return

    raise AssertionError(
        f"Timed out waiting for gesture monitor count >= {min_count}. "
        f"Current count={monitor.count}"
    )


async def wait_after_reads_done(dut):
    """
    Gives the boot/program FSM time to settle after EVT_READS_DONE.
    """
    await ClockCycles(dut.clk, 100)

    try:
        dut._log.info(f"debug_bus after READS_DONE = 0x{int(dut.debug_bus.value):08X}")
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Higher-level system helpers
# ---------------------------------------------------------------------------

async def boot_core_over_spi(dut):
    """
    Load all weights and thresholds over SPI using EVT2 boot command words.
    """
    weights = load_weights_from_mem()
    thresholds = load_thresholds()

    boot_words = build_boot_stream_words(weights, thresholds)

    dut._log.info(
        f"Boot stream contains {len(boot_words)} words: "
        f"BOOT_REQ + {NUM_CLASSES} classes * {FEATURE_COUNT} weights + "
        f"{len(thresholds)} thresholds * 2 + READS_DONE"
    )

    await spi_mode0_stream_words(
        dut,
        boot_words,
        tag="boot_weights_thresholds_reads_done",
        capture_miso=False,
    )

    await wait_after_reads_done(dut)

    return weights, thresholds


async def stream_bin_recording_over_spi(dut, bin_path):
    words = _read_evt2_bin(bin_path)

    dut._log.info(
        f"Streaming EVT2 recording {bin_path} over SPI: {len(words)} words"
    )

    return await spi_mode0_stream_words(
        dut,
        words,
        tag=f"evt2_bin_{Path(bin_path).stem}",
        capture_miso=False,
    )


async def read_classification_over_spi(dut):
    """
    Send a harmless dummy word after classification.

    MISO should return the wrapper's latched classification:
        {confidence, gesture[1:0]} in bits [31:29].
    """
    dummy_word = build_evt2_debug_page(0)

    miso = await spi_mode0_transfer_word(
        dut,
        dummy_word,
        tag="classification_readback",
    )

    return miso


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@logged_test()
async def test_system_package_boot_stream_over_spi(dut):
    """
    Sanity test:
        1. Reset system_package.
        2. Wait for spi_ready.
        3. Stream all weights over SPI.
        4. Stream all thresholds over SPI.
        5. Stream EVT_READS_DONE over SPI.

    This does not stream a .bin file yet.
    It proves the top-level boot/programming path can accept the full SPI stream.
    """
    await setup_system(dut)
    await wait_for_spi_ready(dut)

    await boot_core_over_spi(dut)

    dut._log.info("PASS: full boot stream was sent over SPI")


@logged_test()
async def test_system_package_boot_then_stream_one_bin_file(dut):
    """
    Full integration test:
        1. Reset system_package.
        2. Wait for spi_ready.
        3. Stream all weights/thresholds over SPI as EVT2 boot commands.
        4. Send EVT_READS_DONE.
        5. Stream one real EVT2 .bin recording over SPI.
        6. Watch gesture_valid from voxel_bin_core.
        7. Read latest latched classification back through MISO.

    Select clip with:
        BIN_INDEX=0,1,2,3

    Debug option:
        MAX_BIN_WORDS=5000
    """
    bin_files = _resolve_bin_files()

    bin_index = int(os.environ.get("BIN_INDEX", "0"))
    assert 0 <= bin_index < len(bin_files), f"Invalid BIN_INDEX={bin_index}"

    bin_path = bin_files[bin_index]
    expected_class = EXPECTED_BIN_FILE_CLASS.get(bin_index)

    await setup_system(dut)
    await wait_for_spi_ready(dut)

    await boot_core_over_spi(dut)

    monitor = GestureMonitor()
    monitor_task = cocotb.start_soon(monitor.run(dut))

    await stream_bin_recording_over_spi(dut, bin_path)

    # Let any final pipeline/classification pulse happen after the stream.
    await ClockCycles(dut.clk, 500)

    await wait_for_monitor_count(dut, monitor, min_count=1)

    gesture = monitor.latest_gesture
    confidence = monitor.latest_confidence

    dut._log.info(
        f"Observed latest classification from core: "
        f"gesture={gesture} ({GESTURE_NAMES.get(gesture, gesture)}), "
        f"confidence={confidence}, pulses={monitor.count}"
    )

    # Give spi_wrapper time to latch latest classification.
    await ClockCycles(dut.clk, 20)

    miso_word = await read_classification_over_spi(dut)
    expected_miso = expected_miso_from_classification(gesture, confidence)

    monitor_task.kill()

    assert miso_word == expected_miso, (
        f"MISO classification readback mismatch: "
        f"got 0x{miso_word:08X}, expected 0x{expected_miso:08X} "
        f"from latest gesture={gesture}, confidence={confidence}"
    )

    if ASSERT_EXPECTED_LABEL and expected_class is not None:
        assert gesture == expected_class, (
            f"Expected {GESTURE_NAMES[expected_class]} for {Path(bin_path).name}, "
            f"but DUT reported {GESTURE_NAMES.get(gesture, gesture)}"
        )

    dut._log.info(
        f"PASS: booted over SPI, streamed {Path(bin_path).name}, "
        f"latest gesture={GESTURE_NAMES.get(gesture, gesture)}, "
        f"confidence={confidence}, MISO=0x{miso_word:08X}"
    )


@logged_test()
async def test_system_package_boot_then_stream_all_bin_files(dut):
    """
    Full four-file integration test.

    This test is intentionally disabled by default because it is very long.

    Enable with:
        RUN_ALL_BIN_FILES=1
    """
    if int(os.environ.get("RUN_ALL_BIN_FILES", "0")) == 0:
        dut._log.info(
            "Skipping long all-bin-files test by default. "
            "Set RUN_ALL_BIN_FILES=1 to enable it."
        )
        return

    bin_files = _resolve_bin_files()

    observed = []

    for i, bin_path in enumerate(bin_files):
        dut._log.info("=" * 80)
        dut._log.info(f"Running full system clip {i}: {bin_path}")
        dut._log.info("=" * 80)

        if i == 0:
            await setup_system(dut)
        else:
            await reset_system_no_new_clock(dut)

        await wait_for_spi_ready(dut)

        await boot_core_over_spi(dut)

        monitor = GestureMonitor()
        monitor_task = cocotb.start_soon(monitor.run(dut))

        await stream_bin_recording_over_spi(dut, bin_path)

        await ClockCycles(dut.clk, 500)

        await wait_for_monitor_count(dut, monitor, min_count=1)

        gesture = monitor.latest_gesture
        confidence = monitor.latest_confidence

        await ClockCycles(dut.clk, 20)

        miso_word = await read_classification_over_spi(dut)
        expected_miso = expected_miso_from_classification(gesture, confidence)

        monitor_task.kill()

        assert miso_word == expected_miso, (
            f"[{Path(bin_path).name}] MISO readback mismatch: "
            f"got 0x{miso_word:08X}, expected 0x{expected_miso:08X}"
        )

        observed.append((gesture, confidence))

        expected_class = EXPECTED_BIN_FILE_CLASS.get(i)

        if ASSERT_EXPECTED_LABEL and expected_class is not None:
            assert gesture == expected_class, (
                f"[{Path(bin_path).name}] expected "
                f"{GESTURE_NAMES[expected_class]}, got "
                f"{GESTURE_NAMES.get(gesture, gesture)}"
            )

        dut._log.info(
            f"[{Path(bin_path).name}] PASS: "
            f"latest gesture={GESTURE_NAMES.get(gesture, gesture)}, "
            f"confidence={confidence}, pulses={monitor.count}, "
            f"MISO=0x{miso_word:08X}"
        )

    hist = Counter(g for g, _ in observed)

    dut._log.info(
        "Observed gesture histogram: "
        + " ".join(
            f"{GESTURE_NAMES.get(k, k)}:{v}"
            for k, v in sorted(hist.items())
        )
    )