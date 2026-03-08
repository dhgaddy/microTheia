#!/usr/bin/env python3
"""EVT2 stream relay to iCE40 FPGA.

Two operating modes:
  Live relay  -- reads EVT 2.0 words from the GenX320 via STM32 CDC UART and
                 forwards them in real-time to voxel_bin_top on the iCE40.
  File replay -- reads a captured Prophesee .raw / .bin file and replays it to
                 the FPGA at a controlled rate so the UART link is not overrun.

The GenX320 emits little-endian 32-bit EVT 2.0 words; voxel_bin_top expects
the same words big-endian (MSB first over UART, matching the relay tools).
This script byte-swaps each word before writing to the FPGA port.
"""

import argparse
import sys
import time
from collections import Counter

try:
    import serial
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)


# EVT 2.0 valid type codes:
#   0x0 = CD_OFF, 0x1 = CD_ON, 0x8 = TIME_HIGH
#   0xA = EXT_TRIGGER, 0xE = CONTINUED, 0xF = OTHER_FILTER
VALID_EVT2_TYPES = {0x0, 0x1, 0x8, 0xA, 0xE, 0xF}

# FPGA gesture encoding: 0=Down, 1=Left, 2=Right, 3=Up
GESTURE_NAMES = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}

# EVT2.0 CD event field masks / shifts
_EVT2_TYPE_MASK   = 0xF0000000
_EVT2_TS_MASK     = 0x0FC00000
_EVT2_X_MASK      = 0x003FF800   # bits [21:11]
_EVT2_Y_MASK      = 0x000007FF   # bits [10:0]


def swap_xy_in_evt2_word(word):
    """Swap the x[21:11] and y[10:0] fields of a CD EVT2.0 word in-place.

    Used when the camera outputs y in the upper coordinate field and x in the
    lower, OR when the camera is mounted 90° rotated so that the sensor x-axis
    corresponds to the physical vertical direction.  In both cases, without this
    swap the gesture classifier sees LEFT/RIGHT gestures as UP/DOWN.

    Non-CD events (type != 0 or 1) are returned unchanged.
    """
    evt_type = (word >> 28) & 0xF
    if evt_type not in (0x0, 0x1):
        return word
    ts_field = word & _EVT2_TS_MASK
    x_val    = (word >> 11) & 0x7FF
    y_val    =  word        & 0x7FF
    return (word & _EVT2_TYPE_MASK) | ts_field | (y_val << 11) | x_val


# Sensor resolution for the GenX320 is 320×320; valid coordinates are 0..319.
_SENSOR_MAX = 319


def flip_x_in_evt2_word(word):
    """Invert x: x_new = 319 - x_old.  Non-CD events returned unchanged."""
    evt_type = (word >> 28) & 0xF
    if evt_type not in (0x0, 0x1):
        return word
    x_val = (word >> 11) & 0x7FF
    return (word & ~_EVT2_X_MASK) | ((_SENSOR_MAX - x_val) << 11)


def flip_y_in_evt2_word(word):
    """Invert y: y_new = 319 - y_old.  Non-CD events returned unchanged."""
    evt_type = (word >> 28) & 0xF
    if evt_type not in (0x0, 0x1):
        return word
    y_val = word & 0x7FF
    return (word & ~_EVT2_Y_MASK) | (_SENSOR_MAX - y_val)


def detect_alignment_offset(data, sample_words=20000):
    """Return (best_offset, valid_ratio, counts)."""
    best = (0, -1.0, Counter())
    for off in range(4):
        n = (len(data) - off) // 4
        if n <= 0:
            continue
        n = min(n, sample_words)
        valid = 0
        counts = Counter()
        for i in range(n):
            word = int.from_bytes(data[off + 4 * i: off + 4 * i + 4], "little")
            evt_type = (word >> 28) & 0xF
            counts[evt_type] += 1
            if evt_type in VALID_EVT2_TYPES:
                valid += 1
        ratio = valid / n
        if ratio > best[1]:
            best = (off, ratio, counts)
    return best


class FPGAResponseParser:
    """Parse voxel_bin_top binary response packets.

    Gesture packet: [0xA0|class, conf_byte]  (2 bytes)
      class bits [1:0]: 0=Down, 1=Left, 2=Right, 3=Up
      conf_byte [7:4]: confidence nibble
    Status packet:  [0xBx]  (1 byte, upper nibble = 0xB)
    """

    def __init__(self):
        self._pending_gesture_byte = None
        self.gesture_counts = Counter()
        self.status_count = 0

    def feed(self, data):
        gestures = []
        for b in data:
            if self._pending_gesture_byte is not None:
                g = self._pending_gesture_byte & 0x03
                name = GESTURE_NAMES.get(g, "UNKNOWN")
                self.gesture_counts[name] += 1
                gestures.append(name)
                self._pending_gesture_byte = None
                continue

            if (b & 0xF0) == 0xA0:
                self._pending_gesture_byte = b
                continue

            if (b & 0xF0) == 0xB0:
                self.status_count += 1
                continue

            # Ignore any other byte (echo 0x55, diag bytes, etc.)

        return gestures


def verify_fpga_connection(fpga, timeout_s=0.3):
    """Send echo command 0xFF; voxel_bin_top must reply 0x55.

    Returns True on success, False otherwise.
    """
    try:
        fpga.reset_input_buffer()
    except Exception:
        pass
    fpga.write(b"\xFF")
    time.sleep(timeout_s)
    resp = fpga.read(fpga.in_waiting or 1)
    if b"\x55" in resp:
        print("[FPGA] Connection verified (echo 0xFF → 0x55)")
        return True
    print(f"[FPGA] WARNING: echo probe did not return 0x55 (got {resp.hex() or 'nothing'})")
    print("[FPGA]   Check: correct COM port? correct bitstream loaded? correct baud rate?")
    return False


def format_top_types(type_counts, top_n=4):
    if not type_counts:
        return "-"
    return ", ".join(f"{t:X}:{c}" for t, c in type_counts.most_common(top_n))


def skip_prophesee_raw_header(data: bytes) -> int:
    """Return byte offset where EVT2.0 words begin in a Prophesee .raw file.

    Prophesee .raw files start with a variable-length ASCII header that ends
    with a line containing only '% end'.  Each header line begins with '%'.
    If no header is detected (e.g. raw binary .bin captures) return 0.
    """
    pos = 0
    while pos < len(data):
        # Find newline
        nl = data.find(b'\n', pos)
        if nl == -1:
            break
        line = data[pos:nl]
        pos = nl + 1
        if line.startswith(b'%'):
            if line.strip() == b'% end':
                return pos  # first byte after header
        else:
            # First non-comment line without header marker — no header present
            return 0
    return 0


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Relay EVT 2.0 events from a Prophesee GenX320 (via STM32 CDC UART) "
            "to the voxel_bin_top iCE40 FPGA over UART, or replay a captured .raw file."
        )
    )
    parser.add_argument("--dvs", default="/dev/ttyACM0",
                        help="DVS/STM32 CDC serial port (live mode)")
    parser.add_argument("--fpga", default="/dev/ttyUSB1",
                        help="FPGA UART serial port")
    parser.add_argument("--dvs-baud", type=int, default=3000000,
                        help="DVS port baud rate (default: 3000000 — typical GenX320/STM32 CDC)")
    parser.add_argument("--fpga-baud", type=int, default=115200,
                        help="FPGA UART baud rate (default: 115200)")
    parser.add_argument("--chunk", type=int, default=4096,
                        help="Read chunk size in bytes (default: 4096)")
    parser.add_argument("--probe-bytes", type=int, default=8192,
                        help="Bytes read for byte-alignment probe (default: 8192)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop after N seconds (0 = run until Ctrl+C)")
    parser.add_argument("--save-raw", type=str, default="",
                        help="Save live DVS stream to file (live mode only)")
    parser.add_argument("--file", type=str, default="",
                        help="Replay a captured Prophesee .raw or .bin file to the FPGA")
    parser.add_argument("--loop", action="store_true", default=False,
                        help="Loop file replay continuously (only with --file)")
    parser.add_argument("--replay-rate", type=float, default=1.0,
                        help=(
                            "File replay speed multiplier (default: 1.0 = real-time). "
                            "Uses timestamps in TIME_HIGH words to pace output so the FPGA "
                            "receives events at the original capture rate. "
                            "Set to 0 to disable pacing (send as fast as UART allows)."
                        ))
    parser.add_argument(
        "--max-write-bytes", type=int, default=512,
        help="Max bytes per FPGA write call (default: 512)",
    )
    parser.add_argument(
        "--rate-limit", type=float, default=-1.0,
        help=(
            "Max EVT2 words/sec forwarded to FPGA (default: auto = fpga_baud/10/4*0.90). "
            "Excess CD events are dropped uniformly, preserving spatial pattern and "
            "real-time gesture timing. Set to 0 to disable. "
            "The first TIME_HIGH in each stream pass is forwarded to prime the decoder."
        ),
    )
    parser.add_argument("--no-echo-check", action="store_true", default=False,
                        help="Skip the 0xFF echo check on the FPGA port (use if FPGA is already running)")
    parser.add_argument("--pre-sync", action="store_true", default=False,
                        help=(
                            "Send soft reset (0xFC) and wait 1s before replay to synchronize FPGA bin timer. "
                            "Required for correct LEFT/RIGHT classification: their weights peak in bins 0+1, "
                            "so gesture data must land in the right phase of the 1-second window."
                        ))
    parser.add_argument("--debug", action="store_true",
                        help="Print gesture detections and extra diagnostics")
    parser.add_argument(
        "--swap-xy", action="store_true", default=False,
        help=(
            "Swap x[21:11] and y[10:0] in every CD word before sending to the FPGA. "
            "Use when LEFT/RIGHT are classified as UP/DOWN (camera mounted 90° rotated). "
            "Run evt2_layout_probe.py to confirm before using this flag."
        ),
    )
    parser.add_argument(
        "--flip-x", action="store_true", default=False,
        help=(
            "Invert x coordinate: x_new = 319 - x_old.  Applied after --swap-xy. "
            "Use when gestures are mirrored left/right (e.g. LEFT classified as RIGHT). "
            "Can be combined with --swap-xy."
        ),
    )
    parser.add_argument(
        "--flip-y", action="store_true", default=False,
        help=(
            "Invert y coordinate: y_new = 319 - y_old.  Applied after --swap-xy. "
            "Use when gestures are mirrored up/down (e.g. UP classified as DOWN). "
            "Can be combined with --swap-xy and/or --flip-x."
        ),
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------ open ports
    file_bytes = None
    file_data_offset = 0   # byte offset past any .raw header
    file_pos = 0
    dvs = None

    if args.file:
        print(f"[OPEN] FILE={args.file}  FPGA={args.fpga} @ {args.fpga_baud}"
              f"  swap_xy={'YES' if args.swap_xy else 'no'}"
              f"  flip_x={'YES' if args.flip_x else 'no'}"
              f"  flip_y={'YES' if args.flip_y else 'no'}"
              f"  loop={'YES' if args.loop else 'no'}"
              f"  replay_rate={args.replay_rate}"
              f"  pre_sync={'YES' if args.pre_sync else 'no'}")
        try:
            file_bytes = open(args.file, "rb").read()
        except OSError as e:
            print(f"[ERROR] Cannot open {args.file}: {e}")
            return 1
        file_data_offset = skip_prophesee_raw_header(file_bytes)
        if file_data_offset:
            print(f"[FILE] Skipped {file_data_offset}-byte Prophesee .raw header")
        print(f"[FILE] {len(file_bytes)} bytes total, "
              f"{(len(file_bytes) - file_data_offset) // 4} EVT2 words")
        file_pos = file_data_offset
    else:
        print(f"[OPEN] DVS={args.dvs} @ {args.dvs_baud}  FPGA={args.fpga} @ {args.fpga_baud}"
              f"  swap_xy={'YES' if args.swap_xy else 'no'}"
              f"  flip_x={'YES' if args.flip_x else 'no'}"
              f"  flip_y={'YES' if args.flip_y else 'no'}")
        try:
            dvs = serial.Serial(args.dvs, args.dvs_baud, timeout=0.05)
        except serial.SerialException as e:
            print(f"[ERROR] Cannot open DVS port {args.dvs}: {e}")
            return 1

    try:
        fpga = serial.Serial(args.fpga, args.fpga_baud, timeout=0.05, write_timeout=0.5)
    except serial.SerialException as e:
        if dvs:
            dvs.close()
        print(f"[ERROR] Cannot open FPGA port {args.fpga}: {e}")
        return 1

    # ------------------------------------------------------------------ FPGA echo check
    time.sleep(0.15)  # let FT2232H / FPGA UART settle after port open
    parser_rx = FPGAResponseParser()
    if not args.no_echo_check:
        if not verify_fpga_connection(fpga):
            print("[WARN] Continuing anyway — use --no-echo-check to suppress this check.")

    # ------------------------------------------------------------------ save file
    save_f = open(args.save_raw, "wb") if args.save_raw else None
    if save_f:
        print(f"[SAVE] Capturing live DVS stream -> {args.save_raw}")

    # ------------------------------------------------------------------ pre-sync (bin timer alignment)
    if args.pre_sync:
        fpga.write(b"\xFC")  # soft reset → FPGA bin timer resets to bin 0
        print("[SYNC] Soft reset sent, waiting 1s for stale bins 1-3 to flush...")
        time.sleep(1.0)
        fpga.reset_input_buffer()  # discard any spurious bytes from warmup period
        print("[SYNC] Done — FPGA bin timer aligned, sending gesture data now.")

    # ------------------------------------------------------------------ rate limiter setup
    # Auto-compute from baud rate if not specified: leave 10% headroom for UART framing overhead.
    # One TIME_HIGH is forwarded per stream pass to prime decoder state; the rate budget
    # is intended for CD/other traffic.
    if args.rate_limit < 0:
        word_rate_limit = (args.fpga_baud / 10 / 4) * 0.90   # e.g. 2592 words/sec at 115200
    else:
        word_rate_limit = args.rate_limit   # 0 = disabled
    if word_rate_limit > 0:
        print(f"[RATE] Limiting to {word_rate_limit:.0f} words/sec "
              f"({'auto' if args.rate_limit < 0 else 'manual'}); first TIME_HIGH forwarded per pass.")
    else:
        print("[RATE] Rate limiting disabled; first TIME_HIGH forwarded per pass.")

    # ------------------------------------------------------------------ byte-alignment probe
    if file_bytes is not None:
        probe = file_bytes[file_pos: file_pos + args.probe_bytes]
        file_pos += len(probe)
    else:
        probe = dvs.read(args.probe_bytes)
        if save_f and probe:
            save_f.write(probe)

    if len(probe) >= 16:
        off, ratio, counts = detect_alignment_offset(probe)
        print(f"[ALIGN] offset={off}  valid_ratio={ratio:.3f}  types={format_top_types(counts)}")
        if ratio < 0.6:
            print("[WARN] Low EVT2 valid ratio — stream may not be EVT2, or wrong baud rate.")
        probe = probe[off:]
    else:
        print(f"[ALIGN] Probe too small ({len(probe)} B), skipping alignment detection.")

    # ------------------------------------------------------------------ relay loop
    buf = bytearray(probe)
    total_words = 0
    valid_words = 0
    invalid_words = 0
    cd_words = 0
    time_high_words = 0
    sent_words = 0
    sent_words_rate_epoch = 0
    dropped_words = 0
    write_errors = 0
    evt_type_counts = Counter()
    decoder_primed = False

    # Real-time pacing for file replay.
    # TIME_HIGH words carry bits [33:6] of the µs timestamp (1 increment = 64 µs).
    # replay_wall_start is synced to the wall clock when the FIRST TIME_HIGH is seen,
    # so pre-loop overhead (alignment probe, etc.) does not inflate the time budget.
    replay_wall_start = None
    replay_sensor_start_us = None   # set on first TIME_HIGH seen
    last_sensor_us = None           # most recent TIME_HIGH value seen in current pass
    rate_epoch_start = time.time()

    start = time.time()
    last_report = start
    mode_label = "File replay" if file_bytes is not None else "Live relay"
    print(f"[RUN] {mode_label} started. Ctrl+C to stop.")

    try:
        while True:
            # ---- feed buffer ----
            if file_bytes is not None:
                chunk_end = min(file_pos + args.chunk, len(file_bytes))
                data = file_bytes[file_pos:chunk_end]
                file_pos = chunk_end
                if not data:
                    if args.loop:
                        file_pos = file_data_offset
                        replay_sensor_start_us = None
                        replay_wall_start = None
                        last_sensor_us = None
                        sent_words_rate_epoch = 0
                        rate_epoch_start = time.time()
                        decoder_primed = False
                        print("[FILE] Looping playback.")
                        continue
                    else:
                        print("[FILE] End of file.")
                        break
                buf.extend(data)
            else:
                try:
                    data = dvs.read(dvs.in_waiting or args.chunk)
                except serial.SerialException as e:
                    print(f"[ERROR] DVS read failed: {e}")
                    break
                if data:
                    if save_f:
                        save_f.write(data)
                    buf.extend(data)

            # ---- process complete words ----
            full_len = (len(buf) // 4) * 4
            if full_len:
                tx_batch = bytearray()
                for i in range(0, full_len, 4):
                    word = int.from_bytes(buf[i:i + 4], "little")
                    evt_type = (word >> 28) & 0xF

                    total_words += 1
                    evt_type_counts[evt_type] += 1

                    if evt_type not in VALID_EVT2_TYPES:
                        invalid_words += 1
                        continue

                    valid_words += 1
                    if evt_type in (0x0, 0x1):
                        cd_words += 1
                        if args.swap_xy:
                            word = swap_xy_in_evt2_word(word)
                        if args.flip_x:
                            word = flip_x_in_evt2_word(word)
                        if args.flip_y:
                            word = flip_y_in_evt2_word(word)
                    elif evt_type == 0x8:
                        time_high_words += 1
                        if file_bytes is not None and args.replay_rate > 0:
                            sensor_us_val = (word & 0x0FFFFFFF) << 6
                            if replay_sensor_start_us is None:
                                replay_sensor_start_us = sensor_us_val
                                replay_wall_start = time.time()  # sync wall clock to first event
                                rate_epoch_start = replay_wall_start
                                sent_words_rate_epoch = 0
                            last_sensor_us = sensor_us_val
                        # Forward one TIME_HIGH per pass so decoder accepts CD events.
                        if not decoder_primed:
                            tx_batch.extend(word.to_bytes(4, "big"))
                            decoder_primed = True
                        continue

                    # Decoder requires at least one TIME_HIGH after reset.
                    if not decoder_primed:
                        dropped_words += 1
                        continue

                    # Rate limiter: drop excess events when UART would be saturated.
                    # Uses elapsed wall time as the budget so gesture timing is preserved.
                    # For file replay, align to replay_wall_start (synced to first TIME_HIGH)
                    # so the budget clock matches the pacing clock.
                    if word_rate_limit > 0:
                        words_in_flight = sent_words_rate_epoch + len(tx_batch) // 4
                        rate_clock = replay_wall_start if replay_wall_start is not None else rate_epoch_start
                        budget = (time.time() - rate_clock) * word_rate_limit
                        if words_in_flight >= budget:
                            dropped_words += 1
                            continue

                    # FPGA expects big-endian (MSB first) 4-byte words over UART
                    tx_batch.extend(word.to_bytes(4, "big"))

                if tx_batch:
                    sent_now = 0
                    for j in range(0, len(tx_batch), args.max_write_bytes):
                        chunk = tx_batch[j:j + args.max_write_bytes]
                        try:
                            sent_now += fpga.write(chunk)
                        except (serial.SerialTimeoutException, serial.SerialException) as e:
                            write_errors += 1
                            if args.debug:
                                print(f"[WARN] FPGA write error: {e}")
                            try:
                                fpga.reset_output_buffer()
                            except Exception:
                                pass
                            break
                    sent_now_words = sent_now // 4
                    sent_words += sent_now_words
                    sent_words_rate_epoch += sent_now_words

                del buf[:full_len]

                # Chunk-level pacing: sleep once per chunk to match sensor elapsed time.
                if (file_bytes is not None and args.replay_rate > 0
                        and last_sensor_us is not None and replay_wall_start is not None):
                    sensor_elapsed_s = (
                        (last_sensor_us - replay_sensor_start_us) / 1e6 / args.replay_rate
                    )
                    wall_elapsed_s = time.time() - replay_wall_start
                    sleep_s = sensor_elapsed_s - wall_elapsed_s
                    if args.debug:
                        print(f"[PACE] sensor={sensor_elapsed_s*1000:.1f}ms "
                              f"wall={wall_elapsed_s*1000:.1f}ms "
                              f"sleep={sleep_s*1000:.1f}ms")
                    if sleep_s > 0.001:
                        time.sleep(sleep_s)

            # ---- read FPGA responses ----
            try:
                rx = fpga.read(fpga.in_waiting or 0)
                if rx:
                    gestures = parser_rx.feed(rx)
                    for g in gestures:
                        print(f"[GESTURE] {g}")
            except serial.SerialException as e:
                print(f"[ERROR] FPGA read failed: {e}")
                break

            # ---- periodic status report ----
            now = time.time()
            if now - last_report >= 1.0:
                elapsed = now - start
                valid_ratio = (valid_words / total_words) if total_words else 0.0
                g_totals = " ".join(
                    f"{GESTURE_NAMES[i]}={parser_rx.gesture_counts.get(GESTURE_NAMES[i], 0)}"
                    for i in range(4)
                )
                print(
                    f"{elapsed:7.1f}s  words={total_words}  valid={valid_ratio:.3f}  "
                    f"cd={cd_words}  th={time_high_words}  sent={sent_words}  "
                    f"dropped={dropped_words}  wr_err={write_errors}  {g_totals}"
                )
                last_report = now

            if args.duration > 0 and (now - start) >= args.duration:
                print(f"[RUN] Duration {args.duration}s reached.")
                break

    except KeyboardInterrupt:
        print("\n[RUN] Stopped by user.")
    finally:
        if save_f:
            save_f.close()
        fpga.close()
        if dvs:
            dvs.close()

    elapsed = time.time() - start
    print("\n[SUMMARY]")
    print(f"  mode={'file_replay' if file_bytes is not None else 'live_relay'}")
    print(f"  elapsed_s={elapsed:.2f}")
    print(f"  words_total={total_words}")
    print(f"  words_valid={valid_words}")
    print(f"  words_invalid={invalid_words}")
    print(f"  words_cd={cd_words}")
    print(f"  words_time_high={time_high_words}")
    print(f"  words_sent={sent_words}")
    print(f"  words_dropped={dropped_words}")
    print(f"  write_errors={write_errors}")
    print(f"  types_top={format_top_types(evt_type_counts, top_n=8)}")
    g_summary = ", ".join(
        f"{GESTURE_NAMES[i]}:{parser_rx.gesture_counts.get(GESTURE_NAMES[i], 0)}"
        for i in range(4)
    )
    print(f"  gestures={g_summary}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
