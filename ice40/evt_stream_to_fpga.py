"""Live relay: GenX320 EVT2 stream -> iCE40 UART, with gesture RX decoding."""

import argparse
import sys
import time
from collections import Counter

try:
    import serial
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install -r requirements.txt")
    sys.exit(1)


VALID_EVT2_TYPES = {0x0, 0x1, 0x8, 0xA, 0xE, 0xF}
GESTURE_NAMES = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}

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
    """Parse both voxel_bin binary responses and gradient_map ASCII lines."""

    def __init__(self):
        self._pending_gesture_byte = None
        self._ascii = bytearray()
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

            if b in (0x0A, 0x0D):
                if self._ascii:
                    line = self._ascii.decode("ascii", errors="ignore").strip().upper()
                    self._ascii.clear()
                    if line in GESTURE_NAMES.values():
                        self.gesture_counts[line] += 1
                        gestures.append(line)
                continue

            if 32 <= b <= 126 or self._ascii:
                self._ascii.append(b)
                if len(self._ascii) > 64:
                    self._ascii.clear()

        return gestures


def probe_fpga_architecture(fpga, timeout_s=0.2):
    """
    Send 0xFF echo probe.
    voxel_bin replies 0x55.
    gradient_map does not reply; send 3 pad bytes to re-align 4-byte assembler.
    """
    try:
        fpga.reset_input_buffer()
    except Exception:
        pass

    fpga.write(b"\xFF")
    time.sleep(timeout_s)
    resp = fpga.read(fpga.in_waiting or 1)
    if b"\x55" in resp:
        return "voxel_bin", resp

    # In gradient_map top, 0xFF is consumed as first byte of a 4-byte word.
    # Push 3 bytes so RX assembler returns to byte-0 alignment.
    fpga.write(b"\x00\x00\x00")
    return "gradient_map", resp


def format_top_types(type_counts, top_n=4):
    if not type_counts:
        return "-"
    return ", ".join(f"{t:X}:{c}" for t, c in type_counts.most_common(top_n))


def main():
    parser = argparse.ArgumentParser(
        description="Relay live EVT2 stream from GenX320 (STM32 CDC) to iCE40 FPGA."
    )
    parser.add_argument("--dvs", default="/dev/ttyACM0", help="DVS serial port")
    parser.add_argument("--fpga", default="/dev/ttyUSB1", help="FPGA serial port")
    parser.add_argument("--dvs-baud", type=int, default=115200)
    parser.add_argument("--fpga-baud", type=int, default=115200)
    parser.add_argument("--chunk", type=int, default=4096, help="Read chunk bytes")
    parser.add_argument("--probe-bytes", type=int, default=4096, help="Alignment probe bytes")
    parser.add_argument(
        "--arch",
        choices=["auto", "voxel_bin", "gradient_map"],
        default="auto",
        help="FPGA response protocol",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds (0=run forever)")
    parser.add_argument("--save-raw", type=str, default="", help="Optional raw DVS capture file")
    parser.add_argument(
        "--max-write-bytes",
        type=int,
        default=512,
        help="Max bytes per UART write call (smaller fails faster on wrong port)",
    )
    parser.add_argument("--debug", action="store_true", help="Print extra diagnostics")
    parser.add_argument(
        "--swap-xy",
        action="store_true",
        default=False,
        help=(
            "Swap x and y fields in every CD EVT2.0 word before sending to the FPGA. "
            "Use this when gestures are mis-classified: LEFT/RIGHT appear as UP/DOWN "
            "(camera mounted 90° rotated, or camera y-axis is in the upper EVT2 field). "
            "Run tools/evt2_layout_probe.py first to confirm the axis orientation."
        ),
    )
    args = parser.parse_args()

    print(f"[OPEN] DVS={args.dvs} @ {args.dvs_baud}, FPGA={args.fpga} @ {args.fpga_baud}"
          f"  swap_xy={'YES' if args.swap_xy else 'no'}")
    try:
        dvs = serial.Serial(args.dvs, args.dvs_baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open DVS port {args.dvs}: {e}")
        return 1
    try:
        fpga = serial.Serial(args.fpga, args.fpga_baud, timeout=0.05, write_timeout=0.2)
    except serial.SerialException as e:
        dvs.close()
        print(f"[ERROR] Cannot open FPGA port {args.fpga}: {e}")
        return 1

    detected_arch = args.arch
    parser_rx = FPGAResponseParser()
    if args.arch == "auto":
        detected_arch, probe_resp = probe_fpga_architecture(fpga)
        probe_hex = probe_resp.hex() if probe_resp else "-"
        print(f"[FPGA] Auto-detected {detected_arch} (probe_rx={probe_hex})")
    else:
        print(f"[FPGA] Using forced architecture: {detected_arch}")

    save_f = open(args.save_raw, "wb") if args.save_raw else None
    if save_f:
        print(f"[SAVE] Raw stream capture -> {args.save_raw}")

    probe = dvs.read(args.probe_bytes)
    if save_f and probe:
        save_f.write(probe)

    if len(probe) >= 16:
        off, ratio, counts = detect_alignment_offset(probe)
        print(
            f"[ALIGN] offset={off} valid_ratio={ratio:.3f} "
            f"types={format_top_types(counts)}"
        )
        if ratio < 0.6:
            print("[WARN] Low EVT2 valid ratio in probe. Stream may not be EVT2.")
        probe = probe[off:]
    else:
        print(f"[ALIGN] Probe too small ({len(probe)} B), skipping offset detection.")

    buf = bytearray(probe)
    total_words = 0
    valid_words = 0
    invalid_words = 0
    cd_words = 0
    time_high_words = 0
    sent_words = 0
    write_errors = 0
    evt_type_counts = Counter()

    start = time.time()
    last_report = start
    print("[RUN] Live relay started. Ctrl+C to stop.")
    try:
        while True:
            try:
                data = dvs.read(dvs.in_waiting or args.chunk)
            except serial.SerialException as e:
                print(f"[ERROR] DVS read failed: {e}")
                break
            if data:
                if save_f:
                    save_f.write(data)
                buf.extend(data)

            full_len = (len(buf) // 4) * 4
            if full_len:
                tx_batch = bytearray()
                for i in range(0, full_len, 4):
                    word_bytes_le = bytes(buf[i:i + 4])
                    word = int.from_bytes(word_bytes_le, "little")
                    evt_type = (word >> 28) & 0xF

                    total_words += 1
                    evt_type_counts[evt_type] += 1
                    if evt_type in VALID_EVT2_TYPES:
                        valid_words += 1
                        if evt_type in (0x0, 0x1):
                            cd_words += 1
                            if args.swap_xy:
                                word = swap_xy_in_evt2_word(word)
                        elif evt_type == 0x8:
                            time_high_words += 1
                        # FPGA top-level expects bytes MSB-first for each 32-bit EVT2 word.
                        tx_batch.extend(word.to_bytes(4, "big"))
                    else:
                        invalid_words += 1

                if tx_batch:
                    sent_now = 0
                    for j in range(0, len(tx_batch), args.max_write_bytes):
                        chunk = tx_batch[j:j + args.max_write_bytes]
                        try:
                            sent_now += fpga.write(chunk)
                        except (serial.SerialTimeoutException, serial.SerialException) as e:
                            write_errors += 1
                            if args.debug:
                                print(f"[WARN] FPGA write failed: {e}")
                            try:
                                fpga.reset_output_buffer()
                            except Exception:
                                pass
                            break
                    sent_words += (sent_now // 4)

                del buf[:full_len]

            try:
                rx = fpga.read(fpga.in_waiting or 0)
                if rx:
                    gestures = parser_rx.feed(rx)
                    if gestures and args.debug:
                        print(f"[GESTURE] {' '.join(gestures)}")
            except serial.SerialException as e:
                print(f"[ERROR] FPGA read failed: {e}")
                break

            now = time.time()
            if now - last_report >= 1.0:
                elapsed = now - start
                valid_ratio = (valid_words / total_words) if total_words else 0.0
                totals = " ".join(
                    f"{g}={parser_rx.gesture_counts.get(g, 0)}"
                    for g in ("UP", "DOWN", "LEFT", "RIGHT")
                )
                print(
                    f"{elapsed:7.1f}s words={total_words} valid={valid_ratio:.3f} "
                    f"cd={cd_words} th={time_high_words} sent={sent_words} wr_err={write_errors} "
                    f"{totals}"
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
        dvs.close()

    elapsed = time.time() - start
    print("\n[SUMMARY]")
    print(f"  arch={detected_arch}")
    print(f"  elapsed_s={elapsed:.2f}")
    print(f"  words_total={total_words}")
    print(f"  words_valid={valid_words}")
    print(f"  words_invalid={invalid_words}")
    print(f"  words_cd={cd_words}")
    print(f"  words_time_high={time_high_words}")
    print(f"  words_sent={sent_words}")
    print(f"  write_errors={write_errors}")
    print(f"  types_top={format_top_types(evt_type_counts, top_n=8)}")
    print("  gestures=" + ", ".join(f"{k}:{v}" for k, v in sorted(parser_rx.gesture_counts.items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
