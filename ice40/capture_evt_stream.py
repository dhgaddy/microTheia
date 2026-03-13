#!/usr/bin/env python3
# usage: python3 capture_evt_stream.py COM3 trial_run.bin

import argparse
import time
import threading
from collections import Counter
import serial

stop_flag = False

VALID_TYPES = {0, 1, 8, 0xA, 0xE, 0xF}  # per evt_2_0.h


def wait_for_key():
    global stop_flag
    input("\nPress ENTER to stop capture...\n")
    stop_flag = True


def detect_best_offset(data: bytes, sample_words: int = 50000):
    best = (None, -1.0, None)

    for off in range(4):
        n = (len(data) - off) // 4
        if n <= 0:
            continue
        n = min(n, sample_words)

        counts = Counter()
        valid = 0
        total = 0

        base = off
        for i in range(n):
            w = int.from_bytes(data[base + 4 * i : base + 4 * i + 4], "little", signed=False)
            t = (w >> 28) & 0xF
            counts[t] += 1
            total += 1
            if t in VALID_TYPES:
                valid += 1

        ratio = valid / total if total else 0.0
        if ratio > best[1]:
            best = (off, ratio, counts)

    return best


def write_aligned(raw_path: str, aligned_path: str = "aligned.bin", sample_words: int = 50000):
    raw = open(raw_path, "rb").read()
    off, ratio, counts = detect_best_offset(raw, sample_words=sample_words)

    if off is None:
        raise RuntimeError("Could not determine alignment (file too small).")

    aligned = raw[off:]
    aligned = aligned[: (len(aligned) // 4) * 4]

    with open(aligned_path, "wb") as f:
        f.write(aligned)

    return off, ratio, counts, len(raw), len(aligned)


def analyze_evt2(aligned_path: str, args):
    data = open(aligned_path, "rb").read()
    if len(data) < 4:
        raise RuntimeError("Aligned file is too small to analyze.")

    total_words = len(data) // 4
    max_words = total_words if args.max_words <= 0 else min(total_words, args.max_words)

    type_counts = Counter()
    cd_events = 0
    invalid_types = 0
    out_of_range = 0
    no_time_high = 0
    time_high_updates = 0
    ts_backwards = 0
    ts_prev = None
    ts_back_min_delta = None

    time_high = None

    for i in range(max_words):
        word = int.from_bytes(data[i * 4 : i * 4 + 4], "little", signed=False)
        t = (word >> args.type_shift) & args.type_mask
        type_counts[t] += 1

        if t not in VALID_TYPES:
            invalid_types += 1
            continue

        if t == args.evt_time_high:
            time_high = word & args.time_high_mask
            time_high_updates += 1
            continue

        if t == args.evt_cd_off or t == args.evt_cd_on:
            cd_events += 1

            x = (word >> args.x_shift) & args.x_mask
            y = (word >> args.y_shift) & args.y_mask
            if x > args.max_x or y > args.max_y:
                out_of_range += 1

            ts_lsb = (word >> args.ts_shift) & args.ts_mask
            if time_high is None:
                no_time_high += 1
                full_ts = ts_lsb
            else:
                full_ts = (time_high << args.ts_bits) | ts_lsb

            if ts_prev is not None and full_ts < ts_prev:
                ts_backwards += 1
                delta = ts_prev - full_ts
                ts_back_min_delta = delta if ts_back_min_delta is None else min(ts_back_min_delta, delta)
            ts_prev = full_ts

    analyzed_bytes = max_words * 4
    valid_ratio = 1.0 - (invalid_types / max_words if max_words else 0.0)

    return {
        "total_words": total_words,
        "max_words": max_words,
        "analyzed_bytes": analyzed_bytes,
        "type_counts": type_counts,
        "valid_ratio": valid_ratio,
        "cd_events": cd_events,
        "invalid_types": invalid_types,
        "out_of_range": out_of_range,
        "no_time_high": no_time_high,
        "time_high_updates": time_high_updates,
        "ts_backwards": ts_backwards,
        "ts_back_min_delta": ts_back_min_delta,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="e.g. COM3")
    parser.add_argument("outfile", help="Raw output file, e.g. trial_run.bin")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Ignored by USB CDC but required by pyserial")
    parser.add_argument("--chunk", type=int, default=4096)
    parser.add_argument("--aligned", default="aligned.bin",
                        help="Aligned output file (default: aligned.bin)")
    parser.add_argument("--sample-words", type=int, default=50000,
                        help="Words to sample for offset detection (default: 50000)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Capture duration in seconds (0 = manual stop)")
    parser.add_argument("--max-words", type=int, default=0,
                        help="Max words to analyze (0 = all)")
    parser.add_argument("--x-shift", type=int, default=11,
                        help="X field shift (default: 11)")
    parser.add_argument("--y-shift", type=int, default=0,
                        help="Y field shift (default: 0)")
    parser.add_argument("--x-mask", type=lambda v: int(v, 0), default=0x7FF,
                        help="X field mask (default: 0x7FF)")
    parser.add_argument("--y-mask", type=lambda v: int(v, 0), default=0x7FF,
                        help="Y field mask (default: 0x7FF)")
    parser.add_argument("--ts-shift", type=int, default=22,
                        help="Timestamp LSB shift (default: 22)")
    parser.add_argument("--ts-mask", type=lambda v: int(v, 0), default=0x3F,
                        help="Timestamp LSB mask (default: 0x3F)")
    parser.add_argument("--ts-bits", type=int, default=6,
                        help="Timestamp LSB bit-width (default: 6)")
    parser.add_argument("--type-shift", type=int, default=28,
                        help="Type field shift (default: 28)")
    parser.add_argument("--type-mask", type=lambda v: int(v, 0), default=0xF,
                        help="Type field mask (default: 0xF)")
    parser.add_argument("--time-high-mask", type=lambda v: int(v, 0), default=0x0FFFFFFF,
                        help="TIME_HIGH payload mask (default: 0x0FFFFFFF)")
    parser.add_argument("--evt-cd-off", type=lambda v: int(v, 0), default=0x0,
                        help="CD_OFF type value (default: 0x0)")
    parser.add_argument("--evt-cd-on", type=lambda v: int(v, 0), default=0x1,
                        help="CD_ON type value (default: 0x1)")
    parser.add_argument("--evt-time-high", type=lambda v: int(v, 0), default=0x8,
                        help="TIME_HIGH type value (default: 0x8)")
    parser.add_argument("--max-x", type=int, default=319,
                        help="Max valid X coordinate (default: 319)")
    parser.add_argument("--max-y", type=int, default=319,
                        help="Max valid Y coordinate (default: 319)")
    args = parser.parse_args()

    print("\nOpening port:", args.port)
    ser = serial.Serial(args.port, args.baud, timeout=1)

    input("\nPress ENTER to start capture...\n")
    if args.duration and args.duration > 0:
        print(f"Capturing for {args.duration:.1f}s...")
    else:
        print("Capturing... (press ENTER again to stop)")

    global stop_flag
    stop_flag = False
    if not args.duration or args.duration <= 0:
        t = threading.Thread(target=wait_for_key, daemon=True)
        t.start()

    total = 0
    start = time.time()
    last_print = start

    with open(args.outfile, "wb") as f:
        while not stop_flag:
            data = ser.read(args.chunk)
            if data:
                f.write(data)
                total += len(data)

            now = time.time()
            if now - last_print >= 1.0:
                elapsed = now - start
                mb = total / (1024 * 1024)
                rate = mb / elapsed if elapsed > 0 else 0
                print(f"{mb:.2f} MB captured  |  {rate:.2f} MB/s")
                last_print = now

            if args.duration and args.duration > 0 and (now - start) >= args.duration:
                stop_flag = True

    ser.close()

    print("\nCapture complete.")
    print("Raw bytes written:", total)
    print("Raw file:", args.outfile)

    off, ratio, counts, raw_len, aligned_len = write_aligned(
        args.outfile, aligned_path=args.aligned, sample_words=args.sample_words
    )

    print("\nAlignment results:")
    print(f"  Best offset: {off}")
    print(f"  Valid EVT2 type ratio (sample): {ratio:.3f}")
    print(f"  Raw size: {raw_len} bytes, Aligned size: {aligned_len} bytes")
    print(f"  Aligned file: {args.aligned}")
    print("  Type counts (top 10):")
    for k, v in counts.most_common(10):
        print(f"    type {k:>2}: {v}")

    stats = analyze_evt2(args.aligned, args)
    print("\nEVT2 validation report:")
    print(f"  Analyzed bytes: {stats['analyzed_bytes']} (words: {stats['max_words']})")
    print(f"  Valid type ratio: {stats['valid_ratio']:.3f}")
    print(f"  CD events: {stats['cd_events']}")
    print(f"  Invalid type words: {stats['invalid_types']}")
    print(f"  Out-of-range coords: {stats['out_of_range']}")
    print(f"  TIME_HIGH updates: {stats['time_high_updates']}")
    print(f"  CD events before TIME_HIGH: {stats['no_time_high']}")
    print(f"  Timestamp regressions: {stats['ts_backwards']}")
    if stats["ts_back_min_delta"] is not None:
        print(f"  Smallest regression delta: {stats['ts_back_min_delta']}")

    print("  Type counts (top 10, analyzed):")
    for k, v in stats["type_counts"].most_common(10):
        print(f"    type {k:>2}: {v}")


if __name__ == "__main__":
    main()
