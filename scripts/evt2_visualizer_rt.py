#!/usr/bin/env python3
"""
EVT2 real-time visualizer for aligned.bin (uint32 stream).

Key fixes vs earlier version:
- Starts mid-stream safely: ignores TD events until first valid EVT_TIME_HIGH
- Robust EVT_TIME_HIGH handling: rejects absurd jumps so one bad word can't turn
  your capture into a ~4.6 hour "recording"
- Timestamp shifting: playback time starts at t=0 at the first TD event kept

Usage examples:
  python3 evt2_visualizer_rt.py aligned.bin --tick-us 1.0 --frame-ms 30 --max-s 5
  python3 evt2_visualizer_rt.py aligned.bin --tick-us 1.0 --frame-ms 30 --start-s 10 --max-s 5
  python3 evt2_visualizer_rt.py aligned.bin --tick-us 1.0 --frame-ms 30 --keep-open
"""

import argparse
import time
import numpy as np
import matplotlib.pyplot as plt


# EVT2 types (matches your evt_2_0.h)
TD_LOW = 0x0
TD_HIGH = 0x1
EV_TIME_HIGH = 0x8


def reconstruct_evt2_time_ticks(words: np.ndarray, max_jump_th: int) -> np.ndarray:
    """
    Robust EVT2 time reconstruction.

    TIME_HIGH payload = low 28 bits.
    Full ticks: (time_high_ext << 6) | ts_low

    Handles:
      - start mid-stream: TD before first TIME_HIGH -> -1
      - rejects absurd forward jumps (corruption)
      - handles wrap/reset of 28-bit TIME_HIGH
    """
    types = (words >> 28) & 0xF
    ts_lo = (words >> 22) & 0x3F
    th_payload = (words & 0x0FFFFFFF).astype(np.int64)

    T = np.full(words.shape[0], -1, dtype=np.int64)

    have_th = False
    base = 0          # adds multiples of 2^28 when wrap occurs
    th = 0            # last accepted 28-bit TIME_HIGH
    th_ext = 0        # base + th

    WRAP_GUARD = 1 << 27  # if it drops by > half-range, assume wrap/reset

    for i in range(words.shape[0]):
        t = int(types[i])

        if t == EV_TIME_HIGH:
            new_th = int(th_payload[i])

            if not have_th:
                have_th = True
                th = new_th
                th_ext = base + th
            else:
                if new_th >= th:
                    # forward move
                    if (new_th - th) <= max_jump_th:
                        th = new_th
                        th_ext = base + th
                    else:
                        # suspicious huge jump -> ignore
                        pass
                else:
                    # backward move: either wrap/reset or glitch
                    if (th - new_th) > WRAP_GUARD:
                        # treat as wrap/reset of 28-bit TIME_HIGH
                        base += (1 << 28)
                        th = new_th
                        th_ext = base + th
                    else:
                        # small backward -> ignore as glitch
                        pass

        if have_th:
            T[i] = (int(th_ext) << 6) | int(ts_lo[i])

    return T



def parse_evt2_td(words: np.ndarray, width: int, height: int, max_jump_th: int):
    """
    Extract TD events with (t_ticks, x, y, pol) from a uint32 word stream.
    Applies:
      - drop until first TIME_HIGH
      - reject out-of-range x/y
      - stable sort by time
      - timestamp shift to start at t=0
    """
    # Decode fields
    types = (words >> 28) & 0xF
    X = (words >> 11) & 0x7FF
    Y = (words >> 0) & 0x7FF

    # Firmware mapping in your decoder: y = EVT20_X, x = EVT20_Y //flipping back
    x = X.astype(np.int32)
    y = Y.astype(np.int32)

    # Reconstruct timestamps (ticks)
    T_all = reconstruct_evt2_time_ticks(words, max_jump_th=max_jump_th)

    # TD + in bounds + has valid TIME_HIGH already
    is_td = (types == TD_LOW) | (types == TD_HIGH)
    inb = (x >= 0) & (x < width) & (y >= 0) & (y < height)
    keep = is_td & inb & (T_all >= 0)

    T = T_all[keep].astype(np.int64)
    x = x[keep].astype(np.int16)
    y = y[keep].astype(np.int16)
    pol = (types[keep] == TD_HIGH).astype(np.uint8)

    if T.size == 0:
        return T, x, y, pol

    # Stable time sort
    order = np.argsort(T, kind="stable")
    T = T[order]
    x = x[order]
    y = y[order]
    pol = pol[order]

    # Timestamp shift: start at 0
    T = T - T[0]

    return T, x, y, pol


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="aligned.bin (uint32 EVT2 words)")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=320)

    ap.add_argument("--frame-ms", type=float, default=30.0,
                    help="accumulation window per displayed frame (ms)")

    ap.add_argument("--tick-us", type=float, default=1.0,
                    help="microseconds per timestamp tick (set this once confirmed)")

    ap.add_argument("--gain", type=float, default=12.0,
                    help="brightness gain per event count per frame")
    ap.add_argument("--decay", type=float, default=0.85,
                    help="frame persistence decay (0..1). Lower = faster fade")

    ap.add_argument("--start-s", type=float, default=0.0,
                    help="skip initial seconds of the recording (after timestamp shift)")
    ap.add_argument("--max-s", type=float, default=0.0,
                    help="limit playback duration in seconds (0 = all)")

    ap.add_argument("--max-jump-th", type=int, default=200_000,
                    help=("max allowed forward jump in TIME_HIGH units before rejecting as corrupted. "
                          "TIME_HIGH units correspond to 64 ticks each."))

    ap.add_argument("--keep-open", action="store_true",
                    help="keep the window open after playback finishes")

    args = ap.parse_args()

    words = np.fromfile(args.path, dtype=np.uint32)
    print(f"Loaded {words.size} words")

    T_ticks, x, y, pol = parse_evt2_td(words, args.width, args.height, max_jump_th=args.max_jump_th)
    if T_ticks.size == 0:
        raise SystemExit("No TD events parsed (after TIME_HIGH sync + bounds filtering).")

    tick_s = args.tick_us * 1e-6
    t_s = T_ticks.astype(np.float64) * tick_s

    # Apply start/limit in seconds (post-shift)
    start = max(0.0, args.start_s)
    end = (t_s[-1] if args.max_s <= 0 else start + args.max_s)

    i0 = np.searchsorted(t_s, start, side="left")
    i1 = np.searchsorted(t_s, end, side="left")
    T_ticks = T_ticks[i0:i1]
    x = x[i0:i1]
    y = y[i0:i1]
    pol = pol[i0:i1]
    t_s = t_s[i0:i1]

    if t_s.size == 0:
        raise SystemExit("No TD events in the requested time range (check --start-s/--max-s).")

    duration = float(t_s[-1] - t_s[0])
    print(f"TD events in playback: {t_s.size}")
    print(f"Playback duration: {duration:.3f} s")
    print(f"tick_us = {args.tick_us} -> tick_s = {tick_s:e}")

    frame_s = args.frame_ms * 1e-3
    frame_edges = np.arange(t_s[0], t_s[-1] + frame_s, frame_s)
    idx_edges = np.searchsorted(t_s, frame_edges)
    n_frames = len(idx_edges) - 1
    print(f"Frames: {n_frames}  (frame_ms={args.frame_ms})")

    # Display buffers
    img = np.zeros((args.height, args.width, 3), dtype=np.float32)

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title("EVT2 (real-time). TD_LOW=green, TD_HIGH=blue")
    ax.set_axis_off()
    im = ax.imshow(img, interpolation="nearest", vmin=0.0, vmax=1.0)
    fig.canvas.draw()
    plt.show(block=False)

    wall_start = time.perf_counter()
    stream_start = frame_edges[0]

    for fi in range(n_frames):
        a = idx_edges[fi]
        b = idx_edges[fi + 1]

        # Real-time pacing
        target_wall = wall_start + (frame_edges[fi] - stream_start)
        now = time.perf_counter()
        if target_wall > now:
            time.sleep(target_wall - now)

        # Decay
        img *= args.decay

        if a < b:
            xs = x[a:b].astype(np.int32)
            ys = y[a:b].astype(np.int32)
            ps = pol[a:b]

            flat = ys * args.width + xs

            on = flat[ps == 1]
            off = flat[ps == 0]

            if on.size:
                c_on = np.bincount(on, minlength=args.width * args.height).reshape(args.height, args.width)
                img[..., 2] += args.gain * c_on  # blue
            if off.size:
                c_off = np.bincount(off, minlength=args.width * args.height).reshape(args.height, args.width)
                img[..., 1] += args.gain * c_off  # green

        im.set_data(np.clip(img, 0.0, 1.0))
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    print("Done.")

    if args.keep_open:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
