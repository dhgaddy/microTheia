#!/usr/bin/env python3
"""
visualize aedat files
added "realtime" setting for smooth playback
Display pacing:
  --fps sets a constant display FPS (wall-clock), decoupled from integration window size.
  --realtime advances through event timestamps at a steady rate; use --speed to scale playback.

  --timesurface uses rolling last-timestamp per pixel with exponential decay (tau), adds slight "trailing" that looks more natural

color:
  --activity-color rg  : G=ON, R=OFF with intensity
  --activity-color hsv : brightness=activity, hue=polarity balance, blue - green
                         
usage:
  python3 aedat_visualizer.py file.aedat --fps 60 --realtime --dt-ms 30 --scale 6

Keys:
  q      quit
  space  pause/resume
  c      clear

  yoinked liam's aedat decoder
"""

import argparse
import struct
import time
import numpy as np
import cv2

HEADER_FMT = "<HHIIIIII"
EVENT_FMT  = "<II"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
EVENT_SIZE  = struct.calcsize(EVENT_FMT)

H, W = 128, 128


def skip_text_header_until_end(fin):
    while True:
        line = fin.readline()
        if not line:
            raise RuntimeError("Reached EOF while searching for #!END-HEADER")
        if line.startswith(b"#!END-HEADER"):
            return


def decode_event(data):
    x = (data >> 17) & 0x1FFF
    y = (data >> 2)  & 0x1FFF
    p = (data >> 1)  & 0x1
    return x, y, p


def _robust_scale(a, pct=99.5):
    if np.any(a):
        s = np.percentile(a, pct)
        return max(float(s), 1e-6)
    return 1.0


def render_bgr_rg(acc_on, acc_off):
    """Green=ON, Red=OFF. Intensity encodes magnitude."""
    on_scale  = _robust_scale(acc_on,  99.5)
    off_scale = _robust_scale(acc_off, 99.5)

    g = np.clip(acc_on  / on_scale,  0.0, 1.0)
    r = np.clip(acc_off / off_scale, 0.0, 1.0)
    b = np.zeros_like(r)

    img = np.stack([b, g, r], axis=-1)  # BGR
    return (img * 255.0).astype(np.uint8)


def render_bgr_hsv_activity(acc_on, acc_off):
    """
    Activity-encoded view:
      - Brightness (V) ~ total activity (on+off)
      - Hue indicates polarity balance

    Requested mapping:
      OFF-heavy -> green
      ON-heavy  -> blue
    """
    tot = acc_on + acc_off
    tot_scale = _robust_scale(tot, 99.5)
    v = np.clip(tot / tot_scale, 0.0, 1.0)

    eps = 1e-6
    bal = (acc_on - acc_off) / (tot + eps)  # [-1..+1], +1 ON-heavy, -1 OFF-heavy
    bal = np.clip(bal, -1.0, 1.0)

    # OpenCV HSV: H in [0..179] corresponds to [0..360) degrees.
    # Green ~ 60deg => H=30, Blue ~ 120deg => H=60
    # OFF-heavy (bal=-1) => green (H=60), ON-heavy (bal=+1) => blue (H=120)
    h = 60.0 + ((bal + 1.0) * 0.5) * 60.0   # [-1..+1] -> [60..120]

    s = np.ones_like(v)

    hsv_u8 = np.empty((H, W, 3), dtype=np.uint8)
    hsv_u8[..., 0] = np.clip(h, 0, 179).astype(np.uint8)
    hsv_u8[..., 1] = np.clip(s * 255.0, 0, 255).astype(np.uint8)
    hsv_u8[..., 2] = np.clip(v * 255.0, 0, 255).astype(np.uint8)

    return cv2.cvtColor(hsv_u8, cv2.COLOR_HSV2BGR)


def timesurface_from_last(last_ts, t_ref, tau_us, cutoff_us=None):
    """
    last_ts: int64 array (us), -1 means never
    t_ref: current time reference (us)
    tau_us: decay constant
    cutoff_us: optional hard cutoff
    """
    age = (t_ref - last_ts).astype(np.float32)
    valid = (last_ts >= 0) & (age >= 0)

    out = np.zeros_like(age, dtype=np.float32)
    if np.any(valid):
        out[valid] = np.exp(-age[valid] / float(tau_us))
        if cutoff_us is not None:
            out[valid & (age > cutoff_us)] = 0.0
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("aedat", help="Path to .aedat ")
    ap.add_argument("--dt-ms", type=float, default=30.0, help="integration window in ms (default 30)")
    ap.add_argument("--fps", type=float, default=60.0, help="display FPS (default 60)")
    ap.add_argument("--realtime", action="store_true",
                    help="advance through event timestamps at a steady rate")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Playback speed when --realtime , not super helpful")
    ap.add_argument("--scale", type=int, default=6, help="display scale factor")
    ap.add_argument("--gain", type=float, default=1.0, help="Per-event increment")
    ap.add_argument("--ts-min", type=int, default=None, help="optional timestamp min filter (inclusive)")
    ap.add_argument("--ts-max", type=int, default=None, help="optional timestamp max filter (inclusive)")

    # Count accumulation mode
    ap.add_argument("--accumulate", action="store_true",
                    help="maybe useful for really high speed events?.")
    ap.add_argument("--decay", type=float, default=0.0,
                    help="Only used with --accumulate. Per-window decay in [0..1].")

    # Time surface mode
    ap.add_argument("--timesurface", action="store_true",
                    help="Rolling time surface mode (last timestamp per pixel + exp decay).")
    ap.add_argument("--tau-ms", type=float, default=15.0,
                    help="Time constant tau for time surface decay in ms ")
    ap.add_argument("--ts-cutoff-ms", type=float, default=None,
                    help="Optional cutoff (ms): havent used")

    # Color
    ap.add_argument("--activity-color", choices=["rg", "hsv"], default="rg",
                    help="rg: green=ON/red=OFF. hsv: brightness=activity, hue=polarity balance (off=green, on=blue).")

    args = ap.parse_args()

    dt_us = max(1, int(args.dt_ms * 1000.0))
    fps = max(1e-3, float(args.fps))
    frame_period = 1.0 / fps

    tau_us = max(1, int(args.tau_ms * 1000.0))
    cutoff_us = None if args.ts_cutoff_ms is None else max(0, int(args.ts_cutoff_ms * 1000.0))

    scale = max(1, args.scale)
    decay = float(np.clip(args.decay, 0.0, 1.0))
    speed = max(1e-6, float(args.speed))

    # Count accumulators
    acc_on  = np.zeros((H, W), dtype=np.float32)
    acc_off = np.zeros((H, W), dtype=np.float32)

    # Time surface state
    last_on  = np.full((H, W), -1, dtype=np.int64)
    last_off = np.full((H, W), -1, dtype=np.int64)

    win = "AEDAT DVS128 128x128 (q quit | space pause | c clear)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def accept_ts(ts):
        if args.ts_min is not None and ts < args.ts_min:
            return False
        if args.ts_max is not None and ts > args.ts_max:
            return False
        return True

    def render_frame(t_ref):
        if args.timesurface:
            surf_on  = timesurface_from_last(last_on,  t_ref, tau_us, cutoff_us=cutoff_us)
            surf_off = timesurface_from_last(last_off, t_ref, tau_us, cutoff_us=cutoff_us)
            if args.activity_color == "hsv":
                return render_bgr_hsv_activity(surf_on, surf_off)
            return render_bgr_rg(surf_on, surf_off)

        if args.activity_color == "hsv":
            return render_bgr_hsv_activity(acc_on, acc_off)
        return render_bgr_rg(acc_on, acc_off)

    def clear_all():
        acc_on.fill(0.0)
        acc_off.fill(0.0)
        last_on.fill(-1)
        last_off.fill(-1)

    # Integration window tracking (event-time)
    t_window_start = None

    def advance_integration_windows(ts):
        """
        Advance the integration window(s) up to timestamp ts.
        clearing/decay happens here for count-based modes
        time-surface is continuous; no per-window
        """
        nonlocal t_window_start

        if t_window_start is None:
            t_window_start = ts
            return

        if args.timesurface:
            if ts >= t_window_start + dt_us:
                t_window_start = ts
            return

        if ts < t_window_start + dt_us:
            return

        n = int((ts - t_window_start) // dt_us)
        t_window_start += n * dt_us

        if args.accumulate:
            if decay > 0.0:
                keep = (1.0 - decay) ** n
                acc_on[:]  *= keep
                acc_off[:] *= keep
        else:
            acc_on.fill(0.0)
            acc_off.fill(0.0)

    with open(args.aedat, "rb") as fin:
        skip_text_header_until_end(fin)

        # Read first block header
        hdr = fin.read(HEADER_SIZE)
        if len(hdr) < HEADER_SIZE:
            raise RuntimeError("No blocks found after header.")

        (eventType, eventSource, eventSize, tsOffset, tsOverflow,
         eventCapacity, eventNumber, eventValid) = struct.unpack(HEADER_FMT, hdr)

        # ---- playback state ----
        pending = None              # (data, ts) not yet consumed
        latest_ts_seen = None

        next_display = time.perf_counter()

        paused = False
        t0_wall = None
        t0_evt  = None

        def read_next_event():
            nonlocal eventNumber
            while True:
                if eventNumber > 0:
                    evb = fin.read(EVENT_SIZE)
                    if len(evb) < EVENT_SIZE:
                        return None
                    eventNumber -= 1
                    return struct.unpack(EVENT_FMT, evb)

                # Need next block
                hdr2 = fin.read(HEADER_SIZE)
                if len(hdr2) < HEADER_SIZE:
                    return None

                (et, es, esz, tso, tsof, cap, num, valid) = struct.unpack(HEADER_FMT, hdr2)
                eventNumber = num

        while True:
            now = time.perf_counter()

            # --- UI handling ---
            k = cv2.waitKey(1 if not paused else 30) & 0xFF
            if k == ord("q"):
                return
            if k == ord(" "):
                paused = not paused
                if not paused:
                    next_display = time.perf_counter()
            if k == ord("c"):
                clear_all()

            if paused:
                if latest_ts_seen is not None:
                    img = render_frame(latest_ts_seen)
                    disp = cv2.resize(img, (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow(win, disp)
                continue

            # Constant FPS cadence
            if now < next_display:
                time.sleep(min(0.001, next_display - now))
                continue

            # ---- display tick ----

            # Determine target event timestamp for this frame
            if args.realtime:
                if t0_wall is None:
                    # Prime with first event
                    ev = read_next_event()
                    if ev is None:
                        return
                    pending = ev
                    _, ts0 = ev
                    t0_wall = time.perf_counter()
                    t0_evt = ts0
                    latest_ts_seen = ts0

                wall_elapsed = time.perf_counter() - t0_wall
                target_ts = t0_evt + int(wall_elapsed * speed * 1e6)
            else:
                # If not realtime, we won't force a steady ts rate; render latest seen
                target_ts = latest_ts_seen if latest_ts_seen is not None else 0

            # Consume events up to target_ts (only meaningful in realtime mode)
            if args.realtime:
                while True:
                    if pending is None:
                        ev = read_next_event()
                        if ev is None:
                            break
                        pending = ev

                    data, ts = pending
                    if ts > target_ts:
                        break

                    pending = None

                    if not accept_ts(ts):
                        continue

                    latest_ts_seen = ts
                    advance_integration_windows(ts)

                    x, y, p = decode_event(data)
                    if 0 <= x < W and 0 <= y < H:
                        if args.timesurface:
                            if p:
                                last_on[y, x] = ts
                            else:
                                last_off[y, x] = ts
                        else:
                            if p:
                                acc_on[y, x] += args.gain
                            else:
                                acc_off[y, x] += args.gain

            # Render using target timestamp in realtime mode 
            t_ref = target_ts if args.realtime else latest_ts_seen
            if t_ref is None:
                t_ref = 0

            img = render_frame(t_ref)
            disp = cv2.resize(img, (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)
            cv2.imshow(win, disp)

            # Schedule next frame 
            next_display += frame_period

            # prevent runaway catch-up if render is too slow?
            if time.perf_counter() - next_display > 0.25:
                next_display = time.perf_counter()


if __name__ == "__main__":
    main()
