#!/usr/bin/env python3
"""
Retrain quantized gesture weights from EVT2 .bin recordings.

This script mirrors the cocotb timestamp-forced replay path:
- EVT2 timestamp decode (time-high + ts_lsb)
- 8-bin circular voxel windowing at WINDOW_MS/READOUT_BINS
- optional coordinate transforms (swap/flip) from config
- oldest->newest feature packing (bin-major, then y, then x)

It fits a non-negative linear multiclass model and exports:
- weights/2048weights_q8_c0.mem ... c3.mem
- weights/thresholds.mem
"""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8


@dataclass
class Config:
    window_ms: int
    grid_size: int
    readout_bins: int
    counter_bits: int
    sensor_width: int
    sensor_height: int
    map_swap_xy: int
    map_flip_x: int
    map_flip_y: int


def load_kv_config(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for raw in path.read_text(encoding="ascii").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip().strip('"')
    return out


def read_config(path: Path) -> Config:
    kv = load_kv_config(path)
    sensor_w = int(kv.get("SENSOR_WIDTH", "320").replace("_", ""))
    sensor_h = int(kv.get("SENSOR_HEIGHT", str(sensor_w)).replace("_", ""))
    return Config(
        window_ms=int(kv.get("WINDOW_MS", "1000").replace("_", "")),
        grid_size=int(kv.get("GRID_SIZE", "16").replace("_", "")),
        readout_bins=int(kv.get("READOUT_BINS", "8").replace("_", "")),
        counter_bits=int(kv.get("COUNTER_BITS", "16").replace("_", "")),
        sensor_width=sensor_w,
        sensor_height=sensor_h,
        map_swap_xy=int(kv.get("MAP_SWAP_XY", "0").replace("_", "")),
        map_flip_x=int(kv.get("MAP_FLIP_X", "0").replace("_", "")),
        map_flip_y=int(kv.get("MAP_FLIP_Y", "0").replace("_", "")),
    )


def map_xy(cfg: Config, x_raw: int, y_raw: int) -> tuple[int, int]:
    x = min(x_raw, cfg.sensor_width - 1)
    y = min(y_raw, cfg.sensor_height - 1)
    if cfg.map_swap_xy:
        x, y = y, x
    x = min(x, cfg.sensor_width - 1)
    y = min(y, cfg.sensor_height - 1)
    if cfg.map_flip_x:
        x = (cfg.sensor_width - 1) - x
    if cfg.map_flip_y:
        y = (cfg.sensor_height - 1) - y
    x_bin_div = max(1, cfg.sensor_width // cfg.grid_size)
    y_bin_div = max(1, cfg.sensor_height // cfg.grid_size)
    gx = min(x // x_bin_div, cfg.grid_size - 1)
    gy = min(y // y_bin_div, cfg.grid_size - 1)
    return gx, gy


def read_evt2_words(path: Path) -> Iterable[int]:
    data = path.read_bytes()
    n_words = len(data) // 4
    return struct.unpack_from(f"<{n_words}I", data, 0)


def extract_windows(cfg: Config, bin_path: Path) -> np.ndarray:
    rb = cfg.readout_bins
    gs = cfg.grid_size
    counter_max = (1 << cfg.counter_bits) - 1
    bin_us = (cfg.window_ms // rb) * 1000

    bins = np.zeros((rb, gs, gs), dtype=np.uint16)
    wr = 0
    completed = 0
    windows: list[np.ndarray] = []
    time_high = 0
    next_bin_boundary_us: int | None = None

    def rollover() -> None:
        nonlocal wr, completed
        wr = (wr + 1) % rb
        bins[wr].fill(0)
        completed = min(completed + 1, rb)
        if completed >= rb:
            oldest = (wr + 1) % rb
            feat = np.empty((rb, gs, gs), dtype=np.uint16)
            for off in range(rb):
                feat[off] = bins[(oldest + off) % rb]
            windows.append(feat.reshape(-1).astype(np.float64))

    for word in read_evt2_words(bin_path):
        pkt = (word >> 28) & 0xF
        if pkt == EVT_TIME_HIGH:
            time_high = word & 0x0FFFFFFF
            continue
        if pkt not in (EVT_CD_OFF, EVT_CD_ON):
            continue

        ts_us = (time_high << 6) | ((word >> 22) & 0x3F)
        if next_bin_boundary_us is None:
            next_bin_boundary_us = (ts_us // bin_us + 1) * bin_us
        while ts_us >= next_bin_boundary_us:
            rollover()
            next_bin_boundary_us += bin_us

        x_raw = (word >> 11) & 0x7FF
        y_raw = word & 0x7FF
        gx, gy = map_xy(cfg, x_raw, y_raw)
        if bins[wr, gy, gx] < counter_max:
            bins[wr, gy, gx] += 1

    for _ in range(rb):
        rollover()

    return np.stack(windows, axis=0)


def train_nonnegative_weights(
    x_raw: np.ndarray,
    y: np.ndarray,
    num_classes: int = 4,
    seeds: int = 24,
    iters: int = 2500,
) -> np.ndarray:
    """Fit a non-negative linear model and return uint8 [C, D] weights."""
    n, d = x_raw.shape
    y_1h = np.eye(num_classes, dtype=np.float64)[y]
    feat_scale = np.maximum(x_raw.max(axis=0), 1.0)
    x_norm = x_raw / feat_scale
    best: tuple[float, np.ndarray] | None = None

    for seed in range(seeds):
        rng = np.random.default_rng(seed)
        w = np.maximum(0.0, rng.normal(0.0, 0.02, size=(d, num_classes)))
        lr = 0.8
        reg = 1e-4

        for _ in range(iters):
            logits = x_norm @ w
            logits -= logits.max(axis=1, keepdims=True)
            prob = np.exp(logits)
            prob /= prob.sum(axis=1, keepdims=True)
            grad = (x_norm.T @ (prob - y_1h)) / n + reg * w
            w -= lr * grad
            np.maximum(w, 0.0, out=w)

        # Convert to raw-feature weights by undoing feature normalization.
        w_raw = (w.T / feat_scale).astype(np.float64)  # [C, D]
        max_val = float(w_raw.max())
        if max_val <= 0:
            continue

        # Quantization sweep: scale to best training accuracy after uint8 quantization.
        for alpha in np.linspace(30, 255, 96):
            q = np.clip(np.round(w_raw / max_val * alpha), 0, 255).astype(np.uint8)
            pred = np.argmax(x_raw @ q.T, axis=1)
            acc = float((pred == y).mean())
            if best is None or acc > best[0]:
                best = (acc, q.copy())

    if best is None:
        raise RuntimeError("Failed to train/quantize non-negative weights.")
    return best[1]


def pick_best_threshold(pos_scores: np.ndarray, neg_scores: np.ndarray) -> int:
    """
    Select threshold for rule score > threshold maximizing balanced accuracy.
    """
    if len(pos_scores) == 0:
        return 0
    candidates = sorted(
        set([0] + [int(x) for x in pos_scores.tolist()] + [int(x) for x in neg_scores.tolist()])
    )
    best_t = 0
    best_bal = -1.0
    for t in candidates:
        tpr = float(np.mean(pos_scores > t)) if len(pos_scores) else 0.0
        tnr = float(np.mean(neg_scores <= t)) if len(neg_scores) else 1.0
        bal = 0.5 * (tpr + tnr)
        if bal > best_bal:
            best_bal = bal
            best_t = t
    return int(best_t)


def write_weight_mem(path: Path, values: np.ndarray) -> None:
    lines = [f"{int(v):02X}" for v in values.tolist()]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_thresholds(path: Path, class_thresh: list[int], diff_thresh: list[int]) -> None:
    vals = list(class_thresh) + list(diff_thresh)
    lines = [f"{int(v) & ((1 << 36) - 1):09X}" for v in vals]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument(
        "--gesture-files",
        nargs=4,
        default=[
            "EVT2_gesture_set/wave_down_sun_test1.bin",
            "EVT2_gesture_set/wave_left_sun_test1.bin",
            "EVT2_gesture_set/wave_right_sun_test1.bin",
            "EVT2_gesture_set/wave_up_sun_test1.bin",
        ],
        help="Exactly 4 files in class order: down left right up",
    )
    args = ap.parse_args()

    repo_root: Path = args.repo_root.resolve()
    cfg = read_config(repo_root / "configs" / "voxel_default.txt")

    x_parts: list[np.ndarray] = []
    y_parts: list[np.ndarray] = []
    per_file: list[tuple[str, np.ndarray, np.ndarray]] = []
    for cls, rel in enumerate(args.gesture_files):
        p = (repo_root / rel).resolve()
        x = extract_windows(cfg, p)
        y = np.full(x.shape[0], cls, dtype=np.int64)
        x_parts.append(x)
        y_parts.append(y)
        per_file.append((p.name, x, y))

    x_all = np.concatenate(x_parts, axis=0)
    y_all = np.concatenate(y_parts, axis=0)
    w_q = train_nonnegative_weights(x_all, y_all, num_classes=4)

    # Class thresholds from one-vs-rest class-score distributions.
    scores = x_all @ w_q.T
    class_thresh: list[int] = []
    for c in range(4):
        pos = scores[y_all == c, c]
        neg = scores[y_all != c, c]
        class_thresh.append(pick_best_threshold(pos, neg))

    # Diff threshold only drives gesture_confidence (not gesture_valid).
    # Keep permissive unless you need confidence filtering.
    diff_thresh = [0, 0, 0, 0]

    weights_dir = repo_root / "weights"
    for c in range(4):
        write_weight_mem(weights_dir / f"2048weights_q8_c{c}.mem", w_q[c])
    write_thresholds(weights_dir / "thresholds.mem", class_thresh, diff_thresh)

    # Compact report for sanity.
    pred = np.argmax(scores, axis=1)
    print(f"Training windows: {x_all.shape[0]}  features/window: {x_all.shape[1]}")
    print(f"Window accuracy (argmax): {(pred == y_all).mean():.3f}")
    for name, x, y in per_file:
        s = x @ w_q.T
        p = np.argmax(s, axis=1)
        vals, cnt = np.unique(p, return_counts=True)
        dom = int(vals[np.argmax(cnt)])
        ratio = float((p == y).mean())
        hist = {int(k): int(v) for k, v in zip(vals.tolist(), cnt.tolist())}
        print(f"{name}: dominant={dom} expected={int(y[0])} ratio={ratio:.3f} hist={hist}")
    print(f"class_thresholds={class_thresh}")
    print("diff_thresholds=[0, 0, 0, 0]")


if __name__ == "__main__":
    main()
