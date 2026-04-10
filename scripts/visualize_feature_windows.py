#!/usr/bin/env python3
"""
Visualize average input feature windows (event activity), not weights.

This makes motion direction easier to see than weight heatmaps.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from retrain_gesture_weights import collect_weight_set_files, extract_windows, read_config


def class_name(idx: int) -> str:
    return ["Down", "Left", "Right", "Up"][idx] if idx < 4 else f"Class{idx}"


def centroid_xy(frame: np.ndarray) -> tuple[float, float]:
    # frame shape: [grid_y, grid_x]
    total = float(frame.sum())
    if total <= 0:
        return (np.nan, np.nan)
    gy, gx = np.indices(frame.shape)
    x = float((gx * frame).sum() / total)
    y = float((gy * frame).sum() / total)
    return (x, y)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    ap.add_argument(
        "--weight-set-dir",
        type=Path,
        default=Path("EVT2_gesture_set/weight_set"),
        help="Directory containing wave_down/left/right/up .bin files",
    )
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=Path("weights/visualizations"),
    )
    ap.add_argument(
        "--show",
        action="store_true",
        help="Show figures interactively",
    )
    args = ap.parse_args()

    repo_root = args.repo_root.resolve()
    cfg = read_config(repo_root / "configs" / "voxel_default.txt")
    bins = cfg.readout_bins
    grid = cfg.grid_size

    class_files = collect_weight_set_files((repo_root / args.weight_set_dir).resolve())
    out_dir = (repo_root / args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    for cls, files in enumerate(class_files):
        windows = []
        for p in files:
            x = extract_windows(cfg, p)
            if x.size:
                windows.append(x)
        if not windows:
            print(f"No windows for class {cls}")
            continue

        all_windows = np.concatenate(windows, axis=0).reshape(-1, bins, grid, grid)
        avg = all_windows.mean(axis=0)  # [bins, grid, grid]

        # Figure 1: average bin activity maps
        rows = int(np.ceil(bins / 4))
        cols = min(4, bins)
        fig, axes = plt.subplots(rows, cols, figsize=(3.5 * cols, 3.2 * rows))
        axes_arr = np.atleast_1d(axes).reshape(rows, cols)
        vmax = max(1.0, float(avg.max()))

        for bi in range(rows * cols):
            r = bi // cols
            c = bi % cols
            ax = axes_arr[r, c]
            if bi < bins:
                im = ax.imshow(avg[bi], cmap="plasma", vmin=0.0, vmax=vmax)
                ax.set_title(f"Bin {bi}")
            else:
                ax.axis("off")
                continue
            ax.set_xticks([])
            ax.set_yticks([])

        fig.colorbar(im, ax=axes_arr.ravel().tolist(), shrink=0.85, label="Avg event count")
        fig.suptitle(f"{class_name(cls)}: average input activity per bin")
        fig.tight_layout()
        out_bins = out_dir / f"class_{cls}_{class_name(cls).lower()}_avg_input_bins.png"
        fig.savefig(out_bins, dpi=160)
        print(f"Saved {out_bins}")
        if args.show:
            plt.show()
        plt.close(fig)

        # Figure 2: centroid motion path across bins
        centroids = np.array([centroid_xy(avg[bi]) for bi in range(bins)], dtype=np.float64)
        fig2, ax2 = plt.subplots(figsize=(5, 5))
        ax2.set_xlim(-0.5, grid - 0.5)
        ax2.set_ylim(grid - 0.5, -0.5)
        ax2.set_title(f"{class_name(cls)}: centroid trajectory across bins")
        ax2.set_xlabel("x")
        ax2.set_ylabel("y")
        ax2.grid(alpha=0.3)

        valid = ~np.isnan(centroids[:, 0]) & ~np.isnan(centroids[:, 1])
        if np.any(valid):
            xs = centroids[valid, 0]
            ys = centroids[valid, 1]
            ax2.plot(xs, ys, "-o")
            for i, (x, y) in enumerate(zip(centroids[:, 0], centroids[:, 1])):
                if not np.isnan(x):
                    ax2.text(x + 0.1, y - 0.1, str(i), fontsize=8)

        fig2.tight_layout()
        out_path = out_dir / f"class_{cls}_{class_name(cls).lower()}_avg_input_centroid_path.png"
        fig2.savefig(out_path, dpi=160)
        print(f"Saved {out_path}")
        if args.show:
            plt.show()
        plt.close(fig2)


if __name__ == "__main__":
    main()
