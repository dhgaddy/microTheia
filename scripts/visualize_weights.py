#!/usr/bin/env python3
"""
Visualize quantized voxel gesture weights as heatmaps.

Usage:
  python3 scripts/visualize_weights.py
  python3 scripts/visualize_weights.py --show
  python3 scripts/visualize_weights.py --output-dir weight_viz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_kv_config(path: Path) -> dict[str, str]:
    cfg: dict[str, str] = {}
    for raw in path.read_text(encoding="ascii").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        cfg[k.strip()] = v.strip().strip('"')
    return cfg


def parse_int(cfg: dict[str, str], key: str, default: int) -> int:
    return int(cfg.get(key, str(default)).replace("_", ""))


def load_mem(path: Path, expected_len: int) -> np.ndarray:
    vals = []
    for raw in path.read_text(encoding="ascii").splitlines():
        line = raw.strip()
        if not line or line.startswith("//"):
            continue
        vals.append(int(line, 16))
    if len(vals) < expected_len:
        vals.extend([0] * (expected_len - len(vals)))
    return np.array(vals[:expected_len], dtype=np.float64)


def class_name(idx: int) -> str:
    return ["Down", "Left", "Right", "Up"][idx] if idx < 4 else f"Class{idx}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Repository root path",
    )
    ap.add_argument(
        "--config",
        type=Path,
        default=Path("configs/voxel_default.txt"),
        help="Config path relative to repo root",
    )
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=Path("weights/visualizations"),
        help="Output directory relative to repo root",
    )
    ap.add_argument(
        "--show",
        action="store_true",
        help="Show plots interactively in addition to saving",
    )
    args = ap.parse_args()

    repo_root = args.repo_root.resolve()
    cfg = load_kv_config((repo_root / args.config).resolve())
    grid = parse_int(cfg, "GRID_SIZE", 16)
    bins = parse_int(cfg, "READOUT_BINS", 8)
    num_classes = parse_int(cfg, "NUM_CLASSES", 4)
    feat_count = grid * grid * bins

    out_dir = (repo_root / args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    for c in range(num_classes):
        mem_path = repo_root / "weights" / f"{feat_count}weights_q8_c{c}.mem"
        if not mem_path.exists():
            # Fall back to standard naming used by this project.
            mem_path = repo_root / "weights" / f"2048weights_q8_c{c}.mem"
        w = load_mem(mem_path, feat_count).reshape(bins, grid, grid)

        rows = int(np.ceil(bins / 4))
        cols = min(4, bins)
        fig, axes = plt.subplots(rows, cols, figsize=(3.5 * cols, 3.2 * rows))
        axes_arr = np.atleast_1d(axes).reshape(rows, cols)
        vmax = max(1.0, float(w.max()))

        for bi in range(rows * cols):
            r = bi // cols
            k = bi % cols
            ax = axes_arr[r, k]
            if bi < bins:
                im = ax.imshow(w[bi], cmap="viridis", vmin=0.0, vmax=vmax)
                ax.set_title(f"Bin {bi} ({class_name(c)})")
            else:
                ax.axis("off")
                continue
            ax.set_xticks([])
            ax.set_yticks([])

        fig.colorbar(im, ax=axes_arr.ravel().tolist(), shrink=0.85, label="Weight value")
        fig.suptitle(f"Class {c}: {class_name(c)} weight maps", fontsize=12)
        fig.tight_layout()
        out_path = out_dir / f"class_{c}_{class_name(c).lower()}_bins.png"
        fig.savefig(out_path, dpi=160)
        print(f"Saved {out_path}")
        if args.show:
            plt.show()
        plt.close(fig)

        # Also save a summed spatial map for quick saliency inspection.
        summed = w.sum(axis=0)
        fig2, ax2 = plt.subplots(figsize=(5, 4.5))
        im2 = ax2.imshow(summed, cmap="magma")
        ax2.set_title(f"Class {c}: {class_name(c)} summed across bins")
        ax2.set_xticks([])
        ax2.set_yticks([])
        fig2.colorbar(im2, ax=ax2, shrink=0.85, label="Sum of weights")
        fig2.tight_layout()
        out_path2 = out_dir / f"class_{c}_{class_name(c).lower()}_sum.png"
        fig2.savefig(out_path2, dpi=160)
        print(f"Saved {out_path2}")
        if args.show:
            plt.show()
        plt.close(fig2)


if __name__ == "__main__":
    main()
