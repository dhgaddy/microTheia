#!/usr/bin/env python3
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt

def load_row(csv_path, label=None, file_name=None):
    """
    Reads a single row from:
      - class_templates.csv: columns: class, w0..w{D-1}
      - clip_vectors.csv: columns: file, class, v0..v{D-1}

    Specify either:
      --label (for class_templates.csv), OR
      --file  (for clip_vectors.csv)

    Returns: (name, vec)
    """
    with open(csv_path, newline="") as f:
        r = csv.reader(f)
        header = next(r)

        for row in r:
            if label is not None:
                if row[0] == label:
                    vec = np.array([float(x) for x in row[1:]], dtype=np.float32)
                    return label, vec

            if file_name is not None:
                if row[0] == file_name:
                    vec = np.array([float(x) for x in row[2:]], dtype=np.float32)
                    return file_name, vec

    raise SystemExit("ERROR: target row not found (check --label/--file and CSV path).")


def vec_to_voxels(vec, tbins, sbins):
    vec_len = tbins * sbins * sbins
    if vec.size != vec_len:
        raise SystemExit(f"ERROR: expected length {vec_len} (= {tbins}*{sbins}*{sbins}), got {vec.size}")
    # Flatten order assumed: for each tbin, row-major over (y,x)
    return vec.reshape((tbins, sbins, sbins))


def print_topk(vec, k, tbins, sbins):
    idx = np.argsort(vec)[::-1][:k]
    plane = sbins * sbins
    print(f"\nTop-{k} voxels (largest values):")
    for rank, i in enumerate(idx, start=1):
        t = i // plane
        rem = i % plane
        y = rem // sbins
        x = rem % sbins
        print(f"{rank:>2d}: idx={i:>5d}  (t={t}, y={y}, x={x})  val={vec[i]:.6f}")


def main():
    ap = argparse.ArgumentParser(description="Visualize EVT2 voxel templates/vectors as TBINS heatmaps (SBINSxSBINS).")
    ap.add_argument("csv", help="Path to class_templates.csv or clip_vectors.csv")

    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--label", help="Class label to visualize (for class_templates.csv)")
    g.add_argument("--file", help="File name to visualize (for clip_vectors.csv; must match first column exactly)")

    ap.add_argument("--tbins", type=int, default=5, help="Number of temporal bins (default: 5)")
    ap.add_argument("--sbins", type=int, default=16, help="Spatial bins per axis (default: 16 -> 16x16)")

    ap.add_argument("--topk", type=int, default=0, help="Print top-k voxel coordinates (t,y,x)")
    ap.add_argument("--title", default=None, help="Override plot title")
    ap.add_argument("--save", default=None, help="If set, save figure to this path (e.g., out.png)")
    ap.add_argument("--show", action="store_true", help="Show interactive window (default if --save not set)")

    args = ap.parse_args()

    name, vec = load_row(args.csv, label=args.label, file_name=args.file)
    vox = vec_to_voxels(vec, args.tbins, args.sbins)

    if args.topk > 0:
        print_topk(vec, args.topk, args.tbins, args.sbins)

    # Plot TBINS heatmaps
    fig, axes = plt.subplots(1, args.tbins, figsize=(3.2 * args.tbins, 3.2), constrained_layout=True)
    if args.tbins == 1:
        axes = [axes]

    vmin = float(np.min(vox))
    vmax = float(np.max(vox))

    for t in range(args.tbins):
        ax = axes[t]
        im = ax.imshow(vox[t], origin="upper", vmin=vmin, vmax=vmax)
        ax.set_title(f"tbin {t}")
        ax.set_xticks([])
        ax.set_yticks([])

    main_title = args.title if args.title is not None else name
    fig.suptitle(main_title)

    cbar = fig.colorbar(im, ax=axes, shrink=0.85, pad=0.02)
    cbar.set_label("value")

    if args.save:
        fig.savefig(args.save, dpi=200)
        print(f"\nSaved figure to: {args.save}")

    if args.show or not args.save:
        plt.show()


if __name__ == "__main__":
    main()
