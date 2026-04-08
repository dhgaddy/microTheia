#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import csv

TYPE_CD_OFF   = 0x0
TYPE_CD_ON    = 0x1
TYPE_TIMEHIGH = 0x8

SENSOR_W = 320
SENSOR_H = 320
CLIP_US  = 1_000_000


def load_words(path):
    data = path.read_bytes()
    m = (len(data)//4)*4
    return np.frombuffer(data[:m], dtype="<u4")


def decode_events(words):
    cur_th = None
    first_t = None

    t_list, x_list, y_list = [], [], []

    for w in words:
        w = int(w)
        typ = (w >> 28) & 0xF

        if typ == TYPE_TIMEHIGH:
            cur_th = w & 0x0FFFFFFF
            continue

        if typ in (TYPE_CD_OFF, TYPE_CD_ON) and cur_th is not None:
            tl = (w >> 22) & 0x3F
            ts = (cur_th << 6) | tl

            if first_t is None:
                first_t = ts

            t_rel = ts - first_t
            if t_rel < 0 or t_rel >= CLIP_US:
                continue

            x = (w >> 11) & 0x7FF
            y = w & 0x7FF

            # If your stream ever includes out-of-range coords, clamp by dropping
            if x >= SENSOR_W or y >= SENSOR_H:
                continue

            t_list.append(t_rel)
            x_list.append(x)
            y_list.append(y)

    return np.array(t_list), np.array(x_list), np.array(y_list)


def l1_norm(v):
    s = np.sum(v)
    return v if s == 0 else v / s


def l2_norm(v):
    n = np.sqrt(np.sum(v*v))
    return v if n == 0 else v / n


def process_dataset(root, tbins, sbins, per_clip_l1, final_l2, mirror_x):
    vec_len = tbins * sbins * sbins

    def voxelize(t, y, x):
        V = np.zeros((tbins, sbins, sbins), dtype=np.float32)

        tb = np.minimum((t * tbins) // CLIP_US, tbins - 1)

        xb = np.minimum((x * sbins) // SENSOR_W, sbins - 1)
        yb = np.minimum((y * sbins) // SENSOR_H, sbins - 1)

        if mirror_x:
            xb = (sbins - 1) - xb

        # increment per event (merged polarity already)
        for i in range(len(tb)):
            V[tb[i], yb[i], xb[i]] += 1.0

        return V.reshape(-1)

    clip_rows = []
    class_acc = {}
    class_count = {}

    for class_dir in sorted(root.iterdir()):
        if not class_dir.is_dir():
            continue

        cname = class_dir.name
        class_acc[cname] = np.zeros(vec_len, dtype=np.float64)
        class_count[cname] = 0

        for f in sorted(class_dir.glob("*.bin")):
            words = load_words(f)
            t, x, y = decode_events(words)
            v = voxelize(t, y, x)

            if per_clip_l1:
                v = l1_norm(v)

            clip_rows.append([f.name, cname, *v])

            class_acc[cname] += v
            class_count[cname] += 1

    templates = []
    for cname in class_acc:
        n = class_count[cname]
        if n == 0:
            continue

        w = class_acc[cname] / n
        if final_l2:
            w = l2_norm(w)

        templates.append([cname, *w])

    return clip_rows, templates, vec_len


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("dataset")
    ap.add_argument("--tbins", type=int, default=5, help="Temporal bins (default 5)")
    ap.add_argument("--sbins", type=int, default=16, help="Spatial bins per axis (default 16 -> 16x16)")

    ap.add_argument("--mirror-x", action="store_true",
                    help="Mirror features horizontally: xbin := (sbins-1)-xbin (left<->right)")

    ap.add_argument("--raw-average", action="store_true",
                    help="Disable ALL normalization (pure average of raw voxel counts)")

    ap.add_argument("--no-final-l2", action="store_true",
                    help="Disable final L2 normalization (still does per-clip L1 unless --raw-average)")

    args = ap.parse_args()

    if args.raw_average:
        per_clip_l1 = False
        final_l2 = False
    else:
        per_clip_l1 = True
        final_l2 = not args.no_final_l2

    clip_rows, templates, vec_len = process_dataset(
        Path(args.dataset),
        args.tbins,
        args.sbins,
        per_clip_l1,
        final_l2,
        args.mirror_x
    )

    vec_header = ["file", "class"] + [f"v{i}" for i in range(vec_len)]
    tmpl_header = ["class"] + [f"w{i}" for i in range(vec_len)]

    write_csv("clip_vectors.csv", vec_header, clip_rows)
    write_csv("class_templates.csv", tmpl_header, templates)

    print(f"Done. vec_len={vec_len} mirror_x={args.mirror_x} per_clip_l1={per_clip_l1} final_l2={final_l2}")


if __name__ == "__main__":
    main()
