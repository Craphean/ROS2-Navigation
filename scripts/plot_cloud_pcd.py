#!/usr/bin/env python3
"""Render an ASCII PCD archive to a headless PNG preview.

Produces three orthogonal views (XY top, XZ side, YZ side) so the 3D
structure can be inspected without RViz, OpenGL or a discrete GPU.

Example:
  python3 scripts/plot_cloud_pcd.py \
    --input maps/tb3_3d_map.pcd --output maps/tb3_3d_map.png
"""

import argparse
import json
import sys

import matplotlib

matplotlib.use("Agg")  # write PNGs without needing a display server

import matplotlib.pyplot as plt
import numpy as np


def read_ascii_pcd(path: str) -> np.ndarray:
    header = {}
    with open(path, "r", encoding="utf-8") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("PCD ended before DATA line")
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            key, _, value = stripped.partition(" ")
            header[key.upper()] = value
            if key.upper() == "DATA":
                break
        if header.get("DATA", "").lower() != "ascii":
            raise ValueError("Only ASCII PCD is supported")
        data = np.loadtxt(handle, dtype=np.float64)
    if data.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(data, dtype=np.float64).reshape(-1, 3)


def add_panel(ax, x, y, z, xlabel, ylabel, title, equal=True):
    scatter = ax.scatter(
        x,
        y,
        c=z,
        s=0.5,
        cmap="viridis",
        linewidths=0,
        rasterized=True,
    )
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, linewidth=0.3, alpha=0.4)
    if equal:
        ax.set_aspect("equal", adjustable="datalim")
    return scatter


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="ASCII PCD path")
    parser.add_argument("--output", required=True, help="Output PNG path")
    parser.add_argument("--dpi", type=int, default=160)
    args = parser.parse_args(argv)

    try:
        points = read_ascii_pcd(args.input)
    except Exception as exc:  # noqa: BLE001 - report the parse error
        print(f"error: {exc}", file=sys.stderr)
        return 1
    if points.size == 0:
        print("error: PCD contains no points", file=sys.stderr)
        return 1

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    scatter = add_panel(
        axes[0], x, y, z, "x (m)", "y (m)", "Top view (XY, color = z)"
    )
    add_panel(axes[1], x, z, z, "x (m)", "z (m)", "Side view (XZ)")
    add_panel(axes[2], y, z, z, "y (m)", "z (m)", "Side view (YZ)")

    colorbar = fig.colorbar(scatter, ax=axes, shrink=0.75, pad=0.02)
    colorbar.set_label("height z (m)")
    fig.suptitle(
        f"{len(points)} occupied voxel centers  |  "
        f"x [{x.min():.2f}, {x.max():.2f}]  "
        f"y [{y.min():.2f}, {y.max():.2f}]  "
        f"z [{z.min():.2f}, {z.max():.2f}]",
        fontsize=11,
    )
    fig.savefig(args.output, dpi=args.dpi, bbox_inches="tight")
    plt.close(fig)

    print(
        json.dumps(
            {
                "input": args.input,
                "output": args.output,
                "points": int(len(points)),
                "bbox": {
                    "min": [float(v) for v in points.min(axis=0)],
                    "max": [float(v) for v in points.max(axis=0)],
                },
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
