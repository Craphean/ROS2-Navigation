#!/usr/bin/env python3
"""Generate maps/gt/<world>/ ground-truth 3D clouds and 2D maps for all 6 benchmark worlds."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hier_explore_bench.pcd_io import write_pcd
from hier_explore_bench.sample_gt_mesh import sample_world_ground_truth
from hier_explore_bringup.worlds import BENCHMARK_WORLDS, WORLD_SPECS, find_repo_root, resolve_world_file


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate GT archives for all 6 Lite-HMRE benchmark worlds.")
    ap.add_argument(
        "--worlds",
        nargs="+",
        default=list(BENCHMARK_WORLDS),
        choices=list(BENCHMARK_WORLDS),
        help="Subset of benchmark worlds to generate (default: all 6).",
    )
    ap.add_argument("--out-root", default="", help="Output directory root (default: <repo>/maps/gt).")
    args = ap.parse_args()

    repo_root = find_repo_root()
    out_root = Path(args.out_root) if args.out_root else (repo_root / "maps" / "gt")

    for wname in args.worlds:
        spec = WORLD_SPECS[wname]
        wfile = resolve_world_file(wname, repo_root)
        if not wfile.is_file():
            raise FileNotFoundError(f"World file missing for '{wname}': {wfile}")
        out_dir = out_root / wname
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"[generate_all_gt_maps] Sampling {wname} from {wfile} (seed={spec.gt_seed_xy}) ...")
        res = sample_world_ground_truth(
            wfile,
            sample_step=spec.gt_sample_step,
            voxel_size=spec.gt_voxel_size,
            min_z=0.08,
            max_z=2.0,
            resolution_2d=spec.gt_resolution_2d,
            seed_xy=spec.gt_seed_xy,
        )
        write_pcd(out_dir / "gt_surface.pcd", res["points_3d"], binary=True)
        np.savez_compressed(
            out_dir / "gt_2d.npz",
            grid=res["grid_2d"],
            origin_x=res["origin_x"],
            origin_y=res["origin_y"],
            resolution=res["resolution"],
            explorable_area_m2=res["explorable_area_m2"],
        )
        meta = {
            "world": str(wfile),
            "world_name": wname,
            "seed_xy": list(spec.gt_seed_xy),
            "points_3d": int(len(res["points_3d"])),
            "explorable_area_m2": round(float(res["explorable_area_m2"]), 3),
            "primitive_count": int(res["primitive_count"]),
            "voxel_size": float(spec.gt_voxel_size),
            "resolution_2d": float(spec.gt_resolution_2d),
            "min_z": 0.08,
            "max_z": 2.0,
        }
        (out_dir / "gt_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
