#!/usr/bin/env python3
"""Automated multi-seed headless benchmark orchestrator for RTX 5090 / Docker."""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List

from .eval_metrics import aggregate_runs, evaluate_run_directory

REPO_ROOT = Path(__file__).resolve().parents[3]

# Feature-flag ablation profiles matching Table II of the paper plan.
ABLATION_PROFILES: Dict[str, Dict[str, str]] = {
    "legacy": {
        "EXPLORATION_MODE": "legacy",
        "COORDINATOR_ARGS": "",
    },
    "e1_2d": {
        "EXPLORATION_MODE": "auction",
        "COORDINATOR_ARGS": (
            "-p use_3d_gain:=false -p local_scheduling_enable:=false "
            "-p residual_enable:=false -p convergence_enable:=false"
        ),
    },
    "e2_3dgain": {
        "EXPLORATION_MODE": "auction",
        "COORDINATOR_ARGS": (
            "-p use_3d_gain:=true -p local_scheduling_enable:=false "
            "-p residual_enable:=false -p convergence_enable:=false"
        ),
    },
    "e3_hier": {
        "EXPLORATION_MODE": "auction",
        "COORDINATOR_ARGS": (
            "-p use_3d_gain:=true -p local_scheduling_enable:=true "
            "-p residual_enable:=false -p convergence_enable:=true"
        ),
    },
    "full": {
        "EXPLORATION_MODE": "auction",
        "COORDINATOR_ARGS": (
            "-p use_3d_gain:=true -p local_scheduling_enable:=true "
            "-p residual_enable:=true -p convergence_enable:=true"
        ),
    },
}

DEFAULT_SUMMARY_METRICS = [
    "final_coverage_ratio",
    "coverage_efficiency_m2_per_m",
    "n_auc",
    "t_80_s",
    "t_90_s",
    "completeness_10cm",
    "completeness_15cm",
    "chamfer_l1_m",
    "occupied_iou_tol1",
    "ate_rmse_mean_m",
    "total_path_length_m",
    "overlap_ratio",
    "cpu_mean_pct",
    "cloud_merger_cpu_mean_pct",
    "map_merge_cpu_mean_pct",
    "ram_peak_mb",
    "cycle_ms_p95",
    "se2_accept_count",
    "se2_degenerate_count",
    "comms_avg_kbps",
    "inter_robot_collisions",
]


def run_single_trial(
    world: str,
    profile: str,
    seed: int,
    out_dir: Path,
    gt_dir: Path,
    max_sim_sec: float,
    robots: List[str],
    dry_run: bool = False,
) -> Dict[str, object]:
    """Launch headless simulation + bench_recorder, clean up, and evaluate."""
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = ABLATION_PROFILES[profile]
    env = os.environ.copy()
    env.update(
        {
            "WORLD": world,
            "HEADLESS": "True",
            "USE_RVIZ": "False",
            "DASHBOARD": "False",
            "RANDOM_SPAWN": "true" if seed > 0 else "false",
            "SPAWN_SEED": str(seed),
            "EXPLORATION_MODE": cfg["EXPLORATION_MODE"],
            "COORDINATOR_ARGS": cfg["COORDINATOR_ARGS"],
        }
    )

    sim_cmd = [str(REPO_ROOT / "scripts" / "tb3_multi_explore_slam.sh")]
    rec_cmd = [
        sys.executable,
        "-m",
        "hier_explore_bench.bench_recorder",
        "--out-dir",
        str(out_dir),
        "--max-sim-sec",
        str(max_sim_sec),
        "--robots",
        *robots,
    ]
    if dry_run:
        return {"world": world, "profile": profile, "seed": seed, "sim_cmd": sim_cmd, "rec_cmd": rec_cmd}

    sim_log = (out_dir / "sim_stdout.log").open("w", encoding="utf-8")
    sim_proc = subprocess.Popen(
        sim_cmd,
        cwd=str(REPO_ROOT),
        env=env,
        stdout=sim_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        # Give Gazebo + Nav2 startup grace before starting recorder spin
        time.sleep(5.0)
        subprocess.run(
            rec_cmd,
            cwd=str(REPO_ROOT),
            env={**env, "PYTHONPATH": f"{REPO_ROOT / 'src' / 'hier_explore_bench'}:{env.get('PYTHONPATH', '')}"},
            timeout=max_sim_sec * 3.0 + 180.0,
            check=False,
        )
    finally:
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
            sim_proc.wait(timeout=15.0)
        except Exception:
            try:
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        sim_log.close()
        subprocess.run([str(REPO_ROOT / "scripts" / "kill_nav2_sim.sh")], check=False)

    report = evaluate_run_directory(out_dir, gt_dir, max_time_s=max_sim_sec)
    report.update({"world": world, "profile": profile, "seed": seed})
    (out_dir / "metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    ap = argparse.ArgumentParser(description="Batch headless benchmark matrix runner.")
    ap.add_argument("--worlds", nargs="+", default=["house"])
    ap.add_argument(
        "--profiles",
        "--methods",
        dest="profiles",
        nargs="+",
        choices=list(ABLATION_PROFILES.keys()),
        default=["e1_2d", "full"],
    )
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1])
    ap.add_argument("--robots", nargs="+", default=["robot1", "robot2"])
    ap.add_argument("--max-sim-sec", type=float, default=240.0)
    ap.add_argument("--gt-root", default=str(REPO_ROOT / "maps" / "gt"))
    ap.add_argument(
        "--out-root",
        "--out-dir",
        dest="out_root",
        default=str(REPO_ROOT / "benchmark_results" / "latest"),
    )
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    all_reports: List[Dict[str, object]] = []

    for world in args.worlds:
        gt_dir = Path(args.gt_root) / world
        for profile in args.profiles:
            profile_runs = []
            for seed in args.seeds:
                run_dir = out_root / world / profile / f"seed_{seed}"
                print(f"[bench] world={world} profile={profile} seed={seed} -> {run_dir}")
                rep = run_single_trial(
                    world=world,
                    profile=profile,
                    seed=seed,
                    out_dir=run_dir,
                    gt_dir=gt_dir,
                    max_sim_sec=args.max_sim_sec,
                    robots=args.robots,
                    dry_run=args.dry_run,
                )
                profile_runs.append(rep)
                all_reports.append(rep)

            if not args.dry_run:
                agg = aggregate_runs(profile_runs, DEFAULT_SUMMARY_METRICS)
                (out_root / world / profile / "aggregate.json").write_text(
                    json.dumps(agg, indent=2), encoding="utf-8"
                )

    (out_root / "all_runs.json").write_text(json.dumps(all_reports, indent=2), encoding="utf-8")
    if not args.dry_run and all_reports:
        csv_path = out_root / "summary.csv"
        cols = ["world", "profile", "seed"] + DEFAULT_SUMMARY_METRICS
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            writer.writeheader()
            for r in all_reports:
                writer.writerow(r)
        print(f"[bench] wrote {csv_path}")


if __name__ == "__main__":
    main()
