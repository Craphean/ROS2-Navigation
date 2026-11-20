#!/usr/bin/env python3
"""Unified offline evaluator for 2D/3D multi-robot exploration benchmarks.

Computes all publication metrics for IEEE RA-L / ICRA / IROS:
  1. 3D Reconstruction Quality: Surface Completeness @ 0.10m / 0.15m, Accuracy,
     Bidirectional Chamfer L1 / L2 (RMSE), and F-score against GT mesh points.
  2. 2D Map Consistency: Coverage ratio vs A_GT, Occupied-cell IoU (with 1-cell
     discretization tolerance), Free-space IoU, and 95th-percentile Hausdorff.
  3. Exploration Efficiency: T80 / T90 / T95, Coverage Efficiency (CE, m^2/m),
     normalized AUC (nAUC), total/per-robot path lengths, load balance CV, and
     redundant path overlap ratio.
  4. Localization Accuracy: ATE RMSE / Max and translational RPE RMSE vs GT pose.
  5. Multi-Seed Statistical Significance: Mean +/- Std, Welch's t-test, and
     Mann-Whitney U test across K random spawn seeds.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import mannwhitneyu, ttest_ind

from .pcd_io import read_pcd


# ---------------------------------------------------------------------------
# 1. 3D Reconstruction metrics
# ---------------------------------------------------------------------------


def evaluate_3d_reconstruction(
    rec_points: np.ndarray,
    gt_points: np.ndarray,
    thresholds: Sequence[float] = (0.10, 0.15),
) -> Dict[str, float]:
    """Compare reconstructed 3D point cloud against GT surface point cloud."""
    rec = np.asarray(rec_points, dtype=np.float64).reshape(-1, 3)
    gt = np.asarray(gt_points, dtype=np.float64).reshape(-1, 3)
    out: Dict[str, float] = {
        "rec_points": float(len(rec)),
        "gt_points": float(len(gt)),
    }
    if len(rec) == 0 or len(gt) == 0:
        out.update(
            {
                "accuracy_mean_m": float("nan"),
                "accuracy_p95_m": float("nan"),
                "completeness_mean_m": float("nan"),
                "chamfer_l1_m": float("nan"),
                "chamfer_l2_m": float("nan"),
            }
        )
        for tau in thresholds:
            tag = f"{int(round(tau * 100))}cm"
            out[f"completeness_{tag}"] = 0.0
            out[f"precision_{tag}"] = 0.0
            out[f"fscore_{tag}"] = 0.0
        return out

    tree_gt = cKDTree(gt)
    tree_rec = cKDTree(rec)

    d_rec_to_gt, _ = tree_gt.query(rec, k=1, workers=-1)
    d_gt_to_rec, _ = tree_rec.query(gt, k=1, workers=-1)

    acc_mean = float(np.mean(d_rec_to_gt))
    acc_p95 = float(np.percentile(d_rec_to_gt, 95))
    comp_mean = float(np.mean(d_gt_to_rec))
    chamfer_l1 = 0.5 * (acc_mean + comp_mean)
    chamfer_l2 = 0.5 * (
        math.sqrt(float(np.mean(d_rec_to_gt ** 2)))
        + math.sqrt(float(np.mean(d_gt_to_rec ** 2)))
    )

    out.update(
        {
            "accuracy_mean_m": acc_mean,
            "accuracy_p95_m": acc_p95,
            "completeness_mean_m": comp_mean,
            "chamfer_l1_m": chamfer_l1,
            "chamfer_l2_m": chamfer_l2,
        }
    )

    for tau in thresholds:
        tag = f"{int(round(tau * 100))}cm"
        comp = float(np.mean(d_gt_to_rec <= tau))
        prec = float(np.mean(d_rec_to_gt <= tau))
        fscore = (2.0 * prec * comp / (prec + comp)) if (prec + comp) > 1e-12 else 0.0
        out[f"completeness_{tag}"] = comp
        out[f"precision_{tag}"] = prec
        out[f"fscore_{tag}"] = fscore
    return out


# ---------------------------------------------------------------------------
# 2. 2D Map alignment & coverage metrics
# ---------------------------------------------------------------------------


def _resample_grid_to_target(
    src_grid: np.ndarray,
    src_origin: Tuple[float, float],
    src_res: float,
    dst_shape: Tuple[int, int],
    dst_origin: Tuple[float, float],
    dst_res: float,
) -> np.ndarray:
    dst_h, dst_w = dst_shape
    xs = dst_origin[0] + (np.arange(dst_w) + 0.5) * dst_res
    ys = dst_origin[1] + (np.arange(dst_h) + 0.5) * dst_res
    sc = np.floor((xs - src_origin[0]) / src_res).astype(np.int64)
    sr = np.floor((ys - src_origin[1]) / src_res).astype(np.int64)
    valid_c = (sc >= 0) & (sc < src_grid.shape[1])
    valid_r = (sr >= 0) & (sr < src_grid.shape[0])
    out = np.full((dst_h, dst_w), -1, dtype=np.int8)
    if valid_r.any() and valid_c.any():
        out[np.ix_(np.where(valid_r)[0], np.where(valid_c)[0])] = src_grid[
            np.ix_(sr[valid_r], sc[valid_c])
        ]
    return out


def evaluate_2d_map(
    rec_grid: np.ndarray,
    rec_origin: Tuple[float, float],
    rec_res: float,
    gt_grid: np.ndarray,
    gt_origin: Tuple[float, float],
    gt_res: float,
    explorable_area_m2: Optional[float] = None,
) -> Dict[str, float]:
    """Evaluate 2D merged OccupancyGrid against GT grid in world coordinates."""
    aligned = _resample_grid_to_target(
        rec_grid, rec_origin, rec_res, gt_grid.shape, gt_origin, gt_res
    )
    gt_free = (gt_grid >= 0) & (gt_grid < 20)
    gt_occ = gt_grid > 50
    rec_free = (aligned >= 0) & (aligned < 20)
    rec_occ = aligned > 50

    gt_area = (
        float(explorable_area_m2)
        if explorable_area_m2 and explorable_area_m2 > 0
        else float(gt_free.sum()) * (gt_res ** 2)
    )
    rec_free_in_gt = float((rec_free & gt_free).sum()) * (gt_res ** 2)
    rec_total_free = float(((rec_grid >= 0) & (rec_grid < 20)).sum()) * (rec_res ** 2)
    cov_ratio = min(1.0, rec_free_in_gt / gt_area) if gt_area > 1e-6 else 0.0

    # Free-space IoU
    free_inter = int((rec_free & gt_free).sum())
    free_union = int((rec_free | gt_free).sum())
    free_iou = free_inter / free_union if free_union > 0 else 0.0

    # Occupied-cell dilated IoU (1-cell tolerance for SLAM wall discretization)
    kernel = np.ones((3, 3), dtype=np.uint8)
    gt_occ_d = cv2.dilate(gt_occ.astype(np.uint8), kernel) > 0
    rec_occ_d = cv2.dilate(rec_occ.astype(np.uint8), kernel) > 0
    occ_inter = int(((rec_occ & gt_occ_d) | (gt_occ & rec_occ_d)).sum())
    occ_union = int((rec_occ | gt_occ).sum())
    occ_iou = min(1.0, occ_inter / occ_union) if occ_union > 0 else 0.0

    # 95th percentile symmetric Hausdorff distance on occupied cells
    hd95 = float("nan")
    if rec_occ.any() and gt_occ.any():
        r_pts = np.argwhere(rec_occ).astype(np.float64) * gt_res
        g_pts = np.argwhere(gt_occ).astype(np.float64) * gt_res
        d_rg, _ = cKDTree(g_pts).query(r_pts, k=1)
        d_gr, _ = cKDTree(r_pts).query(g_pts, k=1)
        hd95 = float(max(np.percentile(d_rg, 95), np.percentile(d_gr, 95)))

    return {
        "gt_area_m2": gt_area,
        "explored_area_m2": rec_total_free,
        "explored_interior_area_m2": rec_free_in_gt,
        "coverage_2d_ratio": cov_ratio,
        "free_iou": free_iou,
        "occupied_iou_tol1": occ_iou,
        "occupied_hd95_m": hd95,
    }


# ---------------------------------------------------------------------------
# 3. Exploration efficiency, multi-robot balance, and overlap
# ---------------------------------------------------------------------------


def _path_length_2d(traj_xy: np.ndarray) -> float:
    pts = np.asarray(traj_xy, dtype=np.float64).reshape(-1, 2)
    if len(pts) < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))


def evaluate_exploration_timeseries(
    times: Sequence[float],
    areas_m2: Sequence[float],
    trajectories: Dict[str, np.ndarray],
    target_area_m2: float,
    overlap_cell_m: float = 0.5,
    max_time_s: Optional[float] = None,
) -> Dict[str, object]:
    """Compute T80/T90/T95, CE, nAUC, load balance, and multi-robot path overlap."""
    t_arr = np.asarray(times, dtype=np.float64)
    a_arr = np.asarray(areas_m2, dtype=np.float64)
    target = max(1e-6, float(target_area_m2))

    per_robot_len = {
        ns: _path_length_2d(np.asarray(pts)[:, :2]) if len(pts) else 0.0
        for ns, pts in trajectories.items()
    }
    total_len = sum(per_robot_len.values())
    lengths_vals = list(per_robot_len.values())
    mean_len = float(np.mean(lengths_vals)) if lengths_vals else 0.0
    load_balance_cv = (
        float(np.std(lengths_vals) / mean_len) if len(lengths_vals) > 1 and mean_len > 1e-6 else 0.0
    )

    # Multi-robot spatial traversal overlap ratio on coarse cells
    cell_visitors: Dict[Tuple[int, int], set] = {}
    for ns, pts in trajectories.items():
        arr = np.asarray(pts, dtype=np.float64)
        if arr.size == 0:
            continue
        xy = arr.reshape(-1, arr.shape[-1])[:, :2]
        gx = np.floor(xy[:, 0] / overlap_cell_m).astype(np.int64)
        gy = np.floor(xy[:, 1] / overlap_cell_m).astype(np.int64)
        for cx, cy in zip(gx.tolist(), gy.tolist()):
            cell_visitors.setdefault((cx, cy), set()).add(ns)
    n_cells = len(cell_visitors)
    n_shared = sum(1 for visitors in cell_visitors.values() if len(visitors) > 1)
    overlap_ratio = n_shared / n_cells if n_cells > 0 else 0.0

    final_area = float(a_arr[-1]) if len(a_arr) else 0.0
    ce = final_area / total_len if total_len > 1e-6 else 0.0

    cov = np.clip(a_arr / target, 0.0, 1.0) if len(a_arr) else np.zeros(0)
    t_thresholds = {}
    for pct in (80, 90, 95):
        hit = np.flatnonzero(cov >= pct / 100.0)
        t_thresholds[f"t_{pct}_s"] = float(t_arr[hit[0]] - t_arr[0]) if len(hit) else float("nan")

    # Normalized AUC over [t0, t_end] (or fixed max_time_s horizon if provided)
    n_auc = 0.0
    if len(t_arr) >= 2:
        t_rel = t_arr - t_arr[0]
        horizon = float(max_time_s) if max_time_s and max_time_s > 0 else float(t_rel[-1])
        if horizon > 1e-6:
            area_under = float(np.trapezoid(cov, t_rel)) if hasattr(np, "trapezoid") else float(np.trapz(cov, t_rel))
            if max_time_s and t_rel[-1] < horizon:
                area_under += float(cov[-1]) * (horizon - float(t_rel[-1]))
            n_auc = min(1.0, area_under / horizon)

    return {
        "final_area_m2": final_area,
        "final_coverage_ratio": min(1.0, final_area / target),
        "total_path_length_m": total_len,
        "per_robot_path_lengths_m": per_robot_len,
        "coverage_efficiency_m2_per_m": ce,
        "n_auc": n_auc,
        "load_balance_cv": load_balance_cv,
        "overlap_ratio": overlap_ratio,
        **t_thresholds,
    }


# ---------------------------------------------------------------------------
# 4. Localization ATE / RPE accuracy
# ---------------------------------------------------------------------------


def evaluate_trajectory_accuracy(
    est_xy: np.ndarray, gt_xy: np.ndarray, rpe_step: int = 1
) -> Dict[str, float]:
    """Compute Absolute Trajectory Error (ATE) and translational RPE RMSE."""
    est = np.asarray(est_xy, dtype=np.float64).reshape(-1, 2)
    gt = np.asarray(gt_xy, dtype=np.float64).reshape(-1, 2)
    n = min(len(est), len(gt))
    if n == 0:
        return {"ate_rmse_m": float("nan"), "ate_max_m": float("nan"), "rpe_trans_rmse_m": float("nan")}
    err = np.linalg.norm(est[:n] - gt[:n], axis=1)
    ate_rmse = math.sqrt(float(np.mean(err ** 2)))
    ate_max = float(np.max(err))
    if n > rpe_step:
        d_est = est[rpe_step:n] - est[: n - rpe_step]
        d_gt = gt[rpe_step:n] - gt[: n - rpe_step]
        rpe_err = np.linalg.norm(d_est - d_gt, axis=1)
        rpe_rmse = math.sqrt(float(np.mean(rpe_err ** 2)))
    else:
        rpe_rmse = float("nan")
    return {
        "ate_rmse_m": ate_rmse,
        "ate_max_m": ate_max,
        "rpe_trans_rmse_m": rpe_rmse,
    }


# ---------------------------------------------------------------------------
# 5. Multi-seed aggregation & statistical significance tests
# ---------------------------------------------------------------------------


def aggregate_runs(run_reports: Sequence[Dict[str, object]], metric_keys: Sequence[str]) -> Dict[str, Dict[str, float]]:
    """Aggregate Mean, Std, Median, and valid count across multiple random seeds."""
    summary: Dict[str, Dict[str, float]] = {}
    for key in metric_keys:
        vals = []
        for rep in run_reports:
            v = rep.get(key)
            if isinstance(v, (int, float)) and math.isfinite(float(v)):
                vals.append(float(v))
        if not vals:
            summary[key] = {"mean": float("nan"), "std": float("nan"), "median": float("nan"), "n": 0}
        else:
            arr = np.asarray(vals, dtype=np.float64)
            summary[key] = {
                "mean": float(np.mean(arr)),
                "std": float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0,
                "median": float(np.median(arr)),
                "n": int(len(arr)),
            }
    return summary


def compare_methods_significance(
    runs_a: Sequence[Dict[str, object]],
    runs_b: Sequence[Dict[str, object]],
    metric_key: str,
) -> Dict[str, float]:
    """Run Welch's t-test and Mann-Whitney U test comparing method A vs method B."""
    va = [
        float(r[metric_key])
        for r in runs_a
        if metric_key in r and isinstance(r[metric_key], (int, float)) and math.isfinite(float(r[metric_key]))
    ]
    vb = [
        float(r[metric_key])
        for r in runs_b
        if metric_key in r and isinstance(r[metric_key], (int, float)) and math.isfinite(float(r[metric_key]))
    ]
    if len(va) < 2 or len(vb) < 2:
        return {"welch_p": float("nan"), "mwu_p": float("nan"), "delta_mean": float("nan")}
    _, welch_p = ttest_ind(va, vb, equal_var=False)
    _, mwu_p = mannwhitneyu(va, vb, alternative="two-sided")
    return {
        "mean_a": float(np.mean(va)),
        "mean_b": float(np.mean(vb)),
        "delta_mean": float(np.mean(va) - np.mean(vb)),
        "welch_p": float(welch_p),
        "mwu_p": float(mwu_p),
    }


def evaluate_run_directory(run_dir: Path, gt_dir: Path, max_time_s: Optional[float] = None) -> Dict[str, object]:
    """Evaluate one recorded benchmark run directory against a GT directory."""
    report: Dict[str, object] = {"run_dir": str(run_dir)}

    gt_meta_path = gt_dir / "gt_meta.json"
    gt_meta = json.loads(gt_meta_path.read_text(encoding="utf-8")) if gt_meta_path.is_file() else {}
    target_area = float(gt_meta.get("explorable_area_m2", 100.0))

    # 3D reconstruction
    gt_pcd = gt_dir / "gt_surface.pcd"
    rec_pcd = run_dir / "cloud_final.pcd"
    if gt_pcd.is_file() and rec_pcd.is_file():
        report.update(evaluate_3d_reconstruction(read_pcd(rec_pcd), read_pcd(gt_pcd)))

    # 2D map
    gt_2d = gt_dir / "gt_2d.npz"
    rec_2d = run_dir / "map_final.npz"
    if gt_2d.is_file() and rec_2d.is_file():
        g = np.load(gt_2d)
        r = np.load(rec_2d)
        report.update(
            evaluate_2d_map(
                r["grid"],
                (float(r["origin_x"]), float(r["origin_y"])),
                float(r["resolution"]),
                g["grid"],
                (float(g["origin_x"]), float(g["origin_y"])),
                float(g["resolution"]),
                explorable_area_m2=float(g["explorable_area_m2"]),
            )
        )

    # Time series + trajectories
    ts_path = run_dir / "timeseries.csv"
    traj_path = run_dir / "trajectories.json"
    if ts_path.is_file():
        times, areas = [], []
        with ts_path.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                times.append(float(row["sim_time_s"]))
                areas.append(float(row["explored_area_m2"]))
        trajs: Dict[str, np.ndarray] = {}
        if traj_path.is_file():
            raw_tr = json.loads(traj_path.read_text(encoding="utf-8"))
            for ns, pts in raw_tr.get("estimated", {}).items():
                trajs[ns] = np.asarray(pts, dtype=np.float64)
            # Optional ATE/RPE when GT trajectories were logged
            ate_vals = []
            for ns, est_pts in raw_tr.get("estimated", {}).items():
                gt_pts = raw_tr.get("ground_truth", {}).get(ns)
                if est_pts and gt_pts:
                    acc = evaluate_trajectory_accuracy(np.asarray(est_pts)[:, :2], np.asarray(gt_pts)[:, :2])
                    ate_vals.append(acc["ate_rmse_m"])
            if ate_vals:
                report["ate_rmse_mean_m"] = float(np.mean(ate_vals))
        report.update(
            evaluate_exploration_timeseries(times, areas, trajs, target_area, max_time_s=max_time_s)
        )

    res_path = run_dir / "resources.json"
    if res_path.is_file():
        report.update(json.loads(res_path.read_text(encoding="utf-8")))

    out_path = run_dir / "metrics.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    ap = argparse.ArgumentParser(description="Evaluate a recorded exploration run against GT.")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--gt-dir", required=True)
    ap.add_argument("--max-time", type=float, default=None)
    args = ap.parse_args()
    rep = evaluate_run_directory(Path(args.run_dir), Path(args.gt_dir), max_time_s=args.max_time)
    print(json.dumps(rep, indent=2))


if __name__ == "__main__":
    main()
