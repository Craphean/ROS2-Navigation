"""End-to-end offline tests for hier_explore_bench (GT mesh sampler, metrics, plotting)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from hier_explore_bench.eval_metrics import (
    aggregate_runs,
    compare_methods_significance,
    evaluate_2d_map,
    evaluate_3d_reconstruction,
    evaluate_exploration_timeseries,
    evaluate_trajectory_accuracy,
)
from hier_explore_bench.pcd_io import read_pcd, write_pcd
from hier_explore_bench.plot_paper_figs import format_latex_table, plot_coverage_curves
from hier_explore_bench.sample_gt_mesh import sample_world_ground_truth

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_pcd_binary_and_ascii_roundtrip(tmp_path: Path):
    pts = np.array([[1.0, 2.5, 0.3], [-4.2, 0.0, 1.8], [0.5, -1.1, 0.9]], dtype=np.float32)
    for binary in (True, False):
        path = tmp_path / f"test_{binary}.pcd"
        write_pcd(path, pts, binary=binary)
        loaded = read_pcd(path)
        assert loaded.shape == pts.shape
        np.testing.assert_allclose(loaded, pts, atol=1e-4)


def test_sample_gt_real_mine_and_house_worlds():
    mine_res = sample_world_ground_truth(
        REPO_ROOT / "worlds" / "mine.sdf.xacro",
        sample_step=0.20,
        voxel_size=0.10,
        seed_xy=(-19.0, 0.0),
    )
    # Mine world has recursive xacro macros (17 main arches + side arches + walls + props)
    assert mine_res["primitive_count"] > 80
    assert len(mine_res["points_3d"]) > 1000
    # 44m * 3.2m main drift + 2 cross drifts + 2 chambers (~250-350 m^2)
    assert 180.0 < mine_res["explorable_area_m2"] < 420.0

    house_res = sample_world_ground_truth(
        REPO_ROOT / "worlds" / "house.sdf.xacro",
        sample_step=0.15,
        voxel_size=0.05,
        seed_xy=(-3.0, -2.0),
    )
    # House outer envelope is 11m x 9m = 99 m^2 minus wall thickness (~85-95 m^2)
    assert 70.0 < house_res["explorable_area_m2"] < 105.0
    assert len(house_res["points_3d"]) > 500


def test_3d_reconstruction_metrics_exact_and_occluded():
    xs = np.linspace(0.0, 4.0, 41)
    ys = np.linspace(0.0, 2.0, 21)
    xx, yy = np.meshgrid(xs, ys)
    gt = np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, 1.0)))

    # Perfect reconstruction shifted by 0.05 m (< 0.10 m threshold)
    rec_good = gt + np.array([0.0, 0.0, 0.05])
    m_good = evaluate_3d_reconstruction(rec_good, gt)
    assert m_good["completeness_10cm"] == pytest.approx(1.0)
    assert m_good["fscore_10cm"] == pytest.approx(1.0)
    assert m_good["accuracy_mean_m"] == pytest.approx(0.05, abs=1e-4)

    # Half of the wall missing (simulating 3D occlusion blind spot)
    rec_half = gt[gt[:, 0] <= 2.0]
    m_half = evaluate_3d_reconstruction(rec_half, gt)
    assert 0.45 < m_half["completeness_10cm"] < 0.60
    assert m_half["precision_10cm"] == pytest.approx(1.0)


def test_2d_map_timeseries_ate_and_significance():
    gt_grid = np.full((100, 100), -1, dtype=np.int8)
    gt_grid[10:90, 10:90] = 0
    gt_grid[10, 10:90] = 100
    gt_grid[89, 10:90] = 100
    gt_grid[10:90, 10] = 100
    gt_grid[10:90, 89] = 100

    # Exact copy gives IoU == 1.0 and coverage == 1.0
    m2d_exact = evaluate_2d_map(gt_grid, (0.0, 0.0), 0.1, gt_grid, (0.0, 0.0), 0.1)
    assert m2d_exact["coverage_2d_ratio"] == pytest.approx(1.0)
    assert m2d_exact["occupied_iou_tol1"] == pytest.approx(1.0)

    # Reconstructed map covers left 80% of interior (missing right wall + 20% top/bottom = ~65% perimeter)
    rec_grid = gt_grid.copy()
    rec_grid[10:90, 74:90] = -1
    m2d = evaluate_2d_map(rec_grid, (0.0, 0.0), 0.1, gt_grid, (0.0, 0.0), 0.1)
    assert 0.75 < m2d["coverage_2d_ratio"] < 0.85
    assert 0.60 < m2d["occupied_iou_tol1"] < 0.72

    # Timeseries & multi-robot trajectories
    times = [0.0, 10.0, 20.0, 30.0, 40.0]
    areas = [10.0, 40.0, 70.0, 85.0, 95.0]
    trajs = {
        "robot1": np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]]),
        "robot2": np.array([[0.0, 0.0], [0.0, 5.0], [0.0, 10.0]]),
    }
    ts_m = evaluate_exploration_timeseries(times, areas, trajs, target_area_m2=100.0)
    assert ts_m["total_path_length_m"] == pytest.approx(20.0)
    assert ts_m["coverage_efficiency_m2_per_m"] == pytest.approx(95.0 / 20.0)
    assert ts_m["load_balance_cv"] == pytest.approx(0.0)
    assert ts_m["t_80_s"] == pytest.approx(30.0)
    assert 0.5 < ts_m["n_auc"] < 0.7

    # Trajectory ATE / RPE
    gt_xy = np.column_stack((np.linspace(0, 10, 11), np.zeros(11)))
    est_xy = gt_xy + np.array([0.0, 0.2])
    acc = evaluate_trajectory_accuracy(est_xy, gt_xy)
    assert acc["ate_rmse_m"] == pytest.approx(0.2)
    assert acc["rpe_trans_rmse_m"] == pytest.approx(0.0, abs=1e-9)

    # Significance test across 5 seeds
    ours = [{"completeness_10cm": x} for x in (0.91, 0.93, 0.90, 0.92, 0.94)]
    base = [{"completeness_10cm": x} for x in (0.72, 0.75, 0.71, 0.74, 0.73)]
    sig = compare_methods_significance(ours, base, "completeness_10cm")
    assert sig["delta_mean"] > 0.15
    assert sig["welch_p"] < 0.001
    assert sig["mwu_p"] < 0.02


def test_ieee_plot_and_latex_table(tmp_path: Path):
    curves = {
        "Ours": [
            (np.array([0.0, 50.0, 100.0]), np.array([5.0, 75.0, 96.0])),
            (np.array([0.0, 50.0, 100.0]), np.array([6.0, 70.0, 94.0])),
        ],
        "2D-Auction": [
            (np.array([0.0, 50.0, 100.0]), np.array([5.0, 55.0, 82.0])),
            (np.array([0.0, 50.0, 100.0]), np.array([5.0, 58.0, 80.0])),
        ],
    }
    pdf = plot_coverage_curves(curves, tmp_path / "fig_cov", max_x=100.0)
    assert pdf.is_file() and pdf.stat().st_size > 1000
    assert (tmp_path / "fig_cov.png").is_file()

    agg_ours = aggregate_runs([{"ce": 2.4}, {"ce": 2.6}], ["ce"])
    tex = format_latex_table([("Ours", agg_ours)], [("ce", r"CE ($\text{m}^2/\text{m}$)", ".2f")])
    assert r"\toprule" in tex and "Ours" in tex and "2.50" in tex
