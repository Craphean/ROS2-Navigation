"""Benchmarking, ground-truth mesh sampling, metrics evaluation, and IEEE figure generation."""

from .eval_metrics import (
    aggregate_runs,
    compare_methods_significance,
    evaluate_2d_map,
    evaluate_3d_reconstruction,
    evaluate_exploration_timeseries,
    evaluate_trajectory_accuracy,
)
from .pcd_io import read_pcd, voxel_downsample, write_pcd

__all__ = [
    "read_pcd",
    "write_pcd",
    "voxel_downsample",
    "evaluate_3d_reconstruction",
    "evaluate_2d_map",
    "evaluate_exploration_timeseries",
    "evaluate_trajectory_accuracy",
    "aggregate_runs",
    "compare_methods_significance",
]
