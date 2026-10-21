#!/usr/bin/env python3
"""Classroom exploration with complete 360-degree sweeps and position-only tasks."""
from classroom_voxel_scene import ROOT
from voxel_replay_io import parse_args, run
from lidar360_observation import Lidar360


def main():
    args = parse_args(defaults={
        "config": ROOT/"config/voxel360_planner_params.yaml",
        "output_dir": ROOT/"maps/classroom_voxel360",
        "scan_hz": Lidar360.load().scan_hz,
    })
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
