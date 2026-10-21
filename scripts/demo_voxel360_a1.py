#!/usr/bin/env python3
"""A1 Algorithm: asynchronous exploration with bounded commitment and early refill."""
from classroom_voxel_scene import ROOT
from voxel_replay_io import parse_args, run
from lidar360_observation import Lidar360


def main():
    return run(parse_args(defaults={
        "config": ROOT/"config/voxel360_a1_params.yaml",
        "output_dir": ROOT/"maps/classroom_voxel360_a1",
        "scan_hz": Lidar360.load().scan_hz,
    }))


if __name__ == "__main__":
    raise SystemExit(main())
