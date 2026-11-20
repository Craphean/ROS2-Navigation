"""M4 unit tests for hier_explore_bringup: 6-world registry, N=1..4 spawns, profiles, and launch."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from hier_explore_bringup import (
    ABLATION_PROFILES,
    BENCHMARK_WORLDS,
    WORLD_SPECS,
    canonical_world_name,
    find_repo_root,
    resolve_profile,
    resolve_world_file,
    sample_spawn_poses,
)

REPO_ROOT = find_repo_root()


def test_all_six_benchmark_worlds_registered_and_files_exist():
    assert set(BENCHMARK_WORLDS) == {
        "house",
        "hospital",
        "mine",
        "warehouse",
        "cmu_indoor",
        "cmu_tunnel",
    }
    for w in BENCHMARK_WORLDS:
        assert w in WORLD_SPECS
        wf = resolve_world_file(w, REPO_ROOT)
        assert wf.is_file(), f"World file missing for {w}: {wf}"
    assert canonical_world_name("indoor") == "cmu_indoor"
    assert canonical_world_name("tunnel") == "cmu_tunnel"
    assert canonical_world_name("coalmine") == "mine"


@pytest.mark.parametrize("world", BENCHMARK_WORLDS)
@pytest.mark.parametrize("num_robots", [1, 2, 3, 4])
def test_spawn_poses_non_overlapping_and_deterministic(world: str, num_robots: int):
    # Fixed formation (seed=0)
    fixed = sample_spawn_poses(world, num_robots=num_robots, spawn_seed=0)
    assert len(fixed) == num_robots
    for i in range(len(fixed)):
        for j in range(i + 1, len(fixed)):
            d = math.hypot(fixed[i].x - fixed[j].x, fixed[i].y - fixed[j].y)
            assert d >= 0.90, f"{world} fixed spawn overlap: {fixed[i]} vs {fixed[j]} ({d:.2f}m)"

    # Seeded station sampling (seeds 1..5): identical across repeated calls and non-overlapping
    for seed in (1, 2, 3, 4, 5):
        s1 = sample_spawn_poses(world, num_robots=num_robots, spawn_seed=seed)
        s2 = sample_spawn_poses(world, num_robots=num_robots, spawn_seed=seed)
        assert s1 == s2
        assert len(s1) == num_robots
        for i in range(len(s1)):
            for j in range(i + 1, len(s1)):
                d = math.hypot(s1[i].x - s1[j].x, s1[i].y - s1[j].y)
                assert d >= 0.90, f"{world} seed={seed} overlap ({d:.2f}m)"


@pytest.mark.parametrize("world", BENCHMARK_WORLDS)
def test_all_spawns_collision_free_in_gt_2d_map(world: str):
    gt_2d = REPO_ROOT / "maps" / "gt" / world / "gt_2d.npz"
    if not gt_2d.is_file():
        pytest.skip(f"GT map not generated yet at {gt_2d}")
    d = np.load(gt_2d)
    grid = d["grid"]
    ox, oy, res = float(d["origin_x"]), float(d["origin_y"]), float(d["resolution"])
    dt = cv2.distanceTransform((grid == 0).astype(np.uint8), cv2.DIST_L2, 5) * res

    spec = WORLD_SPECS[world]
    for kind, poses, min_clr in (
        ("default", spec.default_spawns, 0.45),
        ("station", spec.station_pool, 0.70),
    ):
        for sp in poses:
            c = int(math.floor((sp.x - ox) / res))
            r = int(math.floor((sp.y - oy) / res))
            assert 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1], f"{world} {kind} {sp} OOB"
            clr = float(dt[r, c])
            assert clr >= min_clr, f"{world} {kind} {sp} clearance {clr:.2f}m < {min_clr}m"


def test_num_robots_validation():
    with pytest.raises(ValueError):
        sample_spawn_poses("mine", num_robots=0)
    with pytest.raises(ValueError):
        sample_spawn_poses("mine", num_robots=5)


def test_ablation_profiles_and_mode_resolution():
    for name in (
        "legacy",
        "e0_legacy",
        "e1_2d",
        "e2_3dgain",
        "e3_hier",
        "e4_geodesic",
        "e5_deconflict",
        "full",
        "e6_full",
        "no_se2",
        "no_degen_gate",
    ):
        assert name in ABLATION_PROFILES
        r = resolve_profile(ablation_profile=name)
        assert r["odom_mode"] == "ekf"
        assert r["use_gt_odom"] is False

    r_leg = resolve_profile("legacy", odom_mode="gt")
    assert r_leg["planner_backend"] == "legacy"
    assert r_leg["use_se2_merge"] is False
    assert r_leg["use_cpp_cloud_merger"] is False
    assert r_leg["use_gt_odom"] is True

    r_full = resolve_profile("full", planner_backend="off")
    assert r_full["planner_backend"] == "off"
    assert r_full["use_se2_merge"] is True
    assert r_full["coordinator_params"]["use_geodesic_distance"] is True
    assert r_full["coordinator_params"]["use_los_fov_optimizer"] is True

    r_nose2 = resolve_profile("no_se2")
    assert r_nose2["se2_merge_params"]["enable_online_alignment"] is False
    r_nodeg = resolve_profile("no_degen_gate")
    assert r_nodeg["se2_merge_params"]["enable_degeneracy_gate"] is False

    with pytest.raises(ValueError):
        resolve_profile("nonexistent_profile")
    with pytest.raises(ValueError):
        resolve_profile("full", odom_mode="invalid_odom")


def test_nav2_params_has_slam_toolbox_for_all_four_robots():
    cfg = yaml.safe_load((REPO_ROOT / "config" / "nav2_multirobot_slam_params.yaml").read_text())
    for i in range(1, 5):
        ns = f"robot{i}"
        assert ns in cfg and "slam_toolbox" in cfg[ns], f"Missing {ns}.slam_toolbox"
        st = cfg[ns]["slam_toolbox"]["ros__parameters"]
        assert st["scan_topic"] == "scan"
        assert st["restamp_tf"] is True
        assert st["throttle_scans"] == 2


def test_multi_sim_launch_declares_required_arguments():
    pytest.importorskip("launch")
    launch_path = REPO_ROOT / "src" / "hier_explore_bringup" / "launch" / "multi_sim.launch.py"
    spec = importlib.util.spec_from_file_location("multi_sim_launch", str(launch_path))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    declared = {
        getattr(ent, "name", "")
        for ent in ld.entities
        if type(ent).__name__ == "DeclareLaunchArgument"
    }
    for req in (
        "num_robots",
        "world",
        "spawn_seed",
        "odom_mode",
        "ablation_profile",
        "planner_backend",
        "headless",
        "use_rviz",
    ):
        assert req in declared, f"Missing LaunchArgument: {req}"
