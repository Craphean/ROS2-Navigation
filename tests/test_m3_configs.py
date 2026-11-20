"""M3 config / launcher wiring: shared global costmap, teammate clouds, coordinator params."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
NAV2 = REPO_ROOT / "config" / "nav2_multirobot_slam_params.yaml"
SE2 = REPO_ROOT / "config" / "se2_map_merge_params.yaml"
COORD = REPO_ROOT / "config" / "coordinator_params.yaml"
LAUNCHER = REPO_ROOT / "scripts" / "tb3_multi_explore_slam.sh"
NODE = REPO_ROOT / "src" / "hier_explore_core" / "hier_explore_core" / "coordinator_node.py"


def _load(p: Path):
    return yaml.safe_load(p.read_text(encoding="utf-8"))


def test_global_costmap_static_layer_uses_shared_map():
    cfg = _load(NAV2)
    static = cfg["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"]
    assert static["map_topic"] == "shared_map"  # relative -> /<ns>/shared_map
    assert static["map_subscribe_transient_local"] is True


def test_local_voxel_layer_has_teammates_source():
    cfg = _load(NAV2)
    vox = cfg["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]
    assert vox["observation_sources"].split() == ["scan", "points", "teammates"]
    tm = vox["teammates"]
    assert tm["topic"] == "teammates_cloud"
    assert tm["data_type"] == "PointCloud2"
    assert tm["marking"] is True and tm["clearing"] is True
    # Clearing ring (2.85 m) must be raytraced but never marked.
    assert tm["obstacle_max_range"] < 2.85 < tm["raytrace_max_range"]
    # Cloud z = 0.17 m lies inside the marking band.
    assert tm["min_obstacle_height"] < 0.17 < tm["max_obstacle_height"]
    # Original sources untouched.
    assert vox["points"]["topic"] == "points" and vox["points"]["clearing"] is False
    assert vox["scan"]["data_type"] == "LaserScan"


def test_se2_merge_publishes_shared_maps():
    p = _load(SE2)["/**"]["ros__parameters"]
    assert p["publish_per_robot_shared_map"] is True


def test_coordinator_m3_params_documented_and_declared():
    p = _load(COORD)["multi_robot_coordinator"]["ros__parameters"]
    src = NODE.read_text(encoding="utf-8")
    keys = [
        "use_geodesic_distance", "geodesic_min_clearance", "geodesic_max_dist",
        "use_los_fov_optimizer", "residual_viewpoint_r_min", "residual_viewpoint_r_max",
        "residual_viewpoint_samples", "lidar_sensor_z", "lidar_vfov_min_deg",
        "lidar_vfov_max_deg", "residual_max_eff_range", "teammate_deconfliction_enable",
        "teammate_cloud_rate_hz", "corridor_mutex_enable", "corridor_narrow_margin",
        "corridor_robot_radius",
    ]
    for k in keys:
        assert k in p, k
        assert re.search(rf'p\("{k}"', src), f"{k} not declared in coordinator_node.py"
    assert p["lidar_vfov_min_deg"] == -30.0 and p["lidar_vfov_max_deg"] == 55.0
    assert p["teammate_cloud_rate_hz"] == 5.0


def test_launcher_syntax_and_m3_ordering():
    subprocess.run(["bash", "-n", str(LAUNCHER)], check=True)
    s = LAUNCHER.read_text(encoding="utf-8")
    # se2_map_merge_node must start before the Nav2 activation wait loop, since
    # every global_costmap static layer waits for /<ns>/shared_map.
    i_merge = s.index("ros2 run se2_map_merge se2_map_merge_node")
    i_wait = s.index("Waiting for /robot1 and /robot2 SLAM & Nav2 stacks")
    assert i_merge < i_wait
    # The wait loop also watches the merger process.
    assert "se2_map_merge_node exited early" in s
    # Legacy ablation falls back to each robot's own SLAM map.
    assert "s/map_topic: shared_map/map_topic: map/" in s
    i_rewrite = s.index("s/map_topic: shared_map/map_topic: map/")
    i_bringup = s.index("exec ros2 launch nav2_bringup bringup_launch.py")
    assert i_rewrite < i_bringup


def test_launcher_spawn_is_deterministic_per_seed():
    s = LAUNCHER.read_text(encoding="utf-8")
    assert "shuf -i" not in s
    start = s.index("pick_two_stations() {")
    end = s.index("\n}\n", start) + 3
    fn = s[start:end]
    script = fn + 'for seed in 1 2 3; do SPAWN_SEED=$seed; pick_two_stations 9; echo "$idx1 $idx2"; done\n'
    out1 = subprocess.run(["bash", "-c", script], capture_output=True, text=True, check=True).stdout
    out2 = subprocess.run(["bash", "-c", script], capture_output=True, text=True, check=True).stdout
    assert out1 == out2
    for line in out1.strip().splitlines():
        a, b = map(int, line.split())
        assert a != b and 0 <= a < 9 and 0 <= b < 9
