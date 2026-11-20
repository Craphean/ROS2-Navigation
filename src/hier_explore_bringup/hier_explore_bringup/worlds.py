"""Central registry of the 6 Lite-HMRE benchmark worlds and vetted N=1..4 spawn pools.

Every default formation (spawn_seed == 0) and random inspection station pool
(spawn_seed > 0) is verified against the world's 2D GT occupancy grid
(maps/gt/<world>/gt_2d.npz) for:
  - Obstacle clearance >= 0.50 m (station pool >= 0.85 m)
  - Pairwise inter-robot separation >= 0.90 m (no spawn overlaps for N=1..4)
  - Deterministic sampling per (world, num_robots, spawn_seed) across methods.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class SpawnPose:
    x: float
    y: float
    yaw: float
    name: str = ""

    def as_list(self) -> List[float]:
        return [float(self.x), float(self.y), float(self.yaw)]


@dataclass(frozen=True)
class WorldSpec:
    name: str
    rel_world_path: str  # relative to repo root, OR "pkg:<pkg>/<rel>"
    gt_seed_xy: Tuple[float, float]
    default_spawns: Tuple[SpawnPose, ...]  # 4 curated non-overlapping portal/corridor poses
    station_pool: Tuple[SpawnPose, ...]    # >= 6 spread-out vetted inspection stations
    gt_sample_step: float = 0.05
    gt_voxel_size: float = 0.02
    gt_resolution_2d: float = 0.05
    aliases: Tuple[str, ...] = ()


WORLD_SPECS: Dict[str, WorldSpec] = {
    "house": WorldSpec(
        name="house",
        rel_world_path="worlds/house.sdf.xacro",
        gt_seed_xy=(-3.0, -2.0),
        default_spawns=(
            SpawnPose(0.00, -0.50, 1.57, "HALLWAY_SOUTH"),
            SpawnPose(0.00, 0.50, 0.00, "HALLWAY_NORTH"),
            SpawnPose(-1.00, -1.15, 0.78, "HALLWAY_SW"),
            SpawnPose(1.35, 1.60, -2.35, "LIVING_NE"),
        ),
        station_pool=(
            SpawnPose(0.03, 0.38, 0.00, "ST_HALL_CENTER"),
            SpawnPose(1.33, -1.37, 1.57, "ST_SE_ROOM"),
            SpawnPose(1.38, 1.78, -1.57, "ST_NE_ROOM"),
            SpawnPose(-0.97, -1.17, 0.00, "ST_SW_HALL"),
            SpawnPose(1.23, -3.27, 1.57, "ST_SOUTH_STUDY"),
            SpawnPose(-3.97, -1.12, 0.00, "ST_WEST_DINING"),
            SpawnPose(3.63, -1.07, 3.14, "ST_EAST_BEDROOM"),
            SpawnPose(-3.92, 1.08, 0.00, "ST_NW_LIVING"),
            SpawnPose(-1.07, 3.43, -1.57, "ST_NORTH_LOUNGE"),
            SpawnPose(-0.97, -3.47, 1.57, "ST_SOUTH_CORNER"),
        ),
        aliases=("tb3_house", "home"),
    ),
    "hospital": WorldSpec(
        name="hospital",
        rel_world_path="worlds/hospital.sdf.xacro",
        gt_seed_xy=(-6.0, 0.0),
        default_spawns=(
            SpawnPose(-6.00, -0.50, 0.00, "WEST_CORRIDOR_S"),
            SpawnPose(-6.00, 0.50, 0.00, "WEST_CORRIDOR_N"),
            SpawnPose(-4.50, -0.50, 0.00, "WEST_CORRIDOR_MID_S"),
            SpawnPose(-4.50, 0.50, 0.00, "WEST_CORRIDOR_MID_N"),
        ),
        station_pool=(
            SpawnPose(-2.57, -0.57, 0.00, "ST_CENTRAL_HUB"),
            SpawnPose(6.33, 3.48, -1.57, "ST_NE_ICU"),
            SpawnPose(1.63, -3.02, 1.57, "ST_SOUTH_PHARMACY"),
            SpawnPose(-5.82, -2.82, 0.00, "ST_SW_TRIAGE"),
            SpawnPose(5.08, -3.62, 1.57, "ST_SE_CLINIC"),
            SpawnPose(1.68, 0.03, 0.00, "ST_MID_CORRIDOR"),
            SpawnPose(-5.42, 0.03, 0.00, "ST_WEST_CORRIDOR"),
            SpawnPose(-4.22, -4.77, 1.57, "ST_SW_LOBBY"),
            SpawnPose(4.68, -0.02, 3.14, "ST_EAST_CORRIDOR"),
            SpawnPose(-6.77, 2.48, 0.00, "ST_NW_WARD"),
        ),
        aliases=("tb3_hospital", "clinic"),
    ),
    "mine": WorldSpec(
        name="mine",
        rel_world_path="worlds/mine.sdf.xacro",
        gt_seed_xy=(-19.0, 0.0),
        default_spawns=(
            SpawnPose(-20.50, -0.70, 0.00, "PORTAL_S"),
            SpawnPose(-20.50, 0.70, 0.00, "PORTAL_N"),
            SpawnPose(-19.00, -0.70, 0.00, "DRIFT_W_S"),
            SpawnPose(-19.00, 0.70, 0.00, "DRIFT_W_N"),
        ),
        station_pool=(
            SpawnPose(-19.20, 0.00, 0.00, "ST_MINE_PORTAL"),
            SpawnPose(-16.00, 0.00, 0.00, "ST_DRIFT_WEST"),
            SpawnPose(-12.00, 0.00, 0.00, "ST_CROSS_A_JUNCTION"),
            SpawnPose(-12.00, 5.50, 1.57, "ST_CROSS_A_NORTH"),
            SpawnPose(-12.00, 11.50, 1.57, "ST_CHAMBER_NORTH"),
            SpawnPose(0.00, 0.00, 0.00, "ST_JUNCTION_MID"),
            SpawnPose(10.00, -5.50, -1.57, "ST_CROSS_B_SOUTH"),
            SpawnPose(10.00, -11.50, -1.57, "ST_CHAMBER_SOUTH"),
            SpawnPose(19.00, 0.00, 3.14, "ST_EAST_FACE"),
        ),
        aliases=("coalmine", "mine_tunnel"),
    ),
    "warehouse": WorldSpec(
        name="warehouse",
        rel_world_path="pkg:nav2_minimal_tb4_sim/worlds/warehouse.sdf",
        gt_seed_xy=(2.0, -19.65),
        default_spawns=(
            SpawnPose(2.00, -19.65, 1.57, "SOUTH_DOCK_1"),
            SpawnPose(0.50, -19.65, 1.57, "SOUTH_DOCK_2"),
            SpawnPose(3.50, -19.65, 1.57, "SOUTH_DOCK_3"),
            SpawnPose(2.00, -18.00, 1.57, "SOUTH_DOCK_4"),
        ),
        station_pool=(
            SpawnPose(2.00, -19.65, 1.57, "ST_SOUTH_DOCK_1"),
            SpawnPose(0.50, -19.65, 1.57, "ST_SOUTH_DOCK_2"),
            SpawnPose(-11.50, -19.65, 1.57, "ST_SW_AISLE"),
            SpawnPose(8.75, -19.65, 1.57, "ST_SE_AISLE"),
            SpawnPose(-6.25, -10.75, 1.57, "ST_SW_RACK"),
            SpawnPose(8.75, -10.75, 1.57, "ST_SE_RACK"),
            SpawnPose(-6.25, 6.00, 0.00, "ST_CENTRAL_WEST"),
            SpawnPose(1.60, 6.00, 0.00, "ST_CENTRAL_MID"),
            SpawnPose(8.75, 6.00, 0.00, "ST_CENTRAL_EAST"),
            SpawnPose(-9.00, 13.50, 1.57, "ST_NW_BAY"),
        ),
        aliases=("tb4_warehouse",),
    ),
    "cmu_indoor": WorldSpec(
        name="cmu_indoor",
        rel_world_path="worlds/cmu_indoor.sdf.xacro",
        gt_seed_xy=(0.0, 0.0),
        default_spawns=(
            SpawnPose(0.00, -0.55, 0.00, "INDOOR_ENTRY_S"),
            SpawnPose(0.00, 0.55, 0.00, "INDOOR_ENTRY_N"),
            SpawnPose(1.60, -0.55, 0.00, "INDOOR_CORRIDOR_S"),
            SpawnPose(1.60, 0.55, 0.00, "INDOOR_CORRIDOR_N"),
        ),
        station_pool=(
            SpawnPose(0.00, 0.00, 0.00, "ST_WEST_PORTAL"),
            SpawnPose(10.30, -0.30, 0.00, "ST_MAIN_WEST"),
            SpawnPose(23.10, -0.10, 0.00, "ST_MAIN_MID"),
            SpawnPose(13.60, 9.80, 1.57, "ST_NORTH_WING_A"),
            SpawnPose(9.30, -14.50, -1.57, "ST_SOUTH_HALL"),
            SpawnPose(23.90, 24.80, 1.57, "ST_NORTH_WING_B"),
            SpawnPose(8.80, 42.50, 0.00, "ST_NW_ATRIUM"),
            SpawnPose(47.80, -6.70, 0.00, "ST_EAST_CORRIDOR"),
            SpawnPose(43.10, -21.20, 1.57, "ST_SE_ROOM"),
            SpawnPose(54.30, 12.80, 0.00, "ST_EAST_HUB"),
        ),
        gt_sample_step=0.08,
        gt_voxel_size=0.05,
        gt_resolution_2d=0.10,
        aliases=("indoor", "aede_indoor"),
    ),
    "cmu_tunnel": WorldSpec(
        name="cmu_tunnel",
        rel_world_path="worlds/cmu_tunnel.sdf.xacro",
        gt_seed_xy=(0.0, 0.0),
        default_spawns=(
            SpawnPose(-2.00, -0.60, 0.00, "TUNNEL_PORTAL_S"),
            SpawnPose(-2.00, 0.60, 0.00, "TUNNEL_PORTAL_N"),
            SpawnPose(-3.60, -0.60, 0.00, "TUNNEL_STAGE_S"),
            SpawnPose(-3.60, 0.60, 0.00, "TUNNEL_STAGE_N"),
        ),
        station_pool=(
            SpawnPose(-2.00, 0.00, 0.00, "ST_TUNNEL_PORTAL"),
            SpawnPose(-5.05, 1.20, 0.00, "ST_WEST_CHAMBER_S"),
            SpawnPose(-5.75, 16.70, 0.00, "ST_WEST_CHAMBER_M"),
            SpawnPose(-6.45, 23.90, -1.57, "ST_WEST_CHAMBER_N"),
            SpawnPose(7.35, 14.20, 0.00, "ST_NORTH_DRIFT_W"),
            SpawnPose(55.75, 14.00, 0.00, "ST_NORTH_DRIFT_E"),
            SpawnPose(68.05, 13.60, 0.00, "ST_JUNCTION_EAST"),
            SpawnPose(67.05, 27.10, -1.57, "ST_BRANCH_NE"),
            SpawnPose(75.45, -1.30, 3.14, "ST_SOUTH_DRIFT_E"),
            SpawnPose(76.85, 8.40, 1.57, "ST_CROSS_CUT_E"),
        ),
        gt_sample_step=0.08,
        gt_voxel_size=0.08,
        gt_resolution_2d=0.10,
        aliases=("tunnel", "aede_tunnel"),
    ),
}

BENCHMARK_WORLDS: Tuple[str, ...] = (
    "house",
    "hospital",
    "mine",
    "warehouse",
    "cmu_indoor",
    "cmu_tunnel",
)

_ALIAS_MAP: Dict[str, str] = {}
for _name, _spec in WORLD_SPECS.items():
    _ALIAS_MAP[_name] = _name
    for _al in _spec.aliases:
        _ALIAS_MAP[_al.lower()] = _name


def find_repo_root(start: Optional[Path] = None) -> Path:
    """Locate repository root containing config/ and worlds/ (works under --symlink-install or install/)."""
    candidates: List[Path] = []
    if start is not None:
        candidates.append(Path(start).resolve())
    candidates.append(Path(__file__).resolve())
    candidates.append(Path.cwd().resolve())
    candidates.append(Path("/ws"))
    for base in candidates:
        for parent in (base, *base.parents):
            if (parent / "config" / "coordinator_params.yaml").is_file() and (parent / "worlds").is_dir():
                return parent
    return Path.cwd().resolve()


def canonical_world_name(world: str) -> str:
    key = world.strip().lower()
    return _ALIAS_MAP.get(key, world.strip())


def resolve_world_file(world: str, repo_root: Optional[Path] = None) -> Path:
    """Resolve a benchmark world preset or explicit file path to an existing SDF/Xacro path."""
    root = find_repo_root(repo_root)
    canon = canonical_world_name(world)
    if canon in WORLD_SPECS:
        rel = WORLD_SPECS[canon].rel_world_path
        if rel.startswith("pkg:"):
            pkg_and_rel = rel[len("pkg:") :]
            pkg_name, sub = pkg_and_rel.split("/", 1)
            try:
                from ament_index_python.packages import get_package_share_directory

                return Path(get_package_share_directory(pkg_name)) / sub
            except Exception:
                return Path("/opt/ros/lyrical/share") / pkg_name / sub
        return root / rel

    if canon in ("sandbox", "tb3_sandbox"):
        try:
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory("nav2_minimal_tb3_sim")) / "worlds" / "tb3_sandbox.sdf.xacro"
        except Exception:
            return Path("/opt/ros/lyrical/share/nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro")

    if canon in ("empty", "tb3_empty", "tb3_empty_world"):
        try:
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory("nav2_minimal_tb3_sim")) / "worlds" / "tb3_empty_world.sdf.xacro"
        except Exception:
            return Path("/opt/ros/lyrical/share/nav2_minimal_tb3_sim/worlds/tb3_empty_world.sdf.xacro")

    p = Path(world)
    return p if p.is_absolute() else (root / p)


def sample_spawn_poses(
    world: str,
    num_robots: int = 2,
    spawn_seed: int = 0,
    min_separation_m: float = 0.90,
) -> List[SpawnPose]:
    """Return N deterministic, non-overlapping spawn poses for ``world``.

    - ``spawn_seed == 0``: returns the first N poses of the world's curated
      default formation (matching the fixed portal/corridor spawns).
    - ``spawn_seed > 0``: deterministically samples N distinct stations from
      ``station_pool`` using ``random.Random(spawn_seed)`` enforcing pairwise
      separation >= ``min_separation_m``.
    """
    if num_robots < 1 or num_robots > 4:
        raise ValueError(f"num_robots must be in [1, 4], got {num_robots}")

    canon = canonical_world_name(world)
    spec = WORLD_SPECS.get(canon)
    if spec is None:
        # Generic fallback formation for sandbox / custom worlds
        defaults = [
            SpawnPose(-2.0, -0.5, 0.0, "DEFAULT_1"),
            SpawnPose(0.0, 0.5, 0.0, "DEFAULT_2"),
            SpawnPose(-2.0, 0.5, 0.0, "DEFAULT_3"),
            SpawnPose(0.0, -0.5, 0.0, "DEFAULT_4"),
        ]
        return defaults[:num_robots]

    if spawn_seed <= 0:
        return list(spec.default_spawns[:num_robots])

    rng = random.Random(int(spawn_seed))
    pool = list(spec.station_pool)
    if len(pool) < num_robots:
        raise ValueError(f"World '{canon}' has only {len(pool)} stations for {num_robots} robots")

    # Deterministic shuffle + greedy separation enforcement (guarantees zero overlap)
    indices = list(range(len(pool)))
    rng.shuffle(indices)
    chosen: List[SpawnPose] = []
    for idx in indices:
        cand = pool[idx]
        if all(math.hypot(cand.x - c.x, cand.y - c.y) >= min_separation_m for c in chosen):
            chosen.append(cand)
            if len(chosen) == num_robots:
                return chosen

    raise RuntimeError(
        f"Could not pick {num_robots} spawns separated by >= {min_separation_m} m in world '{canon}'"
    )
