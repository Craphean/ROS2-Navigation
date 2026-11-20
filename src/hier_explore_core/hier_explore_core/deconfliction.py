"""Two-layer multi-robot deconfliction: kinematic teammate clouds + corridor mutex reservation.

Layer 1 (Kinematic Local Costmap Injection @ 5 Hz):
  Generates per-robot `/<ns>/teammates_cloud` (PointCloud2 in the robot's local
  `map` frame) containing:
    - Teammate footprint rings (radius = robot_radius + buffer)
    - Future 2.0 s predicted trajectory points sampled from teammate plans
    - Outer raytrace-clearing ring beyond obstacle_max_range so Nav2 VoxelLayer
      clears vacated teammate cells within one cycle (zero ghost trails).

Layer 2 (Topological Corridor Mutex Reservation):
  Uses C++ `_geom_kernel.extract_corridor_bottlenecks` to identify narrow
  doorways and tight tunnel bottlenecks (`w(p) < 2*r_robot + 0.35m`), tracks
  priority-ordered mutex ownership, diverts lower-priority robots to alternate
  branches when available, or stages them at spacious Yield Points to eliminate
  head-on deadlocks and inter-robot collisions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

from .geom_kernel import extract_corridor_bottlenecks_cpp
from .geometry import se2_apply
from .occupancy_view import OccupancyView


# ---------------------------------------------------------------------------
# Layer 1: Kinematic Teammate Footprint + 2 s Predicted Trajectory Cloud
# ---------------------------------------------------------------------------


@dataclass
class TeammateCloudParams:
    footprint_radius: float = 0.26
    footprint_samples: int = 12
    predict_horizon_s: float = 2.0
    nominal_speed_mps: float = 0.42
    predict_step_m: float = 0.16
    predict_ring_radius: float = 0.18
    predict_ring_samples: int = 6
    max_teammate_range: float = 3.5
    ego_safe_radius: float = 0.36
    cloud_z: float = 0.17
    # Sparse ring placed strictly between obstacle_max_range (2.5m) and
    # raytrace_max_range (3.0m) so Nav2 VoxelLayer raytrace-clears stale cells
    # without marking the outer ring itself.
    clear_ring_radius: float = 2.85
    clear_ring_samples: int = 24


def sample_plan_horizon(
    plan_xy: Sequence[Tuple[float, float]],
    origin_xy: Tuple[float, float],
    max_length_m: float,
    step_m: float = 0.16,
) -> List[Tuple[float, float]]:
    """Subsample a world-frame path up to `max_length_m` ahead of `origin_xy`."""
    if not plan_xy or max_length_m <= 0.0:
        return []
    ox, oy = float(origin_xy[0]), float(origin_xy[1])
    # Find closest point on plan to robot's current pose so stale prefix is skipped
    best_i = 0
    best_d2 = float("inf")
    for i, (px, py) in enumerate(plan_xy):
        d2 = (px - ox) ** 2 + (py - oy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i

    samples: List[Tuple[float, float]] = []
    acc = 0.0
    next_emit = step_m
    prev_x, prev_y = float(plan_xy[best_i][0]), float(plan_xy[best_i][1])
    for i in range(best_i + 1, len(plan_xy)):
        cx, cy = float(plan_xy[i][0]), float(plan_xy[i][1])
        seg = math.hypot(cx - prev_x, cy - prev_y)
        if seg < 1e-6:
            continue
        while acc + seg >= next_emit and next_emit <= max_length_m + 1e-6:
            t = (next_emit - acc) / seg
            sx = prev_x + t * (cx - prev_x)
            sy = prev_y + t * (cy - prev_y)
            samples.append((sx, sy))
            next_emit += step_m
        acc += seg
        prev_x, prev_y = cx, cy
        if acc >= max_length_m:
            break
    return samples


def build_teammate_cloud_points(
    ego_ns: str,
    robot_poses_world: Dict[str, Tuple[float, float, float]],
    robot_plans_world: Dict[str, Sequence[Tuple[float, float]]],
    tf_world_to_local: Tuple[float, float, float],
    params: Optional[TeammateCloudParams] = None,
) -> np.ndarray:
    """Build (N, 3) float32 points in ego's local `map` frame for `/<ego>/teammates_cloud`."""
    cfg = params or TeammateCloudParams()
    ego_pose = robot_poses_world.get(ego_ns)
    if ego_pose is None:
        return np.empty((0, 3), dtype=np.float32)

    ex, ey, _ = ego_pose
    world_xy: List[Tuple[float, float]] = []

    fp_angles = np.linspace(0.0, 2.0 * math.pi, max(4, cfg.footprint_samples), endpoint=False)
    pred_angles = np.linspace(0.0, 2.0 * math.pi, max(4, cfg.predict_ring_samples), endpoint=False)
    horizon_m = max(0.0, cfg.predict_horizon_s * cfg.nominal_speed_mps)

    for ns, (rx, ry, ryaw) in robot_poses_world.items():
        if ns == ego_ns:
            continue
        if not (math.isfinite(rx) and math.isfinite(ry)):
            continue
        dist_to_ego = math.hypot(rx - ex, ry - ey)
        if dist_to_ego > cfg.max_teammate_range + horizon_m:
            continue

        # 1. Teammate physical footprint circle (+ core center)
        if dist_to_ego <= cfg.max_teammate_range:
            if dist_to_ego >= cfg.ego_safe_radius:
                world_xy.append((rx, ry))
            for ang in fp_angles:
                px = rx + cfg.footprint_radius * math.cos(float(ang))
                py = ry + cfg.footprint_radius * math.sin(float(ang))
                if math.hypot(px - ex, py - ey) >= cfg.ego_safe_radius:
                    world_xy.append((px, py))

        # 2. Future 2 s trajectory from Nav2 plan (or short heading extrapolation if active)
        plan = robot_plans_world.get(ns) or ()
        if plan:
            traj_pts = sample_plan_horizon(plan, (rx, ry), horizon_m, cfg.predict_step_m)
        else:
            traj_pts = []
        for tx, ty in traj_pts:
            if math.hypot(tx - ex, ty - ey) > cfg.max_teammate_range:
                continue
            if math.hypot(tx - ex, ty - ey) >= cfg.ego_safe_radius:
                world_xy.append((tx, ty))
            for ang in pred_angles:
                px = tx + cfg.predict_ring_radius * math.cos(float(ang))
                py = ty + cfg.predict_ring_radius * math.sin(float(ang))
                if math.hypot(px - ex, py - ey) >= cfg.ego_safe_radius:
                    world_xy.append((px, py))

    # 3. Outer raytrace-clearing ring around ego so VoxelLayer clears vacated teammate cells
    if cfg.clear_ring_radius > 0.0 and cfg.clear_ring_samples > 0:
        clr_angles = np.linspace(0.0, 2.0 * math.pi, cfg.clear_ring_samples, endpoint=False)
        for ang in clr_angles:
            world_xy.append(
                (
                    ex + cfg.clear_ring_radius * math.cos(float(ang)),
                    ey + cfg.clear_ring_radius * math.sin(float(ang)),
                )
            )

    if not world_xy:
        return np.empty((0, 3), dtype=np.float32)

    pts = np.empty((len(world_xy), 3), dtype=np.float32)
    z_val = float(cfg.cloud_z)
    tx_w, ty_w, tyaw_w = tf_world_to_local
    c_yaw = math.cos(tyaw_w)
    s_yaw = math.sin(tyaw_w)
    for i, (wx, wy) in enumerate(world_xy):
        pts[i, 0] = tx_w + c_yaw * wx - s_yaw * wy
        pts[i, 1] = ty_w + s_yaw * wx + c_yaw * wy
        pts[i, 2] = z_val
    return pts


def pack_xyz32_cloud(points_xyz: np.ndarray, frame_id: str, stamp_msg):
    """Pack an (N, 3) float32 array into a tightly packed sensor_msgs/PointCloud2."""
    from sensor_msgs.msg import PointCloud2, PointField

    arr = np.ascontiguousarray(points_xyz, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp_msg
    msg.height = 1
    msg.width = int(len(arr))
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = arr.tobytes()
    return msg


# ---------------------------------------------------------------------------
# Layer 2: Topological Corridor Mutex Reservation
# ---------------------------------------------------------------------------


@dataclass
class CorridorMutexParams:
    robot_radius: float = 0.22
    narrow_margin_m: float = 0.35
    min_segment_cells: int = 4
    yield_search_radius_m: float = 3.0
    yield_min_separation_m: float = 1.0
    proximity_radius_m: float = 0.45
    head_on_detect_dist_m: float = 4.5
    head_on_cos_threshold: float = -0.25
    reservation_ttl_s: float = 15.0


@dataclass
class CorridorInfo:
    id: int
    cell_count: int
    min_half_width_m: float
    length_m: float
    centroid_x: float
    centroid_y: float
    yield_points: List[Tuple[float, float, float]] = field(default_factory=list)


class CorridorMutexManager:
    """Topological narrow-passage bottleneck extractor and priority mutex arbitrator."""

    def __init__(self, params: Optional[CorridorMutexParams] = None) -> None:
        self.params = params or CorridorMutexParams()
        self.labels: Optional[np.ndarray] = None
        self.occ: Optional[OccupancyView] = None
        self.segments: Dict[int, CorridorInfo] = {}
        # corridor_id -> (owner_ns, timestamp_s)
        self.reservations: Dict[int, Tuple[str, float]] = {}

    def update_map(self, occ: OccupancyView) -> Dict[int, CorridorInfo]:
        """Extract narrow corridor bottlenecks E_narrow from the latest OccupancyView."""
        self.occ = occ
        raw = extract_corridor_bottlenecks_cpp(
            occ,
            robot_radius=self.params.robot_radius,
            narrow_margin_m=self.params.narrow_margin_m,
            min_segment_cells=self.params.min_segment_cells,
            yield_search_radius_m=self.params.yield_search_radius_m,
            yield_min_separation_m=self.params.yield_min_separation_m,
        )
        self.labels = np.asarray(raw["labels"], dtype=np.int32)
        self.segments = {}
        for s in raw["segments"]:
            cid = int(s["id"])
            self.segments[cid] = CorridorInfo(
                id=cid,
                cell_count=int(s["cell_count"]),
                min_half_width_m=float(s["min_half_width_m"]),
                length_m=float(s["length_m"]),
                centroid_x=float(s["centroid_x"]),
                centroid_y=float(s["centroid_y"]),
                yield_points=[
                    (float(yp[0]), float(yp[1]), float(yp[2]))
                    for yp in s.get("yield_points", [])
                ],
            )
        return self.segments

    def corridors_near_point(self, x: float, y: float, radius_m: Optional[float] = None) -> Set[int]:
        """Return bottleneck segment IDs overlapping a disk around (x, y)."""
        if self.labels is None or self.occ is None or not self.segments:
            return set()
        rad = self.params.proximity_radius_m if radius_m is None else float(radius_m)
        r0, c0 = self.occ.world_to_cell(x, y)
        rc = max(0, int(math.ceil(rad / self.occ.resolution)))
        r_min = max(0, r0 - rc)
        r_max = min(self.occ.height, r0 + rc + 1)
        c_min = max(0, c0 - rc)
        c_max = min(self.occ.width, c0 + rc + 1)
        if r_min >= r_max or c_min >= c_max:
            return set()
        sub = self.labels[r_min:r_max, c_min:c_max]
        vals = np.unique(sub[sub > 0])
        return {int(v) for v in vals.tolist()}

    def corridors_on_path(self, path_xy: Sequence[Tuple[float, float]]) -> List[int]:
        """Return ordered unique bottleneck segment IDs traversed by `path_xy`."""
        if self.labels is None or self.occ is None or not self.segments or len(path_xy) == 0:
            return []
        seen: Set[int] = set()
        ordered: List[int] = []
        h, w = self.occ.height, self.occ.width
        ox, oy, res = self.occ.origin_x, self.occ.origin_y, self.occ.resolution
        for pt in path_xy:
            c = int(math.floor((float(pt[0]) - ox) / res))
            r = int(math.floor((float(pt[1]) - oy) / res))
            if 0 <= r < h and 0 <= c < w:
                lbl = int(self.labels[r, c])
                if lbl > 0 and lbl not in seen:
                    seen.add(lbl)
                    ordered.append(lbl)
        return ordered

    def update_reservations(
        self,
        robot_poses: Dict[str, Tuple[float, float, float]],
        active_paths: Dict[str, Sequence[Tuple[float, float]]],
        now: float,
    ) -> Dict[int, str]:
        """Rebuild priority-ordered corridor mutex reservations for this cycle.

        Priority:
          1. A robot already physically inside a bottleneck always holds it.
          2. Otherwise, the navigating robot closer to the bottleneck centroid
             (tie-broken lexicographically by namespace) acquires the mutex.
        """
        new_res: Dict[int, str] = {}
        if not self.segments:
            self.reservations = {}
            return new_res

        # Pass 1: Robots physically inside/entering a bottleneck own it immediately
        for ns in sorted(robot_poses.keys()):
            rx, ry, _ = robot_poses[ns]
            for cid in sorted(self.corridors_near_point(rx, ry)):
                new_res.setdefault(cid, ns)

        # Pass 2: Robots whose active path crosses an unclaimed bottleneck
        candidates: List[Tuple[float, str, int]] = []
        for ns, path in active_paths.items():
            if not path or ns not in robot_poses:
                continue
            rx, ry, _ = robot_poses[ns]
            for cid in self.corridors_on_path(path):
                if cid in new_res:
                    continue
                seg = self.segments.get(cid)
                if seg is None:
                    continue
                d = math.hypot(rx - seg.centroid_x, ry - seg.centroid_y)
                candidates.append((d, ns, cid))

        candidates.sort(key=lambda item: (item[0], item[1], item[2]))
        for _d, ns, cid in candidates:
            new_res.setdefault(cid, ns)

        self.reservations = {cid: (ns, now) for cid, ns in new_res.items()}
        return new_res

    def reserve_for_robot(self, robot_ns: str, path_xy: Sequence[Tuple[float, float]], now: float) -> None:
        """Immediately reserve all currently unclaimed bottlenecks along a newly dispatched path."""
        for cid in self.corridors_on_path(path_xy):
            if cid not in self.reservations:
                self.reservations[cid] = (robot_ns, now)

    def blocking_corridor_for_path(
        self, robot_ns: str, path_xy: Sequence[Tuple[float, float]]
    ) -> Optional[Tuple[int, str]]:
        """Return (corridor_id, owner_ns) if `path_xy` enters a bottleneck held by another robot."""
        for cid in self.corridors_on_path(path_xy):
            holder = self.reservations.get(cid)
            if holder is not None and holder[0] != robot_ns:
                return cid, holder[0]
        return None

    def select_yield_point(
        self,
        corridor_id: int,
        robot_xy: Tuple[float, float],
        blocker_xy: Optional[Tuple[float, float]] = None,
    ) -> Optional[Tuple[float, float, float]]:
        """Pick the best spacious Yield Point on the robot's side of `corridor_id`."""
        seg = self.segments.get(corridor_id)
        if seg is None or not seg.yield_points:
            return None

        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        best_yp = None
        best_score = -float("inf")
        for yx, yy, clr in seg.yield_points:
            d_robot = math.hypot(yx - rx, yy - ry)
            # Prefer yield points close to ego, with high clearance, and farther
            # from the oncoming blocker (i.e. on ego's own side of the bottleneck).
            side_bonus = 0.0
            if blocker_xy is not None:
                bx, by = float(blocker_xy[0]), float(blocker_xy[1])
                d_block = math.hypot(yx - bx, yy - by)
                # Vector from corridor centroid to yield point vs robot vs blocker
                vx_y, vy_y = yx - seg.centroid_x, yy - seg.centroid_y
                vx_r, vy_r = rx - seg.centroid_x, ry - seg.centroid_y
                dot_r = vx_y * vx_r + vy_y * vy_r
                side_bonus = (1.5 if dot_r > 0.0 else -2.0) + 0.35 * min(d_block, 4.0)
            score = side_bonus + 0.8 * min(clr, 1.5) - 0.55 * d_robot
            if score > best_score:
                best_score = score
                best_yp = (yx, yy, math.atan2(seg.centroid_y - yy, seg.centroid_x - yx))
        return best_yp

    def detect_head_on_yielders(
        self,
        robot_poses: Dict[str, Tuple[float, float, float]],
        active_paths: Dict[str, Sequence[Tuple[float, float]]],
    ) -> List[Tuple[str, int, str, Optional[Tuple[float, float, float]]]]:
        """Detect active navigating robots heading into a bottleneck reserved by another robot.

        Returns list of `(yielding_ns, corridor_id, owner_ns, yield_pose_xyyaw)`.
        """
        yielders: List[Tuple[str, int, str, Optional[Tuple[float, float, float]]]] = []
        if not self.segments or not self.reservations:
            return yielders

        for ns, path in active_paths.items():
            if not path or ns not in robot_poses:
                continue
            conflict = self.blocking_corridor_for_path(ns, path)
            if conflict is None:
                continue
            cid, owner_ns = conflict
            if owner_ns not in robot_poses:
                continue

            rx, ry, _ = robot_poses[ns]
            ox, oy, _ = robot_poses[owner_ns]
            seg = self.segments.get(cid)
            if seg is None:
                continue

            # Only interrupt when robot is actually approaching the conflicted bottleneck
            dist_to_seg = math.hypot(rx - seg.centroid_x, ry - seg.centroid_y)
            dist_robots = math.hypot(rx - ox, ry - oy)
            if dist_to_seg > self.params.head_on_detect_dist_m and dist_robots > self.params.head_on_detect_dist_m:
                continue

            # If robot is already physically inside the same corridor, do NOT stop it inside!
            if cid in self.corridors_near_point(rx, ry, radius_m=0.25):
                continue

            # Check opposing travel directions (or both heading into the same bottleneck)
            gx, gy = float(path[-1][0]), float(path[-1][1])
            owner_path = active_paths.get(owner_ns) or ()
            opposing = True
            if owner_path:
                ogx, ogy = float(owner_path[-1][0]), float(owner_path[-1][1])
                v1x, v1y = gx - rx, gy - ry
                v2x, v2y = ogx - ox, ogy - oy
                n1 = math.hypot(v1x, v1y)
                n2 = math.hypot(v2x, v2y)
                if n1 > 0.2 and n2 > 0.2:
                    cos_ang = (v1x * v2x + v1y * v2y) / (n1 * n2)
                    # Same-direction convoy with plenty of separation doesn't need yielding
                    if cos_ang > 0.4 and dist_robots > 1.5:
                        opposing = False

            if opposing:
                yp = self.select_yield_point(cid, (rx, ry), (ox, oy))
                yielders.append((ns, cid, owner_ns, yp))
        return yielders
