"""Hierarchical Local Open-TSP tour building, ownership assignment, and Global ROI bidding."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import wrap_angle

DistanceFn = Callable[[float, float, float, float], float]


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


@dataclass
class TourParams:
    max_waypoints: int = 6
    min_spacing: float = 1.2
    max_tour_length: float = 10.0
    score_dist_weight: float = 1.0
    two_opt_passes: int = 2


@dataclass
class AgentState:
    namespace: str
    x: float
    y: float
    yaw: float
    idle: bool
    component_label: int = -1


@dataclass
class Ownership:
    owner: Optional[str]
    pool: str  # 'local' or 'global'
    distance: float
    reason: str = ""


@dataclass
class RoiParams:
    distance_offset: float = 1.0
    roi_scale: float = 5.0
    min_roi: float = 0.05
    max_distance: float = 0.0
    orientation_scale: float = 0.5


@dataclass
class UtilityParams:
    gain_scale: float = 1.0
    potential_scale: float = 2.0
    orientation_scale: float = 0.5
    min_robot_clearance: float = 0.35
    long_dist_threshold: float = 5.0
    long_dist_penalty: float = 2.5


def nearest_neighbor_order(
    points: np.ndarray,
    start: np.ndarray,
    dist_matrix: Optional[np.ndarray] = None,
    start_dists: Optional[np.ndarray] = None,
) -> List[int]:
    """Open-path nearest-neighbor ordering from a fixed start point.

    Supports optional precomputed geodesic ``dist_matrix`` (N, N) and
    ``start_dists`` (N,) so M3 can swap Euclidean distances seamlessly.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    n = len(pts)
    if n == 0:
        return []
    remaining = set(range(n))
    order: List[int] = []
    if dist_matrix is not None and start_dists is not None:
        j = min(remaining, key=lambda k: (float(start_dists[k]), k))
        order.append(j)
        remaining.remove(j)
        while remaining:
            prev = order[-1]
            j = min(remaining, key=lambda k: (float(dist_matrix[prev, k]), k))
            order.append(j)
            remaining.remove(j)
        return order

    cur = np.asarray(start, dtype=np.float64).reshape(2)
    while remaining:
        j = min(remaining, key=lambda k: (float(np.linalg.norm(pts[k] - cur)), k))
        order.append(j)
        remaining.remove(j)
        cur = pts[j]
    return order


def path_length(
    order: Sequence[int],
    points: np.ndarray,
    start: np.ndarray,
    dist_matrix: Optional[np.ndarray] = None,
    start_dists: Optional[np.ndarray] = None,
) -> float:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if not order:
        return 0.0
    if dist_matrix is not None and start_dists is not None:
        total = float(start_dists[order[0]])
        for a, b in zip(order[:-1], order[1:]):
            total += float(dist_matrix[a, b])
        return total

    cur = np.asarray(start, dtype=np.float64).reshape(2)
    total = 0.0
    for idx in order:
        total += float(np.linalg.norm(pts[idx] - cur))
        cur = pts[idx]
    return total


def open_path_2opt(
    order: Sequence[int],
    points: np.ndarray,
    start: np.ndarray,
    max_passes: int = 2,
    dist_matrix: Optional[np.ndarray] = None,
    start_dists: Optional[np.ndarray] = None,
) -> List[int]:
    """2-opt local improvement for an open path with fixed start and free end."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    order = list(order)
    m = len(order)
    if m < 2:
        return order
    start = np.asarray(start, dtype=np.float64).reshape(2)

    def edge_cost(u_idx: Optional[int], v_idx: int) -> float:
        if dist_matrix is not None and start_dists is not None:
            return float(start_dists[v_idx]) if u_idx is None else float(dist_matrix[u_idx, v_idx])
        p_u = start if u_idx is None else pts[u_idx]
        return float(np.linalg.norm(p_u - pts[v_idx]))

    for _ in range(max(0, max_passes)):
        improved = False
        for i in range(m - 1):
            a_idx = None if i == 0 else order[i - 1]
            b_idx = order[i]
            for j in range(i + 1, m):
                c_idx = order[j]
                d_idx = order[j + 1] if j + 1 < m else None
                before = edge_cost(a_idx, b_idx)
                after = edge_cost(a_idx, c_idx)
                if d_idx is not None:
                    before += edge_cost(c_idx, d_idx)
                    after += edge_cost(b_idx, d_idx)
                if after + 1e-9 < before:
                    order[i : j + 1] = reversed(order[i : j + 1])
                    improved = True
        if not improved:
            break
    return order


def build_local_tour(
    candidates: Sequence,
    start_xy,
    params: TourParams,
    distance_fn: DistanceFn = euclidean_distance,
    start_yaw: Optional[float] = None,
    orientation_weight: float = 0.5,
) -> List:
    """Select and order near-field candidates into a bounded open TSP tour."""
    if not candidates:
        return []
    sx, sy = float(start_xy[0]), float(start_xy[1])

    def _cand_score(c) -> float:
        d = distance_fn(sx, sy, float(c.x), float(c.y))
        s = float(getattr(c, "total_gain", 0.0)) - params.score_dist_weight * d
        if start_yaw is not None and orientation_weight > 0.0 and d > 0.25:
            ang = abs(wrap_angle(math.atan2(float(c.y) - sy, float(c.x) - sx) - start_yaw))
            s -= orientation_weight * ang
        return s

    ranked = sorted(
        candidates,
        key=lambda c: (
            -_cand_score(c),
            float(c.x),
            float(c.y),
        ),
    )

    selected: List = []
    for cand in ranked:
        if len(selected) >= params.max_waypoints:
            break
        if any(
            math.hypot(cand.x - s.x, cand.y - s.y) < params.min_spacing
            for s in selected
        ):
            continue
        selected.append(cand)
    if not selected:
        return []

    points = np.array([[c.x, c.y] for c in selected], dtype=np.float64)
    start = np.array([sx, sy], dtype=np.float64)

    dist_matrix: Optional[np.ndarray] = None
    start_dists: Optional[np.ndarray] = None
    if distance_fn is not euclidean_distance:
        geo_obj = getattr(distance_fn, "__self__", None)
        if geo_obj is not None and hasattr(geo_obj, "precompute_pairwise") and len(selected) > 1:
            geo_obj.precompute_pairwise(
                [(sx, sy)] + [(float(c.x), float(c.y)) for c in selected],
                max_dist_m=max(params.max_tour_length * 1.5, 12.0),
            )
        n = len(selected)
        start_dists = np.array(
            [distance_fn(sx, sy, float(c.x), float(c.y)) for c in selected],
            dtype=np.float64,
        )
        dist_matrix = np.zeros((n, n), dtype=np.float64)
        for i in range(n):
            for j in range(i + 1, n):
                d = distance_fn(
                    float(selected[i].x),
                    float(selected[i].y),
                    float(selected[j].x),
                    float(selected[j].y),
                )
                dist_matrix[i, j] = d
                dist_matrix[j, i] = d

    order = nearest_neighbor_order(points, start, dist_matrix, start_dists)
    order = open_path_2opt(
        order, points, start, params.two_opt_passes, dist_matrix, start_dists
    )

    while (
        len(order) > 1
        and path_length(order, points, start, dist_matrix, start_dists)
        > params.max_tour_length
    ):
        order.pop()
    return [selected[i] for i in order]


def assign_ownership(
    agents: Sequence[AgentState],
    candidates: Sequence,
    local_radius: float,
    distance_fn: DistanceFn = euclidean_distance,
) -> List[Ownership]:
    """Assign each candidate to a Local owner or to the Global pool.

    A candidate is Local only when its nearest robot is idle, within
    ``local_radius`` and in the same 4-connected free-space component.
    """
    results: List[Ownership] = []
    valid_agents = [a for a in agents if math.isfinite(a.x) and math.isfinite(a.y)]
    for cand in candidates:
        cand_component = int(getattr(cand, "component_label", -1))
        if not valid_agents:
            results.append(Ownership(None, "global", float("inf"), "no valid pose"))
            continue

        nearest = min(
            valid_agents,
            key=lambda a: (
                distance_fn(a.x, a.y, float(cand.x), float(cand.y)),
                a.namespace,
            ),
        )
        dist = distance_fn(nearest.x, nearest.y, float(cand.x), float(cand.y))
        same_component = (
            cand_component < 0
            or nearest.component_label < 0
            or cand_component == nearest.component_label
        )
        if nearest.idle and dist <= local_radius and same_component:
            results.append(Ownership(nearest.namespace, "local", dist, "nearest idle"))
        else:
            results.append(Ownership(None, "global", dist, "not local-eligible"))
    return results


def global_bid(
    agent: AgentState,
    candidate,
    params: RoiParams,
    distance_fn: DistanceFn = euclidean_distance,
    other_agents: Sequence[AgentState] = (),
    other_goals: Sequence[Tuple[float, float]] = (),
) -> Tuple[float, float, float]:
    """Return (bid, roi, distance) for a Global candidate with Voronoi wing preference."""
    dist = distance_fn(agent.x, agent.y, float(candidate.x), float(candidate.y))
    gain = float(
        getattr(candidate, "bid_gain", 0.0) or getattr(candidate, "total_gain", 0.0)
    )
    roi = gain / max(dist + params.distance_offset, 1e-6)
    heading = abs(
        wrap_angle(math.atan2(candidate.y - agent.y, candidate.x - agent.x) - agent.yaw)
    )
    # Spatial Voronoi division: discourage crossing into another robot's wing when
    # a teammate is substantially closer to this global candidate, or chasing the
    # same branch/direction as an already-dispatched teammate goal.
    voronoi_penalty = 0.0
    if other_agents and math.isfinite(dist):
        min_other = min(
            (
                distance_fn(o.x, o.y, float(candidate.x), float(candidate.y))
                for o in other_agents
                if o.namespace != agent.namespace and math.isfinite(o.x) and math.isfinite(o.y)
            ),
            default=float("inf"),
        )
        if math.isfinite(min_other) and dist > min_other + 3.0:
            voronoi_penalty += 0.60 * params.roi_scale * roi + 0.08 * (dist - min_other)
    if other_goals and math.isfinite(dist):
        min_gdist = min(
            (
                math.hypot(float(candidate.x) - gx, float(candidate.y) - gy)
                for gx, gy in other_goals
            ),
            default=float("inf"),
        )
        if min_gdist < 8.0:
            voronoi_penalty += 0.55 * params.roi_scale * roi * (1.0 - min_gdist / 8.0)
    bid = params.roi_scale * roi - params.orientation_scale * heading - voronoi_penalty
    return bid, roi, dist


def exclude_near_waypoints(
    candidates: Sequence, waypoints: Sequence[Tuple[float, float]], radius: float
) -> List:
    """Drop candidates too close to any still-active tour waypoint."""
    if not waypoints:
        return list(candidates)
    return [
        c
        for c in candidates
        if all(math.hypot(c.x - wx, c.y - wy) >= radius for wx, wy in waypoints)
    ]


def filter_corridor_safe_candidates(
    robot_ns: str,
    robot_xy: Tuple[float, float],
    candidates: Sequence,
    corridor_mgr,
    path_fn: Callable[[Tuple[float, float], Tuple[float, float]], Tuple[float, np.ndarray]],
) -> Tuple[List, Optional[Tuple[int, str]]]:
    """Partition candidates by topological corridor-mutex conflicts.

    Returns ``(safe_candidates, first_conflict)``. When ``safe_candidates`` is
    non-empty, the robot switches cleanly to an alternate branch avoiding any
    narrow bottleneck reserved by a higher-priority teammate. When all
    candidates require crossing an occupied bottleneck, ``safe_candidates`` is
    empty and ``first_conflict = (corridor_id, owner_ns)`` signals the
    coordinator to stage the robot at a spacious Yield Point.
    """
    reservations = getattr(corridor_mgr, "reservations", None)
    if corridor_mgr is None or not reservations:
        return list(candidates), None
    other_cids = [cid for cid, holder in reservations.items() if holder[0] != robot_ns]
    if not other_cids:
        return list(candidates), None

    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    # Fast spatial skip: if every teammate-reserved bottleneck is far away, avoid running A*
    segments = getattr(corridor_mgr, "segments", None) or {}
    params = getattr(corridor_mgr, "params", None)
    near_rad = max(6.0, 2.0 * float(getattr(params, "reserve_lookahead_m", 4.0)))
    if segments and all(
        (cid not in segments)
        or math.hypot(rx - segments[cid].centroid_x, ry - segments[cid].centroid_y) > near_rad
        for cid in other_cids
    ):
        return list(candidates), None

    safe: List = []
    first_conflict: Optional[Tuple[int, str]] = None
    # Bound per-cycle A* path checks on large maps to the nearest 16 candidates
    ordered = (
        sorted(candidates, key=lambda c: math.hypot(float(c.x) - rx, float(c.y) - ry))[:16]
        if len(candidates) > 16
        else candidates
    )
    for cand in ordered:
        cx, cy = float(cand.x), float(cand.y)
        dist_m, path_xy = path_fn((rx, ry), (cx, cy))
        if not math.isfinite(dist_m) or len(path_xy) == 0:
            continue
        conflict = corridor_mgr.blocking_corridor_for_path(robot_ns, path_xy, origin_xy=(rx, ry))
        if conflict is None:
            safe.append(cand)
        elif first_conflict is None:
            first_conflict = conflict
    return safe, first_conflict


def calculate_legacy_utility(
    agent: AgentState,
    all_agents: Sequence[AgentState],
    cluster,
    res: float,
    params: UtilityParams,
) -> float:
    """Legacy 2D utility used for ablation baseline E1 and liveness fallback."""
    dx = cluster.world_x - agent.x
    dy = cluster.world_y - agent.y
    dist = math.hypot(dx, dy)

    if any(
        math.hypot(cluster.world_x - r.x, cluster.world_y - r.y)
        < params.min_robot_clearance
        for r in all_agents
    ):
        return -float("inf")

    heading_to_goal = math.atan2(dy, dx)
    angle_diff = abs(wrap_angle(heading_to_goal - agent.yaw))

    gain = float(getattr(cluster, "total_gain", 0.0))
    if gain <= 0.0:
        capped_size = min(cluster.size, 1500)
        gain = params.gain_scale * capped_size * res

    dist_cost = dist
    if dist > params.long_dist_threshold:
        dist_cost += (dist - params.long_dist_threshold) * params.long_dist_penalty

    return (
        gain
        - params.potential_scale * dist_cost
        - params.orientation_scale * angle_diff
    )
