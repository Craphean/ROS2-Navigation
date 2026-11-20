"""Vectorized 2D frontier extraction, recursive cluster splitting, and safe-goal retraction."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .occupancy_view import OccupancyView
from .voxel_index import InfoGainParams, VoxelColumnIndex


@dataclass
class FrontierParams:
    min_frontier_size: int = 5
    min_clearance: float = 0.55
    blacklist_radius: float = 0.9
    dedupe_radius: float = 0.4
    max_retract_m: float = 1.5
    split_min_spread_m: float = 2.0
    split_recurse_spread_m: float = 2.5
    gain_size_cap: int = 1500


class FrontierCluster:
    def __init__(
        self,
        world_x: float,
        world_y: float,
        size: int,
        grid_r: int,
        grid_c: int,
        yaw_world: float = 0.0,
        gain_2d: float = 0.0,
        gain_3d: float = 0.0,
        total_gain: float = 0.0,
        kind: str = "frontier",
        component_label: int = -1,
    ):
        self.world_x = world_x
        self.world_y = world_y
        self.size = size
        self.grid_r = grid_r
        self.grid_c = grid_c
        self.yaw_world = yaw_world
        self.gain_2d = gain_2d
        self.gain_3d = gain_3d
        self.total_gain = total_gain
        self.kind = kind
        self.component_label = component_label

    @property
    def x(self) -> float:
        return self.world_x

    @property
    def y(self) -> float:
        return self.world_y


def split_cluster_if_large(
    cluster_cells: List[Tuple[int, int]],
    res: float,
    min_size: int = 8,
    min_spread_m: float = 2.0,
    recurse_spread_m: float = 2.5,
) -> List[List[Tuple[int, int]]]:
    """Recursively bisect elongated frontier clusters along their principal axis."""
    if len(cluster_cells) < min_size * 2:
        return [cluster_cells]

    coords = np.array(cluster_cells)
    span = np.ptp(coords, axis=0)
    spread_m = math.hypot(span[0], span[1]) * res
    if spread_m < min_spread_m:
        return [cluster_cells]

    var = np.var(coords, axis=0)
    split_dim = 1 if var[1] > var[0] else 0
    median_val = np.median(coords[:, split_dim])

    part1 = [c for c in cluster_cells if c[split_dim] <= median_val]
    part2 = [c for c in cluster_cells if c[split_dim] > median_val]
    if len(part1) < min_size or len(part2) < min_size:
        return [cluster_cells]

    results: List[List[Tuple[int, int]]] = []
    for part in (part1, part2):
        p_coords = np.array(part)
        p_span = np.ptp(p_coords, axis=0)
        p_spread = math.hypot(p_span[0], p_span[1]) * res
        if p_spread >= recurse_spread_m and len(part) >= min_size * 2:
            results.extend(
                split_cluster_if_large(
                    part, res, min_size, min_spread_m, recurse_spread_m
                )
            )
        else:
            results.append(part)
    return results


def find_safe_goal(
    cluster_cells: Sequence[Tuple[int, int]],
    is_free: np.ndarray,
    obstacle_dt: np.ndarray,
    res: float,
    min_clearance: float = 0.55,
    max_retract_m: float = 1.5,
) -> Optional[Tuple[int, int]]:
    """Retract a frontier centroid into free space with >= min_clearance."""
    height, width = is_free.shape
    min_clearance_cells = min_clearance / res
    max_retract_cells = int(max_retract_m / res)

    mean_r = sum(c[0] for c in cluster_cells) / len(cluster_cells)
    mean_c = sum(c[1] for c in cluster_cells) / len(cluster_cells)

    safe_cells = [
        (r, c)
        for (r, c) in cluster_cells
        if obstacle_dt[r, c] >= min_clearance_cells
    ]
    if safe_cells:
        return min(
            safe_cells,
            key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2,
        )

    sorted_cluster = sorted(
        cluster_cells,
        key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2,
    )
    q: deque = deque()
    visited = set()
    for r, c in sorted_cluster:
        q.append((r, c, 0))
        visited.add((r, c))

    while q:
        r, c, steps = q.popleft()
        if steps > max_retract_cells:
            break
        if obstacle_dt[r, c] >= min_clearance_cells:
            return (r, c)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and (nr, nc) not in visited:
                visited.add((nr, nc))
                if is_free[nr, nc]:
                    q.append((nr, nc, steps + 1))
    return None


def frontier_yaw(occ: OccupancyView, safe_r: int, safe_c: int) -> float:
    """Compute world yaw from a retracted frontier cell toward nearest unknown cell."""
    r0 = max(0, safe_r - 2)
    r1 = min(occ.height, safe_r + 3)
    c0 = max(0, safe_c - 2)
    c1 = min(occ.width, safe_c + 3)
    sub = occ.is_unknown[r0:r1, c0:c1]
    if not sub.any():
        return 0.0
    rr, cc = np.nonzero(sub)
    gr = rr + r0
    gc = cc + c0
    k = int(np.argmin((gr - safe_r) ** 2 + (gc - safe_c) ** 2))
    ux = occ.origin_x + (gc[k] + 0.5) * occ.resolution
    uy = occ.origin_y + (gr[k] + 0.5) * occ.resolution
    wx = occ.origin_x + (safe_c + 0.5) * occ.resolution
    wy = occ.origin_y + (safe_r + 0.5) * occ.resolution
    return math.atan2(uy - wy, ux - wx)


def extract_frontiers(
    occ: OccupancyView,
    params: FrontierParams,
    blacklisted_goals: Sequence[Tuple[float, float]] = (),
) -> List[FrontierCluster]:
    """Vectorized 2D frontier extraction with safe-goal retraction and deduplication."""
    height, width = occ.height, occ.width
    res = occ.resolution

    unknown_padded = np.pad(
        occ.is_unknown, 1, mode="constant", constant_values=False
    )
    lethal_padded = np.pad(
        occ.is_lethal, 1, mode="constant", constant_values=False
    )

    has_unknown_neighbor = np.zeros((height, width), dtype=bool)
    has_lethal_neighbor = np.zeros((height, width), dtype=bool)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            has_unknown_neighbor |= unknown_padded[
                1 + dr : height + 1 + dr, 1 + dc : width + 1 + dc
            ]
            has_lethal_neighbor |= lethal_padded[
                1 + dr : height + 1 + dr, 1 + dc : width + 1 + dc
            ]

    frontier_mask = occ.is_free & has_unknown_neighbor & (~has_lethal_neighbor)
    frontier_coords = np.argwhere(frontier_mask)
    visited = np.zeros((height, width), dtype=bool)
    raw_clusters: List[List[Tuple[int, int]]] = []

    for r, c in frontier_coords:
        if visited[r, c]:
            continue
        cluster_cells: List[Tuple[int, int]] = []
        q = deque([(int(r), int(c))])
        visited[r, c] = True
        while q:
            curr_r, curr_c = q.popleft()
            cluster_cells.append((curr_r, curr_c))
            r_start = max(0, curr_r - 1)
            r_end = min(height, curr_r + 2)
            c_start = max(0, curr_c - 1)
            c_end = min(width, curr_c + 2)
            for nr in range(r_start, r_end):
                for nc in range(c_start, c_end):
                    if frontier_mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        q.append((nr, nc))

        if len(cluster_cells) >= params.min_frontier_size:
            raw_clusters.extend(
                split_cluster_if_large(
                    cluster_cells,
                    res,
                    params.min_frontier_size,
                    params.split_min_spread_m,
                    params.split_recurse_spread_m,
                )
            )

    clusters: List[FrontierCluster] = []
    for sub_cells in raw_clusters:
        safe_cell = find_safe_goal(
            sub_cells,
            occ.is_free,
            occ.obstacle_dt,
            res,
            min_clearance=params.min_clearance,
            max_retract_m=params.max_retract_m,
        )
        if safe_cell is None:
            continue
        safe_r, safe_c = safe_cell
        wx = occ.origin_x + (safe_c + 0.5) * res
        wy = occ.origin_y + (safe_r + 0.5) * res

        if any(
            math.hypot(wx - bx, wy - by) < params.blacklist_radius
            for bx, by in blacklisted_goals
        ):
            continue
        if any(
            math.hypot(wx - c.world_x, wy - c.world_y) < params.dedupe_radius
            for c in clusters
        ):
            continue

        clusters.append(
            FrontierCluster(
                wx,
                wy,
                len(sub_cells),
                safe_r,
                safe_c,
                yaw_world=frontier_yaw(occ, safe_r, safe_c),
                gain_2d=min(len(sub_cells), params.gain_size_cap) * res,
                component_label=int(occ.free_labels[safe_r, safe_c]),
            )
        )
    return clusters


def score_frontiers(
    clusters: Sequence[FrontierCluster],
    resolution: float,
    gain_scale: float,
    gain_3d_scale: float,
    voxel_index: Optional[VoxelColumnIndex],
    info_gain_params: InfoGainParams,
    use_3d_gain: bool = True,
    skip_3d: bool = False,
    gain_size_cap: int = 1500,
) -> None:
    """Populate gain_2d, gain_3d, and total_gain in place on frontier clusters."""
    for c in clusters:
        g3d = 0.0
        if not skip_3d and use_3d_gain and voxel_index is not None:
            stats = voxel_index.window_stats(
                c.world_x,
                c.world_y,
                info_gain_params.window_radius,
                info_gain_params,
            )
            g3d = stats.g3d
        c.gain_2d = min(c.size, gain_size_cap) * resolution
        c.gain_3d = g3d
        c.total_gain = gain_scale * c.gain_2d + gain_3d_scale * g3d
