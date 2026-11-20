"""Vectorized 2D frontier extraction, recursive cluster splitting, and safe-goal retraction."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
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


def _split_coords_if_large(
    coords: np.ndarray,
    res: float,
    min_size: int = 8,
    min_spread_m: float = 2.0,
    recurse_spread_m: float = 2.5,
) -> List[np.ndarray]:
    """Fast NumPy-array recursive bisection of elongated frontier clusters."""
    if len(coords) < min_size * 2:
        return [coords]

    span0 = int(coords[:, 0].max() - coords[:, 0].min())
    span1 = int(coords[:, 1].max() - coords[:, 1].min())
    spread_m = math.hypot(span0, span1) * res
    if spread_m < min_spread_m:
        return [coords]

    var0 = float(np.var(coords[:, 0]))
    var1 = float(np.var(coords[:, 1]))
    split_dim = 1 if var1 > var0 else 0
    median_val = float(np.median(coords[:, split_dim]))

    mask1 = coords[:, split_dim] <= median_val
    part1 = coords[mask1]
    part2 = coords[~mask1]
    if len(part1) < min_size or len(part2) < min_size:
        return [coords]

    results: List[np.ndarray] = []
    for part in (part1, part2):
        ps0 = int(part[:, 0].max() - part[:, 0].min())
        ps1 = int(part[:, 1].max() - part[:, 1].min())
        p_spread = math.hypot(ps0, ps1) * res
        if p_spread >= recurse_spread_m and len(part) >= min_size * 2:
            results.extend(
                _split_coords_if_large(
                    part, res, min_size, min_spread_m, recurse_spread_m
                )
            )
        else:
            results.append(part)
    return results


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
    arr = np.asarray(cluster_cells, dtype=np.int32)
    parts = _split_coords_if_large(arr, res, min_size, min_spread_m, recurse_spread_m)
    if len(parts) == 1 and len(parts[0]) == len(cluster_cells):
        return [cluster_cells]
    return [[(int(r), int(c)) for r, c in p] for p in parts]


def find_safe_goal(
    cluster_cells: Sequence[Tuple[int, int]],
    is_free: np.ndarray,
    obstacle_dt: np.ndarray,
    res: float,
    min_clearance: float = 0.55,
    max_retract_m: float = 1.5,
    free_labels: Optional[np.ndarray] = None,
) -> Optional[Tuple[int, int]]:
    """Retract a frontier centroid into free space with >= min_clearance (vectorized)."""
    height, width = is_free.shape
    min_clearance_cells = min_clearance / res
    max_retract_cells = int(max_retract_m / res)

    arr = np.asarray(cluster_cells, dtype=np.int32)
    mean_r = float(arr[:, 0].mean())
    mean_c = float(arr[:, 1].mean())

    dt_vals = obstacle_dt[arr[:, 0], arr[:, 1]]
    safe_mask = dt_vals >= min_clearance_cells
    if safe_mask.any():
        safe_arr = arr[safe_mask]
        d2 = (safe_arr[:, 0] - mean_r) ** 2 + (safe_arr[:, 1] - mean_c) ** 2
        idx = int(np.argmin(d2))
        return (int(safe_arr[idx, 0]), int(safe_arr[idx, 1]))

    r_c = int(round(mean_r))
    c_c = int(round(mean_c))
    r0 = max(0, r_c - max_retract_cells)
    r1 = min(height, r_c + max_retract_cells + 1)
    c0 = max(0, c_c - max_retract_cells)
    c1 = min(width, c_c + max_retract_cells + 1)
    if r0 >= r1 or c0 >= c1:
        return None

    sub_ok = is_free[r0:r1, c0:c1] & (obstacle_dt[r0:r1, c0:c1] >= min_clearance_cells)
    if free_labels is not None:
        comp = int(free_labels[int(arr[0, 0]), int(arr[0, 1])])
        if comp > 0:
            sub_ok = sub_ok & (free_labels[r0:r1, c0:c1] == comp)
    if not sub_ok.any():
        return None

    rr, cc = np.nonzero(sub_ok)
    gr = rr + r0
    gc = cc + c0
    d2 = (gr - mean_r) ** 2 + (gc - mean_c) ** 2
    if float(d2.min()) > float((max_retract_cells + 4) ** 2):
        return None
    idx = int(np.argmin(d2))
    return (int(gr[idx]), int(gc[idx]))


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


_KERNEL_3X3 = np.ones((3, 3), dtype=np.uint8)


def extract_frontiers(
    occ: OccupancyView,
    params: FrontierParams,
    blacklisted_goals: Sequence[Tuple[float, float]] = (),
) -> List[FrontierCluster]:
    """Vectorized 2D frontier extraction with safe-goal retraction and deduplication."""
    res = occ.resolution

    # An is_free cell has an 8-connected unknown neighbor iff the 3x3 dilation
    # of is_unknown is non-zero there, and has no 8-connected lethal neighbor
    # iff its Euclidean obstacle distance transform is >= 1.5 cells (> sqrt(2)).
    unk_dilated = cv2.dilate(occ.is_unknown.view(np.uint8), _KERNEL_3X3)
    frontier_mask = occ.is_free & (occ.obstacle_dt >= 1.5) & (unk_dilated != 0)
    raw_clusters: List[np.ndarray] = []

    if frontier_mask.any():
        n_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            frontier_mask.view(np.uint8), connectivity=8, ltype=cv2.CV_32S
        )
        valid_labels = [
            lbl
            for lbl in range(1, n_labels)
            if int(stats[lbl, cv2.CC_STAT_AREA]) >= params.min_frontier_size
        ]
        if valid_labels:
            rr, cc = np.nonzero(frontier_mask)
            lbl_vals = labels[rr, cc]
            order = np.argsort(lbl_vals, kind="mergesort")
            coords_s = np.column_stack((rr[order], cc[order])).astype(np.int32, copy=False)
            lbl_s = lbl_vals[order]
            uniq_lbls, starts, counts = np.unique(
                lbl_s, return_index=True, return_counts=True
            )
            for _lbl_id, st, cnt in zip(
                uniq_lbls.tolist(), starts.tolist(), counts.tolist()
            ):
                if cnt < params.min_frontier_size:
                    continue
                raw_clusters.extend(
                    _split_coords_if_large(
                        coords_s[st : st + cnt],
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
            free_labels=occ.free_labels,
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
