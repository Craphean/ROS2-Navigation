#!/usr/bin/env python3
"""Pure-Python helpers for 3D-aware multi-robot exploration.

This module deliberately has no ROS dependencies so the geometry, scoring,
residual blind-spot detection, TSP ordering and convergence logic can be unit
tested without a running ROS graph.  The coordinator converts ROS messages into
plain numpy arrays and calls into these helpers.

Data flow:
    OccupancyView (2D /map)  ─┐
                              ├─> residual detection, free-space components
    VoxelColumnIndex (3D)   ──┘
                              └─> InfoGainEvaluator
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# Coarse map states used by the residual detector.
FREE = 0
UNKNOWN = 1
LETHAL = 2


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# 2D occupancy view
# ---------------------------------------------------------------------------


@dataclass
class OccupancyView:
    """Vectorized view of the merged 2D occupancy grid.

    ``free_labels`` is a 4-connected labeling of free cells (0 = not free).
    Four-connectivity is intentional: diagonal free-cell connections can leak
    around wall corners and would defeat reachability checks.
    """

    data: np.ndarray  # int8 (height, width), ROS occupancy values
    is_free: np.ndarray
    is_unknown: np.ndarray
    is_lethal: np.ndarray
    free_labels: np.ndarray  # int32 (height, width)
    obstacle_dt: np.ndarray  # float32 (height, width), cells
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int

    @classmethod
    def from_grid(cls, grid_msg) -> "OccupancyView":
        """Build from a nav_msgs/OccupancyGrid (duck-typed, no ROS import)."""
        width = int(grid_msg.info.width)
        height = int(grid_msg.info.height)
        data = np.asarray(grid_msg.data, dtype=np.int8).reshape((height, width))
        return cls.from_numpy(
            data,
            float(grid_msg.info.origin.position.x),
            float(grid_msg.info.origin.position.y),
            float(grid_msg.info.resolution),
        )

    @classmethod
    def from_numpy(
        cls, data: np.ndarray, origin_x: float, origin_y: float, resolution: float
    ) -> "OccupancyView":
        data = np.asarray(data, dtype=np.int8)
        height, width = data.shape
        is_free = (data >= 0) & (data < 20)
        is_unknown = data == -1
        is_lethal = data > 50

        # Free-space components (4-connected).  Unknown/lethal cells get 0.
        n_labels, labels = cv2.connectedComponents(
            is_free.astype(np.uint8), connectivity=4
        )
        del n_labels
        labels[~is_free] = 0

        # Clearance to nearest lethal cell, in cells.
        obstacle_binary = np.ones((height, width), dtype=np.uint8) * 255
        obstacle_binary[is_lethal] = 0
        obstacle_dt = cv2.distanceTransform(obstacle_binary, cv2.DIST_L2, 5)

        return cls(
            data=data,
            is_free=is_free,
            is_unknown=is_unknown,
            is_lethal=is_lethal,
            free_labels=labels.astype(np.int32),
            obstacle_dt=obstacle_dt,
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            resolution=float(resolution),
            width=width,
            height=height,
        )

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int(math.floor((x - self.origin_x) / self.resolution))
        r = int(math.floor((y - self.origin_y) / self.resolution))
        return r, c

    def cell_to_world(self, r: int, c: int) -> Tuple[float, float]:
        return (
            self.origin_x + (c + 0.5) * self.resolution,
            self.origin_y + (r + 0.5) * self.resolution,
        )

    def contains_cell(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def component_at_world(self, x: float, y: float) -> int:
        r, c = self.world_to_cell(x, y)
        if not self.contains_cell(r, c):
            return -1
        return int(self.free_labels[r, c])

    def clearance_at_world(self, x: float, y: float) -> float:
        r, c = self.world_to_cell(x, y)
        if not self.contains_cell(r, c):
            return 0.0
        return float(self.obstacle_dt[r, c]) * self.resolution

    def state_at_world(self, x: float, y: float) -> int:
        r, c = self.world_to_cell(x, y)
        if not self.contains_cell(r, c):
            return LETHAL
        v = int(self.data[r, c])
        if v == -1:
            return UNKNOWN
        if v > 50:
            return LETHAL
        if v >= 0:
            return FREE
        return UNKNOWN

    def component_is_valid(self, component: int) -> bool:
        return component > 0


# ---------------------------------------------------------------------------
# 3D voxel column index
# ---------------------------------------------------------------------------


@dataclass
class VoxelParams:
    resolution: float = 0.10
    min_z: float = 0.08
    max_z: float = 2.0
    min_points_per_column: int = 3
    max_points: int = 400_000
    low_z: float = 0.5
    mid_z: float = 1.0
    margin: float = 2.0
    max_cells: int = 4_000_000


@dataclass
class WindowStats:
    density: float = 0.0
    unknown: float = 0.0
    floating: float = 0.0
    height: float = 0.0
    occupancy: float = 0.0
    g3d: float = 0.0


class VoxelColumnIndex:
    """Coarse 2D column rasterization of OctoMap occupied-voxel centers.

    Each coarse cell stores how many occupied voxels fall into its column, the
    vertical extent, and counts per height band.  This is enough to reason about
    structural density, height span, overhangs (table tops) and 3D structure that
    the 2D map still marks unknown, without ray casting.
    """

    def __init__(
        self,
        resolution: float,
        origin_x: float,
        origin_y: float,
        nx: int,
        ny: int,
        raw_count: np.ndarray,
        zmin: np.ndarray,
        zmax: np.ndarray,
        low_count: np.ndarray,
        mid_count: np.ndarray,
        high_count: np.ndarray,
        map_state: np.ndarray,
        column_keys: set,
    ):
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.nx = int(nx)
        self.ny = int(ny)
        self.raw_count = raw_count
        self.zmin = zmin
        self.zmax = zmax
        self.low_count = low_count
        self.mid_count = mid_count
        self.high_count = high_count
        self.map_state = map_state  # int8 (ny, nx): FREE/UNKNOWN/LETHAL
        self.column_keys = column_keys
        self.observed = raw_count >= 0  # replaced by build() before use
        self.total_columns = len(column_keys)

    @classmethod
    def build(
        cls,
        points_xyz: np.ndarray,
        view: OccupancyView,
        params: VoxelParams,
    ) -> "VoxelColumnIndex":
        """Build the index from an (N, 3) array of occupied voxel centers."""
        pts = np.asarray(points_xyz, dtype=np.float64)
        if pts.size == 0:
            pts = np.empty((0, 3), dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ValueError("points_xyz must be an (N, 3) array")

        # Crop vertically.
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts):
            pts = pts[(pts[:, 2] >= params.min_z) & (pts[:, 2] <= params.max_z)]
        if len(pts) > params.max_points > 0:
            stride = int(math.ceil(len(pts) / float(params.max_points)))
            pts = pts[::stride]

        res = float(params.resolution)
        if res <= 0.0:
            raise ValueError("voxel resolution must be positive")

        # Bounds follow the 2D map plus a margin.  If there are no points we
        # still build a valid (all-zero) grid so callers can query it safely.
        min_x = view.origin_x - params.margin
        min_y = view.origin_y - params.margin
        max_x = view.origin_x + view.width * view.resolution + params.margin
        max_y = view.origin_y + view.height * view.resolution + params.margin
        if len(pts):
            min_x = min(min_x, float(pts[:, 0].min()) - params.margin)
            min_y = min(min_y, float(pts[:, 1].min()) - params.margin)
            max_x = max(max_x, float(pts[:, 0].max()) + params.margin)
            max_y = max(max_y, float(pts[:, 1].max()) + params.margin)

        nx = max(1, int(math.ceil((max_x - min_x) / res)))
        ny = max(1, int(math.ceil((max_y - min_y) / res)))
        if nx * ny > params.max_cells:
            raise ValueError(
                f"voxel index too large: {nx}x{ny} cells (limit {params.max_cells})"
            )

        size = nx * ny
        raw_count = np.zeros(size, dtype=np.int32)
        zmin_flat = np.full(size, np.inf, dtype=np.float64)
        zmax_flat = np.full(size, -np.inf, dtype=np.float64)
        low_flat = np.zeros(size, dtype=np.int32)
        mid_flat = np.zeros(size, dtype=np.int32)
        high_flat = np.zeros(size, dtype=np.int32)

        if len(pts):
            ix = np.floor((pts[:, 0] - min_x) / res).astype(np.int64)
            iy = np.floor((pts[:, 1] - min_y) / res).astype(np.int64)
            valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            if valid.any():
                x = pts[valid, 0]
                y = pts[valid, 1]
                z = pts[valid, 2]
                ix = ix[valid]
                iy = iy[valid]
                lin = iy * nx + ix
                raw_count += np.bincount(lin, minlength=size).astype(np.int32)
                np.minimum.at(zmin_flat, lin, z)
                np.maximum.at(zmax_flat, lin, z)
                low_flat += np.bincount(
                    lin[z < params.low_z], minlength=size
                ).astype(np.int32)
                mid_mask = (z >= params.low_z) & (z < params.mid_z)
                mid_flat += np.bincount(lin[mid_mask], minlength=size).astype(
                    np.int32
                )
                high_flat += np.bincount(
                    lin[z >= params.mid_z], minlength=size
                ).astype(np.int32)

        raw_count = raw_count.reshape(ny, nx)
        zmin = zmin_flat.reshape(ny, nx)
        zmax = zmax_flat.reshape(ny, nx)
        low_count = low_flat.reshape(ny, nx)
        mid_count = mid_flat.reshape(ny, nx)
        high_count = high_flat.reshape(ny, nx)

        # Sample the 2D map state at coarse-cell centers.
        map_state = np.full((ny, nx), LETHAL, dtype=np.int8)
        if nx > 0 and ny > 0:
            cx = min_x + (np.arange(nx) + 0.5) * res
            cy = min_y + (np.arange(ny) + 0.5) * res
            columns = np.floor((cx - view.origin_x) / view.resolution).astype(int)
            rows = np.floor((cy - view.origin_y) / view.resolution).astype(int)
            valid_c = (columns >= 0) & (columns < view.width)
            valid_r = (rows >= 0) & (rows < view.height)
            sub = view.data[np.ix_(rows[valid_r], columns[valid_c])]
            sampled = np.full((ny, nx), LETHAL, dtype=np.int8)
            sampled[np.ix_(np.where(valid_r)[0], np.where(valid_c)[0])] = _classify(
                sub
            )
            map_state = sampled

        index = cls(
            resolution=res,
            origin_x=min_x,
            origin_y=min_y,
            nx=nx,
            ny=ny,
            raw_count=raw_count,
            zmin=zmin,
            zmax=zmax,
            low_count=low_count,
            mid_count=mid_count,
            high_count=high_count,
            map_state=map_state,
            column_keys=set(),
        )
        observed = raw_count >= int(params.min_points_per_column)
        index.observed = observed
        flat_lin = np.flatnonzero(observed.reshape(-1))
        # Keys must not depend on this build's origin/bounds.  The bounds
        # follow the growing SLAM map, so grid-linear indices (iy*nx+ix)
        # change for every column whenever nx changes, which would fake
        # voxel growth and reset convergence.  Quantize the column centers
        # on a fixed world-anchored grid instead.
        oy, ox = np.divmod(flat_lin, nx)
        wx = min_x + (ox + 0.5) * res
        wy = min_y + (oy + 0.5) * res
        index.column_keys = set(
            zip(
                (wx // res).astype(np.int64).tolist(),
                (wy // res).astype(np.int64).tolist(),
            )
        )
        index.total_columns = int(len(index.column_keys))
        return index

    def new_column_count(self, previous_keys: Optional[set]) -> int:
        """Number of occupied columns that were not present in the previous index."""
        if not previous_keys:
            return int(len(self.column_keys))
        return int(len(self.column_keys - previous_keys))

    def window_stats(
        self, wx: float, wy: float, radius: float, params: "InfoGainParams"
    ) -> WindowStats:
        """Structural statistics in a square window around a world point."""
        r0, r1, c0, c1 = self._window_bounds(wx, wy, radius)
        if r0 >= r1 or c0 >= c1:
            return WindowStats()

        obs = self.observed[r0:r1, c0:c1]
        cells = obs.size
        n_obs = int(obs.sum())
        if cells == 0:
            return WindowStats()

        density = n_obs / float(cells)
        density_norm = min(density / params.density_ref, 1.0) if params.density_ref > 0 else 0.0

        unknown = 0.0
        floating = 0.0
        height_norm = 0.0
        occupancy_norm = 0.0
        if n_obs:
            raw = self.raw_count[r0:r1, c0:c1][obs]
            occupancy_norm = min(
                float(raw.mean()) / params.occupancy_ref, 1.0
            ) if params.occupancy_ref > 0 else 0.0

            unknown = float(
                (obs & (self.map_state[r0:r1, c0:c1] == UNKNOWN)).sum()
            ) / n_obs
            high = self.high_count[r0:r1, c0:c1] > 0
            low = self.low_count[r0:r1, c0:c1] > 0
            mid = self.mid_count[r0:r1, c0:c1] > 0
            floating = float((obs & high & ~low & ~mid).sum()) / n_obs

            wall = (
                obs
                & (self.zmin[r0:r1, c0:c1] < params.wall_zmin)
                & (self.zmax[r0:r1, c0:c1] > params.wall_zmax)
            )
            valid = obs & ~wall
            if valid.any():
                zmin = self.zmin[r0:r1, c0:c1][valid]
                zmax = self.zmax[r0:r1, c0:c1][valid]
                span = float(np.percentile(zmax, 90) - np.percentile(zmin, 10))
                if params.height_ref > 0:
                    height_norm = min(max(span / params.height_ref, 0.0), 1.0)

        weight_sum = (
            params.w_density
            + params.w_unknown
            + params.w_floating
            + params.w_height
            + params.w_occupancy
        )
        if weight_sum <= 0:
            weight_sum = 1.0
        g3d = (
            params.w_density * density_norm
            + params.w_unknown * unknown
            + params.w_floating * floating
            + params.w_height * height_norm
            + params.w_occupancy * occupancy_norm
        ) / weight_sum

        return WindowStats(
            density=density_norm,
            unknown=unknown,
            floating=floating,
            height=height_norm,
            occupancy=occupancy_norm,
            g3d=float(min(max(g3d, 0.0), 1.0)),
        )

    def _window_bounds(
        self, wx: float, wy: float, radius: float
    ) -> Tuple[int, int, int, int]:
        c0 = int(math.floor((wx - radius - self.origin_x) / self.resolution))
        c1 = int(math.ceil((wx + radius - self.origin_x) / self.resolution)) + 1
        r0 = int(math.floor((wy - radius - self.origin_y) / self.resolution))
        r1 = int(math.ceil((wy + radius - self.origin_y) / self.resolution)) + 1
        return (
            max(0, r0),
            min(self.ny, r1),
            max(0, c0),
            min(self.nx, c1),
        )


def _classify(values: np.ndarray) -> np.ndarray:
    """Map raw occupancy values to FREE/UNKNOWN/LETHAL."""
    out = np.full(values.shape, LETHAL, dtype=np.int8)
    out[(values >= 0) & (values < 20)] = FREE
    out[values == -1] = UNKNOWN
    return out


@dataclass
class InfoGainParams:
    window_radius: float = 1.5
    density_ref: float = 0.18
    occupancy_ref: float = 8.0
    height_ref: float = 0.80
    wall_zmin: float = 0.25
    wall_zmax: float = 1.30
    w_density: float = 0.30
    w_unknown: float = 0.35
    w_floating: float = 0.15
    w_height: float = 0.10
    w_occupancy: float = 0.10


# ---------------------------------------------------------------------------
# Residual blind-spot detection
# ---------------------------------------------------------------------------


@dataclass
class ResidualParams:
    activation_cycles: int = 3
    min_cluster_cells: int = 25
    min_density: float = 0.08
    min_unknown_frac: float = 0.20
    structure_radius: float = 2.5
    min_structure_columns: int = 3
    ring_radius: float = 1.0
    ring_min_empty_sectors: int = 3
    viewpoint_search_radius: float = 2.5
    min_clearance: float = 0.35
    max_candidates: int = 4
    dedupe_radius: float = 1.5
    score_scale: float = 10.0
    blacklist_radius: float = 1.0
    coverage_hole_enable: bool = False


@dataclass
class ResidualCandidate:
    x: float
    y: float
    yaw: float
    score: float
    cluster_cells: int
    area: float
    component_label: int = -1
    # Blind-spot cluster centroid (the viewpoint above is where the robot
    # should drive).  Blacklists/attempt counters key on the cluster, because
    # the viewpoint may shift slightly between detections.
    cluster_x: float = 0.0
    cluster_y: float = 0.0

    # Common candidate interface used by the scheduler.
    kind: str = "residual"
    total_gain: float = 0.0
    gain_2d: float = 0.0
    gain_3d: float = 0.0
    size: int = 0

    @property
    def world_x(self) -> float:
        """Alias matching the coordinator's candidate interface."""
        return self.x

    @property
    def world_y(self) -> float:
        return self.y


def detect_residuals(
    view: OccupancyView,
    index: VoxelColumnIndex,
    params: ResidualParams,
    blacklist: Sequence[Tuple[float, float]] = (),
    attempts: Optional[Dict[Tuple[float, float], int]] = None,
    max_attempts: int = 2,
) -> Tuple[List[ResidualCandidate], float]:
    """Find 3D blind spots left after the 2D frontier is exhausted.

    Returns ``(candidates, raw_gain)`` where ``raw_gain`` is the sum of all
    qualifying cluster scores *before* the cap, used by the convergence tracker.
    """
    if index.nx <= 0 or index.ny <= 0:
        return [], 0.0

    obs = index.observed
    if not obs.any():
        return [], 0.0

    res = index.resolution
    k_small = _odd_kernel(max(1.0, 1.0 / res))
    k_structure = _odd_kernel(max(1.0, params.structure_radius * 2.0 / res))
    k_density = _odd_kernel(max(1.0, 1.0 / res))

    density_small = cv2.boxFilter(
        obs.astype(np.float32), -1, (k_density, k_density), normalize=True,
        borderType=cv2.BORDER_CONSTANT,
    )
    unknown_small = cv2.boxFilter(
        (index.map_state == UNKNOWN).astype(np.float32), -1,
        (k_small, k_small), normalize=True, borderType=cv2.BORDER_CONSTANT,
    )
    structure_count = cv2.boxFilter(
        obs.astype(np.float32), -1, (k_structure, k_structure),
        normalize=False, borderType=cv2.BORDER_CONSTANT,
    )
    empty_sectors = _empty_sector_count(obs, params.ring_radius, res)

    map_free = index.map_state == FREE
    map_unknown = index.map_state == UNKNOWN

    mask_a = (
        map_free
        & (density_small < params.min_density)
        & (unknown_small >= params.min_unknown_frac)
        & (structure_count >= params.min_structure_columns)
        & (empty_sectors >= params.ring_min_empty_sectors)
    )
    # 3D structure visible inside a 2D-unknown region (e.g. over a low wall or
    # a table top seen before the room floor is mapped).
    mask_b = map_unknown & obs

    mask = mask_a | mask_b
    if params.coverage_hole_enable:
        # Sparse-but-surrounded columns: possible occlusion hole.  Off by
        # default because open floor legitimately has no occupied columns.
        sparse = obs & (index.raw_count <= 1)
        dense_neighbourhood = density_small >= params.min_density
        mask = mask | (map_free & sparse & dense_neighbourhood)

    if not mask.any():
        return [], 0.0

    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    del n_labels

    raw_candidates: List[ResidualCandidate] = []
    raw_gain = 0.0
    blacklist = list(blacklist)
    attempts = attempts or {}

    for label in range(1, stats.shape[0]):
        cell_count = int(stats[label, cv2.CC_STAT_AREA])
        if cell_count < params.min_cluster_cells:
            continue

        cx = index.origin_x + (centroids[label][0] + 0.5) * res
        cy = index.origin_y + (centroids[label][1] + 0.5) * res

        if _is_blacklisted(cx, cy, blacklist, params.blacklist_radius):
            continue
        key = _attempt_key(cx, cy)
        if attempts.get(key, 0) >= max_attempts:
            continue

        view_xy = _find_viewpoint(
            view, cx, cy, params.viewpoint_search_radius, params.min_clearance
        )
        if view_xy is None:
            continue
        vx, vy = view_xy
        yaw = math.atan2(cy - vy, cx - vx)
        area = cell_count * res * res
        score = params.score_scale * area
        component = view.component_at_world(vx, vy)

        raw_gain += score
        raw_candidates.append(
            ResidualCandidate(
                x=vx,
                y=vy,
                yaw=yaw,
                score=score,
                cluster_cells=cell_count,
                area=area,
                component_label=component,
                cluster_x=cx,
                cluster_y=cy,
                total_gain=score,
                size=cell_count,
            )
        )

    if not raw_candidates:
        return [], raw_gain

    raw_candidates.sort(key=lambda c: (-c.score, c.x, c.y))
    selected: List[ResidualCandidate] = []
    for cand in raw_candidates:
        if len(selected) >= params.max_candidates:
            break
        if any(
            math.hypot(cand.x - s.x, cand.y - s.y) < params.dedupe_radius
            for s in selected
        ):
            continue
        selected.append(cand)
    return selected, raw_gain


def _odd_kernel(size_cells: float) -> int:
    k = int(round(size_cells))
    if k < 1:
        k = 1
    if k % 2 == 0:
        k += 1
    return k


def _empty_sector_count(
    observed: np.ndarray, ring_radius: float, resolution: float
) -> np.ndarray:
    """Number of the 8 compass sectors around each cell with no structure."""
    occ = cv2.dilate(observed.astype(np.uint8), np.ones((3, 3), np.uint8))
    radius_cells = max(1, int(round(ring_radius / max(resolution, 1e-6))))
    offsets = [
        (0, radius_cells),
        (0, -radius_cells),
        (radius_cells, 0),
        (-radius_cells, 0),
        (radius_cells, radius_cells),
        (radius_cells, -radius_cells),
        (-radius_cells, radius_cells),
        (-radius_cells, -radius_cells),
    ]
    empty = np.zeros(observed.shape, dtype=np.int16)
    for dr, dc in offsets:
        shifted = _shift2d(occ, dr, dc)
        empty += (shifted == 0).astype(np.int16)
    return empty


def _shift2d(a: np.ndarray, dr: int, dc: int) -> np.ndarray:
    out = np.zeros_like(a)
    h, w = a.shape
    r0 = max(0, dr)
    r1 = min(h, h + dr)
    c0 = max(0, dc)
    c1 = min(w, w + dc)
    if r0 >= r1 or c0 >= c1:
        return out
    out[r0:r1, c0:c1] = a[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
    return out


def _is_blacklisted(
    x: float, y: float, blacklist: Sequence[Tuple[float, float]], radius: float
) -> bool:
    return any(math.hypot(x - bx, y - by) < radius for bx, by in blacklist)


def _attempt_key(x: float, y: float) -> Tuple[float, float]:
    return (round(x, 1), round(y, 1))


def _find_viewpoint(
    view: OccupancyView,
    target_x: float,
    target_y: float,
    search_radius: float,
    min_clearance: float,
) -> Optional[Tuple[float, float]]:
    """Nearest free cell with enough clearance to the target cluster."""
    tr, tc = view.world_to_cell(target_x, target_y)
    radius_cells = max(1, int(math.ceil(search_radius / view.resolution)))
    clearance_cells = min_clearance / view.resolution

    r0 = max(0, tr - radius_cells)
    r1 = min(view.height, tr + radius_cells + 1)
    c0 = max(0, tc - radius_cells)
    c1 = min(view.width, tc + radius_cells + 1)
    if r0 >= r1 or c0 >= c1:
        return None

    free = view.is_free[r0:r1, c0:c1]
    clear = view.obstacle_dt[r0:r1, c0:c1] >= clearance_cells
    valid = free & clear
    if not valid.any():
        return None

    rows, cols = np.nonzero(valid)
    rows = rows + r0
    cols = cols + c0
    dist2 = (rows - tr) ** 2 + (cols - tc) ** 2
    best = int(np.argmin(dist2))
    r = int(rows[best])
    c = int(cols[best])
    return view.cell_to_world(r, c)


# ---------------------------------------------------------------------------
# Near-field TSP (open path)
# ---------------------------------------------------------------------------


@dataclass
class TourParams:
    max_waypoints: int = 6
    min_spacing: float = 1.2
    max_tour_length: float = 10.0
    score_dist_weight: float = 1.0
    two_opt_passes: int = 2


def nearest_neighbor_order(points: np.ndarray, start: np.ndarray) -> List[int]:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    n = len(pts)
    if n == 0:
        return []
    remaining = set(range(n))
    order: List[int] = []
    cur = np.asarray(start, dtype=np.float64).reshape(2)
    while remaining:
        j = min(remaining, key=lambda k: (float(np.linalg.norm(pts[k] - cur)), k))
        order.append(j)
        remaining.remove(j)
        cur = pts[j]
    return order


def path_length(order: Sequence[int], points: np.ndarray, start: np.ndarray) -> float:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if not order:
        return 0.0
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
) -> List[int]:
    """2-opt for an open path with a fixed start and free end."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    order = list(order)
    m = len(order)
    if m < 2:
        return order
    start = np.asarray(start, dtype=np.float64).reshape(2)

    for _ in range(max(0, max_passes)):
        improved = False
        for i in range(m - 1):
            a = start if i == 0 else pts[order[i - 1]]
            b = pts[order[i]]
            for j in range(i + 1, m):
                c = pts[order[j]]
                d = pts[order[j + 1]] if j + 1 < m else None
                before = float(np.linalg.norm(a - b))
                after = float(np.linalg.norm(a - c))
                if d is not None:
                    before += float(np.linalg.norm(c - d))
                    after += float(np.linalg.norm(b - d))
                if after + 1e-9 < before:
                    order[i : j + 1] = reversed(order[i : j + 1])
                    improved = True
        if not improved:
            break
    return order


def build_local_tour(candidates: Sequence, start_xy, params: TourParams) -> List:
    """Select and order near-field candidates into an open TSP tour."""
    if not candidates:
        return []
    sx, sy = float(start_xy[0]), float(start_xy[1])

    ranked = sorted(
        candidates,
        key=lambda c: (
            -(float(getattr(c, "total_gain", 0.0))
              - params.score_dist_weight * math.hypot(c.x - sx, c.y - sy)),
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
    order = nearest_neighbor_order(points, start)
    order = open_path_2opt(order, points, start, params.two_opt_passes)

    # Trim the tail if the optimized tour is too long.  The tour is a route, so
    # dropping the last waypoints preserves the beginning of the route.
    while len(order) > 1 and path_length(order, points, start) > params.max_tour_length:
        order.pop()
    return [selected[i] for i in order]


# ---------------------------------------------------------------------------
# Ownership and Global ROI
# ---------------------------------------------------------------------------


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


def assign_ownership(
    agents: Sequence[AgentState], candidates: Sequence, local_radius: float
) -> List[Ownership]:
    """Assign each candidate to a Local owner or to the Global pool.

    A candidate is Local only when its nearest robot is idle, within
    ``local_radius`` and in the same 4-connected free-space component.  Busy
    robots do not own candidates; active-tour exclusion protects their route.
    """
    results: List[Ownership] = []
    valid_agents = [
        a for a in agents if math.isfinite(a.x) and math.isfinite(a.y)
    ]
    for cand in candidates:
        cand_component = int(getattr(cand, "component_label", -1))
        if not valid_agents:
            results.append(Ownership(None, "global", float("inf"), "no valid pose"))
            continue

        nearest = min(
            valid_agents,
            key=lambda a: (
                math.hypot(cand.x - a.x, cand.y - a.y),
                a.namespace,
            ),
        )
        dist = math.hypot(cand.x - nearest.x, cand.y - nearest.y)
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


@dataclass
class RoiParams:
    distance_offset: float = 1.0
    roi_scale: float = 5.0
    min_roi: float = 0.05
    max_distance: float = 0.0
    orientation_scale: float = 0.5


def global_bid(
    agent: AgentState, candidate, params: RoiParams
) -> Tuple[float, float, float]:
    """Return (bid, roi, distance) for a Global candidate."""
    dist = math.hypot(candidate.x - agent.x, candidate.y - agent.y)
    # ``bid_gain`` is an optional scheduling penalty (e.g. a released Local
    # leftover); fall back to the plain information gain.
    gain = float(
        getattr(candidate, "bid_gain", 0.0)
        or getattr(candidate, "total_gain", 0.0)
    )
    roi = gain / max(dist + params.distance_offset, 1e-6)
    heading = abs(
        wrap_angle(math.atan2(candidate.y - agent.y, candidate.x - agent.x) - agent.yaw)
    )
    bid = params.roi_scale * roi - params.orientation_scale * heading
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
        if all(
            math.hypot(c.x - wx, c.y - wy) >= radius for wx, wy in waypoints
        )
    ]


# ---------------------------------------------------------------------------
# Convergence tracking
# ---------------------------------------------------------------------------


@dataclass
class ConvergenceParams:
    enabled: bool = True
    cycles_required: int = 6
    residual_gain_threshold: float = 0.50
    voxel_growth_epsilon: int = 25
    voxel_growth_rate: float = 0.005
    require_3d: bool = True
    fallback_timeout: float = 120.0


@dataclass
class ConvergenceState:
    converged: bool = False
    streak: int = 0
    two_d_done: bool = False
    residual_done: bool = False
    growth_done: bool = False
    reason: str = ""


class ConvergenceTracker:
    def __init__(self, params: ConvergenceParams):
        self.params = params
        self.converged = False
        self.streak = 0
        self._last_activity = 0.0
        self._initialized = False

    def reset(self, reason: str = "") -> None:
        self.converged = False
        self.streak = 0
        self._last_activity = 0.0
        self._initialized = False
        self.last_reset_reason = reason

    def update(
        self,
        now: float,
        *,
        map_valid: bool,
        frontier_count: int,
        all_idle: bool,
        residual_count: int,
        residual_raw_gain: float,
        index_available: bool,
        new_columns: int,
        total_columns: int,
    ) -> ConvergenceState:
        p = self.params
        if not p.enabled:
            self.reset("disabled")
            return ConvergenceState(reason="disabled")

        if not self._initialized:
            self._last_activity = now
            self._initialized = True

        busy = (
            not map_valid
            or frontier_count > 0
            or residual_count > 0
            or not all_idle
        )
        if busy:
            self._last_activity = now

        two_d_done = bool(map_valid and frontier_count == 0)

        if index_available:
            residual_done = (
                residual_count == 0
                and residual_raw_gain <= p.residual_gain_threshold
            )
            growth_done = (
                new_columns <= p.voxel_growth_epsilon
                or new_columns <= p.voxel_growth_rate * max(total_columns, 1)
            )
        elif p.require_3d:
            residual_done = False
            growth_done = False
        else:
            quiet_for = now - self._last_activity
            residual_done = quiet_for >= p.fallback_timeout
            growth_done = True

        ready = (
            all_idle
            and two_d_done
            and residual_done
            and growth_done
        )
        if ready:
            self.streak += 1
        else:
            self.streak = 0
            # Convergence is a live state, not a one-way latch.  If new
            # evidence appears after latching (a frontier, a residual blind
            # spot, voxel growth, or a busy robot), reopen dispatch so the
            # new area can be cleaned.  Losing the 3D index alone does not
            # reopen it: with require_3d we simply cannot re-verify.
            reopened = (
                not map_valid
                or frontier_count > 0
                or residual_count > 0
                or not all_idle
                or (index_available and not growth_done)
            )
            if self.converged and reopened:
                self.converged = False
                self.last_reset_reason = "convergence conditions no longer hold"

        if not self.converged and self.streak >= max(1, p.cycles_required):
            self.converged = True

        return ConvergenceState(
            converged=self.converged,
            streak=self.streak,
            two_d_done=two_d_done,
            residual_done=residual_done,
            growth_done=growth_done,
            reason=(
                f"2d={two_d_done} residual={residual_done} "
                f"growth={growth_done} idle={all_idle} streak={self.streak}"
            ),
        )
