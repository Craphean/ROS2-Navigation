"""2.5D voxel-column structural index and 3D information gain evaluation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .occupancy_view import LETHAL, UNKNOWN, OccupancyView, classify_occupancy


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


@dataclass
class WindowStats:
    density: float = 0.0
    unknown: float = 0.0
    floating: float = 0.0
    height: float = 0.0
    occupancy: float = 0.0
    g3d: float = 0.0


class VoxelColumnIndex:
    """Coarse 2.5D column rasterization of OctoMap occupied-voxel centers.

    Each column tracks vertical extent, height-band counts, and 2D map state,
    enabling lightweight structural gain and occlusion reasoning without
    expensive full-3D volumetric ray casting.
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
        self.map_state = map_state  # int8 (ny, nx): FREE / UNKNOWN / LETHAL
        self.column_keys = column_keys
        self.observed = raw_count >= 0
        self.total_columns = len(column_keys)

    @classmethod
    def build(
        cls,
        points_xyz: np.ndarray,
        view: OccupancyView,
        params: VoxelParams,
    ) -> "VoxelColumnIndex":
        """Build the column index from an (N, 3) array of occupied voxel centers."""
        pts = np.asarray(points_xyz, dtype=np.float64)
        if pts.size == 0:
            pts = np.empty((0, 3), dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ValueError("points_xyz must be an (N, 3) array")

        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts):
            pts = pts[(pts[:, 2] >= params.min_z) & (pts[:, 2] <= params.max_z)]
        if len(pts) > params.max_points > 0:
            stride = int(math.ceil(len(pts) / float(params.max_points)))
            pts = pts[::stride]

        res = float(params.resolution)
        if res <= 0.0:
            raise ValueError("voxel resolution must be positive")

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
                z = pts[valid, 2]
                lin = iy[valid] * nx + ix[valid]
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
            sampled[np.ix_(np.where(valid_r)[0], np.where(valid_c)[0])] = (
                classify_occupancy(sub)
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
        # Quantize column centers on a world-anchored grid so keys stay
        # invariant as the SLAM map bounding box grows.
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
        """Number of occupied columns absent from the previous index."""
        if not previous_keys:
            return int(len(self.column_keys))
        return int(len(self.column_keys - previous_keys))

    def window_stats(
        self, wx: float, wy: float, radius: float, params: InfoGainParams
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
        density_norm = (
            min(density / params.density_ref, 1.0) if params.density_ref > 0 else 0.0
        )

        unknown = 0.0
        floating = 0.0
        height_norm = 0.0
        occupancy_norm = 0.0
        if n_obs:
            raw = self.raw_count[r0:r1, c0:c1][obs]
            occupancy_norm = (
                min(float(raw.mean()) / params.occupancy_ref, 1.0)
                if params.occupancy_ref > 0
                else 0.0
            )

            unknown = (
                float((obs & (self.map_state[r0:r1, c0:c1] == UNKNOWN)).sum()) / n_obs
            )
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
