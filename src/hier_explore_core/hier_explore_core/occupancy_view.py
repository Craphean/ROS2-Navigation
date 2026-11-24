"""Vectorized 2D occupancy grid view with 4-connected free components and clearance."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import cv2
import numpy as np

# Avoid OpenCV spawning 128 worker threads inside a 4-core container
cv2.setNumThreads(1)

# Coarse map states shared by the 2D frontier extractor and 3D residual detector.
FREE = 0
UNKNOWN = 1
LETHAL = 2


def classify_occupancy(values: np.ndarray) -> np.ndarray:
    """Map raw ROS OccupancyGrid values to FREE / UNKNOWN / LETHAL."""
    out = np.full(values.shape, LETHAL, dtype=np.int8)
    out[(values >= 0) & (values < 20)] = FREE
    out[values == -1] = UNKNOWN
    return out


# Alias preserved for backwards compatibility with existing imports.
_classify = classify_occupancy


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
    def from_grid(
        cls,
        grid_msg,
        clear_robot_xy: Sequence[Tuple[float, float]] = (),
        clear_radius_m: float = 0.28,
    ) -> "OccupancyView":
        """Build from a nav_msgs/OccupancyGrid (duck-typed, no ROS import)."""
        width = int(grid_msg.info.width)
        height = int(grid_msg.info.height)
        raw = grid_msg.data
        try:
            data = np.frombuffer(raw, dtype=np.int8).reshape((height, width))
        except (TypeError, BufferError, ValueError):
            data = np.asarray(raw, dtype=np.int8).reshape((height, width))
        ox = float(grid_msg.info.origin.position.x)
        oy = float(grid_msg.info.origin.position.y)
        res = float(grid_msg.info.resolution)
        if clear_robot_xy and res > 0.0 and clear_radius_m > 0.0:
            rad_cells = max(1, int(math.ceil(clear_radius_m / res)))
            r2_max = (clear_radius_m / res) ** 2
            copied = False
            for rx, ry in clear_robot_xy:
                if not (math.isfinite(rx) and math.isfinite(ry)):
                    continue
                rc = int(math.floor((ry - oy) / res))
                cc = int(math.floor((rx - ox) / res))
                r0 = max(0, rc - rad_cells)
                r1 = min(height, rc + rad_cells + 1)
                c0 = max(0, cc - rad_cells)
                c1 = min(width, cc + rad_cells + 1)
                if r0 >= r1 or c0 >= c1:
                    continue
                sub = data[r0:r1, c0:c1]
                if not (sub > 50).any():
                    continue
                if not copied:
                    data = data.copy()
                    copied = True
                    sub = data[r0:r1, c0:c1]
                yy, xx = np.ogrid[r0:r1, c0:c1]
                disk = ((yy - rc) ** 2 + (xx - cc) ** 2) <= r2_max
                sub[disk & (sub > 50)] = 0
        return cls.from_numpy(data, ox, oy, res)

    @classmethod
    def from_numpy(
        cls, data: np.ndarray, origin_x: float, origin_y: float, resolution: float
    ) -> "OccupancyView":
        data = np.asarray(data, dtype=np.int8)
        height, width = data.shape
        is_free = (data >= 0) & (data < 20)
        is_unknown = data == -1
        is_lethal = data > 50

        n_labels, labels = cv2.connectedComponents(
            is_free.view(np.uint8), connectivity=4, ltype=cv2.CV_32S
        )
        del n_labels

        not_lethal_u8 = (data <= 50).view(np.uint8)
        obstacle_dt = cv2.distanceTransform(not_lethal_u8, cv2.DIST_L2, 3)

        return cls(
            data=data,
            is_free=is_free,
            is_unknown=is_unknown,
            is_lethal=is_lethal,
            free_labels=labels,
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
