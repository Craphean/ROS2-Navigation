"""Live copy of the exported room cloud.

``export_merged_cloud.py`` accumulates ``/points_merged``: both robots' 3D
scans are already transformed into ``world`` and cropped to the lidar range.
That cloud stays inside the house. ``/octomap_point_cloud_centers`` does not —
misses that fly over the 1.5 m walls are stored as occupied cells out to 12 m,
which is the halo around the floor plan.

The console keeps the same voxel hash as the PCD exporter and sends the
centres to the browser.
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Any

import numpy as np

_FLOAT32 = 7
# Stored voxels can keep growing for the whole exploration. The browser
# payload is downsampled so a long run does not stall the websocket.
_STORE_MAX = 600_000
_PACK_MAX = 200_000


class CloudCache:
    def __init__(
        self,
        voxel: float = 0.04,
        zmin: float = 0.05,
        zmax: float = 2.0,
        min_interval: float = 0.5,
    ) -> None:
        self.voxel = float(voxel)
        self.zmin = float(zmin)
        self.zmax = float(zmax)
        self._min_interval = min_interval
        self._lock = threading.Lock()
        self._keys: set[tuple[int, int, int]] = set()
        self._payload = b""
        self._seq = 0
        self._count = 0
        self._last_pack = 0.0

    def clear(self) -> None:
        with self._lock:
            self._keys.clear()
            self._payload = b""
            self._count = 0
            self._seq += 1

    def update(self, msg: Any) -> None:
        xyz = _xyz(msg)
        if xyz is None or xyz.shape[0] == 0:
            return
        keep = (xyz[:, 2] >= self.zmin) & (xyz[:, 2] <= self.zmax)
        xyz = xyz[keep]
        if xyz.shape[0] == 0:
            return
        idx = np.floor(xyz / self.voxel).astype(np.int32)
        fresh = {
            (int(ix), int(iy), int(iz))
            for ix, iy, iz in idx
        }
        now = time.monotonic()
        with self._lock:
            before = len(self._keys)
            if len(self._keys) < _STORE_MAX:
                self._keys.update(fresh)
            grew = len(self._keys) > before
            if not grew and now - self._last_pack < self._min_interval:
                return
            self._last_pack = now
            payload, count = _pack_keys(self._keys, self.voxel)
            self._payload = payload
            self._count = count
            self._seq += 1

    def snapshot(self) -> tuple[bytes, int, int]:
        with self._lock:
            return self._payload, self._seq, self._count


def _pack_keys(keys: set[tuple[int, int, int]], voxel: float) -> tuple[bytes, int]:
    count = len(keys)
    if count == 0:
        return b"", 0
    centers = (np.asarray(list(keys), dtype=np.float32) + 0.5) * np.float32(voxel)
    if count > _PACK_MAX:
        centers = centers[:: int(np.ceil(count / _PACK_MAX))]
        count = int(centers.shape[0])
    return struct.pack("<I", count) + centers.astype(np.float32, copy=False).tobytes(), count


def _xyz(msg: Any) -> np.ndarray | None:
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        return None
    if any(fields[name].datatype != _FLOAT32 for name in ("x", "y", "z")):
        return None
    point_step = int(msg.point_step)
    count = int(msg.width) * int(msg.height)
    if point_step < 12 or count <= 0:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    need = count * point_step
    if raw.size < need:
        return None
    points = raw[:need].reshape(count, point_step)

    def column(name: str) -> np.ndarray:
        offset = int(fields[name].offset)
        chunk = np.ascontiguousarray(points[:, offset : offset + 4])
        return np.frombuffer(chunk, dtype=np.float32)

    xyz = np.stack([column("x"), column("y"), column("z")], axis=1)
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]
