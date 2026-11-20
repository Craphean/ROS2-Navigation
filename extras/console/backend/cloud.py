"""Live copy of the exported room cloud.

``export_merged_cloud.py`` accumulates ``/points_merged``: both robots' 3D
scans are already transformed into ``world`` and cropped to the lidar range.
That cloud stays inside the house. ``/octomap_point_cloud_centers`` does not —
misses that fly over the 1.5 m walls are stored as occupied cells out to 12 m,
which is the halo around the floor plan.

The console keeps the same voxel hash as the PCD exporter and sends the
centres to the browser.

Voxels are packed into ``int64`` ids and kept in a sorted numpy array.  A
Python ``set`` of ``(ix, iy, iz)`` tuples cost ~150 B per voxel and forced a
full re-pack on every message, which held the cache lock — and therefore
``/api/cloud.bin`` — for half a second once the map passed a few hundred
thousand voxels.
"""

from __future__ import annotations

import logging
import struct
import threading
from typing import Any

import numpy as np

LOGGER = logging.getLogger(__name__)

_FLOAT32 = 7
# Stored voxels can keep growing for the whole exploration. The browser
# payload is downsampled so a long run does not stall the websocket.
_STORE_MAX = 600_000
# The 3D view needs the density; the real bandwidth hog turned out to be DDS
# multicast on the egress NIC, not this payload. 60k points (~720 KB) keeps the
# original 4 cm spacing and costs ~3 Mbps at the 2 s poll.
_PACK_MAX = 60_000
# Coarsen until the payload is within this multiple of _PACK_MAX, then stride.
# Striding the store directly leaves bands because keys are sorted by (x, y, z).
_PACK_COARSEN_TO = 3
# 21 bits per axis -> +-2^20 voxels (+-41.9 km at 4 cm), so three indices pack
# into one signed int64 without collisions.
_AXIS_BITS = 21
_AXIS_MAX = (1 << _AXIS_BITS) - 1
_AXIS_OFFSET = 1 << (_AXIS_BITS - 1)
_EMPTY_KEYS = np.empty(0, dtype=np.int64)


class CloudCache:
    def __init__(
        self,
        voxel: float = 0.04,
        zmin: float = 0.05,
        zmax: float = 2.0,
    ) -> None:
        self.voxel = float(voxel)
        self._base_voxel = float(voxel)
        self.zmin = float(zmin)
        self.zmax = float(zmax)
        self._lock = threading.Lock()
        self._keys = _EMPTY_KEYS
        self._payload = b""
        self._seq = 0
        self._count = 0
        # Bumped on every change to _keys; a pack that finishes after a newer
        # merge must not overwrite the newer payload.
        self._gen = 0

    def clear(self) -> None:
        with self._lock:
            self._keys = _EMPTY_KEYS
            self.voxel = self._base_voxel
            self._payload = b""
            self._count = 0
            self._gen += 1
            self._seq += 1

    def update(self, msg: Any) -> None:
        xyz = _xyz(msg)
        if xyz is None or xyz.shape[0] == 0:
            return
        keep = (xyz[:, 2] >= self.zmin) & (xyz[:, 2] <= self.zmax)
        xyz = xyz[keep]
        if xyz.shape[0] == 0:
            return

        with self._lock:
            changed, gen, keys, voxel = self._merge_locked(xyz)
        if not changed:
            return

        # Decode + serialise outside the lock: /api/cloud.bin only waits for
        # the merge, never for the payload build.
        payload, count = _pack_payload(keys, voxel)
        with self._lock:
            if self._gen != gen:
                return  # a newer cloud already superseded this pack
            self._payload = payload
            self._count = count
            self._seq += 1

    def snapshot(self) -> tuple[bytes, int, int]:
        with self._lock:
            return self._payload, self._seq, self._count

    def points(self) -> tuple[np.ndarray, float]:
        """Full-resolution (N, 3) voxel centres in world + the voxel size.

        The browser payload is downsampled to ~60k points; a PCD export wants
        every stored voxel, so decode the keys directly instead of reusing
        _payload.
        """
        with self._lock:
            keys = self._keys
            voxel = self.voxel
        if keys.size == 0:
            return np.empty((0, 3), dtype=np.float32), voxel
        centers = (_unpack_voxels(keys).astype(np.float32) + 0.5) * np.float32(voxel)
        return centers, voxel

    # ------------------------------------------------------------------
    def _merge_locked(self, xyz: np.ndarray) -> tuple[bool, int, np.ndarray, float]:
        """Fold one cloud into the voxel store. Caller holds the lock."""
        idx = np.floor(xyz / self.voxel).astype(np.int32)
        keys = np.unique(_pack_voxels(idx))
        if keys.size == 0:
            return False, 0, _EMPTY_KEYS, self.voxel

        before = self._keys.size
        self._keys = keys if before == 0 else np.union1d(self._keys, keys)

        coarsened = False
        if self._keys.size > _STORE_MAX:
            self._coarsen_locked()
            coarsened = True
        if not coarsened and self._keys.size == before and self._payload:
            return False, 0, _EMPTY_KEYS, self.voxel

        self._gen += 1
        return True, self._gen, self._keys, self.voxel

    def _coarsen_locked(self) -> None:
        """Double the voxel size until the store fits.

        Refusing new voxels at the cap (the old behaviour) silently froze the
        3D view for the rest of the run.  Halving the resolution keeps every
        region visible instead of dropping whatever was explored last.
        """
        while self._keys.size > _STORE_MAX:
            idx = _unpack_voxels(self._keys)
            self.voxel *= 2.0
            self._keys = np.unique(_pack_voxels(idx >> 1))
        LOGGER.warning(
            "cloud cache exceeded %d voxels: coarsened to %.3f m, %d kept",
            _STORE_MAX,
            self.voxel,
            self._keys.size,
        )


def _pack_voxels(idx: np.ndarray) -> np.ndarray:
    """(N, 3) integer voxel indices -> int64 ids (out-of-range rows dropped)."""
    ids = idx.astype(np.int64) + _AXIS_OFFSET
    inside = np.all((ids >= 0) & (ids <= _AXIS_MAX), axis=1)
    if not inside.all():
        ids = ids[inside]
    if ids.shape[0] == 0:
        return _EMPTY_KEYS
    return (ids[:, 0] << (2 * _AXIS_BITS)) | (ids[:, 1] << _AXIS_BITS) | ids[:, 2]


def _unpack_voxels(keys: np.ndarray) -> np.ndarray:
    """int64 ids -> (N, 3) int32 voxel indices."""
    mask = (1 << _AXIS_BITS) - 1
    return np.stack(
        [
            (keys >> (2 * _AXIS_BITS)) - _AXIS_OFFSET,
            ((keys >> _AXIS_BITS) & mask) - _AXIS_OFFSET,
            (keys & mask) - _AXIS_OFFSET,
        ],
        axis=1,
    ).astype(np.int32)


def _pack_payload(keys: np.ndarray, voxel: float) -> tuple[bytes, int]:
    total = int(keys.size)
    if total == 0:
        return b"", 0
    if total > _PACK_MAX:
        # Doubling the grid keeps the coverage even; a plain stride over the
        # (x, y, z)-sorted keys would sample in bands and punch holes in walls.
        while keys.size > _PACK_MAX * _PACK_COARSEN_TO:
            keys = np.unique(_pack_voxels(_unpack_voxels(keys) >> 1))
            voxel *= 2.0
        if keys.size > _PACK_MAX:
            # Even spacing beats an integer stride: a stride of 2 turned a
            # 66k store into 33k points instead of the 60k the caller asked for.
            idx = np.linspace(0, keys.size - 1, _PACK_MAX).astype(np.int64)
            keys = keys[idx]
    count = int(keys.size)
    centers = (_unpack_voxels(keys).astype(np.float32) + 0.5) * np.float32(voxel)
    return struct.pack("<I", count) + centers.tobytes(), count


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
