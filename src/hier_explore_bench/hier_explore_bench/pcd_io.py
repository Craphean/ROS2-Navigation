"""Fast binary & ASCII PCD point-cloud I/O and voxel downsampling (no PCL/Open3D required)."""

from __future__ import annotations

from pathlib import Path
from typing import Union

import numpy as np


def write_pcd(path: Union[str, Path], points: np.ndarray, binary: bool = True) -> Path:
    """Write an (N, 3) float32 XYZ point cloud to binary or ASCII PCD format."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    pts = np.ascontiguousarray(np.asarray(points, dtype=np.float32).reshape(-1, 3))
    n = int(pts.shape[0])
    mode = "binary" if binary else "ascii"
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA {mode}\n"
    ).encode("ascii")

    if binary:
        with out.open("wb") as f:
            f.write(header)
            if n > 0:
                f.write(pts.tobytes())
    else:
        with out.open("wb") as f:
            f.write(header)
            if n > 0:
                np.savetxt(f, pts, fmt="%.5f %.5f %.5f")
    return out


def read_pcd(path: Union[str, Path]) -> np.ndarray:
    """Read an XYZ PCD file (binary or ASCII) into an (N, 3) float32 numpy array."""
    p = Path(path)
    raw = p.read_bytes()
    header_lines = []
    pos = 0
    while pos < len(raw):
        nl = raw.find(b"\n", pos)
        if nl == -1:
            line = raw[pos:].decode("ascii", errors="ignore").strip()
            pos = len(raw)
        else:
            line = raw[pos:nl].decode("ascii", errors="ignore").strip()
            pos = nl + 1
        header_lines.append(line)
        if line.upper().startswith("DATA "):
            break

    meta = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        meta[parts[0].upper()] = parts[1:]

    fields = [f.lower() for f in meta.get("FIELDS", ["x", "y", "z"])]
    sizes = [int(s) for s in meta.get("SIZE", ["4"] * len(fields))]
    types = [t.upper() for t in meta.get("TYPE", ["F"] * len(fields))]
    counts = [int(c) for c in meta.get("COUNT", ["1"] * len(fields))]
    n_points = int(meta.get("POINTS", ["0"])[0])
    data_mode = meta.get("DATA", ["ascii"])[0].lower()

    if n_points == 0:
        return np.empty((0, 3), dtype=np.float32)

    ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
    body = raw[pos:]

    if data_mode == "ascii":
        text_lines = [ln for ln in body.decode("ascii", errors="ignore").splitlines() if ln.strip()]
        arr = np.loadtxt(text_lines, dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        pts = np.column_stack((arr[:, ix], arr[:, iy], arr[:, iz])).astype(np.float32)
    elif data_mode == "binary":
        dtype_map = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 4): "<i4", ("U", 4): "<u4", ("U", 1): "<u1"}
        dt_fields = []
        for f, s, t, c in zip(fields, sizes, types, counts):
            dt = dtype_map.get((t, s), f"V{s}")
            dt_fields.append((f, dt) if c == 1 else (f, dt, (c,)))
        struct_dt = np.dtype(dt_fields)
        rec = np.frombuffer(body[: n_points * struct_dt.itemsize], dtype=struct_dt)
        pts = np.column_stack((rec["x"], rec["y"], rec["z"])).astype(np.float32)
    else:
        raise ValueError(f"Unsupported PCD DATA mode: {data_mode}")

    finite = np.isfinite(pts).all(axis=1)
    return pts[finite] if not finite.all() else pts


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Fast 1D int64-packed voxel grid deduplication."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if pts.size == 0 or voxel_size <= 0.0:
        return pts
    keys = np.floor(pts / float(voxel_size)).astype(np.int64) + (1 << 20)
    np.clip(keys, 0, (1 << 21) - 1, out=keys)
    packed = (keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]
    _, first = np.unique(packed, return_index=True)
    return pts[np.sort(first)]
