"""Python facade for the C++17 pybind11 geometry kernel (_geom_kernel).

Provides:
  - GeodesicCache: bounded multi-source Dial-queue Dijkstra distance queries &
    exact A* geodesic shortest paths on an OccupancyView.
  - optimize_residual_viewpoints_cpp: 2D Bresenham LoS + 3D vertical FoV +
    submodular greedy set-cover viewpoint optimizer.
  - extract_corridor_bottlenecks_cpp: medial-axis narrow passage extraction and
    spacious yield-point localization.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .occupancy_view import OccupancyView

try:
    from . import _geom_kernel as _cpp  # type: ignore[attr-defined]
except ImportError:
    _cpp = None
    # Support running pytest directly with PYTHONPATH=src/hier_explore_core after colcon build
    _repo_root = Path(__file__).resolve().parents[3]
    _candidates = list(
        (_repo_root / "install" / "hier_explore_core").glob(
            "lib/python*/site-packages/hier_explore_core/_geom_kernel*.so"
        )
    ) + list((_repo_root / "build" / "hier_explore_core").glob("_geom_kernel*.so"))
    for _so in _candidates:
        try:
            _spec = importlib.util.spec_from_file_location("_geom_kernel", str(_so))
            if _spec is not None and _spec.loader is not None:
                _mod = importlib.util.module_from_spec(_spec)
                _spec.loader.exec_module(_mod)
                _cpp = _mod
                break
        except Exception:
            continue


def has_cpp_kernel() -> bool:
    """Return True when the compiled C++17 _geom_kernel extension is loaded."""
    return _cpp is not None


class GeodesicCache:
    """Per-cycle geodesic distance & path query helper backed by C++17 _geom_kernel.

    Precomputes robot-to-candidate geodesic distances in a single multi-source
    C++ Dial-queue Dijkstra pass and exposes a drop-in ``distance_fn(x1, y1, x2, y2)``
    callback for ``scheduler.py``.
    """

    def __init__(
        self,
        occ: OccupancyView,
        min_clearance_m: float = 0.22,
        max_dist_m: float = 0.0,
        snap_radius_m: float = 0.65,
    ) -> None:
        self.occ = occ
        self.traversable = np.ascontiguousarray(occ.is_free, dtype=np.uint8)
        self.obstacle_dt = np.ascontiguousarray(occ.obstacle_dt, dtype=np.float32)
        # Cap clearance used for geodesic graph connectivity at 0.15 m so tight
        # doorways and tunnel timber-prop passages remain connected in Dijkstra.
        eff_clr_m = min(float(min_clearance_m), 0.15)
        self.min_clearance_cells = max(0.0, eff_clr_m / max(occ.resolution, 1e-6))
        self.max_dist_m = float(max_dist_m)
        self.snap_radius_m = max(float(snap_radius_m), 0.90)
        self.allow_on_demand_astar: bool = True
        self._pair_cache: Dict[Tuple[int, int, int, int], float] = {}
        self._path_cache: Dict[Tuple[int, int, int, int], Tuple[float, np.ndarray]] = {}

    def _same_or_snapped_component(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        c1 = self.occ.component_at_world(x1, y1)
        c2 = self.occ.component_at_world(x2, y2)
        if c1 > 0 and c2 > 0:
            return c1 == c2
        # If robot pose or retracted goal momentarily sits on a non-free cell,
        # treat as reachable so scheduling never starves.
        return True

    @staticmethod
    def _qkey(x: float, y: float) -> Tuple[int, int]:
        return (int(round(x * 20.0)), int(round(y * 20.0)))

    def precompute_distances(
        self,
        sources_xy: Sequence[Tuple[float, float]],
        targets_xy: Sequence[Tuple[float, float]],
        max_dist_m: Optional[float] = None,
    ) -> np.ndarray:
        """Batch-compute S x T geodesic distances in a single C++ call and populate cache."""
        ns = len(sources_xy)
        nt = len(targets_xy)
        if ns == 0 or nt == 0:
            return np.empty((ns, nt), dtype=np.float64)

        srcs = np.asarray(sources_xy, dtype=np.float64).reshape(ns, 2)
        tgts = np.asarray(targets_xy, dtype=np.float64).reshape(nt, 2)
        md = self.max_dist_m if max_dist_m is None else float(max_dist_m)

        if _cpp is not None:
            mat = np.asarray(
                _cpp.geodesic_distances(
                    self.traversable,
                    self.obstacle_dt,
                    self.occ.resolution,
                    self.occ.origin_x,
                    self.occ.origin_y,
                    srcs,
                    tgts,
                    md,
                    self.min_clearance_cells,
                    self.snap_radius_m,
                ),
                dtype=np.float64,
            )
        else:
            diff = srcs[:, None, :] - tgts[None, :, :]
            mat = np.hypot(diff[:, :, 0], diff[:, :, 1])
            if md > 0.0:
                mat = np.where(mat <= md, mat, np.inf)

        for i in range(ns):
            sx, sy = float(srcs[i, 0]), float(srcs[i, 1])
            sk = self._qkey(sx, sy)
            for j in range(nt):
                tx, ty = float(tgts[j, 0]), float(tgts[j, 1])
                tk = self._qkey(tx, ty)
                d = float(mat[i, j])
                if not math.isfinite(d) and self._same_or_snapped_component(sx, sy, tx, ty):
                    d = 1.35 * math.hypot(tx - sx, ty - sy)
                    mat[i, j] = d
                self._pair_cache[(sk[0], sk[1], tk[0], tk[1])] = d
                self._pair_cache[(tk[0], tk[1], sk[0], sk[1])] = d
        return mat

    def precompute_pairwise(
        self,
        points_xy: Sequence[Tuple[float, float]],
        max_dist_m: Optional[float] = None,
    ) -> np.ndarray:
        """Batch-compute symmetric M x M geodesic distance matrix for Open-TSP."""
        n = len(points_xy)
        if n == 0:
            return np.empty((0, 0), dtype=np.float64)
        pts = np.asarray(points_xy, dtype=np.float64).reshape(n, 2)
        md = self.max_dist_m if max_dist_m is None else float(max_dist_m)

        if _cpp is not None:
            mat = np.asarray(
                _cpp.geodesic_matrix(
                    self.traversable,
                    self.obstacle_dt,
                    self.occ.resolution,
                    self.occ.origin_x,
                    self.occ.origin_y,
                    pts,
                    md,
                    self.min_clearance_cells,
                    self.snap_radius_m,
                ),
                dtype=np.float64,
            )
        else:
            diff = pts[:, None, :] - pts[None, :, :]
            mat = np.hypot(diff[:, :, 0], diff[:, :, 1])

        for i in range(n):
            xi, yi = float(pts[i, 0]), float(pts[i, 1])
            ki = self._qkey(xi, yi)
            for j in range(i, n):
                xj, yj = float(pts[j, 0]), float(pts[j, 1])
                kj = self._qkey(xj, yj)
                d = float(mat[i, j])
                if not math.isfinite(d) and self._same_or_snapped_component(xi, yi, xj, yj):
                    d = 1.35 * math.hypot(xj - xi, yj - yi)
                    mat[i, j] = d
                    mat[j, i] = d
                self._pair_cache[(ki[0], ki[1], kj[0], kj[1])] = d
                self._pair_cache[(kj[0], kj[1], ki[0], ki[1])] = d
        return mat

    def shortest_path(
        self,
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
        max_dist_m: Optional[float] = None,
    ) -> Tuple[float, np.ndarray]:
        """Compute exact geodesic distance (m) and world path (P, 2) via C++ A*."""
        sx, sy = float(start_xy[0]), float(start_xy[1])
        gx, gy = float(goal_xy[0]), float(goal_xy[1])
        sk = self._qkey(sx, sy)
        gk = self._qkey(gx, gy)
        ck = (sk[0], sk[1], gk[0], gk[1])
        if ck in self._path_cache:
            return self._path_cache[ck]

        md = self.max_dist_m if max_dist_m is None else float(max_dist_m)
        if _cpp is not None:
            dist_m, path_xy = _cpp.geodesic_path(
                self.traversable,
                self.obstacle_dt,
                self.occ.resolution,
                self.occ.origin_x,
                self.occ.origin_y,
                sx,
                sy,
                gx,
                gy,
                md,
                self.min_clearance_cells,
                self.snap_radius_m,
            )
            dist_f = float(dist_m)
            path_arr = np.asarray(path_xy, dtype=np.float64).reshape(-1, 2)
        else:
            dist_f = math.hypot(gx - sx, gy - sy)
            path_arr = np.array([[sx, sy], [gx, gy]], dtype=np.float64)

        self._path_cache[ck] = (dist_f, path_arr)
        self._pair_cache[ck] = dist_f
        self._pair_cache[(gk[0], gk[1], sk[0], sk[1])] = dist_f
        return dist_f, path_arr

    def distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Drop-in DistanceFn(x1, y1, x2, y2) returning cached or on-demand geodesic distance."""
        if abs(x1 - x2) < 1e-6 and abs(y1 - y2) < 1e-6:
            return 0.0
        k1 = self._qkey(x1, y1)
        k2 = self._qkey(x2, y2)
        ck = (k1[0], k1[1], k2[0], k2[1])
        cached = self._pair_cache.get(ck)
        if cached is not None:
            return cached
        if not self.allow_on_demand_astar:
            dist_m = (
                1.35 * math.hypot(x2 - x1, y2 - y1)
                if self._same_or_snapped_component(x1, y1, x2, y2)
                else float("inf")
            )
            self._pair_cache[ck] = dist_m
            return dist_m
        dist_m, _ = self.shortest_path((x1, y1), (x2, y2))
        if not math.isfinite(dist_m) and self._same_or_snapped_component(x1, y1, x2, y2):
            dist_m = 1.35 * math.hypot(x2 - x1, y2 - y1)
            self._pair_cache[ck] = dist_m
        return dist_m


def has_line_of_sight_cpp(
    occ: OccupancyView,
    r0: int,
    c0: int,
    r1: int,
    c1: int,
    lethal_threshold: int = 50,
) -> bool:
    """2D Bresenham line-of-sight query on an OccupancyView."""
    if _cpp is not None:
        return bool(
            _cpp.has_line_of_sight(
                np.ascontiguousarray(occ.data, dtype=np.int8),
                int(r0),
                int(c0),
                int(r1),
                int(c1),
                int(lethal_threshold),
            )
        )
    # Pure-Python Bresenham fallback
    r, c = int(r0), int(c0)
    dr = abs(int(r1) - r)
    dc = abs(int(c1) - c)
    sr = 1 if r < r1 else -1
    sc = 1 if c < c1 else -1
    err = dc - dr
    while True:
        if r == r1 and c == c1:
            return True
        if not (r == r0 and c == c0):
            if not occ.contains_cell(r, c) or int(occ.data[r, c]) > lethal_threshold:
                return False
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


def optimize_residual_viewpoints_cpp(
    occ: OccupancyView,
    columns_xyzz: np.ndarray,
    cluster_ids: np.ndarray,
    *,
    r_min: float = 0.8,
    r_max: float = 3.5,
    num_samples: int = 24,
    min_clearance: float = 0.55,
    sensor_z: float = 0.25,
    vfov_min_deg: float = -30.0,
    vfov_max_deg: float = 55.0,
    max_eff_range: float = 5.0,
    max_viewpoints: int = 4,
    dedupe_radius: float = 1.5,
) -> List[dict]:
    """Run C++ 2D Bresenham LoS + 3D vertical FoV + submodular set-cover viewpoint optimizer."""
    if _cpp is None or len(columns_xyzz) == 0:
        return []
    return list(
        _cpp.optimize_residual_viewpoints(
            np.ascontiguousarray(occ.data, dtype=np.int8),
            np.ascontiguousarray(occ.obstacle_dt, dtype=np.float32),
            float(occ.resolution),
            float(occ.origin_x),
            float(occ.origin_y),
            np.ascontiguousarray(columns_xyzz, dtype=np.float64),
            np.ascontiguousarray(cluster_ids, dtype=np.int32),
            float(r_min),
            float(r_max),
            int(num_samples),
            float(min_clearance),
            float(sensor_z),
            math.radians(float(vfov_min_deg)),
            math.radians(float(vfov_max_deg)),
            float(max_eff_range),
            int(max_viewpoints),
            float(dedupe_radius),
            50,
        )
    )


def extract_corridor_bottlenecks_cpp(
    occ: OccupancyView,
    *,
    robot_radius: float = 0.22,
    narrow_margin_m: float = 0.35,
    min_passable_margin_m: float = 0.02,
    min_segment_cells: int = 4,
    yield_search_radius_m: float = 3.0,
    yield_min_separation_m: float = 1.0,
    max_yield_points: int = 4,
    min_segment_length_m: float = 0.0,
) -> dict:
    """Extract narrow-passage bottlenecks E_narrow and yield points via C++ kernel."""
    if _cpp is None:
        return {"labels": np.zeros((occ.height, occ.width), dtype=np.int32), "segments": []}
    return dict(
        _cpp.extract_corridor_bottlenecks(
            np.ascontiguousarray(occ.is_free, dtype=np.uint8),
            np.ascontiguousarray(occ.obstacle_dt, dtype=np.float32),
            float(occ.resolution),
            float(occ.origin_x),
            float(occ.origin_y),
            float(robot_radius),
            float(narrow_margin_m),
            float(min_passable_margin_m),
            int(min_segment_cells),
            float(yield_search_radius_m),
            float(yield_min_separation_m),
            int(max_yield_points),
            float(min_segment_length_m),
        )
    )
