"""Residual 3D blind-spot detection and viewpoint selection."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .geom_kernel import has_cpp_kernel, optimize_residual_viewpoints_cpp
from .occupancy_view import FREE, UNKNOWN, OccupancyView
from .voxel_index import VoxelColumnIndex


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
    # Pillar 3: C++ 2D Bresenham LoS + 3D vertical FoV + submodular viewpoint optimization
    use_los_fov_optimizer: bool = True
    viewpoint_r_min: float = 0.8
    viewpoint_r_max: float = 3.5
    viewpoint_samples: int = 24
    sensor_z: float = 0.25
    vfov_min_deg: float = -30.0
    vfov_max_deg: float = 55.0
    max_eff_range: float = 5.0


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
    # should drive). Blacklists/attempt counters key on the cluster, because
    # the viewpoint may shift slightly between detections.
    cluster_x: float = 0.0
    cluster_y: float = 0.0

    # Common candidate interface shared with FrontierCluster.
    kind: str = "residual"
    total_gain: float = 0.0
    gain_2d: float = 0.0
    gain_3d: float = 0.0
    size: int = 0
    yaw_world: float = 0.0

    def __post_init__(self) -> None:
        self.yaw_world = self.yaw

    @property
    def world_x(self) -> float:
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
    qualifying cluster scores before the cap, used by the convergence tracker.
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
        obs.astype(np.float32),
        -1,
        (k_density, k_density),
        normalize=True,
        borderType=cv2.BORDER_CONSTANT,
    )
    unknown_small = cv2.boxFilter(
        (index.map_state == UNKNOWN).astype(np.float32),
        -1,
        (k_small, k_small),
        normalize=True,
        borderType=cv2.BORDER_CONSTANT,
    )
    structure_count = cv2.boxFilter(
        obs.astype(np.float32),
        -1,
        (k_structure, k_structure),
        normalize=False,
        borderType=cv2.BORDER_CONSTANT,
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
        sparse = obs & (index.raw_count <= 1)
        dense_neighbourhood = density_small >= params.min_density
        mask = mask | (map_free & sparse & dense_neighbourhood)

    if not mask.any():
        return [], 0.0

    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    del n_labels

    blacklist = list(blacklist)
    attempts = attempts or {}

    # Identify qualifying clusters (size, blacklist, max attempts)
    cluster_meta: Dict[int, Tuple[float, float, int, float, float]] = {}
    raw_gain = 0.0
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
        area = cell_count * res * res
        score = params.score_scale * area
        cluster_meta[label] = (cx, cy, cell_count, area, score)

    if not cluster_meta:
        return [], 0.0

    # Preferred Pillar 3 path: C++ 2D Bresenham LoS + 3D vertical FoV + submodular set cover
    if params.use_los_fov_optimizer and has_cpp_kernel():
        qual_mask = np.isin(labels, list(cluster_meta.keys()))
        ry, rx = np.nonzero(qual_mask)
        if len(ry) > 0:
            # Cap very large clusters to <= 600 sampled columns so LoS raycasting stays < 2 ms
            if len(ry) > 600:
                stride = int(math.ceil(len(ry) / 600.0))
                ry = ry[::stride]
                rx = rx[::stride]
            cxs = index.origin_x + (rx.astype(np.float64) + 0.5) * res
            cys = index.origin_y + (ry.astype(np.float64) + 0.5) * res
            zmins = index.zmin[ry, rx].astype(np.float64)
            zmaxs = index.zmax[ry, rx].astype(np.float64)
            bad_z = (~np.isfinite(zmins)) | (~np.isfinite(zmaxs)) | (zmaxs < zmins)
            zmins = np.where(bad_z, 0.25, zmins)
            zmaxs = np.where(bad_z, 1.50, zmaxs)
            cols_xyzz = np.column_stack((cxs, cys, zmins, zmaxs))
            cids = labels[ry, rx].astype(np.int32)

            cpp_views = optimize_residual_viewpoints_cpp(
                view,
                cols_xyzz,
                cids,
                r_min=params.viewpoint_r_min,
                r_max=params.viewpoint_r_max,
                num_samples=params.viewpoint_samples,
                min_clearance=params.min_clearance,
                sensor_z=params.sensor_z,
                vfov_min_deg=params.vfov_min_deg,
                vfov_max_deg=params.vfov_max_deg,
                max_eff_range=params.max_eff_range,
                max_viewpoints=params.max_candidates,
                dedupe_radius=params.dedupe_radius,
            )

            selected: List[ResidualCandidate] = []
            for vp in cpp_views:
                vx, vy = float(vp["x"]), float(vp["y"])
                if _is_blacklisted(vx, vy, blacklist, params.blacklist_radius):
                    continue
                cid = int(vp["cluster_id"])
                cx, cy, cell_count, area, base_score = cluster_meta[cid]
                cov_ratio = max(0.25, min(1.5, float(vp.get("coverage_ratio", 1.0))))
                score = base_score * cov_ratio
                component = view.component_at_world(vx, vy)
                selected.append(
                    ResidualCandidate(
                        x=vx,
                        y=vy,
                        yaw=float(vp["yaw"]),
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
            if selected:
                raw_gain = sum(m[4] for m in cluster_meta.values())
                return selected, raw_gain

    # Fallback path (or ablation when use_los_fov_optimizer=False)
    raw_candidates: List[ResidualCandidate] = []
    for _label, (cx, cy, cell_count, area, score) in cluster_meta.items():
        view_xy = _find_viewpoint(
            view, cx, cy, params.viewpoint_search_radius, params.min_clearance
        )
        if view_xy is None:
            continue
        vx, vy = view_xy
        if _is_blacklisted(vx, vy, blacklist, params.blacklist_radius):
            continue
        yaw = math.atan2(cy - vy, cx - vx)
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
    selected_fb: List[ResidualCandidate] = []
    for cand in raw_candidates:
        if len(selected_fb) >= params.max_candidates:
            break
        if any(
            math.hypot(cand.x - s.x, cand.y - s.y) < params.dedupe_radius
            for s in selected_fb
        ):
            continue
        selected_fb.append(cand)
    return selected_fb, raw_gain


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
    out[r0:r1, c0:c1] = a[r0 - dr : r1 - dr, c0 - dc : c1 - dc]
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
