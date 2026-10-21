"""Position-only full-revolution observations using a finite, documented beam grid."""
from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

from voxel_exploration_core import OCCUPIED, UNKNOWN, SensorModel, evaluate_candidates
from voxel_native import library
from voxel_frontier_policy import FrontierPolicy, prepare_targets, select_frontier_batch

DEFAULT_PROFILE = Path(__file__).resolve().parents[1]/"config/lidar360_os0.yaml"


@dataclass(frozen=True)
class Lidar360:
    name: str
    horizontal_samples: int
    vertical_samples: int
    vertical_min_deg: float
    vertical_max_deg: float
    scan_hz: float
    min_range: float
    max_range: float
    height: float
    offset_x: float = 0.
    offset_y: float = 0.
    azimuth_phase_deg: float = -180.

    def __post_init__(self):
        if (self.horizontal_samples < 4 or self.vertical_samples < 2 or self.scan_hz <= 0
                or self.offset_x != 0 or self.offset_y != 0):
            raise ValueError("360 mode needs a centered sensor, positive rate and a full beam grid")
        self.sensor()  # Validate range and vertical field of view.

    @classmethod
    def load(cls, path=DEFAULT_PROFILE):
        path = Path(path)
        if not path.is_absolute():
            path = DEFAULT_PROFILE.parents[1]/path
        return cls(**yaml.safe_load(path.read_text()))

    def sensor(self, max_range=None):
        limit = self.max_range if max_range is None else float(max_range)
        if not self.min_range < limit <= self.max_range:
            raise ValueError(f"360 effective range must be in ({self.min_range}, {self.max_range}] m")
        return SensorModel(self.height, self.min_range, limit, np.deg2rad(self.vertical_min_deg),
                           np.deg2rad(self.vertical_max_deg), self.offset_x, self.offset_y)

    @lru_cache(maxsize=8)
    def vectors(self, max_range=None, planar=False):
        sensor = self.sensor(max_range)
        azimuth = np.deg2rad(self.azimuth_phase_deg)+np.arange(self.horizontal_samples)*2*np.pi/self.horizontal_samples
        elevation = np.array([0.]) if planar else np.linspace(sensor.vfov_min, sensor.vfov_max, self.vertical_samples)
        el, az = np.meshgrid(elevation, azimuth, indexing="ij")
        directions = sensor.max_range*np.column_stack((
            (np.cos(el)*np.cos(az)).ravel(), (np.cos(el)*np.sin(az)).ravel(), np.sin(el).ravel()))
        directions.flags.writeable = False
        return directions

    def metadata(self):
        return dict(name=self.name, horizontal_fov_deg=360., horizontal_samples=self.horizontal_samples,
                    vertical_samples=self.vertical_samples, horizontal_step_deg=360/self.horizontal_samples,
                    vertical_fov_deg=[self.vertical_min_deg, self.vertical_max_deg], scan_hz=self.scan_hz,
                    rays_per_scan=self.horizontal_samples*self.vertical_samples,
                    rays_per_second=self.horizontal_samples*self.vertical_samples*self.scan_hz,
                    sensor_offset=[self.offset_x, self.offset_y], sensor_height=self.height,
                    observation_heading_required=False, offline_azimuth_phase="world_fixed",
                    source="https://data.ouster.io/downloads/datasheets/datasheet-revd-v2p1-os0.pdf")


def first_unknowns(voxels, origin, vectors, min_range):
    """First unknown touched by each finite beam; known occupied cells occlude.

    Return only measured-volume intersections beyond the blind range. No beam
    is synthesized towards voxel centers and no free-space dilation is used.
    """
    origin = np.ascontiguousarray(origin, dtype=np.float64)
    source, inside = voxels.indices(origin)
    if inside:
        value = voxels.state.ravel()[source]
        # The first cell blocks every ray. Avoid allocating/tracing a full
        # revolution when this entire unknown cell is inside the blind range.
        farthest = np.abs(voxels.centers[source]-origin)+voxels.resolution/2
        if value == OCCUPIED or (value == UNKNOWN and np.linalg.norm(farthest) <= min_range):
            return np.empty(0, dtype=np.int64)
    endpoints = np.ascontiguousarray(origin+vectors)
    hits = np.full(len(vectors), -1, dtype=np.int64)
    native = library()
    if native is not None:
        native.voxel_first_unknown(voxels.state, voxels.shape_xyz, voxels.origin, voxels.resolution,
                                   origin, endpoints, len(endpoints), min_range, hits)
    else:
        active = np.ones(len(vectors), dtype=bool)
        lengths = np.linalg.norm(vectors, axis=1)
        for rays, ids, inside, _ in voxels._walk(origin, endpoints, active):
            rows = rays[inside & active[rays]]
            cells = ids[inside & active[rays]]
            if not len(rows):
                continue
            # Entry/exit distances distinguish actual intersections from an
            # endpoint cell touched only at the maximum range.
            low = voxels.centers[cells]-voxels.resolution/2
            with np.errstate(divide="ignore", invalid="ignore"):
                a, b = (low-origin)/vectors[rows], (low+voxels.resolution-origin)/vectors[rows]
            entering = np.where(np.abs(vectors[rows]) > 1e-15, np.minimum(a, b), -np.inf).max(axis=1)
            leaving = np.where(np.abs(vectors[rows]) > 1e-15, np.maximum(a, b), np.inf).min(axis=1)
            values = voxels.state.ravel()[cells]
            unknown = values == UNKNOWN
            usable = unknown & (leaving*lengths[rows] > min_range+1e-9) & (entering < 1.-1e-9)
            hits[rows[usable]] = cells[usable]
            # A blind unknown can occlude: it is not automatically free.
            active[rows[unknown | (values == OCCUPIED)]] = False
    return np.unique(hits[hits >= 0])


class FullCircleVisibility:
    def __init__(self, voxels, profile):
        self.voxels, self.profile = voxels, profile

    def visible_views(self, origins, sensor):
        """Full finite-beam unknown sets, independently of robot ownership."""
        origins = np.ascontiguousarray(origins, dtype=np.float64).reshape(-1, 3)
        if not len(origins):
            return []
        native = library()
        vectors = self.profile.vectors(sensor.max_range)
        if native is not None:
            allowed = np.ones(self.voxels.state.size, dtype=np.uint8)
            owners = np.full(self.voxels.state.size, -1, dtype=np.int32)
            offsets = np.zeros(len(origins)+1, dtype=np.int64)
            hits = np.empty(max(1024, len(origins)*2048), dtype=np.int64)
            while True:
                owners.fill(-1)
                count = native.voxel_collect_views(
                    self.voxels.state, self.voxels.shape_xyz, self.voxels.origin,
                    self.voxels.resolution, origins, len(origins), vectors, len(vectors),
                    sensor.min_range, allowed, owners, hits, len(hits), offsets)
                if count < 0:
                    break
                if count <= len(hits):
                    return [np.sort(hits[a:b].copy()) for a, b in zip(offsets, offsets[1:])]
                hits = np.empty(count, dtype=np.int64)
        return [first_unknowns(self.voxels, origin, vectors, sensor.min_range) for origin in origins]

    def claim_views(self, origins, targets, sensor):
        """Each target belongs only to its first (nearest ordered) valid view."""
        origins = np.ascontiguousarray(origins, dtype=np.float64).reshape(-1, 3)
        allowed = np.zeros(self.voxels.state.size, dtype=np.uint8)
        allowed[targets] = 1
        owners = np.full(self.voxels.state.size, -1, dtype=np.int32)
        native = library()
        vectors = self.profile.vectors(sensor.max_range)
        if native is not None and native.voxel_claim_views(
                self.voxels.state, self.voxels.shape_xyz, self.voxels.origin, self.voxels.resolution,
                origins, len(origins), vectors, len(vectors), sensor.min_range, allowed, owners):
            ids = np.flatnonzero(owners >= 0)
            order = np.argsort(owners[ids], kind="stable")
            counts = np.bincount(owners[ids], minlength=len(origins))
            return np.split(ids[order], np.cumsum(counts)[:-1]) if len(origins) else []
        result = []
        for origin in origins:
            ids = first_unknowns(self.voxels, origin, vectors, sensor.min_range)
            ids = ids[allowed[ids].astype(bool)]
            result.append(ids)
            allowed[ids] = 0
        return result

    def __call__(self, origin, targets, sensor, *, sorted_targets=False, optimistic=False):
        if optimistic:
            # Exclusion remains the same optimistic geometric proof as the
            # original algorithm. Missing one azimuth phase is not proof that
            # a voxel is permanently unobservable.
            return self.voxels.visible(origin, targets, sensor, optimistic=True)
        ids = first_unknowns(self.voxels, origin, self.profile.vectors(sensor.max_range), sensor.min_range)
        if sorted_targets:
            # The boundary array is already sorted. intersect1d re-sorted its
            # entire volume for every parking position (including empty hits).
            offsets = np.searchsorted(targets, ids)
            valid = offsets < len(targets)
            ids, offsets = ids[valid], offsets[valid]
            return ids[targets[offsets] == ids]
        return np.intersect1d(ids, targets, assume_unique=True)


def evaluate_candidates_360(voxels, ground, starts, sensor, stride=.25, profile=None,
                            *, ownership="nearest", audit_when_idle=True, blocked=(),
                            target_policy="exhaustive", frontier_policy=None,
                            speeds=(.22, .22), observation_time=1.):
    """One task per position; coverage is one complete 360-degree acquisition."""
    profile = Lidar360.load() if profile is None else profile
    if target_policy not in ("exhaustive", "frontier"):
        raise ValueError("Unknown target policy")
    policy = frontier_policy or FrontierPolicy()
    ignored, support = prepare_targets(voxels, policy) if target_policy == "frontier" else (set(), None)
    coverage = evaluate_candidates(voxels, ground, starts, sensor, stride,
                                   visibility=FullCircleVisibility(voxels, profile),
                                   ownership=ownership, audit_when_idle=audit_when_idle,
                                   blocked=blocked, ignored=ignored)
    coverage = replace(coverage, ignored=ignored, target_policy=target_policy)
    if target_policy == "frontier":
        coverage = select_frontier_batch(coverage, voxels, ground, starts, support, policy,
                                         speeds, observation_time, sensor.height)
    return coverage
