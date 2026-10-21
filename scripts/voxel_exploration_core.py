#!/usr/bin/env python3
"""Finite-volume, three-state exploration and coverage-aware two-robot routing.

No ROS dependency. Unknown is a real state, never an inflated/free proxy.
All visibility and exclusion claims are relative to the configured candidate
lattice, sensor model and bounded region, not to a continuous-space oracle.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from collections import Counter, OrderedDict
import heapq
import math
import hashlib
import threading
import zlib
from typing import Optional

import cv2
import numpy as np

from voxel_route_solver import _held_karp, _reconstruct_route
from voxel_native import library

UNKNOWN, FREE, OCCUPIED = -1, 0, 1


@dataclass(frozen=True)
class SensorModel:
    height: float = 0.22
    min_range: float = 0.10
    max_range: float = 5.0
    vfov_min: float = -0.5236
    vfov_max: float = 0.9599
    offset_x: float = 0.0
    offset_y: float = 0.0

    def __post_init__(self):
        if not 0 <= self.min_range < self.max_range:
            raise ValueError("Sensor range must satisfy 0 <= min < max")
        if not -math.pi / 2 <= self.vfov_min < self.vfov_max <= math.pi / 2:
            raise ValueError("Invalid vertical FOV")

    def origin(self, xy):
        # All viewpoint goals use world yaw=0 for a horizontal 360 degree LiDAR.
        return xy[0] + self.offset_x, xy[1] + self.offset_y, self.height


class VoxelMap:
    """Dense bounded occupancy, indexed [z, y, x], with batched 3D DDA rays."""

    def __init__(self, bounds, resolution=0.05):
        self.bounds = np.asarray(bounds, dtype=float)
        if self.bounds.shape != (6,) or not np.isfinite(self.bounds).all():
            raise ValueError("bounds must be [xmin,xmax,ymin,ymax,zmin,zmax]")
        self.resolution = float(resolution)
        self.origin = self.bounds[::2].copy()
        extent = self.bounds[1::2] - self.origin
        if self.resolution <= 0 or np.any(extent <= 0):
            raise ValueError("Positive resolution and bounds extents required")
        cells = extent / self.resolution
        if not np.allclose(cells, np.round(cells)):
            raise ValueError("Each bounds extent must be a multiple of voxel_resolution")
        self.shape_xyz = np.round(cells).astype(int)
        if int(np.prod(self.shape_xyz)) > 5_000_000:
            raise ValueError("ROI exceeds 5 million voxels; use a coarser resolution")
        self.state = np.full(tuple(self.shape_xyz[::-1]), UNKNOWN, dtype=np.int8)
        self.revision = 0
        ids = np.arange(self.state.size, dtype=np.int64)
        nx, ny, _ = self.shape_xyz
        self.centers = np.empty((len(ids), 3), dtype=np.float64)
        for axis, coordinates in enumerate((ids % nx, ids // nx % ny, ids // (nx * ny))):
            self.centers[:, axis] = self.origin[axis] + (coordinates + .5) * self.resolution
        self.centers.flags.writeable = False

    def snapshot(self, empty=False):
        """Copy evidence while sharing immutable geometry (important at 5 cm)."""
        result = object.__new__(VoxelMap)
        result.bounds, result.origin = self.bounds.copy(), self.origin.copy()
        result.resolution, result.shape_xyz = self.resolution, self.shape_xyz.copy()
        result.centers = self.centers
        result.state = np.full_like(self.state, UNKNOWN) if empty else self.state.copy()
        result.revision = 0 if empty else self.revision
        return result

    def indices(self, points):
        xyz = np.floor((np.asarray(points) - self.origin) / self.resolution).astype(int)
        inside = ((xyz >= 0) & (xyz < self.shape_xyz)).all(axis=-1)
        clipped = np.clip(xyz, 0, self.shape_xyz - 1)
        ids = (clipped[..., 2] * self.shape_xyz[1] + clipped[..., 1]) * self.shape_xyz[0] + clipped[..., 0]
        return ids, inside

    @property
    def unknown(self):
        return np.flatnonzero(self.state.ravel() == UNKNOWN)

    def _walk(self, origin, endpoints, active=None):
        """Yield (ray IDs, voxel IDs, in-ROI, endpoint-cell) along each ray.

        Cross all tied axes together at exact edge/corner crossings. Zero
        length contacts with neighbouring cells are not volume observations.
        Rays may start/end outside the ROI; their interior is still integrated.
        """
        endpoints = np.asarray(endpoints, dtype=float).reshape(-1, 3)
        if not len(endpoints):
            return
        origin = np.asarray(origin, dtype=float)
        delta = endpoints - origin
        current = np.tile(np.floor((origin - self.origin) / self.resolution).astype(int), (len(endpoints), 1))
        end = np.floor((endpoints - self.origin) / self.resolution).astype(int)
        step = np.sign(delta).astype(int)
        with np.errstate(divide="ignore", invalid="ignore"):
            t_delta = np.where(delta != 0, self.resolution / np.abs(delta), np.inf)
            boundary = self.origin + (current + (step > 0)) * self.resolution
            t_next = np.where(delta != 0, (boundary - origin) / delta, np.inf)
        rays = np.arange(len(endpoints))
        limit = int(np.abs(end - current).sum(axis=1).max()) + 1
        for _ in range(limit):
            inside = ((current >= 0) & (current < self.shape_xyz)).all(axis=1)
            clipped = np.clip(current, 0, self.shape_xyz - 1)
            ids = (clipped[:, 2] * self.shape_xyz[1] + clipped[:, 1]) * self.shape_xyz[0] + clipped[:, 0]
            finished = (current == end).all(axis=1)
            yield rays, ids, inside, finished
            keep = ~finished
            if active is not None:
                keep &= active[rays]
            if not keep.any():
                return
            rays, current, end = rays[keep], current[keep], end[keep]
            step, t_next, t_delta = step[keep], t_next[keep], t_delta[keep]
            axes = t_next <= t_next.min(axis=1)[:, None] + 1e-12
            current += axes * step
            t_next = np.where(axes, t_next + t_delta, t_next)

    def integrate(self, origin, endpoints, hit=None):
        """Integrate finite world-frame rays; hit=False represents a known miss.

        Endpoints override free evidence in the same scan. No free dilation,
        no inference from missing/NaN beams, no clipping away useful rays whose
        endpoints are above/below the ROI.
        """
        endpoints = np.asarray(endpoints, dtype=float).reshape(-1, 3)
        origin = np.asarray(origin, dtype=float)
        if not np.isfinite(origin).all():
            raise ValueError("Non-finite sensor origin")
        valid = np.isfinite(endpoints).all(axis=1)
        hit = np.ones(len(endpoints), dtype=bool) if hit is None else np.asarray(hit, dtype=bool)
        if hit.shape != (len(endpoints),):
            raise ValueError("One hit flag per endpoint required")
        endpoints, hit = endpoints[valid], hit[valid]
        before = self.state.copy()
        flat = self.state.ravel()
        native = library()
        if native is not None:
            native.voxel_integrate(
                self.state, self.shape_xyz, self.origin, self.resolution,
                np.ascontiguousarray(origin), np.ascontiguousarray(endpoints),
                np.ascontiguousarray(hit, dtype=np.uint8), len(endpoints))
            changed = int(np.count_nonzero(before != self.state))
            if changed:
                self.revision += 1
            return changed
        for offset in range(0, len(endpoints), 4096):
            batch, flags = endpoints[offset:offset + 4096], hit[offset:offset + 4096]
            for rays, ids, inside, finished in self._walk(origin, batch):
                free = inside & ~(finished & flags[rays])
                flat[ids[free]] = FREE
        ids, inside = self.indices(endpoints)
        flat[ids[inside & hit]] = OCCUPIED
        changed = int(np.count_nonzero(before != self.state))
        if changed:
            self.revision += 1
        return changed

    def integrate_cloud(self, points_sensor, world_from_sensor, stamp=0.0,
                        min_range=0.1, max_range=10.0):
        points = np.asarray(points_sensor, dtype=float).reshape(-1, 3)
        ranges = np.linalg.norm(points, axis=1)
        valid = np.isfinite(points).all(axis=1) & (ranges >= min_range) & (ranges <= max_range)
        points, ranges = points[valid], ranges[valid]
        transform = np.asarray(world_from_sensor)
        endpoints = points @ transform[:3, :3].T + transform[:3, 3]
        # Finite endpoints at the configured maximum are no-return beams.
        self.integrate(transform[:3, 3], endpoints, ranges < max_range - 1e-4)
        return len(points)

    def visible(self, origin, target_ids, sensor, optimistic=False, sorted_targets=False):
        """Return target IDs visible through free space (or possible unknown).

        The first unknown on a ray is observable; deeper unknown voxels are
        deferred. Optimistic mode ignores unknown occlusion solely to prove
        exclusions, never to mark a voxel observed.
        """
        target_ids = np.asarray(target_ids, dtype=int)
        if not len(target_ids):
            return target_ids
        if not optimistic:
            source = np.floor((np.asarray(origin) - self.origin) / self.resolution).astype(int)
            lo, hi = np.maximum(0, source-1), np.minimum(self.shape_xyz, source+2)
            if np.all(hi > lo):
                local = self.state[lo[2]:hi[2], lo[1]:hi[1], lo[0]:hi[0]]
                if not np.any(local == FREE):
                    # Without a free first step, only the source's immediate
                    # neighbours can be the first unknown on a ray. This is an
                    # exact visibility implication, not a candidate/task cap.
                    z, y, x = np.mgrid[lo[2]:hi[2], lo[1]:hi[1], lo[0]:hi[0]]
                    nearby = ((z*self.shape_xyz[1]+y)*self.shape_xyz[0]+x).ravel()
                    if sorted_targets:
                        locations = np.searchsorted(target_ids, nearby)
                        valid = locations < len(target_ids)
                        locations = locations[valid]
                        target_ids = target_ids[locations[target_ids[locations] == nearby[valid]]]
                    else:
                        target_ids = target_ids[np.isin(target_ids, nearby)]
                    if not len(target_ids):
                        return target_ids
        native = library()
        if native is not None:
            targets = np.ascontiguousarray(target_ids, dtype=np.int64)
            output = np.empty_like(targets)
            count = native.voxel_visible(
                self.state, self.shape_xyz, self.origin, self.resolution,
                np.ascontiguousarray(origin, dtype=np.float64), targets, len(targets),
                sensor.min_range, sensor.max_range, sensor.vfov_min, sensor.vfov_max,
                int(optimistic), output)
            return output[:count]
        points = self.centers[target_ids]
        delta = points - np.asarray(origin)
        distance = np.linalg.norm(delta, axis=1)
        angle = np.arctan2(delta[:, 2], np.linalg.norm(delta[:, :2], axis=1))
        valid = ((distance >= sensor.min_range) & (distance <= sensor.max_range)
                 & (angle >= sensor.vfov_min) & (angle <= sensor.vfov_max))
        target_ids, points = target_ids[valid], points[valid]
        result = []
        source, _ = self.indices(np.asarray(origin))
        flat = self.state.ravel()
        for offset in range(0, len(points), 4096):
            batch = points[offset:offset + 4096]
            alive = np.ones(len(batch), dtype=bool)
            for rays, ids, inside, finished in self._walk(origin, batch, alive):
                # Ignore the source cell: the sensor is located there.
                interior = ~finished & (ids != source)
                blocked = (~inside) | (flat[ids] == OCCUPIED)
                if not optimistic:
                    blocked |= flat[ids] == UNKNOWN
                alive[rays[interior & blocked]] = False
            result.extend(target_ids[offset:offset + len(batch)][alive])
        return np.asarray(result, dtype=int)

    def save(self, path):
        np.savez_compressed(path, bounds=self.bounds, resolution=self.resolution, state=self.state)

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            result = cls(data["bounds"], float(data["resolution"]))
            state = data["state"]
            if state.shape != result.state.shape or not np.isin(state, [-1, 0, 1]).all():
                raise ValueError("Invalid voxel snapshot")
            result.state[:] = state
        return result


class GroundDistanceCache:
    """Bounded, lossless cache shared by successive plans of the same graph."""

    def __init__(self, max_bytes=64*1024*1024):
        self.max_bytes = max_bytes
        self.entries = OrderedDict()
        self.size = 0
        self.lock = threading.Lock()

    def get(self, key, shape):
        with self.lock:
            data = self.entries.get(key)
            if data is None:
                return None
            self.entries.move_to_end(key)
        return np.frombuffer(zlib.decompress(data), dtype=np.float64).reshape(shape)

    def put(self, key, distance):
        data = zlib.compress(distance.tobytes(), level=1)
        with self.lock:
            previous = self.entries.pop(key, b"")
            self.size += len(data)-len(previous)
            self.entries[key] = data
            while self.size > self.max_bytes:
                _, removed = self.entries.popitem(last=False)
                self.size -= len(removed)


class GroundGrid:
    """Footprint-inflated graph; unknown ground is only traversed optimistically."""

    def __init__(self, data, resolution, origin=(0.0, 0.0), clearance=0.25, distance_cache=None):
        self.data = np.asarray(data, dtype=np.int8)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.clearance = float(clearance)
        self.safe = self._inflate(self.data == FREE)
        self.potential = self._inflate(self.data != OCCUPIED)
        self._cache = OrderedDict()
        self.distance_cache = distance_cache
        self.graph_key = (self.safe.shape, self.resolution,
                          hashlib.sha256(self.safe.tobytes()).digest())

    def _inflate(self, free):
        # Treat the ROI boundary as a physical limit of candidate base positions.
        padded = np.pad(free.astype(np.uint8), 1)
        distance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
        # Bounding each obstacle square by its circumcircle is conservative,
        # including diagonal approaches (subtracting only half a cell is not).
        return free & (distance * self.resolution >= self.clearance + self.resolution / math.sqrt(2) - 1e-9)

    def cell(self, xy):
        col, row = np.floor((np.asarray(xy[:2]) - self.origin) / self.resolution).astype(int)
        return int(row), int(col)

    def xy(self, cell):
        row, col = cell
        return tuple(self.origin + np.array([col + .5, row + .5]) * self.resolution)

    def valid(self, cell, mask):
        r, c = cell
        return 0 <= r < mask.shape[0] and 0 <= c < mask.shape[1] and bool(mask[r, c])

    def neighbors(self, cell, mask):
        r, c = cell
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nxt = r + dr, c + dc
            if not self.valid(nxt, mask):
                continue
            if dr and dc and not (mask[r + dr, c] and mask[r, c + dc]):
                continue
            yield nxt, self.resolution * math.hypot(dr, dc)

    def distances(self, xy, optimistic=False):
        start = self.cell(xy)
        key = (tuple(xy[:2]), optimistic)
        if key in self._cache:
            self._cache.move_to_end(key)
            return self._cache[key]
        mask = self.potential if optimistic else self.safe
        offset = math.dist(tuple(xy[:2]), self.xy(start))
        shared_key = (self.graph_key, start)
        # Only centred safe-graph queries are shared. Optimistic graphs and
        # moving off-centre starts retain their own exact distance calculation.
        shared = self.distance_cache if not optimistic and offset < 1e-12 else None
        if shared is not None:
            cached = shared.get(shared_key, mask.shape)
            if cached is not None:
                return cached
        distance = np.full(mask.shape, np.inf)
        native = library()
        if native is not None:
            native.grid_distances(
                np.ascontiguousarray(mask, dtype=np.uint8), *mask.shape, *start,
                self.resolution, math.dist(tuple(xy[:2]), self.xy(start)), distance)
            self._cache[key] = distance
            if len(self._cache) > 64:
                self._cache.popitem(last=False)
            if shared is not None:
                shared.put(shared_key, distance)
            return distance
        # Never snap a start/goal across a wall.
        if self.valid(start, mask):
            distance[start] = math.dist(tuple(xy[:2]), self.xy(start))
            queue = [(distance[start], start)]
            while queue:
                cost, cell = heapq.heappop(queue)
                if cost > distance[cell] + 1e-10:
                    continue
                for nxt, step in self.neighbors(cell, mask):
                    candidate = cost + step
                    if candidate < distance[nxt] - 1e-10:
                        distance[nxt] = candidate
                        heapq.heappush(queue, (candidate, nxt))
        self._cache[key] = distance
        if len(self._cache) > 64:
            self._cache.popitem(last=False)
        if shared is not None:
            shared.put(shared_key, distance)
        return distance

    def path(self, start, end):
        distance = self.distances(end)
        cell, goal = self.cell(start), self.cell(end)
        if not self.valid(cell, self.safe) or not np.isfinite(distance[cell]):
            return []
        cells = [cell]
        while cell != goal:
            nxt, step = min(self.neighbors(cell, self.safe), key=lambda item: distance[item[0]] + item[1])
            if distance[nxt] >= distance[cell]:
                raise RuntimeError("Invalid path distance gradient")
            cells.append(nxt)
            cell = nxt
        points = [tuple(start[:2])] + [self.xy(cell) for cell in cells]
        if math.dist(points[-1], tuple(end[:2])) > 1e-9:
            points.append(tuple(end[:2]))
        return points

    def candidate_cells(self, starts, stride, optimistic=False):
        mask = self.potential if optimistic else self.safe
        reachable = np.zeros(mask.shape, dtype=bool)
        for start in starts:
            reachable |= np.isfinite(self.distances(start, optimistic))
        spacing = max(1, int(round(stride / self.resolution)))
        lattice = np.zeros(mask.shape, dtype=bool)
        lattice[::spacing, ::spacing] = True
        # Current robot cells belong to the domain in both evaluations.
        for start in starts:
            cell = self.cell(start)
            if self.valid(cell, mask):
                lattice[cell] = True
        return [tuple(cell) for cell in np.argwhere(lattice & reachable & mask)]


@dataclass(frozen=True)
class Candidate:
    cell: tuple
    xy: tuple
    coverage: frozenset


@dataclass
class Coverage:
    candidates: list
    actionable: set
    deferred: set
    excluded: set
    evaluated_positions: int
    optimistic_positions: int
    classification_complete: bool = True
    ownership: str = "overlap"
    ignored: set = field(default_factory=set)
    target_policy: str = "exhaustive"
    selection_phase: str = "all_targets"
    frontier_candidates: int = 0
    available_candidates: int = 0
    planning_stats: dict = field(default_factory=dict)


def coverage_exhausted(voxels, coverage):
    """All retained unknowns need proof; small policy-ignored regions stay unknown."""
    if not coverage.classification_complete or coverage.actionable or coverage.deferred:
        return False
    if coverage.ignored and coverage.target_policy != "frontier":
        return False
    return (not coverage.excluded.intersection(coverage.ignored)
            and set(map(int, voxels.unknown)) == coverage.excluded | coverage.ignored)


@dataclass
class CompletionMonitor:
    """Require a fresh, idle, unchanged completion candidate for a stable window."""
    stable_seconds: float = 8.
    since: Optional[float] = None
    version: object = None

    def reset(self):
        self.since, self.version = None, None

    def update(self, now, *, exhausted, version, fresh, idle, planning=False):
        if not (exhausted and fresh and idle) or planning:
            self.reset()
            return False
        if self.since is None or self.version != version or now < self.since:
            self.since, self.version = now, version
        return now - self.since >= self.stable_seconds - 1e-9


def evaluate_candidates(voxels, ground, starts, sensor, stride=0.5, visibility=None,
                        *, ownership="overlap", audit_when_idle=False, blocked=(), ignored=()):
    """Evaluate every unknown against every in-range candidate, without caps.

    Exhaust the optimistic reachable candidate domain before excluding anything.
    The optimistic graph treats unknown ground as passable, so a closed unknown
    door or unobserved corridor cannot by itself prove permanent infeasibility.
    """
    if len(starts) != 2 or any(not ground.valid(ground.cell(s), ground.safe) for s in starts):
        raise ValueError("Two localized robots on known safe ground are required")
    if stride <= 0:
        raise ValueError("Candidate stride must be positive")
    if ownership not in ("overlap", "nearest"):
        raise ValueError("Unknown candidate ownership policy")
    visible_from = voxels.visible if visibility is None else visibility
    unknown = voxels.unknown
    eligible = voxels.state == UNKNOWN
    if len(ignored):
        eligible.ravel()[np.fromiter(ignored, dtype=np.int64)] = False
        unknown = np.flatnonzero(eligible)
    candidates, actionable = [], set()
    cells = [cell for cell in ground.candidate_cells(starts, stride) if cell not in blocked]
    if ownership == "nearest":
        # Weighted multi-source BFS: diagonal steps cost sqrt(2), and walls/
        # footprint clearance are respected. Ties use a stable row/col order.
        distance = np.minimum.reduce([ground.distances(start) for start in starts])
        cells.sort(key=lambda cell: (float(distance[cell]), cell))
    # A strictly visible unknown must touch measured free space or the source
    # cell. This removes provably occluded targets, not arbitrary task limits.
    adjacent_source = voxels.state == FREE
    for cell in cells:
        source, inside = voxels.indices(sensor.origin(ground.xy(cell)))
        if inside:
            adjacent_source.ravel()[source] = True
    near = np.stack([cv2.dilate(layer.astype(np.uint8), np.ones((3, 3), np.uint8))
                     for layer in adjacent_source])
    neighbors = near.copy()
    neighbors[1:] |= near[:-1]
    neighbors[:-1] |= near[1:]
    boundary = np.flatnonzero(eligible & neighbors.astype(bool))
    actionable_mask = np.zeros(voxels.state.size, dtype=bool)
    origins = [sensor.origin(ground.xy(cell)) for cell in cells]
    batches = (visible_from.claim_views(origins, boundary, sensor)
               if ownership == "nearest" and hasattr(visible_from, "claim_views") else None)
    for index, cell in enumerate(cells):
        xy = ground.xy(cell)
        ids = (batches[index] if batches is not None
               else visible_from(sensor.origin(xy), boundary, sensor, sorted_targets=True))
        if ownership == "nearest":
            ids = ids[~actionable_mask[ids]]
        visible = frozenset(map(int, ids))
        if visible:
            candidates.append(Candidate(cell, xy, visible))
            actionable.update(visible)
            actionable_mask[ids] = True
    remaining = unknown[~actionable_mask[unknown]]
    if audit_when_idle and actionable:
        # Exclusion is a separate exhaustive proof. Keep unclassified unknowns
        # deferred during motion; a no-task result MUST run the full audit.
        return Coverage(candidates, actionable, set(map(int, remaining)), set(),
                        len(cells), 0, False, ownership)
    possible_mask = np.zeros(voxels.state.size, dtype=bool)
    optimistic_cells = ground.candidate_cells(starts, stride, optimistic=True)
    # Spread early proof views across the entire room. A row-major sweep keeps
    # rechecking millions of far-away targets until the last row is reached.
    # Every candidate is still considered when any unresolved targets remain.
    ordered = []
    if optimistic_cells:
        coordinates = np.asarray(optimistic_cells, dtype=float)
        first = int(np.argmin(np.sum((coordinates-coordinates.mean(axis=0))**2, axis=1)))
        nearest = np.full(len(coordinates), np.inf)
        selected = np.zeros(len(coordinates), dtype=bool)
        index = first
        for _ in optimistic_cells:
            ordered.append(optimistic_cells[index])
            selected[index] = True
            nearest = np.minimum(nearest, np.sum((coordinates-coordinates[index])**2, axis=1))
            nearest[selected] = -1
            index = int(np.argmax(nearest))
    for cell in ordered:
        if not len(remaining):
            break
        visible = visible_from(sensor.origin(ground.xy(cell)), remaining, sensor, optimistic=True)
        possible_mask[visible] = True
        remaining = remaining[~possible_mask[remaining]]
    return Coverage(candidates, actionable, set(map(int, np.flatnonzero(possible_mask))),
                    set(map(int, remaining)), len(cells), len(optimistic_cells), True, ownership)


@dataclass
class RoutePlan:
    routes: list
    times: tuple
    makespan: float
    method: str
    paths: list


def _route_cost(route, start, pair):
    if not route:
        return 0.0
    return float(start[route[0]] + sum(pair[a, b] for a, b in zip(route, route[1:])))


def _insertion_deltas(route, start, pair):
    """Rows are insertion slots, columns are candidates; all costs in seconds."""
    if not route:
        return start[None, :]
    first = start + pair[:, route[0]] - start[route[0]]
    middle = (pair[route[:-1], :] + pair[:, route[1:]].T
              - pair[route[:-1], route[1:]][:, None])
    return np.vstack((first, middle, pair[route[-1], :]))


class _CoverageGains:
    """Exact incremental set gains, indexed by the owners of each voxel."""

    def __init__(self, candidates, actionable):
        targets = np.array(sorted(actionable), dtype=np.int64)
        self.remaining = np.ones(len(targets), dtype=bool)
        self.ids = [np.searchsorted(targets, np.fromiter(c.coverage, dtype=np.int64))
                    for c in candidates]
        self.gains = np.array([len(ids) for ids in self.ids])
        flat = np.concatenate(self.ids)
        owners = np.repeat(np.arange(len(candidates)), self.gains)
        self.owners = owners[np.argsort(flat, kind="stable")]
        self.offsets = np.r_[0, np.cumsum(np.bincount(flat, minlength=len(targets)))]

    def cover(self, candidate):
        ids = self.ids[candidate]
        new = ids[self.remaining[ids]]
        self.remaining[new] = False
        if len(new):
            affected = np.concatenate([self.owners[self.offsets[k]:self.offsets[k+1]] for k in new])
            self.gains -= np.bincount(affected, minlength=len(self.gains))


def pending_route(route, candidates, voxels, blocked=()):
    """Drop only measured requirements (or temporarily blocked positions)."""
    state = voxels.state.ravel()
    return [i for i in route if candidates[i].cell not in blocked
            and np.any(state[np.fromiter(candidates[i].coverage, dtype=np.int64)] == UNKNOWN)]


def continue_route(route, candidates, active_cell, ground=None, start=None):
    """Reanchor a delayed tour at the live pose and any still-useful goal.

    Check every opening and both directions of the same cyclic tour. Without a
    committed goal, the old open order is one option, so travel cannot increase.
    Preserve every task and its robot assignment; a useful active goal stays
    first. The next full replan again optimizes joint allocation and selection.
    """
    if not route:
        return route
    active = None
    for offset, index in enumerate(route):
        if candidates[index].cell == active_cell:
            active = offset
            break
    if ground is None or start is None or len(route) < 2:
        return route[active:]+route[:active] if active is not None else route
    cells = np.asarray([candidates[i].cell for i in route])
    initial = ground.distances(start)[cells[:, 0], cells[:, 1]]
    edges = np.array([ground.distances(candidates[i].xy)[tuple(cells[(k+1) % len(route)])]
                      for k, i in enumerate(route)])
    if not np.isfinite(edges).all():
        return route  # Feasibility is checked separately before dispatch.
    forward = initial+edges.sum()-np.roll(edges, 1)
    backward = initial+edges.sum()-edges
    openings = range(len(route)) if active is None else [active]
    _, _, offset, direction = min(
        (float(cost[k]), float(initial[k]), k, direction)
        for k in openings for direction, cost in ((1, forward), (-1, backward)))
    return [route[(offset+direction*k) % len(route)] for k in range(len(route))]


def solve_coverage_routes(coverage, ground, starts, speeds=(0.22, 0.22),
                          observation_time=1.0, exact_limit=10):
    """Joint viewpoint selection, assignment and order minimizing max(T0,T1).

    Exact for <= exact_limit candidates (optional-subset Held-Karp).
    Larger instances use makespan-aware coverage insertion + route relocation;
    every currently actionable voxel must be covered even for large instances.
    """
    if len(starts) != 2 or len(speeds) != 2 or min(speeds) <= 0 or observation_time < 0:
        raise ValueError("Exactly two robots with positive speeds required")
    candidates = coverage.candidates
    n = len(candidates)
    if not n:
        return RoutePlan([[], []], (0.0, 0.0), 0.0, "empty", [[], []])
    rows, cols = np.array([c.cell for c in candidates]).T
    start_dist = np.array([ground.distances(start)[rows, cols] for start in starts])
    distance = np.array([ground.distances(c.xy)[rows, cols] for c in candidates])
    costs = [start_dist[r] / speeds[r] + observation_time for r in (0, 1)]
    pairs = [distance / speeds[r] + observation_time for r in (0, 1)]
    if n <= exact_limit:
        tables = [_held_karp(costs[r], pairs[r]) for r in (0, 1)]
        ids = {key: index for index, key in enumerate(sorted(coverage.actionable))}
        bitsets = [sum(1 << ids[key] for key in c.coverage) for c in candidates]
        unions = [0] * (1 << n)
        for mask in range(1, 1 << n):
            bit = mask & -mask
            unions[mask] = unions[mask ^ bit] | bitsets[bit.bit_length() - 1]
        full = (1 << len(ids)) - 1
        best, masks = (float("inf"), float("inf")), None
        # Each candidate can be skipped, visited by robot 0, or by robot 1.
        for first in range(1 << n):
            t0 = float(tables[0][0][first])
            if not np.isfinite(t0) or t0 > best[0]:
                continue
            available = ((1 << n) - 1) ^ first
            second = available
            while True:
                if unions[first] | unions[second] == full:
                    t1 = float(tables[1][0][second])
                    score = (max(t0, t1), t0 + t1)
                    if np.isfinite(t1) and score < best:
                        best, masks = score, (first, second)
                if second == 0:
                    break
                second = (second - 1) & available
        if masks is None:
            raise RuntimeError("No finite coverage route for reachable candidates")
        routes = [_reconstruct_route(masks[r], tables[r][1], tables[r][2]) for r in (0, 1)]
        method = "exact_joint_coverage"
    else:
        routes, remaining = [[], []], set(coverage.actionable)
        tracker = _CoverageGains(candidates, coverage.actionable)
        while remaining:
            best = None
            current_times = [_route_cost(routes[r], costs[r], pairs[r]) for r in (0, 1)]
            gains = tracker.gains
            for r in (0, 1):
                deltas = _insertion_deltas(routes[r], costs[r], pairs[r])
                positions = np.argmin(deltas, axis=0)
                increase = deltas[positions, np.arange(n)]
                new_times = current_times[r] + increase
                makespans = np.maximum(new_times, current_times[1 - r])
                valid = (gains > 0) & np.isfinite(new_times)
                choices = np.flatnonzero(valid)
                if not len(choices):
                    continue
                # A hidden travel increase still carries a small tie-break cost.
                scores = (makespans - max(current_times) + .05 * increase + 1e-6) / np.maximum(gains, 1)
                order = np.lexsort((choices, makespans[choices], scores[choices]))
                i = int(choices[order[0]])
                score = (float(scores[i]), float(makespans[i]), i, r, int(positions[i]))
                if best is None or score < best:
                    best = score
            if best is None:
                raise RuntimeError("Uncovered actionable voxels have no finite route")
            _, _, i, r, pos = best
            routes[r].insert(pos, i)
            remaining.difference_update(candidates[i].coverage)
            tracker.cover(i)
        # Delete redundant visits, then relocate visits between/within routes.
        counts = Counter(k for route in routes for i in route for k in candidates[i].coverage)
        for r in (0, 1):
            for i in list(routes[r]):
                if all(counts[k] > 1 for k in candidates[i].coverage):
                    routes[r].remove(i)
                    counts.subtract(candidates[i].coverage)
        for _ in range(20):
            times = [_route_cost(routes[r], costs[r], pairs[r]) for r in (0, 1)]
            best_score, best_routes = (max(times), sum(times)), None
            for src in (0, 1):
                for k, item in enumerate(routes[src]):
                    base = [list(route) for route in routes]
                    base[src].pop(k)
                    base_times = [_route_cost(base[r], costs[r], pairs[r]) for r in (0, 1)]
                    for dst in (0, 1):
                        # Only the best insertion of this visit into this route
                        # can improve makespan; avoid rebuilding every trial.
                        route = base[dst]
                        if not route:
                            increments = np.array([costs[dst][item]])
                        else:
                            increments = np.concatenate((
                                [costs[dst][item] + pairs[dst][item, route[0]] - costs[dst][route[0]]],
                                pairs[dst][route[:-1], item] + pairs[dst][item, route[1:]]
                                - pairs[dst][route[:-1], route[1:]],
                                [pairs[dst][route[-1], item]]))
                        pos = int(np.argmin(increments))
                        t = base_times[dst] + increments[pos]
                        score = max(t, base_times[1 - dst]), t + base_times[1 - dst]
                        if (score[0] < best_score[0] - 1e-9
                                or (abs(score[0] - best_score[0]) < 1e-9 and score[1] < best_score[1] - 1e-9)):
                            trial = [list(route) for route in base]
                            trial[dst].insert(pos, item)
                            best_score, best_routes = score, trial
            if best_routes is None:
                break
            routes = best_routes
        method = "heuristic_joint_coverage"
    times = tuple(_route_cost(routes[r], costs[r], pairs[r]) for r in (0, 1))
    covered = set().union(*(candidates[i].coverage for route in routes for i in route))
    if not covered >= coverage.actionable or not np.isfinite(times).all():
        raise RuntimeError("Route coverage/feasibility invariant violated")
    paths = []
    for r in (0, 1):
        points, previous = [], starts[r]
        for i in routes[r]:
            segment = ground.path(previous, candidates[i].xy)
            points.extend(segment if not points else segment[1:])
            previous = candidates[i].xy
        paths.append(points)
    return RoutePlan(routes, times, max(times), method, paths)
