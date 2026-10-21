"""Budgeted frontier views, independent visibility caches and resumable audits.

The measured map and finite LiDAR beams are unchanged. Nearest ownership is
exact within the evaluated candidate pool; candidate selection is approximate.
Only a completed optimistic audit may classify targets as excluded.
"""
from collections import OrderedDict
from collections.abc import Set
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, replace
import hashlib
import math
import time

import cv2
import numpy as np

from lidar360_observation import FullCircleVisibility
from voxel_exploration_core import Candidate, Coverage, FREE, OCCUPIED, UNKNOWN
from voxel_frontier_policy import FrontierPolicy, prepare_targets, select_frontier_batch
from voxel_incremental_frontier import IncrementalFrontier


class VoxelIdSet(Set):
    """Immutable bitmap set for millions of deferred IDs (one byte per cell).

    Small active task sets remain normal sets. Keeping deferred IDs compact
    avoids constructing/copying millions of Python integers in each replan.
    """

    def __init__(self, values=()):
        ids = np.fromiter(values, dtype=np.int64)
        if len(ids) and ids.min() < 0:
            raise ValueError("Voxel IDs must be nonnegative")
        mask = np.zeros(int(ids.max())+1 if len(ids) else 0, dtype=bool)
        mask[ids] = True
        self._finish(mask)

    def _finish(self, mask):
        self.mask = mask
        self.mask.flags.writeable = False
        self.count = int(np.count_nonzero(mask))

    @classmethod
    def from_mask(cls, mask):
        result = object.__new__(cls)
        result._finish(np.asarray(mask, dtype=bool).ravel().copy())
        return result

    def __len__(self):
        return self.count

    def __iter__(self):
        return map(int, np.flatnonzero(self.mask))

    def __contains__(self, value):
        return isinstance(value, (int, np.integer)) and 0 <= value < len(self.mask) and bool(self.mask[value])

    def _operation(self, other, operation):
        if not isinstance(other, Set):
            return NotImplemented
        if not other and operation != 'intersection':
            return self
        ids = np.flatnonzero(other.mask) if isinstance(other, VoxelIdSet) else np.fromiter(other, dtype=np.int64)
        size = max(len(self.mask), int(ids.max())+1 if len(ids) and operation == 'union' else 0)
        mask = np.zeros(size, dtype=bool)
        if operation != 'intersection':
            mask[:len(self.mask)] = self.mask
        inside = ids[(ids >= 0) & (ids < size)]
        if operation == 'union':
            mask[inside] = True
        elif operation == 'difference':
            mask[inside] = False
        else:
            mask[inside] = self.mask[inside]
        result = object.__new__(type(self))
        result._finish(mask)
        return result

    def __or__(self, other):
        return self._operation(other, 'union')

    __ror__ = __or__

    def __sub__(self, other):
        return self._operation(other, 'difference')

    def __and__(self, other):
        return self._operation(other, 'intersection')

    __rand__ = __and__

    def copy(self):
        return self


@dataclass(frozen=True)
class FastPolicy:
    fast_incremental_targets: bool = False
    fast_view_budget: int = 32
    fast_audit_budget: int = 24
    fast_cluster_size: float = 1.5
    fast_cache_mb: float = 64.
    fast_recovery_limit: int = 192
    fast_ray_workers: int = 2

    def __post_init__(self):
        for value in (self.fast_view_budget, self.fast_audit_budget, self.fast_recovery_limit,
                      self.fast_ray_workers):
            if not math.isfinite(value) or value < 1 or int(value) != value:
                raise ValueError("Fast view, audit and recovery budgets must be positive integers")
        if not all(math.isfinite(v) and v > 0 for v in (self.fast_cluster_size, self.fast_cache_mb)):
            raise ValueError("Positive fast cluster size and cache budget required")

    @classmethod
    def from_config(cls, config):
        return cls(**{name: config[name] for name in cls.__dataclass_fields__ if name in config})


class FastFrontierEvaluator:
    """One instance per planner worker. All cached visibility is unowned."""

    def __init__(self):
        self.geometry = None
        self.previous = None
        self.visibility = OrderedDict()
        self.cache_bytes = 0
        self.epoch = 0
        self.audit = None
        self.certificate = None
        self.recovery = set()
        self.recovery_attempted = False
        self.ray_pool = None
        self.ray_workers = 0

    def close(self):
        if self.ray_pool is not None:
            self.ray_pool.shutdown(wait=True)
            self.ray_pool = None
            self.ray_workers = 0

    def _trace(self, visibility, origins, sensor, workers):
        if workers <= 1 or len(origins) <= 1:
            return visibility.visible_views(origins, sensor)
        if self.ray_workers != workers:
            self.close()
            self.ray_pool = ThreadPoolExecutor(max_workers=workers, thread_name_prefix='voxel-rays')
            self.ray_workers = workers
        # Independent unowned sets can run concurrently. The native kernel has
        # per-thread immutable ray geometry and ctypes releases the Python GIL.
        # Preserve input order so ownership and route ties stay deterministic.
        batches = [batch for batch in np.array_split(np.asarray(origins), min(workers, len(origins))) if len(batch)]
        futures = [self.ray_pool.submit(visibility.visible_views, batch, sensor) for batch in batches]
        return [ids for future in futures for ids in future.result()]

    def _sync(self, voxels, sensor, profile, policy, incremental=False):
        key = (tuple(voxels.bounds), voxels.resolution, sensor, profile, policy, incremental)
        invalidated = 0
        if key != self.geometry:
            self.geometry, self.previous = key, None
            self.visibility.clear()
            self.cache_bytes = 0
            self.certificate = None
            self.target_tracker = IncrementalFrontier() if incremental else None
        changed = self.previous is None or not np.array_equal(self.previous, voxels.state)
        if changed:
            if self.previous is not None:
                if np.any(((self.previous == OCCUPIED) & (voxels.state != OCCUPIED))
                          | ((self.previous != UNKNOWN) & (voxels.state == UNKNOWN))):
                    self.certificate = None
                dirty = np.any(self.previous != voxels.state, axis=0).astype(np.float32)
                # A conservative XY square contains every finite ray, including
                # the extent of cells intersected near its range endpoint.
                radius = math.ceil(sensor.max_range/voxels.resolution)+2
                affected = cv2.boxFilter(dirty, -1, (2*radius+1, 2*radius+1),
                                        normalize=False, borderType=cv2.BORDER_CONSTANT) > .5
                # An empty unfiltered beam set cannot gain a distant target
                # when unknowns become known: it contained no measurable first
                # unknown there. Only clearing a blind unknown, removing an
                # occupied occluder, or introducing an unknown can change it.
                opened = np.any(((self.previous == OCCUPIED) & (voxels.state != OCCUPIED))
                                | ((self.previous != UNKNOWN) & (voxels.state == UNKNOWN)), axis=0)
                blind = np.any((self.previous == UNKNOWN) & (voxels.state == FREE), axis=0)
                blind_radius = math.ceil(sensor.min_range/voxels.resolution)+2
                affected_empty = (
                    cv2.boxFilter(opened.astype(np.float32), -1, (2*radius+1, 2*radius+1),
                                  normalize=False, borderType=cv2.BORDER_CONSTANT) > .5)
                affected_empty |= cv2.boxFilter(
                    blind.astype(np.float32), -1, (2*blind_radius+1, 2*blind_radius+1),
                    normalize=False, borderType=cv2.BORDER_CONSTANT) > .5
                for origin in list(self.visibility):
                    x, y = np.floor((np.asarray(origin[:2])-voxels.origin[:2])/voxels.resolution).astype(int)
                    mask = affected if len(self.visibility[origin]) else affected_empty
                    if not (0 <= y < mask.shape[0] and 0 <= x < mask.shape[1]) or mask[y, x]:
                        self.cache_bytes -= self.visibility.pop(origin).nbytes
                        invalidated += 1
            self.previous = voxels.state.copy()
            self.epoch += 1
            self.audit = None
            self.recovery.clear()
            self.recovery_attempted = False
            self.ignored, self.support = (self.target_tracker.update(voxels, policy)
                                         if incremental else prepare_targets(voxels, policy))
            self.eligible = voxels.state.ravel() == UNKNOWN
            if self.ignored:
                self.eligible[np.fromiter(self.ignored, dtype=np.int64)] = False
            self.targets = np.flatnonzero(self.eligible)
            self.target_set = VoxelIdSet.from_mask(self.eligible)
            if self.certificate is not None:
                ids = self.certificate['ids']
                self.certificate['ids'] = ids[self.eligible[ids]]
        return changed, invalidated

    def _store(self, origin, ids, budget):
        old = self.visibility.pop(origin, None)
        if old is not None:
            self.cache_bytes -= old.nbytes
        self.visibility[origin] = ids
        self.cache_bytes += ids.nbytes
        while self.cache_bytes > budget and len(self.visibility) > 1:
            _, removed = self.visibility.popitem(last=False)
            self.cache_bytes -= removed.nbytes

    @staticmethod
    def _source_blocked(voxels, origin, sensor):
        index, inside = voxels.indices(origin)
        if not inside:
            return False
        value = voxels.state.ravel()[index]
        farthest = np.abs(voxels.centers[index]-origin)+voxels.resolution/2
        return value == OCCUPIED or (value == UNKNOWN and np.linalg.norm(farthest) <= sensor.min_range)

    @staticmethod
    def _sources_blocked(voxels, origins, sensor):
        """The same blind-source test, batched without per-position NumPy calls."""
        points = np.asarray(origins, dtype=float).reshape(-1, 3)
        indices, inside = voxels.indices(points)
        values = voxels.state.ravel()[indices]
        farthest = np.abs(voxels.centers[indices]-points)+voxels.resolution/2
        return inside & ((values == OCCUPIED) | ((values == UNKNOWN)
                         & (np.linalg.norm(farthest, axis=1) <= sensor.min_range)))

    def _shortlist(self, voxels, ground, starts, cells, sensor, policy, fast, distances, origins):
        """Cluster/diversify cheap frontier proxies before tracing any 3D rays."""
        if not cells or all(origin in self.visibility for origin in origins):
            return [], 0
        xy = np.asarray([ground.xy(c) for c in cells])
        columns = np.floor((xy-voxels.origin[:2])/voxels.resolution).astype(int)
        valid = ((columns >= 0) & (columns < voxels.shape_xyz[:2])).all(axis=1)
        z = math.floor((sensor.height-voxels.origin[2])/voxels.resolution)
        if self.target_tracker is not None:
            projection = self.target_tracker.projection
            boundary = self.target_tracker.boundary
        else:
            free = voxels.state == FREE
            near = np.stack([cv2.dilate(layer.astype(np.uint8), np.ones((3, 3), np.uint8)) for layer in free])
            adjacent = near.copy()
            adjacent[1:] |= near[:-1]
            adjacent[:-1] |= near[1:]
            boundary = self.eligible.reshape(voxels.state.shape) & adjacent.astype(bool)
            substantial = boundary & (self.support >= policy.frontier_unknown_fraction)
            projection = (substantial if substantial.any() else boundary).sum(axis=0).astype(np.float32)
        radius = math.ceil(sensor.max_range/voxels.resolution)
        gain_image = cv2.boxFilter(projection, -1, (2*radius+1, 2*radius+1),
                                  normalize=False, borderType=cv2.BORDER_CONSTANT)
        frontier = np.zeros(voxels.state.shape[1:], bool)
        if 0 <= z < voxels.state.shape[0]:
            reachable = np.minimum.reduce([ground.distances(s, optimistic=True) for s in starts])
            yy, xx = np.indices(frontier.shape)
            gx = np.floor((voxels.origin[0]+(xx+.5)*voxels.resolution-ground.origin[0])/ground.resolution).astype(int)
            gy = np.floor((voxels.origin[1]+(yy+.5)*voxels.resolution-ground.origin[1])/ground.resolution).astype(int)
            inside = (gx >= 0) & (gy >= 0) & (gx < reachable.shape[1]) & (gy < reachable.shape[0])
            feasible = np.zeros_like(frontier)
            feasible[inside] = np.isfinite(reachable[gy[inside], gx[inside]])
            frontier = (self.eligible.reshape(voxels.state.shape)[z] & feasible
                        & cv2.dilate((voxels.state[z] == FREE).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool))
        if not frontier.any():
            frontier = boundary.any(axis=0)
        if frontier.any():
            front_distance, labels = cv2.distanceTransformWithLabels(
                (~frontier).astype(np.uint8), cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_CCOMP)
            front_distance *= voxels.resolution
        else:
            front_distance, labels = np.full(frontier.shape, np.inf), np.zeros(frontier.shape, np.int32)
        gains, close, clusters = np.zeros(len(cells)), np.zeros(len(cells), bool), []
        gains[valid] = gain_image[columns[valid, 1], columns[valid, 0]]
        close[valid] = front_distance[columns[valid, 1], columns[valid, 0]] <= policy.frontier_view_distance
        for i, (x, y) in enumerate(xy):
            label = int(labels[columns[i, 1], columns[i, 0]]) if valid[i] else 0
            clusters.append((label, math.floor(x/fast.fast_cluster_size), math.floor(y/fast.fast_cluster_size)))
        utility = gains[None, :]/(distances+max(.1, sensor.min_range))
        pending = [i for i, origin in enumerate(origins) if origin not in self.visibility]
        orders = [sorted(pending, key=lambda i: (not close[i], -utility[r, i], distances[r, i], cells[i]))
                  for r in range(2)]
        selected, used, used_clusters = [], set(), set()
        nearest_selected = np.full(len(cells), np.inf)
        reachable_views = np.isfinite(distances)
        for slot in range(min(len(pending), fast.fast_view_budget)):
            robot = slot % 2
            available = [i for i in orders[robot] if i not in used and reachable_views[robot, i]]
            if not available:
                available = [i for i in orders[1-robot] if i not in used]
            if not available:
                break
            front = [i for i in available if close[i]]
            choices = front or available
            diverse = [i for i in choices if clusters[i] not in used_clusters]
            if diverse:
                choices = diverse
            else:
                separated = ([i for i in choices if nearest_selected[i] >= policy.frontier_separation]
                             if fast.fast_incremental_targets else
                             [i for i in choices if all(np.linalg.norm(xy[i]-xy[j]) >= policy.frontier_separation
                                                       for j in selected)])
                choices = separated or choices
            index = choices[0]
            selected.append(index)
            used.add(index)
            used_clusters.add(clusters[index])
            if fast.fast_incremental_targets:
                nearest_selected = np.minimum(nearest_selected, np.linalg.norm(xy-xy[index], axis=1))
        return selected, len(set(clusters[i] for i in pending if close[i]))

    def _audit_step(self, voxels, ground, starts, sensor, stride, budget):
        """Incremental conservative proof over the whole optimistic domain."""
        cells = set(ground.candidate_cells(starts, stride, optimistic=True)) | self.recovery
        graph = hashlib.sha256(ground.potential.tobytes()).digest()
        context = (self.epoch, graph, ground.resolution, tuple(ground.origin), stride, sensor)
        a = self.audit
        calls = 0
        if a is None or a['context'] != context or not a['cells'].issubset(cells):
            # Unknown->known and additional occupied cells cannot make a
            # previously invisible target visible. Reuse that certificate,
            # but validate it against any newly admitted viewpoint first.
            certificate = self.certificate
            preproved = np.empty(0, dtype=np.int64)
            if certificate is not None:
                extra = sorted(cells-certificate['cells'])
                for cell in extra:
                    if not len(certificate['ids']):
                        certificate['cells'].update(extra)
                        break
                    if calls >= budget:
                        return False, calls, len(cells)
                    visible = voxels.visible(sensor.origin(ground.xy(cell)), certificate['ids'], sensor,
                                             optimistic=True)
                    certificate['ids'] = np.setdiff1d(certificate['ids'], visible, assume_unique=True)
                    certificate['cells'].add(cell)
                    calls += 1
                preproved = certificate['ids'].copy()
            a = self.audit = dict(context=context, cells=set(), pending=[], cursor=0,
                                 remaining=np.setdiff1d(self.targets, preproved, assume_unique=True),
                                 preproved=preproved, possible=set())
        extra = cells-a['cells']
        # Spread the proof views spatially; this order affects cost, not scope.
        if extra:
            if a['pending'] and len(a['preproved']):
                # A recovery domain expanded after the preceding proof. Its
                # new origins must also test all previously excluded targets.
                a['remaining'] = np.union1d(a['remaining'], a['preproved'])
                a['preproved'] = np.empty(0, dtype=np.int64)
            ordered = sorted(extra)
            coordinates = np.asarray(ordered, dtype=float)
            index = int(np.argmin(np.sum((coordinates-coordinates.mean(axis=0))**2, axis=1)))
            nearest, used = np.full(len(ordered), np.inf), np.zeros(len(ordered), bool)
            for _ in ordered:
                a['pending'].append(ordered[index])
                used[index] = True
                nearest = np.minimum(nearest, np.sum((coordinates-coordinates[index])**2, axis=1))
                nearest[used] = -1
                index = int(np.argmax(nearest))
            a['cells'] = cells
        while a['cursor'] < len(a['pending']) and len(a['remaining']) and calls < budget:
            cell = a['pending'][a['cursor']]
            ids = voxels.visible(sensor.origin(ground.xy(cell)), a['remaining'], sensor, optimistic=True)
            if len(ids):
                a['possible'].update(map(int, ids))
                a['remaining'] = np.setdiff1d(a['remaining'], ids, assume_unique=True)
            a['cursor'] += 1
            calls += 1
        done = a['cursor'] == len(a['pending']) or not len(a['remaining'])
        if done:
            self.certificate = dict(cells=a['cells'].copy(), ids=np.union1d(a['remaining'], a['preproved']))
        return done, calls, len(a['cells'])

    def _recovery_views(self, voxels, ground, starts, sensor, targets, stride, limit):
        """Additional positions, not headings, to recover finite-beam misses."""
        if not targets:
            return set()
        points = voxels.centers[np.fromiter(targets, dtype=np.int64)]
        bins = np.floor(points/.5).astype(int)
        _, indices = np.unique(bins, axis=0, return_index=True)
        anchors = points[indices]
        reachable = np.minimum.reduce([ground.distances(s) for s in starts])
        original = set(ground.candidate_cells(starts, stride))
        groups = []
        for anchor in anchors:
            group = []
            vertical = abs(anchor[2]-sensor.height)
            for radius in sorted(set((.35, .7, 1.2, 2., 3., min(4., vertical+.25)))):
                for angle in np.arange(12)*2*np.pi/12:
                    cell = ground.cell(anchor[:2]+radius*np.array([np.cos(angle), np.sin(angle)]))
                    if (cell in original or not ground.valid(cell, ground.safe)
                            or not np.isfinite(reachable[cell])):
                        continue
                    if not self._source_blocked(voxels, sensor.origin(ground.xy(cell)), sensor):
                        group.append(cell)
            groups.append(group)
        selected = set()
        for rank in range(max(map(len, groups), default=0)):
            for group in groups:
                if rank < len(group):
                    selected.add(group[rank])
                    if len(selected) >= limit:
                        return selected
        return selected

    def evaluate(self, voxels, ground, starts, sensor, stride, profile, *, blocked=(),
                 policy=None, fast=None, speeds=(.22, .22), observation_time=1.):
        begin = time.perf_counter()
        policy, fast = policy or FrontierPolicy(), fast or FastPolicy()
        if len(starts) != 2 or any(not ground.valid(ground.cell(s), ground.safe) for s in starts):
            raise ValueError("Two localized robots on known safe ground are required")
        if stride <= 0:
            raise ValueError("Positive candidate stride required")
        changed, invalidated = self._sync(voxels, sensor, profile, policy, fast.fast_incremental_targets)
        ground_key = (ground.graph_key, tuple(ground.origin),
                      hashlib.sha256(ground.potential.tobytes()).digest(), stride, fast.fast_recovery_limit)
        if ground_key != getattr(self, 'ground_key', None):
            ground_geometry = (ground.resolution, tuple(ground.origin), ground.data.shape)
            if ground_geometry != getattr(self, 'ground_geometry', None):
                self.certificate = None
            self.ground_geometry = ground_geometry
            self.ground_key = ground_key
            self.audit = None
            self.recovery.clear()
            self.recovery_attempted = False
        targets_done = time.perf_counter()
        raw_cells = set(ground.candidate_cells(starts, stride)) | self.recovery
        cells = sorted(raw_cells-set(blocked))
        origins = [tuple(sensor.origin(ground.xy(cell))) for cell in cells]
        budget = int(fast.fast_cache_mb*1024*1024)
        reused_views = sum(origin in self.visibility for origin in origins)
        source_rejected = 0
        uncached = [origin for origin in origins if origin not in self.visibility]
        blocked_sources = (self._sources_blocked(voxels, uncached, sensor) if fast.fast_incremental_targets else
                           [self._source_blocked(voxels, origin, sensor) for origin in uncached])
        for origin, blocked_source in zip(uncached, blocked_sources):
            if blocked_source:
                self._store(origin, np.empty(0, dtype=np.int64), budget)
                source_rejected += 1
        # Fetch each compressed distance field once, not once per candidate.
        fields = [ground.distances(s) for s in starts]
        distances = np.asarray([[field[cell] for cell in cells] for field in fields])
        if not len(cells):
            distances = np.empty((2, 0))
        selected, clusters = self._shortlist(voxels, ground, starts, cells, sensor, policy, fast, distances, origins)
        shortlist_done = time.perf_counter()
        visibility = FullCircleVisibility(voxels, profile)
        batches = self._trace(visibility, [origins[i] for i in selected], sensor, fast.fast_ray_workers)
        for i, ids in zip(selected, batches):
            self._store(origins[i], ids, budget)
        visibility_done = time.perf_counter()
        # Only these cheap ownership operations depend on current car poses.
        order = sorted(range(len(cells)), key=lambda i: (float(distances[:, i].min()), cells[i]))
        claimed = np.zeros(voxels.state.size, dtype=bool)
        candidates = []
        cached_views = 0
        for i in order:
            ids = self.visibility.get(origins[i])
            if ids is None:
                continue
            cached_views += 1
            ids = ids[self.eligible[ids] & ~claimed[ids]]
            if len(ids):
                candidates.append(Candidate(cells[i], ground.xy(cells[i]), frozenset(map(int, ids))))
                claimed[ids] = True
        actionable = set(map(int, np.flatnonzero(claimed)))
        deferred, excluded = self.target_set-actionable, set()
        untested = len(cells)-cached_views
        audit_done, audit_calls, optimistic_positions = False, 0, 0
        if not candidates and not untested:
            audit_done, audit_calls, optimistic_positions = self._audit_step(
                voxels, ground, starts, sensor, stride, fast.fast_audit_budget)
            if audit_done and self.audit['possible'] and not self.recovery_attempted:
                self.recovery = self._recovery_views(voxels, ground, starts, sensor,
                                                     self.audit['possible'], stride, fast.fast_recovery_limit)
                self.recovery_attempted = True
                if self.recovery:
                    audit_done = False  # Additional domain needs exact scans and an extended proof.
            if audit_done:
                deferred = self.audit['possible'].copy()
                excluded = set(map(int, np.union1d(self.audit['remaining'], self.audit['preproved'])))
        if not len(self.targets):
            audit_done = True
        stats = dict(candidate_evaluation="incremental_frontier", map_changed=changed,
                     invalidated_views=invalidated, cached_views=reused_views,
                     source_rejected_views=source_rejected,
                     traced_views=len(selected), candidate_domain=len(cells), untested_views=untested,
                     ray_workers=fast.fast_ray_workers,
                     proxy_clusters=clusters, cache_bytes=self.cache_bytes, audit_views=audit_calls,
                     audit_complete=audit_done, recovery_views=len(self.recovery),
                     search_pending=not candidates and not audit_done,
                     ownership_scope="evaluated_candidate_pool",
                     target_seconds=targets_done-begin, shortlist_seconds=shortlist_done-targets_done,
                     visibility_seconds=visibility_done-shortlist_done,
                     audit_and_ownership_seconds=time.perf_counter()-visibility_done)
        if audit_done and deferred and not candidates:
            stats['stalled_reason'] = "no_finite_beam_view_for_deferred_targets"
        if self.target_tracker is not None:
            stats.update(self.target_tracker.stats if changed else dict(target_update='unchanged', changed_voxels=0))
        coverage = Coverage(candidates, actionable, deferred, excluded, len(selected), optimistic_positions,
                            audit_done, "nearest", ignored=self.ignored.copy(), target_policy="frontier",
                            planning_stats=stats)
        coverage = select_frontier_batch(coverage, voxels, ground, starts, self.support, policy,
                                          speeds, observation_time, sensor.height)
        coverage.planning_stats['evaluation_seconds'] = time.perf_counter()-begin
        return coverage
