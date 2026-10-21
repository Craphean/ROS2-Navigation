"""Select substantial unknown fronts without modifying measured voxel evidence."""
from dataclasses import dataclass, replace
import math

import cv2
import numpy as np

from voxel_exploration_core import UNKNOWN
from voxel_native import library


@dataclass(frozen=True)
class FrontierPolicy:
    # Fill unused batch capacity with retained 3D detail tasks after fronts.
    # Opt-in for V2; the original frontier-only baseline remains reproducible.
    frontier_fill_detail_tasks: bool = False
    # Physical thresholds remain meaningful when voxel resolution changes.
    min_unknown_component_volume: float = .005
    frontier_support_radius: float = .4
    frontier_unknown_fraction: float = .35
    min_frontier_area: float = .10
    frontier_task_limit: int = 8
    frontier_separation: float = .75
    frontier_view_distance: float = 1.

    def __post_init__(self):
        values = (self.min_unknown_component_volume, self.frontier_support_radius,
                  self.min_frontier_area, self.frontier_separation, self.frontier_unknown_fraction,
                  self.frontier_view_distance)
        if (not all(math.isfinite(v) and v >= 0 for v in values)
                or self.frontier_support_radius <= 0
                or not 0 < self.frontier_unknown_fraction <= 1
                or self.frontier_task_limit < 2 or int(self.frontier_task_limit) != self.frontier_task_limit):
            raise ValueError("Invalid frontier thresholds or task limit (at least two)")

    @classmethod
    def from_config(cls, config):
        return cls(**{name: config[name] for name in cls.__dataclass_fields__ if name in config})


def unknown_components(voxels):
    """26 connectivity deliberately preserves diagonal and narrow connections."""
    labels = np.zeros(voxels.state.shape, dtype=np.int32)
    native = library()
    if native is not None:
        native.voxel_unknown_components(voxels.state, voxels.shape_xyz, labels)
    else:
        # Slow but exact fallback, primarily for small maps / no compiler.
        nz, ny, nx = labels.shape
        component = 0
        for z, y, x in np.argwhere(voxels.state == UNKNOWN):
            if labels[z, y, x]:
                continue
            component += 1
            labels[z, y, x] = component
            queue = [(z, y, x)]
            for zz, yy, xx in queue:
                for dz in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            a, b, c = zz+dz, yy+dy, xx+dx
                            if (0 <= a < nz and 0 <= b < ny and 0 <= c < nx
                                    and not labels[a, b, c] and voxels.state[a, b, c] == UNKNOWN):
                                labels[a, b, c] = component
                                queue.append((a, b, c))
    return labels, np.bincount(labels.ravel())


def prepare_targets(voxels, policy):
    """Return ignored islands and local unknown support for frontier scoring."""
    labels, sizes = unknown_components(voxels)
    small = sizes*voxels.resolution**3 < policy.min_unknown_component_volume-1e-12
    small[0] = False
    ignored_mask = small[labels]
    ignored = set(map(int, np.flatnonzero(ignored_mask)))
    retained = ((voxels.state == UNKNOWN) & ~ignored_mask).astype(np.float32)
    radius = max(1, math.ceil(policy.frontier_support_radius/voxels.resolution))
    width = 2*radius+1
    # Separable 3D box mean. Outside the ROI contributes zero unknown support.
    xy = np.stack([cv2.boxFilter(layer, -1, (width, width), normalize=True,
                                borderType=cv2.BORDER_CONSTANT) for layer in retained])
    padded = np.pad(xy, ((radius, radius), (0, 0), (0, 0)))
    sums = np.concatenate((np.zeros_like(xy[:1]), np.cumsum(padded, axis=0)), axis=0)
    support = (sums[width:]-sums[:-width])/width
    return ignored, support


def select_frontier_batch(coverage, voxels, ground, starts, support, policy,
                          speeds=(.22, .22), observation_time=1., sensor_height=.22):
    """Prioritize bulk frontiers, then refine; optimize only this joint batch.

    Visibility and nearest ownership have already been computed. Targets of
    unselected views remain deferred and are re-evaluated after the next arrival.
    """
    candidates = coverage.candidates
    if not candidates:
        return replace(coverage, selection_phase="idle", available_candidates=0)
    cells = np.asarray([c.cell for c in candidates])
    distance = np.asarray([ground.distances(s)[cells[:, 0], cells[:, 1]] for s in starts])
    support = support.ravel()
    areas = np.asarray([
        np.count_nonzero(support[np.fromiter(c.coverage, dtype=np.int64)]
                         >= policy.frontier_unknown_fraction)*voxels.resolution**2
        for c in candidates])
    frontier = areas >= policy.min_frontier_area-1e-12
    # Exploration fronts are measured horizontal boundaries, not sparse ray
    # gaps above/below the car. Use the sensor's actual source-voxel layer:
    # combining distant heights could mark unknown source cells as observed,
    # making every useful frontier unreachable by the strict visibility model.
    # This is utility only, never a navigation/free-space update.
    source_z = math.floor((sensor_height-voxels.origin[2])/voxels.resolution)
    unseen = ((voxels.state[source_z] == UNKNOWN) if 0 <= source_z < voxels.state.shape[0]
              else np.zeros(voxels.state.shape[1:], bool))
    ignored = np.fromiter(coverage.ignored, dtype=np.int64)
    layer_size = unseen.size
    in_layer = (ignored >= source_z*layer_size) & (ignored < (source_z+1)*layer_size)
    unseen.ravel()[ignored[in_layer]-source_z*layer_size] = False
    # Unknown inside walls / outside the footprint-reachable ground is not an
    # exploration front. In particular, do not steer back to an unseen strip
    # behind a measured room wall. This only filters horizontal utility, not
    # the 3D target map or its geometric exclusion proof.
    possible = np.minimum.reduce([ground.distances(s, optimistic=True) for s in starts])
    xs = voxels.origin[0]+(np.arange(voxels.shape_xyz[0])+.5)*voxels.resolution
    ys = voxels.origin[1]+(np.arange(voxels.shape_xyz[1])+.5)*voxels.resolution
    gx = np.floor((xs-ground.origin[0])/ground.resolution).astype(int)
    gy = np.floor((ys-ground.origin[1])/ground.resolution).astype(int)
    valid_x, valid_y = (gx >= 0) & (gx < possible.shape[1]), (gy >= 0) & (gy < possible.shape[0])
    reachable = np.zeros_like(unseen)
    reachable[np.ix_(valid_y, valid_x)] = np.isfinite(possible[np.ix_(gy[valid_y], gx[valid_x])])
    unseen &= reachable
    if unseen.any():
        distance_to_unseen = cv2.distanceTransform((~unseen).astype(np.uint8),
                                                  cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*voxels.resolution
        xy = np.asarray([c.xy for c in candidates])
        columns = np.floor((xy-voxels.origin[:2])/voxels.resolution).astype(int)
        in_roi = ((columns >= 0) & (columns < voxels.shape_xyz[:2])).all(axis=1)
        near_front = np.zeros(len(candidates), bool)
        near_front[in_roi] = distance_to_unseen[columns[in_roi, 1], columns[in_roi, 0]] <= policy.frontier_view_distance
        if (frontier & near_front).any():
            frontier &= near_front
    phase = "frontier" if frontier.any() else "detail"
    eligible = np.flatnonzero(frontier) if frontier.any() else np.arange(len(candidates))
    gain = areas if frontier.any() else np.asarray([len(c.coverage)*voxels.resolution**2 for c in candidates])
    travel = distance/np.asarray(speeds)[:, None]
    utility = gain[None, :]/(travel+max(observation_time, .1))
    # Reserve opportunities for each car, with spatial separation so eight
    # neighbouring parking cells cannot crowd out another exploration front.
    selected = []
    limit = min(int(policy.frontier_task_limit), len(candidates) if policy.frontier_fill_detail_tasks else len(eligible))
    detail_utility = (np.asarray([len(c.coverage)*voxels.resolution**2 for c in candidates])[None, :]
                      /(travel+max(observation_time, .1)))
    detail_count = 0
    for slot in range(limit):
        robot = slot % 2
        pool, scores = eligible, utility
        if policy.frontier_fill_detail_tasks and not any(i not in selected for i in eligible):
            pool, scores = np.arange(len(candidates)), detail_utility
        choices = [int(i) for i in pool if i not in selected and np.isfinite(travel[robot, i])]
        separated = [i for i in choices if all(math.dist(candidates[i].xy, candidates[j].xy)
                                               >= policy.frontier_separation for j in selected)]
        if separated:
            choices = separated
        if not choices:
            robot = 1-robot
            choices = [int(i) for i in pool if i not in selected and np.isfinite(travel[robot, i])]
        if not choices:
            break
        chosen = min(choices, key=lambda i: (-scores[robot, i], travel[robot, i], candidates[i].cell))
        detail_count += int(not frontier[chosen])
        selected.append(chosen)
    available = [candidates[i] for i in selected]
    active = set().union(*(c.coverage for c in available))
    stats = dict(coverage.planning_stats)
    stats.update(selected_frontier_tasks=len(selected)-detail_count, selected_detail_tasks=detail_count)
    return replace(coverage, candidates=available, actionable=active, planning_stats=stats,
                   deferred=coverage.deferred | (coverage.actionable-active),
                   selection_phase='frontier+detail' if phase == 'frontier' and detail_count else phase,
                   frontier_candidates=int(frontier.sum()),
                   available_candidates=len(candidates))
