"""Exact incremental target utility; never changes measured occupancy.

Deletion-only updates use threshold-limited component searches. New UNKNOWN
cells can merge islands, so those updates (and missing native code) use the
original full component pass. Local support is recomputed with a complete halo.
"""
import math

import cv2
import numpy as np

from voxel_exploration_core import FREE, UNKNOWN
from voxel_frontier_policy import unknown_components
from voxel_native import library


def box_support(retained, radius):
    width = 2*radius+1
    xy = np.stack([cv2.boxFilter(layer.astype(np.float32), -1, (width, width),
                                normalize=True, borderType=cv2.BORDER_CONSTANT)
                   for layer in retained])
    # Treat flattened XY columns as independent pixels for a vertical box
    # filter. OpenCV's rolling sums avoid a padded 3D cumulative-sum volume
    # and its strided memory traffic, even when both cars dirty most of the ROI.
    return cv2.boxFilter(xy.reshape(len(xy), -1), -1, (1, width), normalize=True,
                         borderType=cv2.BORDER_CONSTANT).reshape(xy.shape)


def region(mask, padding=0):
    bounds = []
    for axis in range(3):
        indices = np.flatnonzero(mask.any(axis=tuple(i for i in range(3) if i != axis)))
        if not len(indices):
            return None
        bounds.append(slice(max(0, int(indices[0])-padding), min(mask.shape[axis], int(indices[-1])+padding+1)))
    return tuple(bounds)


class IncrementalFrontier:
    def __init__(self):
        self.key = self.previous = None
        self.stats = {}

    def update(self, voxels, policy):
        state = voxels.state
        key = (tuple(voxels.bounds), voxels.resolution, policy)
        reset = key != self.key
        changed = np.ones(state.shape, bool) if reset else self.previous != state
        if not changed.any():
            self.stats = dict(target_update='unchanged', changed_voxels=0)
            return self.ignored, self.support
        added = not reset and np.any((self.previous != UNKNOWN) & (state == UNKNOWN))
        native = library()
        if reset or added or native is None:
            labels, sizes = unknown_components(voxels)
            small = sizes*voxels.resolution**3 < policy.min_unknown_component_volume-1e-12
            small[0] = False
            ignored = small[labels].astype(np.uint8)
            mode = 'full_components'
        else:
            ignored = self.ignored_mask.copy()
            removed = np.flatnonzero((self.previous == UNKNOWN) & (state != UNKNOWN))
            threshold = max(1, math.ceil((policy.min_unknown_component_volume-1e-12)/voxels.resolution**3))
            native.voxel_update_islands(state, voxels.shape_xyz, removed, len(removed), threshold, ignored)
            mode = 'incremental_components'
        retained = (state == UNKNOWN) & ~ignored.astype(bool)
        radius = max(1, math.ceil(policy.frontier_support_radius/voxels.resolution))
        utility_changed = np.ones(state.shape, bool) if reset else retained != self.retained
        output = region(utility_changed, radius)
        if reset:
            self.support = box_support(retained, radius)
        elif output is not None:
            source = tuple(slice(max(0, s.start-radius), min(n, s.stop+radius))
                           for s, n in zip(output, state.shape))
            local = box_support(retained[source], radius)
            crop = tuple(slice(o.start-s.start, o.stop-s.start) for o, s in zip(output, source))
            self.support[output] = local[crop]
        # Cache the expensive 3D adjacency operation; recompute only the changed
        # region plus its one-cell halo, including utility-only island changes.
        boundary_region = region(changed | utility_changed, 1)
        if reset:
            self.boundary = np.zeros(state.shape, bool)
        source = tuple(slice(max(0, s.start-1), min(n, s.stop+1))
                       for s, n in zip(boundary_region, state.shape))
        near = np.stack([cv2.dilate(layer.astype(np.uint8), np.ones((3, 3), np.uint8))
                         for layer in (state[source] == FREE)])
        adjacent = near.copy()
        adjacent[1:] |= near[:-1]
        adjacent[:-1] |= near[1:]
        crop = tuple(slice(o.start-s.start, o.stop-s.start) for o, s in zip(boundary_region, source))
        self.boundary[boundary_region] = (retained[source] & adjacent.astype(bool))[crop]
        substantial = self.boundary & (self.support >= policy.frontier_unknown_fraction)
        self.projection = (substantial if substantial.any() else self.boundary).sum(axis=0).astype(np.float32)
        self.ignored_mask, self.retained = ignored, retained
        self.ignored = set(map(int, np.flatnonzero(ignored)))
        self.previous, self.key = state.copy(), key
        self.stats = dict(target_update=mode, changed_voxels=int(changed.sum()),
                          support_updated_voxels=0 if output is None else math.prod(s.stop-s.start for s in output))
        return self.ignored, self.support
