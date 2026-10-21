#!/usr/bin/env python3
"""Unit tests for exploration_3d (no ROS required)."""

import math
import unittest
from types import SimpleNamespace

import numpy as np

from exploration_3d import (
    AgentState,
    ConvergenceParams,
    ConvergenceTracker,
    InfoGainParams,
    OccupancyView,
    ResidualParams,
    RoiParams,
    TourParams,
    VoxelColumnIndex,
    VoxelParams,
    assign_ownership,
    build_local_tour,
    detect_residuals,
    exclude_near_waypoints,
    global_bid,
    nearest_neighbor_order,
    open_path_2opt,
    path_length,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_grid(width_m=20.0, height_m=20.0, res=0.1, fill=0, walls=True):
    """Return (data, view) with optional lethal map border."""
    width = int(round(width_m / res))
    height = int(round(height_m / res))
    data = np.full((height, width), fill, dtype=np.int8)
    if walls:
        data[0:2, :] = 100
        data[-2:, :] = 100
        data[:, 0:2] = 100
        data[:, -2:] = 100
    view = OccupancyView.from_numpy(data, 0.0, 0.0, res)
    return data, view


def rebuild_view(data, res=0.1):
    return OccupancyView.from_numpy(data, 0.0, 0.0, res)


def set_rect(data, res, x0, y0, x1, y1, value):
    c0 = int(round(x0 / res))
    c1 = int(round(x1 / res))
    r0 = int(round(y0 / res))
    r1 = int(round(y1 / res))
    data[r0:r1, c0:c1] = value


def points_on_grid(x0, x1, y0, y1, zs, step=0.2):
    pts = []
    x = x0
    while x <= x1 + 1e-9:
        y = y0
        while y <= y1 + 1e-9:
            for z in zs:
                pts.append([x, y, z])
            y += step
        x += step
    return np.array(pts, dtype=np.float64)


def column_of(index, x, y):
    c = int(math.floor((x - index.origin_x) / index.resolution))
    r = int(math.floor((y - index.origin_y) / index.resolution))
    return r, c


class Cand(SimpleNamespace):
    def __init__(self, x, y, gain=1.0, component=1, **kw):
        super().__init__(x=x, y=y, total_gain=gain, component_label=component, **kw)


# ---------------------------------------------------------------------------
# OccupancyView
# ---------------------------------------------------------------------------


class TestOccupancyView(unittest.TestCase):
    def test_free_components_are_four_connected(self):
        data, _ = make_grid(10, 10, 0.1, walls=False)
        data[:] = 0
        data[50, :] = 100  # horizontal wall splits the map
        view = rebuild_view(data)
        self.assertNotEqual(
            view.component_at_world(2.0, 2.0), view.component_at_world(2.0, 8.0)
        )
        self.assertGreater(view.component_at_world(2.0, 2.0), 0)
        self.assertEqual(view.component_at_world(2.0, 5.0), 0)  # lethal cell

    def test_world_cell_roundtrip(self):
        _, view = make_grid(10, 10, 0.1)
        x, y = view.cell_to_world(20, 30)
        self.assertEqual(view.world_to_cell(x, y), (20, 30))
        self.assertAlmostEqual(x, 3.05, places=6)


# ---------------------------------------------------------------------------
# VoxelColumnIndex
# ---------------------------------------------------------------------------


class TestVoxelColumnIndex(unittest.TestCase):
    def setUp(self):
        data, self.view = make_grid(10, 10, 0.1, fill=0)
        self.params = VoxelParams(resolution=0.1, margin=1.0)

    def test_column_statistics_and_height_bands(self):
        pts = np.array(
            [
                [1.05, 1.05, 0.2],
                [1.05, 1.05, 0.6],
                [1.05, 1.05, 1.1],
                [1.05, 1.05, 1.5],
                [1.05, 1.05, 0.05],  # below min_z
                [1.05, 1.05, 2.50],  # above max_z
            ]
        )
        idx = VoxelColumnIndex.build(pts, self.view, self.params)
        r, c = column_of(idx, 1.05, 1.05)
        self.assertEqual(int(idx.raw_count[r, c]), 4)
        self.assertAlmostEqual(float(idx.zmin[r, c]), 0.2)
        self.assertAlmostEqual(float(idx.zmax[r, c]), 1.5)
        self.assertEqual(int(idx.low_count[r, c]), 1)  # 0.2
        self.assertEqual(int(idx.mid_count[r, c]), 1)  # 0.6
        self.assertEqual(int(idx.high_count[r, c]), 2)  # 1.1, 1.5
        self.assertTrue(idx.observed[r, c])
        self.assertEqual(idx.total_columns, 1)

    def test_min_points_threshold_and_new_column_count(self):
        two = np.array([[2.05, 2.05, 0.5], [2.05, 2.05, 0.6]])
        idx = VoxelColumnIndex.build(two, self.view, self.params)
        self.assertEqual(idx.total_columns, 0)
        self.assertEqual(idx.new_column_count(None), 0)

        pts = np.array(
            [[2.05, 2.05, 0.5], [2.05, 2.05, 0.6], [2.05, 2.05, 0.7]]
        )
        idx2 = VoxelColumnIndex.build(pts, self.view, self.params)
        self.assertEqual(idx2.total_columns, 1)
        self.assertEqual(idx2.new_column_count(set()), 1)
        idx3 = VoxelColumnIndex.build(pts, self.view, self.params)
        self.assertEqual(idx3.new_column_count(idx2.column_keys), 0)

    def test_empty_cloud_is_safe(self):
        idx = VoxelColumnIndex.build(np.empty((0, 3)), self.view, self.params)
        self.assertEqual(idx.total_columns, 0)
        stats = idx.window_stats(5.0, 5.0, 1.5, InfoGainParams())
        self.assertEqual(stats.g3d, 0.0)
        self.assertFalse(idx.observed.any())

    def test_column_keys_stable_across_map_growth(self):
        pts = np.array(
            [
                [2.05, 2.05, 0.5],
                [2.05, 2.05, 0.6],
                [2.05, 2.05, 0.7],
                [7.05, 6.05, 0.5],
                [7.05, 6.05, 0.6],
                [7.05, 6.05, 0.7],
            ]
        )
        small = VoxelColumnIndex.build(pts, self.view, self.params)
        _, big_view = make_grid(14.0, 12.0, 0.1, fill=0)
        grown = VoxelColumnIndex.build(pts, big_view, self.params)
        self.assertEqual(small.total_columns, 2)
        self.assertEqual(grown.total_columns, 2)
        # The grid grew, so grid-linear keys would all change; canonical
        # world-anchored keys must not.
        self.assertEqual(grown.new_column_count(small.column_keys), 0)

    def test_map_state_sampling(self):
        data, view = make_grid(10, 10, 0.1, fill=0)
        set_rect(data, 0.1, 4.0, 4.0, 6.0, 6.0, -1)
        view = rebuild_view(data)
        idx = VoxelColumnIndex.build(
            np.array([[4.55, 4.55, 0.5], [4.55, 4.55, 0.6], [4.55, 4.55, 0.7]]),
            view,
            self.params,
        )
        r, c = column_of(idx, 4.55, 4.55)
        from exploration_3d import UNKNOWN

        self.assertEqual(int(idx.map_state[r, c]), UNKNOWN)


# ---------------------------------------------------------------------------
# 3D information gain
# ---------------------------------------------------------------------------


class TestInfoGain(unittest.TestCase):
    def setUp(self):
        self.params = VoxelParams(resolution=0.1, margin=1.0)
        self.gain_params = InfoGainParams(window_radius=1.0)

    def test_structured_unknown_scores_higher_than_empty(self):
        data, view = make_grid(10, 10, 0.1, fill=0)
        set_rect(data, 0.1, 3.0, 3.0, 5.0, 5.0, -1)
        view = rebuild_view(data)
        pts = points_on_grid(3.1, 4.9, 3.1, 4.9, [0.3, 0.8, 1.3], step=0.2)
        idx = VoxelColumnIndex.build(pts, view, self.params)

        near = idx.window_stats(4.0, 4.0, 1.0, self.gain_params)
        far = idx.window_stats(8.5, 8.5, 1.0, self.gain_params)
        self.assertGreater(near.g3d, far.g3d)
        self.assertGreater(near.unknown, 0.5)
        self.assertEqual(far.g3d, 0.0)
        self.assertGreaterEqual(near.g3d, 0.0)
        self.assertLessEqual(near.g3d, 1.0)

    def test_wall_columns_do_not_count_towards_height_span(self):
        data, view = make_grid(10, 10, 0.1, fill=0)
        pts = points_on_grid(3.1, 3.5, 3.1, 3.5, [0.1, 0.5, 0.9, 1.5], step=0.2)
        idx = VoxelColumnIndex.build(pts, view, self.params)
        stats = idx.window_stats(3.3, 3.3, 0.6, self.gain_params)
        self.assertEqual(stats.height, 0.0)
        self.assertGreater(stats.density, 0.0)

    def test_height_span_uses_non_wall_columns(self):
        data, view = make_grid(10, 10, 0.1, fill=0)
        pts = points_on_grid(3.1, 3.9, 3.1, 3.9, [0.3, 0.7, 1.1], step=0.2)
        idx = VoxelColumnIndex.build(pts, view, self.params)
        stats = idx.window_stats(3.5, 3.5, 0.8, self.gain_params)
        self.assertGreater(stats.height, 0.0)


# ---------------------------------------------------------------------------
# Residual blind-spot detection
# ---------------------------------------------------------------------------


class TestResidualDetector(unittest.TestCase):
    def setUp(self):
        self.voxel_params = VoxelParams(resolution=0.1, margin=1.0)
        self.residual_params = ResidualParams(
            min_cluster_cells=20,
            max_candidates=4,
            viewpoint_search_radius=2.5,
            min_clearance=0.2,
        )

    def test_3d_structure_inside_unknown_region_is_detected(self):
        data, _ = make_grid(20, 20, 0.1, fill=0)
        set_rect(data, 0.1, 8.0, 8.0, 10.0, 10.0, -1)
        view = rebuild_view(data)
        pts = points_on_grid(8.1, 9.9, 8.1, 9.9, [0.3, 0.8, 1.3], step=0.2)
        idx = VoxelColumnIndex.build(pts, view, self.voxel_params)

        candidates, raw_gain = detect_residuals(
            view, idx, self.residual_params
        )
        self.assertTrue(candidates)
        self.assertGreater(raw_gain, 0.0)
        for cand in candidates:
            self.assertGreater(view.clearance_at_world(cand.x, cand.y), 0.0)
            self.assertTrue(view.is_free[view.world_to_cell(cand.x, cand.y)])
            self.assertGreater(cand.component_label, 0)

    def test_no_3d_points_means_no_residual(self):
        data, _ = make_grid(20, 20, 0.1, fill=0)
        set_rect(data, 0.1, 8.0, 8.0, 10.0, 10.0, -1)
        view = rebuild_view(data)
        idx = VoxelColumnIndex.build(
            np.empty((0, 3)), view, self.voxel_params
        )
        candidates, raw_gain = detect_residuals(view, idx, self.residual_params)
        self.assertEqual(candidates, [])
        self.assertEqual(raw_gain, 0.0)

    def test_fully_free_observed_room_has_no_residual(self):
        data, _ = make_grid(20, 20, 0.1, fill=0)
        view = rebuild_view(data)
        pts = points_on_grid(1.5, 18.5, 1.5, 18.5, [0.3, 0.8, 1.3], step=0.4)
        idx = VoxelColumnIndex.build(pts, view, self.voxel_params)
        candidates, _ = detect_residuals(view, idx, self.residual_params)
        self.assertEqual(candidates, [])

    def test_occlusion_pocket_next_to_wall_is_detected(self):
        data, _ = make_grid(20, 20, 0.1, fill=0)
        set_rect(data, 0.1, 8.0, 8.0, 9.0, 9.0, -1)
        view = rebuild_view(data)
        # A wall 1.5 m east of the unknown pocket; the pocket itself has no
        # 3D columns, so this exercises mask A (occlusion pocket).
        wall = points_on_grid(10.5, 10.5, 6.0, 10.0, [0.3, 0.8, 1.3], step=0.1)
        idx = VoxelColumnIndex.build(wall, view, self.voxel_params)
        candidates, raw_gain = detect_residuals(view, idx, self.residual_params)
        self.assertTrue(candidates)
        self.assertGreater(raw_gain, 0.0)

    def test_blacklist_suppresses_candidate(self):
        data, _ = make_grid(20, 20, 0.1, fill=0)
        set_rect(data, 0.1, 8.0, 8.0, 10.0, 10.0, -1)
        view = rebuild_view(data)
        pts = points_on_grid(8.1, 9.9, 8.1, 9.9, [0.3, 0.8, 1.3], step=0.2)
        idx = VoxelColumnIndex.build(pts, view, self.voxel_params)
        candidates, _ = detect_residuals(view, idx, self.residual_params)
        self.assertTrue(candidates)
        c = candidates[0]
        blocked, _ = detect_residuals(
            view, idx, self.residual_params, blacklist=[(c.x, c.y)]
        )
        self.assertEqual(blocked, [])


# ---------------------------------------------------------------------------
# TSP
# ---------------------------------------------------------------------------


class TestTSP(unittest.TestCase):
    def test_empty_and_single(self):
        start = np.array([0.0, 0.0])
        self.assertEqual(nearest_neighbor_order(np.empty((0, 2)), start), [])
        self.assertEqual(nearest_neighbor_order(np.array([[1.0, 0.0]]), start), [0])
        self.assertEqual(open_path_2opt([0], np.array([[1.0, 0.0]]), start), [0])
        self.assertAlmostEqual(path_length([], np.empty((0, 2)), start), 0.0)
        self.assertEqual(build_local_tour([], (0, 0), TourParams()), [])

    def test_nearest_neighbor_order(self):
        pts = np.array([[3.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        self.assertEqual(nearest_neighbor_order(pts, np.array([0.0, 0.0])), [1, 2, 0])

    def test_two_opt_never_increases_path_length(self):
        rng = np.random.default_rng(7)
        for _ in range(100):
            n = int(rng.integers(2, 10))
            pts = rng.uniform(-5.0, 5.0, size=(n, 2))
            start = rng.uniform(-1.0, 1.0, size=2)
            order = nearest_neighbor_order(pts, start)
            before = path_length(order, pts, start)
            improved = open_path_2opt(order, pts, start, max_passes=4)
            after = path_length(improved, pts, start)
            self.assertLessEqual(after, before + 1e-9)

    def test_build_local_tour_respects_spacing(self):
        cands = [Cand(1.0, 0.0), Cand(1.5, 0.0), Cand(5.0, 0.0)]
        tour = build_local_tour(cands, (0.0, 0.0), TourParams(min_spacing=1.2))
        self.assertEqual(len(tour), 2)
        xs = sorted(c.x for c in tour)
        self.assertEqual(xs, [1.0, 5.0])

    def test_build_local_tour_limits_count_and_length(self):
        cands = [Cand(float(x), 0.0) for x in range(2, 21, 2)]
        tour = build_local_tour(
            cands,
            (0.0, 0.0),
            TourParams(max_waypoints=6, min_spacing=1.2, max_tour_length=100.0),
        )
        self.assertEqual(len(tour), 6)

        pts = np.array([[c.x, c.y] for c in tour])
        start = np.array([0.0, 0.0])
        self.assertLessEqual(path_length(list(range(len(tour))), pts, start), 100.0)

        trimmed = build_local_tour(
            cands,
            (0.0, 0.0),
            TourParams(max_waypoints=6, min_spacing=1.2, max_tour_length=5.0),
        )
        pts = np.array([[c.x, c.y] for c in trimmed])
        self.assertLessEqual(
            path_length(list(range(len(trimmed))), pts, start), 5.0 + 1e-9
        )

    def test_exclude_near_waypoints(self):
        cands = [Cand(1.0, 0.0), Cand(3.0, 0.0), Cand(5.0, 0.0)]
        kept = exclude_near_waypoints(cands, [(2.0, 0.0)], 0.9)
        self.assertEqual([c.x for c in kept], [1.0, 3.0, 5.0])
        kept = exclude_near_waypoints(cands, [(2.0, 0.0)], 1.5)
        self.assertEqual([c.x for c in kept], [5.0])


# ---------------------------------------------------------------------------
# Ownership and ROI
# ---------------------------------------------------------------------------


class TestOwnership(unittest.TestCase):
    def test_local_only_for_nearest_idle_in_range_and_component(self):
        agents = [
            AgentState("robot1", 0.0, 0.0, 0.0, idle=True, component_label=1),
            AgentState("robot2", 10.0, 10.0, 0.0, idle=False, component_label=2),
        ]
        cands = [
            Cand(1.0, 0.0, component=1),   # robot1 local
            Cand(9.0, 9.0, component=2),   # robot2 busy -> global
            Cand(7.0, 0.0, component=1),   # robot1 nearest, > 6 m -> global
            Cand(2.0, 0.0, component=2),   # component mismatch -> global
        ]
        own = assign_ownership(agents, cands, local_radius=6.0)
        self.assertEqual((own[0].pool, own[0].owner), ("local", "robot1"))
        self.assertEqual(own[1].pool, "global")
        self.assertEqual(own[2].pool, "global")
        self.assertEqual(own[3].pool, "global")

    def test_tie_breaks_by_namespace(self):
        agents = [
            AgentState("robot2", -1.0, 0.0, 0.0, idle=True, component_label=1),
            AgentState("robot1", 1.0, 0.0, 0.0, idle=True, component_label=1),
        ]
        own = assign_ownership(agents, [Cand(0.0, 0.0, component=1)], 6.0)
        self.assertEqual(own[0].owner, "robot1")

    def test_global_bid_prefers_high_roi(self):
        agent = AgentState("robot1", 0.0, 0.0, 0.0, idle=True)
        params = RoiParams(roi_scale=5.0, orientation_scale=0.5)
        near_low = Cand(3.0, 0.0, gain=3.0)
        far_high = Cand(5.0, 0.0, gain=10.0)
        self.assertGreater(
            global_bid(agent, far_high, params)[0],
            global_bid(agent, near_low, params)[0],
        )

    def test_global_bid_penalizes_targets_behind(self):
        agent = AgentState("robot1", 0.0, 0.0, 0.0, idle=True)
        params = RoiParams(roi_scale=5.0, orientation_scale=0.5)
        front = Cand(5.0, 0.0, gain=10.0)
        behind = Cand(-5.0, 0.0, gain=10.0)
        self.assertGreater(
            global_bid(agent, front, params)[0],
            global_bid(agent, behind, params)[0],
        )


# ---------------------------------------------------------------------------
# Convergence
# ---------------------------------------------------------------------------


class TestConvergence(unittest.TestCase):
    def _kwargs(self, **overrides):
        kwargs = dict(
            map_valid=True,
            frontier_count=0,
            all_idle=True,
            residual_count=0,
            residual_raw_gain=0.0,
            index_available=True,
            new_columns=0,
            total_columns=1000,
        )
        kwargs.update(overrides)
        return kwargs

    def test_requires_six_consecutive_cycles(self):
        tracker = ConvergenceTracker(ConvergenceParams(cycles_required=6))
        kwargs = self._kwargs()
        for i in range(5):
            state = tracker.update(float(i), **kwargs)
            self.assertFalse(state.converged)
            self.assertEqual(state.streak, i + 1)
        state = tracker.update(6.0, **kwargs)
        self.assertTrue(state.converged)

    def test_voxel_growth_resets_streak(self):
        tracker = ConvergenceTracker(ConvergenceParams(cycles_required=3))
        for i in range(2):
            tracker.update(float(i), **self._kwargs())
        state = tracker.update(2.0, **self._kwargs(new_columns=1000))
        self.assertEqual(state.streak, 0)
        self.assertFalse(state.converged)
        for i in range(2):
            state = tracker.update(float(3 + i), **self._kwargs())
            self.assertFalse(state.converged)
        state = tracker.update(6.0, **self._kwargs())
        self.assertTrue(state.converged)

    def test_new_frontier_resets_streak(self):
        tracker = ConvergenceTracker(ConvergenceParams(cycles_required=2))
        tracker.update(0.0, **self._kwargs())
        state = tracker.update(1.0, **self._kwargs(frontier_count=1))
        self.assertEqual(state.streak, 0)
        self.assertFalse(state.two_d_done)

    def test_no_3d_never_converges_when_required(self):
        tracker = ConvergenceTracker(
            ConvergenceParams(cycles_required=2, require_3d=True)
        )
        for i in range(20):
            tracker.update(float(i), **self._kwargs(index_available=False))
        self.assertFalse(tracker.converged)

    def test_no_3d_fallback_when_not_required(self):
        tracker = ConvergenceTracker(
            ConvergenceParams(
                cycles_required=2, require_3d=False, fallback_timeout=10.0
            )
        )
        for i in range(10):
            state = tracker.update(float(i), **self._kwargs(index_available=False))
            self.assertFalse(state.converged)
        tracker.update(10.0, **self._kwargs(index_available=False))
        state = tracker.update(11.0, **self._kwargs(index_available=False))
        self.assertTrue(state.converged)

    def test_residual_gain_blocks_convergence(self):
        tracker = ConvergenceTracker(ConvergenceParams(cycles_required=2))
        for i in range(5):
            state = tracker.update(
                float(i), **self._kwargs(residual_raw_gain=1.0)
            )
            self.assertFalse(state.converged)
            self.assertFalse(state.residual_done)

    def test_reset_clears_latch(self):
        tracker = ConvergenceTracker(ConvergenceParams(cycles_required=1))
        state = tracker.update(0.0, **self._kwargs())
        self.assertTrue(state.converged)
        tracker.reset("test")
        self.assertFalse(tracker.converged)
        state = tracker.update(1.0, **self._kwargs())
        self.assertTrue(state.converged)

    def test_latch_reopens_when_new_residual_appears(self):
        tracker = ConvergenceTracker(ConvergenceParams(cycles_required=1))
        self.assertTrue(tracker.update(0.0, **self._kwargs()).converged)
        state = tracker.update(1.0, **self._kwargs(residual_count=1))
        self.assertFalse(state.converged)
        self.assertFalse(tracker.converged)
        self.assertTrue(tracker.update(2.0, **self._kwargs()).converged)

    def test_latch_reopens_on_voxel_growth(self):
        tracker = ConvergenceTracker(ConvergenceParams(cycles_required=1))
        self.assertTrue(tracker.update(0.0, **self._kwargs()).converged)
        state = tracker.update(1.0, **self._kwargs(new_columns=1000))
        self.assertFalse(state.converged)

    def test_latch_survives_index_loss(self):
        tracker = ConvergenceTracker(
            ConvergenceParams(cycles_required=1, require_3d=True)
        )
        self.assertTrue(tracker.update(0.0, **self._kwargs()).converged)
        state = tracker.update(1.0, **self._kwargs(index_available=False))
        self.assertTrue(state.converged)


if __name__ == "__main__":
    unittest.main()
