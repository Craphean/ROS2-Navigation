"""Frontier utility, small unknown islands, and honest completion semantics."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from lidar360_observation import Lidar360, evaluate_candidates_360
from voxel_exploration_core import (
    FREE, OCCUPIED, UNKNOWN, Candidate, Coverage, GroundGrid, SensorModel, VoxelMap,
    coverage_exhausted, evaluate_candidates, solve_coverage_routes,
)
from voxel_frontier_policy import (
    FrontierPolicy, prepare_targets, select_frontier_batch, unknown_components,
)


class FrontierPolicyTests(unittest.TestCase):
    def test_component_native_and_fallback_agree_on_diagonal_connectivity(self):
        voxels = VoxelMap([0, 1, 0, 1, 0, 1], .1)
        voxels.state[:] = FREE
        voxels.state[1, 1, 1] = UNKNOWN
        voxels.state[2, 2, 2] = UNKNOWN
        voxels.state[3, 3, 3] = UNKNOWN
        voxels.state[8, 8, 8] = UNKNOWN
        before = voxels.state.copy()
        labels, sizes = unknown_components(voxels)
        self.assertEqual(labels[1, 1, 1], labels[3, 3, 3])
        self.assertNotEqual(labels[1, 1, 1], labels[8, 8, 8])
        self.assertEqual(sorted(sizes[1:]), [1, 3])
        with patch("voxel_frontier_policy.library", return_value=None):
            reference, counts = unknown_components(voxels)
        np.testing.assert_array_equal(labels, reference)
        np.testing.assert_array_equal(sizes, counts)
        np.testing.assert_array_equal(voxels.state, before)

    def test_volume_threshold_scales_and_does_not_erase_unknowns(self):
        for resolution in (.05, .1):
            voxels = VoxelMap([0, 1, 0, 1, 0, 1], resolution)
            voxels.state[:] = FREE
            edge = round(.2/resolution)
            voxels.state[1:1+edge, 1:1+edge, 1:1+edge] = UNKNOWN  # .008 m^3.
            voxels.state[-2, -2, -2] = UNKNOWN
            before = voxels.state.copy()
            ignored, _ = prepare_targets(voxels, FrontierPolicy())
            self.assertEqual(len(ignored), 1)
            np.testing.assert_array_equal(voxels.state, before)
            self.assertEqual(voxels.revision, 0)

    def test_new_observations_reclassify_a_fragmented_region(self):
        voxels = VoxelMap([0, 1, 0, 1, 0, 1], .1)
        voxels.state[:] = FREE
        voxels.state[2, 2, 1:8] = UNKNOWN  # .007 m^3 retained.
        ignored, _ = prepare_targets(voxels, FrontierPolicy())
        self.assertFalse(ignored)
        voxels.state[2, 2, 3:6] = FREE
        ignored, _ = prepare_targets(voxels, FrontierPolicy())
        self.assertEqual(len(ignored), 4)  # Two disconnected .002 m^3 islands.

    def test_unknown_support_does_not_wrap_around_roi(self):
        voxels = VoxelMap([0, 1, 0, 1, 0, 1], .1)
        voxels.state[:] = FREE
        voxels.state[:2, :2, :2] = UNKNOWN
        _, support = prepare_targets(voxels, FrontierPolicy(frontier_support_radius=.1))
        self.assertGreater(support[0, 0, 0], 0)
        self.assertEqual(support[-1, -1, -1], 0)
        self.assertLess(support[0, 0, 0], 1.)

    def fixture(self):
        voxels = VoxelMap([0, 16, 0, 3, 0, 1], 1.)
        ground = GroundGrid(np.zeros((3, 16), dtype=np.int8), 1., clearance=0.)
        starts = [(.5, 1.5), (15.5, 1.5)]
        candidates = [Candidate((1, c), (c+.5, 1.5), frozenset(ids))
                      for c, ids in ((0, range(0, 12)), (3, range(12, 15)),
                                     (12, range(15, 18)), (15, range(18, 30)))]
        active = set().union(*(c.coverage for c in candidates))
        coverage = Coverage(candidates, active, set(range(30, 48)), set(), 4, 0,
                            False, "nearest", target_policy="frontier")
        support = np.zeros(voxels.state.shape, dtype=np.float32)
        support.ravel()[12:18] = .8
        policy = FrontierPolicy(min_frontier_area=2., frontier_task_limit=2)
        return voxels, ground, starts, coverage, support, policy

    def test_bulk_frontier_beats_more_nearby_low_density_residuals(self):
        voxels, ground, starts, coverage, support, policy = self.fixture()
        result = select_frontier_batch(coverage, voxels, ground, starts, support, policy)
        self.assertEqual(result.selection_phase, "frontier")
        self.assertEqual([c.cell for c in result.candidates], [(1, 3), (1, 12)])
        self.assertEqual(result.frontier_candidates, 2)
        self.assertEqual(result.available_candidates, 4)
        self.assertEqual(result.actionable, set(range(12, 18)))
        self.assertTrue(set(range(12)) <= result.deferred)
        self.assertEqual(result.actionable | result.deferred, coverage.actionable | coverage.deferred)
        self.assertFalse(result.actionable & result.deferred)
        plan = solve_coverage_routes(result, ground, starts)
        self.assertEqual(plan.routes, [[0], [1]])

    def test_detail_fallback_keeps_retained_targets_after_frontiers_resolve(self):
        voxels, ground, starts, coverage, support, policy = self.fixture()
        support[:] = 0
        result = select_frontier_batch(coverage, voxels, ground, starts, support, policy)
        self.assertEqual(result.selection_phase, "detail")
        self.assertEqual(len(result.candidates), 2)
        self.assertEqual(result.actionable | result.deferred, coverage.actionable | coverage.deferred)
        self.assertFalse(coverage_exhausted(voxels, result))

    def test_one_bulk_front_cannot_starve_other_robot_of_retained_3d_tasks(self):
        voxels, ground, starts, coverage, support, policy = self.fixture()
        support.ravel()[15:18] = 0.
        before = voxels.state.copy()
        legacy = select_frontier_batch(coverage, voxels, ground, starts, support, policy)
        self.assertEqual(len(legacy.candidates), 1)
        self.assertEqual(solve_coverage_routes(legacy, ground, starts).routes[1], [])
        filled = select_frontier_batch(coverage, voxels, ground, starts, support,
                                       replace(policy, frontier_fill_detail_tasks=True))
        self.assertEqual([c.cell for c in filled.candidates], [(1, 3), (1, 15)])
        self.assertEqual(solve_coverage_routes(filled, ground, starts).routes, [[0], [1]])
        self.assertEqual(filled.selection_phase, 'frontier+detail')
        self.assertEqual(filled.planning_stats['selected_detail_tasks'], 1)
        self.assertEqual(filled.actionable | filled.deferred, coverage.actionable | coverage.deferred)
        self.assertFalse(filled.actionable & filled.deferred)
        np.testing.assert_array_equal(before, voxels.state)

    def test_detail_fill_preserves_priority_when_frontier_batch_is_full(self):
        voxels, ground, starts, coverage, support, policy = self.fixture()
        original = select_frontier_batch(coverage, voxels, ground, starts, support, policy)
        filled = select_frontier_batch(coverage, voxels, ground, starts, support,
                                       replace(policy, frontier_fill_detail_tasks=True))
        self.assertEqual(original.candidates, filled.candidates)
        self.assertEqual(filled.planning_stats['selected_detail_tasks'], 0)

    def test_measured_horizontal_front_beats_near_car_sampling_gaps(self):
        voxels, ground, starts, coverage, support, policy = self.fixture()
        voxels.state[:, :, :12] = FREE
        support[:] = .8  # Even high local unknown support must not dominate.
        result = select_frontier_batch(coverage, voxels, ground, starts, support,
                                        replace(policy, frontier_view_distance=1.), sensor_height=.5)
        self.assertEqual({c.cell for c in result.candidates}, {(1, 12), (1, 15)})
        self.assertTrue(set(range(12)) <= result.deferred)

    def test_unseen_wall_volume_cannot_create_a_horizontal_exploration_front(self):
        voxels, ground, starts, coverage, support, policy = self.fixture()
        voxels.state[:, :, :12] = FREE
        voxels.state[:, :, 2] = UNKNOWN
        data = ground.data.copy()
        data[:, 2] = OCCUPIED
        ground = GroundGrid(data, 1., clearance=0.)
        support[:] = .8
        result = select_frontier_batch(coverage, voxels, ground, starts, support,
                                        replace(policy, frontier_view_distance=1.), sensor_height=.5)
        self.assertEqual({c.cell for c in result.candidates}, {(1, 12), (1, 15)})
        self.assertTrue(set(range(12)) <= result.deferred)

    def test_small_ignored_unknown_still_occludes_retained_target(self):
        voxels = VoxelMap([0, 6, 0, 1, 0, 1], 1.)
        voxels.state[:] = FREE
        voxels.state.ravel()[[2, 4]] = UNKNOWN
        ground = GroundGrid(np.zeros((1, 6), dtype=np.int8), 1., clearance=0.)
        with patch.object(ground, "candidate_cells", return_value=[(0, 0)]):
            result = evaluate_candidates(voxels, ground, [(.5, .5)]*2,
                                         SensorModel(height=.5, min_range=.1, max_range=6.),
                                         ignored={2}, ownership="nearest", audit_when_idle=True)
        self.assertFalse(result.actionable)
        self.assertEqual(result.deferred, {4})
        self.assertEqual(voxels.state.ravel()[2], UNKNOWN)

    def test_policy_completion_reports_ignored_without_claiming_observed(self):
        voxels = VoxelMap([0, 2, 0, 1, 0, 1], 1.)
        coverage = Coverage([], set(), set(), {0}, 0, 0, ignored={1}, target_policy="frontier")
        self.assertTrue(coverage_exhausted(voxels, coverage))
        self.assertFalse(coverage_exhausted(voxels, replace(coverage, target_policy="exhaustive")))
        self.assertFalse(coverage_exhausted(voxels, replace(coverage, deferred={1})))
        self.assertFalse(coverage_exhausted(voxels, replace(coverage, excluded={0, 1})))
        np.testing.assert_array_equal(voxels.state, UNKNOWN)

    def test_real_360_pipeline_ignores_small_island_and_keeps_map(self):
        voxels = VoxelMap([0, 1, 0, 1, 0, 1], .1)
        voxels.state[:] = FREE
        voxels.state[5, 5, 5] = UNKNOWN
        ground = GroundGrid(np.zeros((10, 10), dtype=np.int8), .1, clearance=0.)
        profile = replace(Lidar360.load(), horizontal_samples=16, vertical_samples=4, height=.35)
        before = voxels.state.copy()
        result = evaluate_candidates_360(voxels, ground, [(.15, .15), (.85, .85)],
                                         profile.sensor(), .2, profile, target_policy="frontier")
        self.assertEqual(len(result.ignored), 1)
        self.assertFalse(result.excluded)
        self.assertTrue(coverage_exhausted(voxels, result))
        np.testing.assert_array_equal(voxels.state, before)

    def test_offline_and_ros_adapters_preserve_policy_and_ignored_classification(self):
        from continuous_voxel_replay import compute_plan
        from voxel_exploration_planner import VoxelExplorationPlanner
        voxels = VoxelMap([0, 1, 0, 1, 0, 1], .1)
        voxels.state[:] = FREE
        voxels.state[5, 5, 5] = UNKNOWN
        ground = GroundGrid(np.zeros((10, 10), dtype=np.int8), .1, clearance=0.)
        starts = [(.15, .15), (.85, .85)]
        profile = replace(Lidar360.load(), horizontal_samples=16, vertical_samples=4, height=.35)
        config = dict(observation_model="lidar360", lidar_profile="fixture", target_policy="frontier",
                      candidate_ownership="nearest", audit_when_idle=True, candidate_position_stride=.2,
                      robot_speeds=[.22, .22], observation_time=1., exact_candidate_limit=10,
                      **vars(FrontierPolicy()))
        with patch("continuous_voxel_replay.Lidar360.load", return_value=profile):
            offline, _, _ = compute_plan(voxels, ground.data, .1, ground.origin, 0., starts,
                                          profile.sensor(), config, set())
        node = object.__new__(VoxelExplorationPlanner)
        node.lidar360, node.sensor = profile, profile.sensor()
        node.speeds = config["robot_speeds"]
        node.config = SimpleNamespace(**config)
        node.get_parameter = lambda name: SimpleNamespace(value=config[name])
        _, _, _, ros, _, _ = node.compute_plan(voxels, ground, starts, set())
        self.assertEqual(offline.ignored, ros.ignored)
        for result in (offline, ros):
            self.assertEqual(result.target_policy, "frontier")
            self.assertEqual(len(result.ignored), 1)
            self.assertTrue(coverage_exhausted(voxels, result))

    def test_invalid_policy_is_rejected(self):
        for values in (dict(frontier_task_limit=1), dict(frontier_task_limit=2.5),
                       dict(frontier_unknown_fraction=0), dict(frontier_support_radius=0),
                       dict(min_unknown_component_volume=-1)):
            with self.assertRaises(ValueError):
                FrontierPolicy(**values)


if __name__ == "__main__":
    unittest.main()
