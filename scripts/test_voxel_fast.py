"""Finite-beam cache correctness, changing ownership and bounded idle audits."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from lidar360_observation import FullCircleVisibility, Lidar360, first_unknowns, evaluate_candidates_360
from voxel_exploration_core import FREE, OCCUPIED, UNKNOWN, GroundGrid, VoxelMap, coverage_exhausted
from voxel_fast_policy import FastFrontierEvaluator, FastPolicy, VoxelIdSet
from voxel_frontier_policy import FrontierPolicy


class FastVisibilityTests(unittest.TestCase):
    def test_compact_deferred_set_preserves_set_operations_and_snapshot_immutability(self):
        values = VoxelIdSet([1, 3, 5])
        self.assertEqual(values, {1, 3, 5})
        self.assertEqual(values | {3, 8}, {1, 3, 5, 8})
        self.assertEqual({3, 8} | values, {1, 3, 5, 8})
        self.assertEqual(values - {3, 8}, {1, 5})
        self.assertEqual(values & {3, 8}, {3})
        self.assertEqual(values | VoxelIdSet([4, 8]), {1, 3, 4, 5, 8})
        self.assertTrue({1, 5} <= values)
        self.assertIs(values | set(), values)
        self.assertFalse(values.mask.flags.writeable)
        self.assertEqual(values, {1, 3, 5})

    def test_independent_native_sets_match_beams_and_preserve_overlaps(self):
        rng = np.random.default_rng(241)
        voxels = VoxelMap([0, 4, 0, 4, 0, 2], .25)
        profile = replace(Lidar360.load(), horizontal_samples=32, vertical_samples=6,
                          height=.375, min_range=.1, max_range=2.)
        origins = np.array([[.625, .625, .375], [.875, .625, .375], [1.125, .625, .375]])
        for _ in range(3):
            voxels.state[:] = rng.choice([FREE, UNKNOWN, OCCUPIED], voxels.state.shape, p=[.9, .08, .02])
            ids, _ = voxels.indices(origins)
            voxels.state.ravel()[ids] = FREE
            expected = [first_unknowns(voxels, p, profile.vectors(), profile.min_range) for p in origins]
            actual = FullCircleVisibility(voxels, profile).visible_views(origins, profile.sensor())
            for a, b in zip(actual, expected):
                np.testing.assert_array_equal(a, b)
        with patch('lidar360_observation.library', return_value=None):
            fallback = FullCircleVisibility(voxels, profile).visible_views(origins, profile.sensor())
        for a, b in zip(fallback, expected):
            np.testing.assert_array_equal(a, b)
        shifted = origins.copy()
        shifted[1, 0] += .04  # No common voxel offset: exact per-ray fallback.
        actual = FullCircleVisibility(voxels, profile).visible_views(shifted, profile.sensor())
        for a, p in zip(actual, shifted):
            np.testing.assert_array_equal(a, first_unknowns(voxels, p, profile.vectors(), profile.min_range))

    def test_overlapping_unknown_is_returned_for_both_views(self):
        v = VoxelMap([0, 5, 0, 1, 0, 1], .25)
        v.state[:] = FREE
        v.state[:, :, 8:] = UNKNOWN
        p = replace(Lidar360.load(), horizontal_samples=4, vertical_samples=2,
                    height=.375, min_range=.1, max_range=3., vertical_min_deg=-1., vertical_max_deg=1.)
        values = FullCircleVisibility(v, p).visible_views([[.625, .375, .375], [.875, .375, .375]], p.sensor())
        self.assertTrue(set(values[0]) & set(values[1]))
        self.assertTrue(all(v.centers[i, 0] < 2.25 for value in values for i in value))

    def test_batched_blind_sources_match_scalar_including_outside_roi(self):
        rng = np.random.default_rng(9)
        voxels = VoxelMap([0, 2, 0, 2, 0, 1], .1)
        voxels.state[:] = rng.choice([FREE, OCCUPIED, UNKNOWN], voxels.state.shape)
        points = rng.uniform([-.2, -.2, -.2], [2.2, 2.2, 1.2], (300, 3))
        sensor = Lidar360.load().sensor()
        expected = [FastFrontierEvaluator._source_blocked(voxels, p, sensor) for p in points]
        np.testing.assert_array_equal(FastFrontierEvaluator._sources_blocked(voxels, points, sensor), expected)
        self.assertEqual(len(FastFrontierEvaluator._sources_blocked(voxels, [], sensor)), 0)


class FastPlannerTests(unittest.TestCase):
    def setUp(self):
        self.voxels = VoxelMap([0, 6, 0, 6, 0, 2], .25)
        self.voxels.state[:] = FREE
        self.voxels.state[:, 10:13, 10:13] = UNKNOWN
        self.ground = GroundGrid(np.zeros((24, 24), np.int8), .25, clearance=0.)
        self.starts = [(.375, .375), (5.625, 5.625)]
        self.profile = replace(Lidar360.load(), horizontal_samples=32, vertical_samples=4,
                               height=.375, min_range=.1, max_range=2.)
        self.policy = FrontierPolicy(min_unknown_component_volume=0.)
        self.fast = FastPolicy(fast_view_budget=12, fast_audit_budget=2, fast_recovery_limit=12)
        self.cache = FastFrontierEvaluator()

    def tearDown(self):
        self.cache.close()

    def evaluate(self, **kwargs):
        return self.cache.evaluate(self.voxels, self.ground, self.starts, self.profile.sensor(), .5,
                                   self.profile, policy=self.policy, fast=kwargs.pop('fast', self.fast), **kwargs)

    def test_v2_incremental_targets_preserve_candidate_selection_and_ownership(self):
        incremental = FastFrontierEvaluator()
        try:
            for index in range(5):
                self.voxels.state[:, 10+index%3, 10] = FREE
                expected = self.evaluate()
                actual = incremental.evaluate(
                    self.voxels, self.ground, self.starts, self.profile.sensor(), .5, self.profile,
                    policy=self.policy, fast=replace(self.fast, fast_incremental_targets=True))
                self.assertEqual([(c.cell, c.coverage) for c in actual.candidates],
                                 [(c.cell, c.coverage) for c in expected.candidates])
                self.assertEqual(actual.ignored, expected.ignored)
                self.assertEqual(actual.deferred, expected.deferred)
        finally:
            incremental.close()

    def test_v2_falls_back_to_3d_boundary_without_sensor_height_front(self):
        self.voxels.state[:] = FREE
        self.voxels.state[-1, 10:13, 10:13] = UNKNOWN
        actual = self.evaluate(fast=replace(self.fast, fast_incremental_targets=True))
        self.assertTrue(actual.candidates or actual.deferred or actual.excluded)

    def test_full_budget_matches_reference_and_reassigns_after_pose_and_cooldown_changes(self):
        fast = replace(self.fast, fast_view_budget=1000)
        for starts, blocked in ((self.starts, set()), ([(4.625, 4.625), (.375, .375)], {(10, 10)})):
            self.starts = starts
            actual = self.evaluate(fast=fast, blocked=blocked)
            reference = evaluate_candidates_360(self.voxels, self.ground, starts, self.profile.sensor(),
                                                 .5, self.profile, target_policy='frontier',
                                                 frontier_policy=self.policy, blocked=blocked)
            self.assertEqual([(c.cell, c.coverage) for c in actual.candidates],
                             [(c.cell, c.coverage) for c in reference.candidates])
            self.assertEqual(actual.actionable, reference.actionable)
            self.assertEqual(sum(len(c.coverage) for c in actual.candidates), len(actual.actionable))
        self.assertFalse(actual.planning_stats['map_changed'])
        self.assertLess(actual.planning_stats['traced_views'], actual.planning_stats['candidate_domain'])

    def test_budget_is_respected_and_map_is_never_changed_by_planning(self):
        before = self.voxels.state.copy()
        result = self.evaluate()
        self.assertLessEqual(result.planning_stats['traced_views'], 12)
        self.assertLessEqual(len(result.candidates), 8)
        self.assertFalse(result.classification_complete)
        self.assertFalse(result.excluded)
        self.assertFalse(coverage_exhausted(self.voxels, result))
        np.testing.assert_array_equal(before, self.voxels.state)

    def test_parallel_visibility_keeps_the_same_tasks_as_serial(self):
        parallel = self.evaluate()
        serial = FastFrontierEvaluator()
        try:
            reference = serial.evaluate(self.voxels, self.ground, self.starts, self.profile.sensor(), .5,
                                        self.profile, policy=self.policy,
                                        fast=replace(self.fast, fast_ray_workers=1))
        finally:
            serial.close()
        self.assertEqual(parallel.candidates, reference.candidates)
        self.assertEqual(parallel.actionable, reference.actionable)

    def test_occupancy_changes_invalidate_empty_and_nonempty_sets_even_without_revision(self):
        self.evaluate(fast=replace(self.fast, fast_view_budget=1000))
        old_epoch = self.cache.epoch
        self.voxels.state[:, 10:13, 10:13] = FREE
        self.voxels.state[:, 8:11, 8:11] = UNKNOWN
        result = self.evaluate(fast=replace(self.fast, fast_view_budget=1000))
        self.assertGreater(result.planning_stats['invalidated_views'], 0)
        self.assertEqual(self.cache.epoch, old_epoch+1)
        reference = evaluate_candidates_360(self.voxels, self.ground, self.starts, self.profile.sensor(),
                                             .5, self.profile, target_policy='frontier', frontier_policy=self.policy)
        self.assertEqual(result.actionable, reference.actionable)

    def test_distant_map_change_keeps_visibility_and_sensor_change_invalidates_it(self):
        sensor = self.profile.sensor()
        self.cache._sync(self.voxels, sensor, self.profile, self.policy)
        origin = (.375, .375, .375)
        saved = np.array([], dtype=np.int64)
        self.cache._store(origin, saved, 1024)
        self.voxels.state[-1, -1, -1] = UNKNOWN
        _, invalidated = self.cache._sync(self.voxels, sensor, self.profile, self.policy)
        self.assertEqual(invalidated, 0)
        self.assertIs(self.cache.visibility[origin], saved)
        different = replace(self.profile, horizontal_samples=64)
        self.cache._sync(self.voxels, different.sensor(), different, self.policy)
        self.assertFalse(self.cache.visibility)

    def test_empty_visibility_survives_distant_unknown_resolution_but_not_occluder_removal(self):
        self.voxels.state[:] = FREE
        self.voxels.state[:, :, 4] = OCCUPIED
        self.voxels.state[1, 1, 8:10] = UNKNOWN
        self.profile = replace(self.profile, horizontal_samples=4, vertical_samples=2,
                               vertical_min_deg=-1., vertical_max_deg=1., max_range=3.)
        sensor, origin = self.profile.sensor(), (.375, .375, .375)
        self.cache._sync(self.voxels, sensor, self.profile, self.policy)
        empty = first_unknowns(self.voxels, origin, self.profile.vectors(), sensor.min_range)
        self.assertEqual(len(empty), 0)
        self.cache._store(origin, empty, 1024)
        self.voxels.state[1, 1, 8] = FREE
        self.cache._sync(self.voxels, sensor, self.profile, self.policy)
        self.assertIs(self.cache.visibility[origin], empty)
        self.voxels.state[:, :, 4] = FREE
        self.cache._sync(self.voxels, sensor, self.profile, self.policy)
        self.assertNotIn(origin, self.cache.visibility)
        self.assertGreater(len(first_unknowns(self.voxels, origin, self.profile.vectors(), sensor.min_range)), 0)

    def test_clearing_blind_unknown_invalidates_empty_visibility(self):
        self.voxels.state[:] = UNKNOWN
        self.profile = replace(self.profile, min_range=.3)
        sensor, origin = self.profile.sensor(), (.375, .375, .375)
        self.cache._sync(self.voxels, sensor, self.profile, self.policy)
        self.assertEqual(len(first_unknowns(self.voxels, origin, self.profile.vectors(), sensor.min_range)), 0)
        self.cache._store(origin, np.empty(0, dtype=np.int64), 1024)
        self.voxels.state[1, 1, 1] = FREE
        self.cache._sync(self.voxels, sensor, self.profile, self.policy)
        self.assertNotIn(origin, self.cache.visibility)
        self.assertGreater(len(first_unknowns(self.voxels, origin, self.profile.vectors(), sensor.min_range)), 0)

    def test_partial_audit_never_excludes_and_completed_proof_is_reused(self):
        self.voxels.state[:] = FREE
        self.voxels.state[-1, -1, -1] = UNKNOWN
        cells = [(1, 1), (1, 3)]
        self.starts = [self.ground.xy(c) for c in cells]
        fast = replace(self.fast, fast_audit_budget=1)
        with patch.object(self.ground, 'candidate_cells', return_value=cells):
            first = self.evaluate(fast=fast)
            self.assertFalse(first.classification_complete)
            self.assertFalse(first.excluded)
            second = self.evaluate(fast=fast)
            self.assertTrue(second.classification_complete)
            self.assertTrue(coverage_exhausted(self.voxels, second))
            with patch.object(self.voxels, 'visible', side_effect=AssertionError('Unchanged proof recomputed')):
                third = self.evaluate(fast=fast)
            self.assertEqual(second.excluded, third.excluded)
            self.assertEqual(third.planning_stats['audit_views'], 0)
            self.assertEqual(third.planning_stats['traced_views'], 0)

    def test_ground_change_resets_proof_but_keeps_geometry_cache(self):
        self.evaluate()
        self.cache.audit = {'sentinel': True}
        data = self.ground.data.copy()
        data[3, 3] = OCCUPIED
        self.ground = GroundGrid(data, .25, clearance=0.)
        self.evaluate()
        self.assertNotEqual(self.cache.audit, {'sentinel': True})

    def test_exclusion_certificate_survives_added_occupancy_and_checks_new_origins(self):
        self.voxels.state[:] = FREE
        self.voxels.state[5, 23, 23] = UNKNOWN
        cells = [(1, 1), (1, 3)]
        fast = replace(self.fast, fast_audit_budget=1)
        with patch.object(self.ground, 'candidate_cells', side_effect=lambda *a, **kw: cells.copy()):
            self.evaluate(fast=fast)
            proved = self.evaluate(fast=fast)
            self.assertTrue(coverage_exhausted(self.voxels, proved))
            self.voxels.state[2, 8, 8] = OCCUPIED
            with patch.object(self.voxels, 'visible', side_effect=AssertionError('Safe proof lost')):
                reused = self.evaluate(fast=fast)
            self.assertEqual(reused.excluded, proved.excluded)
            self.assertEqual(reused.planning_stats['audit_views'], 0)
            cells.append((23, 19))  # A new origin can see the previously excluded target.
            with patch.object(FullCircleVisibility, 'visible_views',
                              side_effect=lambda origins, sensor: [np.empty(0, np.int64) for _ in origins]), \
                 patch.object(self.cache, '_recovery_views', return_value=set()):
                for _ in range(8):
                    updated = self.evaluate(fast=fast)
                    if updated.classification_complete:
                        break
            self.assertTrue(updated.classification_complete)
            self.assertFalse(updated.excluded)
            self.assertEqual(len(updated.deferred), 1)
            self.assertFalse(coverage_exhausted(self.voxels, updated))
            self.voxels.state[2, 8, 8] = FREE
            self.cache._sync(self.voxels, self.profile.sensor(), self.profile, self.policy)
            self.assertIsNone(self.cache.certificate)

    def test_recovery_finds_real_gain_when_regular_lattice_misses_finite_beams(self):
        from voxel_replay_io import observe
        self.voxels.state[:] = FREE
        self.voxels.state[1, 9, 9] = UNKNOWN
        self.profile = replace(self.profile, horizontal_samples=4, vertical_samples=2,
                               vertical_min_deg=-1., vertical_max_deg=1.)
        fast = replace(self.fast, fast_view_budget=1000, fast_audit_budget=1000, fast_recovery_limit=24)
        def search():
            return self.cache.evaluate(self.voxels, self.ground, self.starts, self.profile.sensor(), 1.,
                                       self.profile, policy=self.policy, fast=fast)
        first = search()
        self.assertFalse(first.actionable)
        self.assertFalse(first.classification_complete)
        self.assertFalse(first.excluded)
        self.assertGreater(first.planning_stats['recovery_views'], 0)
        recovered = search()
        self.assertTrue(recovered.actionable)
        truth, measured = self.voxels.snapshot(), self.voxels.snapshot()
        truth.state[1, 9, 9] = OCCUPIED
        gained = observe(truth, measured, recovered.candidates[0].xy, self.profile.sensor(), self.profile.vectors())
        self.assertEqual(gained, 1)
        self.assertEqual(measured.state[1, 9, 9], OCCUPIED)
        self.assertEqual(self.voxels.state[1, 9, 9], UNKNOWN)

    def test_ignored_voxels_remain_unknown_and_can_finish_policy(self):
        self.voxels.state[:] = FREE
        self.voxels.state[3, 3, 3] = UNKNOWN
        self.policy = replace(self.policy, min_unknown_component_volume=.1)
        result = self.evaluate()
        self.assertEqual(len(result.ignored), 1)
        self.assertTrue(coverage_exhausted(self.voxels, result))
        self.assertEqual(self.voxels.state[3, 3, 3], UNKNOWN)

    def test_ros_and_offline_fast_adapters_use_same_policy(self):
        from continuous_voxel_replay import compute_plan
        from voxel_exploration_planner import VoxelExplorationPlanner
        config = dict(observation_model='lidar360', target_policy='frontier',
                      candidate_evaluation='incremental_frontier', lidar_profile='fixture',
                      candidate_position_stride=.5, robot_speeds=[.22, .22], observation_time=1.,
                      exact_candidate_limit=10, **vars(self.policy), **vars(self.fast))
        with patch('continuous_voxel_replay.Lidar360.load', return_value=self.profile):
            offline, plan, _ = compute_plan(self.voxels, self.ground.data, .25, self.ground.origin,
                                            0., self.starts, self.profile.sensor(), config, set())
        node = object.__new__(VoxelExplorationPlanner)
        node.fast_evaluator, node.fast_policy = FastFrontierEvaluator(), self.fast
        node.lidar360, node.sensor = self.profile, self.profile.sensor()
        node.config, node.speeds = SimpleNamespace(**config), config['robot_speeds']
        node.get_parameter = lambda name: SimpleNamespace(value=config[name])
        _, _, _, ros, ros_plan, _ = node.compute_plan(self.voxels, self.ground, self.starts, set())
        self.assertEqual(offline.candidates, ros.candidates)
        self.assertEqual(plan.routes, ros_plan.routes)

    def test_invalid_budgets_fail(self):
        for args in ({'fast_view_budget': 0}, {'fast_audit_budget': 1.5}, {'fast_cache_mb': float('nan')},
                     {'fast_recovery_limit': -1}, {'fast_cluster_size': 0}):
            with self.assertRaises(ValueError):
                FastPolicy(**args)


if __name__ == '__main__':
    unittest.main()
