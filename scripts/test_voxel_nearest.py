"""Nearest reachable ownership, finite-beam equivalence, and cache invalidation."""
from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np

from lidar360_observation import FullCircleVisibility, Lidar360, first_unknowns
from voxel_exploration_core import (
    FREE, OCCUPIED, UNKNOWN, GroundDistanceCache, GroundGrid, SensorModel,
    VoxelMap, coverage_exhausted, evaluate_candidates,
)


class NearestOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.voxels = VoxelMap([0, 7, 0, 7, 0, 1], 1.)
        self.voxels.state[:] = FREE
        self.voxels.state.ravel()[[40, 41]] = UNKNOWN
        data = np.zeros((7, 7), dtype=np.int8)
        data[:6, 2] = OCCUPIED  # Only the far end of this wall has an opening.
        self.ground = GroundGrid(data, 1., clearance=0.)
        self.starts = [(1.5, 1.5), (1.5, 2.5)]
        self.views = {(1, 0): {40}, (1, 3): {40}, (6, 1): {41}, (6, 6): {41}}

    def visible(self, origin, targets, sensor, **kwargs):
        return np.array(sorted(self.views.get(self.ground.cell(origin), set()) & set(targets)),
                        dtype=np.int64)

    def evaluate(self, **kwargs):
        return evaluate_candidates(self.voxels, self.ground, self.starts,
                                   SensorModel(height=.5, min_range=.1, max_range=10.),
                                   1., self.visible, **kwargs)

    def test_claims_are_disjoint_and_use_path_distance_around_walls(self):
        before = self.voxels.state.copy()
        original = self.evaluate()
        result = self.evaluate(ownership="nearest")
        owners = {i: c.cell for c in result.candidates for i in c.coverage}
        self.assertEqual(owners, {40: (1, 0), 41: (6, 1)})
        self.assertEqual(result.actionable, original.actionable)
        self.assertEqual(sum(len(c.coverage) for c in result.candidates), len(result.actionable))
        np.testing.assert_array_equal(self.voxels.state, before)
        self.assertEqual(result.actionable, set(self.voxels.unknown))

    def test_both_live_robot_positions_participate_in_nearest_search(self):
        self.starts[1] = (3.5, 1.5)
        result = self.evaluate(ownership="nearest")
        owners = {i: c.cell for c in result.candidates for i in c.coverage}
        self.assertEqual(owners[40], (1, 3))
        # A new invocation recomputes ownership; no global "claimed" state leaks.
        self.starts[1] = (1.5, 2.5)
        self.assertIn(40, next(c.coverage for c in self.evaluate(ownership="nearest").candidates
                              if c.cell == (1, 0)))

    def test_blocked_near_view_does_not_steal_far_views_gain(self):
        result = self.evaluate(ownership="nearest", blocked={(1, 0)})
        self.assertEqual(next(c.cell for c in result.candidates if 40 in c.coverage), (1, 3))
        self.assertEqual(result.actionable, {40, 41})

    def test_equal_distance_tie_is_stable(self):
        self.views = {(1, 0): {40, 41}, (2, 0): {40, 41}}
        result = self.evaluate(ownership="nearest")
        self.assertEqual([c.cell for c in result.candidates], [(1, 0)])

    def test_quick_classification_cannot_report_completion_or_exclusions(self):
        self.voxels.state.ravel()[42] = UNKNOWN
        result = self.evaluate(ownership="nearest", audit_when_idle=True)
        self.assertFalse(result.classification_complete)
        self.assertEqual(result.deferred, {42})
        self.assertEqual(result.excluded, set())
        self.assertFalse(coverage_exhausted(self.voxels, result))
        # Empty strict visibility must run the optimistic audit, even in quick mode.
        self.views = {}
        result = self.evaluate(ownership="nearest", audit_when_idle=True)
        self.assertTrue(result.classification_complete)
        self.assertGreater(result.optimistic_positions, 0)
        self.assertEqual(result.excluded, {40, 41, 42})
        self.assertTrue(coverage_exhausted(self.voxels, result))

    def test_batch_ray_claims_match_sequential_reference_after_map_changes(self):
        rng = np.random.default_rng(63)
        for resolution in (.25, .5):
            voxels = VoxelMap([-3, 3, -3, 3, 0, 3], resolution)
            profile = replace(Lidar360.load(), horizontal_samples=64, vertical_samples=8,
                              height=1.125, max_range=3.)
            visibility = FullCircleVisibility(voxels, profile)
            origins = np.array([[-.375, -.375, 1.125], [.625, -.375, 1.125],
                                [.625, .625, 1.125]])
            for phase in range(3):
                voxels.state[:] = rng.choice([FREE, UNKNOWN, OCCUPIED],
                                              size=voxels.state.shape, p=[.94, .04, .02])
                ids, _ = voxels.indices(origins)
                voxels.state.ravel()[ids] = FREE
                targets = voxels.unknown
                actual = visibility.claim_views(origins, targets, profile.sensor())
                remaining, expected = set(targets), []
                for origin in origins:
                    visible = set(first_unknowns(voxels, origin, profile.vectors(), profile.min_range))
                    claim = visible & remaining
                    expected.append(claim)
                    remaining -= claim
                self.assertEqual([set(a) for a in actual], expected)
                if phase == 0:
                    with patch("lidar360_observation.library", return_value=None):
                        fallback = visibility.claim_views(origins, targets, profile.sensor())
                    self.assertEqual([set(a) for a in fallback], expected)

    def test_claimed_unknown_still_occludes_and_does_not_become_free(self):
        voxels = VoxelMap([0, 5, 0, 1, 0, 1], .25)
        voxels.state[:] = FREE
        voxels.state[:, :, 8:] = UNKNOWN
        profile = replace(Lidar360.load(), horizontal_samples=4, vertical_samples=2,
                          height=.375, max_range=3., vertical_min_deg=-1., vertical_max_deg=1.)
        origins = np.array([[.625, .375, .375], [.875, .375, .375]])
        before = voxels.state.copy()
        claims = FullCircleVisibility(voxels, profile).claim_views(origins, voxels.unknown, profile.sensor())
        self.assertGreater(len(claims[0]), 0)
        self.assertEqual(len(claims[1]), 0)
        self.assertTrue(all(voxels.centers[i, 0] < 2.25 for i in claims[0]))
        np.testing.assert_array_equal(voxels.state, before)

    def test_nonmatching_origins_fall_back_without_losing_any_claims(self):
        voxels = VoxelMap([0, 4, 0, 4, 0, 2], .25)
        voxels.state[:] = FREE
        voxels.state[:, :, 10:] = UNKNOWN
        profile = replace(Lidar360.load(), horizontal_samples=32, vertical_samples=4, height=.6)
        origins = [[.3, .6, .6], [1.125, .625, .6]]
        visibility = FullCircleVisibility(voxels, profile)
        actual = visibility.claim_views(origins, voxels.unknown, profile.sensor())
        with patch("lidar360_observation.library", return_value=None):
            expected = visibility.claim_views(origins, voxels.unknown, profile.sensor())
        self.assertEqual([set(a) for a in actual], [set(a) for a in expected])


class GroundDistanceCacheTests(unittest.TestCase):
    def test_lossless_reuse_and_changed_obstacle_invalidates_cache(self):
        data = np.zeros((10, 12), dtype=np.int8)
        cache = GroundDistanceCache()
        first = GroundGrid(data, .5, clearance=0., distance_cache=cache)
        expected = first.distances((.75, .75))
        second = GroundGrid(data, .5, clearance=0., distance_cache=cache)
        with patch("voxel_exploration_core.library", side_effect=AssertionError("cache miss")):
            np.testing.assert_array_equal(second.distances((.75, .75)), expected)
        data[1:9, 4] = OCCUPIED
        changed = GroundGrid(data, .5, clearance=0., distance_cache=cache)
        actual = changed.distances((.75, .75))
        np.testing.assert_array_equal(actual, GroundGrid(data, .5, clearance=0.).distances((.75, .75)))
        self.assertGreater(actual[1, 6], expected[1, 6])

    def test_compressed_cache_is_bounded_even_for_single_large_entry(self):
        cache = GroundDistanceCache(max_bytes=128)
        rng = np.random.default_rng(2)
        for index in range(8):
            cache.put(index, rng.random((20, 20)))
            self.assertLessEqual(cache.size, 128)
        self.assertIsNone(cache.get(7, (20, 20)))


if __name__ == "__main__":
    unittest.main()
