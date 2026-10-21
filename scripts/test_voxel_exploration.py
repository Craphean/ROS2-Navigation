#!/usr/bin/env python3
"""Regression tests for occupancy evidence, reachability, coverage and makespan."""
import itertools
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from voxel_exploration_core import (
    Candidate, Coverage, FREE, GroundGrid, OCCUPIED, SensorModel, UNKNOWN,
    VoxelMap, _CoverageGains, continue_route, evaluate_candidates, solve_coverage_routes,
)


class VoxelEvidenceTests(unittest.TestCase):
    def test_incremental_gains_equal_set_intersections_after_overlapping_visits(self):
        rng = np.random.default_rng(42)
        candidates = [Candidate((0, i), (i, 0), frozenset(map(int, rng.choice(500, 150, replace=False))))
                      for i in range(25)]
        remaining = set().union(*(c.coverage for c in candidates))
        tracker = _CoverageGains(candidates, remaining)
        for i in rng.permutation(len(candidates)):
            np.testing.assert_equal(tracker.gains, [len(c.coverage & remaining) for c in candidates])
            tracker.cover(i)
            remaining.difference_update(candidates[i].coverage)
        self.assertFalse(tracker.remaining.any())
        self.assertTrue((tracker.gains == 0).all())

    def test_reanchoring_retains_all_tasks_and_does_not_increase_travel(self):
        ground = GroundGrid(np.zeros((8, 9), dtype=np.int8), 1., clearance=0)
        candidates = [Candidate(cell, ground.xy(cell), frozenset([i]))
                      for i, cell in enumerate([(1, 1), (1, 6), (5, 6), (5, 3), (3, 2)])]
        original = list(range(len(candidates)))
        for cell in ((1, 2), (4, 5), (6, 1)):
            start = ground.xy(cell)
            order = continue_route(original, candidates, None, ground, start)

            def cost(route):
                locations = [start]+[candidates[i].xy for i in route]
                return sum(ground.distances(a)[ground.cell(b)] for a, b in zip(locations, locations[1:]))

            self.assertEqual(set(order), set(original))
            self.assertLessEqual(cost(order), cost(original)+1e-9)
            active = continue_route(original, candidates, candidates[2].cell, ground, start)
            self.assertEqual(active[0], 2)
            self.assertEqual(set(active), set(original))

    def test_ray_updates_only_traversed_voxels(self):
        m = VoxelMap([0, 5, 0, 3, 0, 2], 1)
        m.integrate([.5, .5, .5], [[3.5, .5, .5]])
        np.testing.assert_array_equal(m.state[0, 0], [FREE, FREE, FREE, OCCUPIED, UNKNOWN])
        self.assertTrue((m.state[1] == UNKNOWN).all())
        self.assertTrue((m.state[:, 1:] == UNKNOWN).all())
        self.assertEqual(m.revision, 1)
        m.integrate([.5, .5, .5], [[3.5, .5, .5]])
        self.assertEqual(m.revision, 1)

    def test_hit_priority_and_clearing_new_evidence(self):
        m = VoxelMap([0, 5, 0, 1, 0, 1], 1)
        m.integrate([.5, .5, .5], [[2.5, .5, .5], [4.5, .5, .5]], [True, False])
        self.assertEqual(m.state[0, 0, 2], OCCUPIED)
        self.assertEqual(m.state[0, 0, 4], FREE)
        m.integrate([.5, .5, .5], [[4.5, .5, .5]], [False])
        self.assertTrue((m.state == FREE).all())

    def test_external_endpoints_still_clear_roi(self):
        m = VoxelMap([0, 2, 0, 2, 0, 2], 1)
        m.integrate([.5, .5, -.5], [[.5, .5, 3.5]])
        self.assertEqual(list(m.state[:, 0, 0]), [FREE, FREE])
        self.assertEqual(np.sum(m.state == OCCUPIED), 0)

    def test_negative_coordinates_and_snapshot(self):
        m = VoxelMap([-2, 2, -1, 1, -.5, 1.5], .5)
        m.integrate([-1.75, -.75, -.25], [[1.75, -.75, -.25]])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.npz"
            m.save(path)
            copy = VoxelMap.load(path)
            np.testing.assert_array_equal(copy.state, m.state)
            np.testing.assert_array_equal(copy.centers, m.centers)

    def test_first_unknown_not_occluded_unknown(self):
        m = VoxelMap([0, 5, 0, 1, 0, 1], 1)
        m.state[0, 0, :2] = FREE
        sensor = SensorModel(.5, 0, 10, -1, 1)
        self.assertEqual(m.visible([.5, .5, .5], m.unknown, sensor).tolist(), [2])
        self.assertEqual(m.visible([.5, .5, .5], m.unknown, sensor, True).tolist(), [2, 3, 4])
        m.state[0, 0, 2] = OCCUPIED
        self.assertEqual(len(m.visible([.5, .5, .5], m.unknown, sensor, True)), 0)

    def test_no_return_max_range_and_nan(self):
        m = VoxelMap([0, 5, 0, 1, 0, 1], 1)
        matrix = np.eye(4)
        matrix[:3, 3] = [.5, .5, .5]
        count = m.integrate_cloud([[4, 0, 0], [np.nan, 0, 0]], matrix, max_range=4)
        self.assertEqual(count, 1)
        self.assertFalse((m.state == OCCUPIED).any())


class ReachabilityTests(unittest.TestCase):
    def test_corner_cutting_and_no_snapping(self):
        grid = GroundGrid([[0, 1], [1, 0]], 1, clearance=0)
        self.assertFalse(np.isfinite(grid.distances([.5, .5])[1, 1]))
        self.assertFalse(np.isfinite(grid.distances([1.5, .5])).any())
        self.assertEqual(grid.path([.5, .5], [1.5, 1.5]), [])

    def test_entire_path_respects_footprint(self):
        data = np.zeros((9, 9), dtype=np.int8)
        data[:, 4] = OCCUPIED
        data[4, 4] = FREE  # one-cell doorway too narrow for this body
        small = GroundGrid(data, .25, clearance=0)
        wide = GroundGrid(data, .25, clearance=.2)
        self.assertTrue(np.isfinite(small.distances([.625, 1.125])[4, 6]))
        self.assertFalse(np.isfinite(wide.distances([.625, 1.125])[4, 6]))

    def test_obstacle_detour_and_path_length(self):
        data = np.zeros((7, 7), dtype=np.int8)
        data[:5, 3] = OCCUPIED
        grid = GroundGrid(data, 1, clearance=0)
        start, end = (1.5, 1.5), (5.5, 1.5)
        path = grid.path(start, end)
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        self.assertGreater(length, 4)
        self.assertAlmostEqual(length, grid.distances(start)[grid.cell(end)])
        self.assertTrue(all(grid.valid(grid.cell(p), grid.safe) for p in path))

    def test_unseen_corridor_is_deferred_not_excluded(self):
        m = VoxelMap([0, 6, 0, 3, 0, 1], 1)
        m.state[:, :, :2] = FREE
        data = np.full((3, 6), UNKNOWN, dtype=np.int8)
        data[:, :2] = FREE
        ground = GroundGrid(data, 1, clearance=0)
        result = evaluate_candidates(m, ground, [(0.5, 1.5)] * 2,
                                     SensorModel(.5, 0, 2, -1, 1), 1)
        far = int(m.indices([5.5, 1.5, .5])[0])
        self.assertIn(far, result.deferred)
        self.assertNotIn(far, result.excluded)
        self.assertEqual(result.actionable | result.deferred | result.excluded, set(m.unknown))
        self.assertFalse(result.actionable & result.deferred)

    def test_closed_wall_exclusion_reactivates_when_opened(self):
        m = VoxelMap([0, 6, 0, 3, 0, 1], 1)
        m.state[:, :, :2] = FREE
        m.state[:, :, 2] = OCCUPIED
        data = np.zeros((3, 6), dtype=np.int8)
        data[:, 2] = OCCUPIED
        sensor = SensorModel(.5, 0, 10, -1, 1)
        starts = [(0.5, 1.5)] * 2
        result = evaluate_candidates(m, GroundGrid(data, 1, clearance=0), starts, sensor, 1)
        self.assertEqual(result.excluded, set(m.unknown))
        data[1, 2] = FREE
        m.state[0, 1, 2] = FREE
        reopened = evaluate_candidates(m, GroundGrid(data, 1, clearance=0), starts, sensor, 1)
        self.assertLess(len(reopened.excluded), len(result.excluded))

    def test_no_visibility_or_candidate_cap(self):
        m = VoxelMap([0, 25, 0, 25, 0, 1], 1)
        grid = GroundGrid(np.zeros((25, 25), dtype=np.int8), 1, clearance=0)
        result = evaluate_candidates(m, grid, [(.5, .5), (24.5, 24.5)],
                                     SensorModel(.5, 0, 2, -1, 1), 1)
        self.assertEqual(result.evaluated_positions, 625)
        self.assertGreater(len(result.actionable), 400)
        self.assertEqual(len(result.excluded), 0)


class RouteTests(unittest.TestCase):
    @staticmethod
    def coverage(candidates):
        return Coverage(candidates, set().union(*(c.coverage for c in candidates)), set(), set(), len(candidates), len(candidates))

    def test_joint_selection_can_skip_far_high_coverage_view(self):
        grid = GroundGrid(np.zeros((3, 20), dtype=np.int8), 1, clearance=0)
        candidates = [
            Candidate((1, 1), (1.5, 1.5), frozenset([0])),
            Candidate((1, 3), (3.5, 1.5), frozenset([1])),
            Candidate((1, 19), (19.5, 1.5), frozenset([0, 1])),
        ]
        result = solve_coverage_routes(self.coverage(candidates), grid,
                                      [(1.5, 1.5), (3.5, 1.5)], (1, 1), 2)
        self.assertEqual(result.routes, [[0], [1]])
        self.assertEqual(result.makespan, 2)

    def test_exact_matches_independent_bruteforce_with_different_speeds(self):
        data = np.zeros((5, 7), dtype=np.int8)
        data[:3, 3] = OCCUPIED
        grid = GroundGrid(data, 1, clearance=0)
        candidates = [Candidate(cell, grid.xy(cell), frozenset(cover))
                      for cell, cover in [((0, 0), [0]), ((4, 2), [1, 2]),
                                          ((0, 6), [2, 3]), ((4, 6), [0, 3])]]
        starts, speeds, dwell = [(1.5, .5), (5.5, .5)], (.5, 1.25), 1.7
        coverage = self.coverage(candidates)
        best = math.inf
        for owners in itertools.product((-1, 0, 1), repeat=len(candidates)):
            chosen = [i for i, owner in enumerate(owners) if owner >= 0]
            if set().union(*(candidates[i].coverage for i in chosen)) != coverage.actionable:
                continue
            partitions = [[i for i, owner in enumerate(owners) if owner == r] for r in (0, 1)]
            for r0 in itertools.permutations(partitions[0]):
                for r1 in itertools.permutations(partitions[1]):
                    times = []
                    for r, route in enumerate((r0, r1)):
                        previous, elapsed = starts[r], 0.
                        for i in route:
                            elapsed += grid.distances(previous)[candidates[i].cell] / speeds[r] + dwell
                            previous = candidates[i].xy
                        times.append(elapsed)
                    best = min(best, max(times))
        result = solve_coverage_routes(coverage, grid, starts, speeds, dwell)
        self.assertAlmostEqual(result.makespan, best)

    def test_disconnected_tasks_only_assigned_to_reachable_robot(self):
        data = np.zeros((3, 7), dtype=np.int8)
        data[:, 3] = OCCUPIED
        grid = GroundGrid(data, 1, clearance=0)
        candidates = [Candidate((1, c), (c + .5, 1.5), frozenset([c])) for c in (0, 1, 5, 6)]
        for limit in (0, 10):
            result = solve_coverage_routes(self.coverage(candidates), grid,
                                          [(.5, 1.5), (6.5, 1.5)], (1, 1), 1, limit)
            self.assertEqual(set(result.routes[0]), {0, 1})
            self.assertEqual(set(result.routes[1]), {2, 3})
            self.assertTrue(math.isfinite(result.makespan))

    def test_heuristic_covers_all_without_task_truncation(self):
        grid = GroundGrid(np.zeros((3, 30), dtype=np.int8), 1, clearance=0)
        candidates = [Candidate((1, c), (c + .5, 1.5), frozenset([c])) for c in range(30)]
        result = solve_coverage_routes(self.coverage(candidates), grid,
                                      [(.5, 1.5), (29.5, 1.5)], (1, 1), 1, 5)
        self.assertEqual(set(result.routes[0] + result.routes[1]), set(range(30)))
        self.assertEqual(result.method, "heuristic_joint_coverage")


if __name__ == "__main__":
    unittest.main()
