"""Fine geometry and accelerated-kernel equivalence regression tests."""
import unittest
from unittest.mock import patch

import numpy as np

from classroom_voxel_scene import Box, DEFAULT_WORLD, load_boxes, scene, voxelize_boxes
from voxel_replay_io import lidar_vectors
from voxel_exploration_core import FREE, OCCUPIED, UNKNOWN, GroundGrid, SensorModel, VoxelMap, evaluate_candidates
from voxel_native import library


class ClassroomTests(unittest.TestCase):
    def test_actual_xacro_and_thin_collision_geometry(self):
        boxes = load_boxes(DEFAULT_WORLD)
        desks = {b.name.split("/")[0] for b in boxes if b.name.startswith("desk_c")}
        self.assertEqual(len(desks), 12)
        self.assertEqual(len(boxes), 103)
        leg = next(b for b in boxes if b.name == "desk_c1_r1/l1")
        np.testing.assert_allclose(leg.center, [-5.16, .92, .36])
        np.testing.assert_allclose(leg.size, [.06, .06, .72])

    def test_sub_voxel_geometry_is_not_lost_by_center_sampling(self):
        box = Box("thin", np.array([.05, .05, .05]), np.eye(3), np.array([.02, .02, .02]))
        voxels = voxelize_boxes([box], [0, .1, 0, .1, 0, .1], .05)
        # None of the 8 voxel centers lie inside the box, but all overlap it.
        self.assertEqual(int(np.count_nonzero(voxels.state == OCCUPIED)), 8)

    def test_classroom_defaults_and_spawn_positions(self):
        truth, known, ground, starts, _, _, _ = scene()
        self.assertEqual(truth.resolution, .05)
        self.assertEqual(truth.state.size, 4_300_800)
        self.assertEqual(starts, [(-6.5, 4.8), (6.5, 4.8)])
        self.assertTrue((known.state == UNKNOWN).all())
        self.assertIs(known.centers, truth.centers)
        self.assertFalse(known.centers.flags.writeable)
        for point in [(-5.16, .92, .3), (-4.5, 1.2, .73)]:
            index, inside = truth.indices(point)
            self.assertTrue(inside)
            self.assertEqual(truth.state.ravel()[index], OCCUPIED)
        self.assertTrue(all(ground.safe[ground.cell(s)] for s in starts))

    def test_lidar_uses_configured_beam_layout(self):
        vectors = lidar_vectors()
        self.assertEqual(vectors.shape, (360*24, 3))
        np.testing.assert_allclose(np.linalg.norm(vectors, axis=1), 5.)


class NativeEquivalenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if library() is None:
            raise unittest.SkipTest("g++ native kernels unavailable")

    def test_native_visibility_matches_numpy_with_occlusions(self):
        rng = np.random.default_rng(51)
        voxels = VoxelMap([-1, 1, -1, 1, 0, 1], .1)
        voxels.state[:] = rng.choice([FREE, OCCUPIED, UNKNOWN], size=voxels.state.shape, p=[.8, .05, .15])
        sensor = SensorModel(.2, .1, 2, -1, 1)
        for origin in ([.05, .05, .2], [-.5, -.5, .2]):
            for optimistic in (False, True):
                actual = voxels.visible(origin, voxels.unknown, sensor, optimistic)
                with patch("voxel_exploration_core.library", return_value=None):
                    reference = voxels.visible(origin, voxels.unknown, sensor, optimistic)
                np.testing.assert_array_equal(actual, reference)

    def test_native_integration_matches_numpy_external_endpoints_and_ties(self):
        rng = np.random.default_rng(19)
        m = VoxelMap([-1, 1, -1, 1, 0, 1], .1)
        copy = m.snapshot()
        endpoints = np.vstack([rng.uniform([-2, -2, -.5], [2, 2, 2], (120, 3)),
                               [[.55, .55, .55], [-.55, -.55, .55]]])
        hit = rng.random(len(endpoints)) > .2
        m.integrate([.05, .05, .05], endpoints, hit)
        with patch("voxel_exploration_core.library", return_value=None):
            copy.integrate([.05, .05, .05], endpoints, hit)
        np.testing.assert_array_equal(m.state, copy.state)

    def test_native_distances_match_numpy_and_off_center_starts(self):
        data = np.zeros((20, 20), dtype=np.int8)
        data[:17, 8] = OCCUPIED
        native = GroundGrid(data, .05, clearance=0)
        reference = GroundGrid(data, .05, clearance=0)
        a = native.distances([.231, .223])
        with patch("voxel_exploration_core.library", return_value=None):
            b = reference.distances([.231, .223])
        np.testing.assert_allclose(a, b)

    def test_frontier_acceleration_keeps_exhaustive_visibility(self):
        rng = np.random.default_rng(75)
        voxels = VoxelMap([0, 3, 0, 3, 0, 1], .25)
        voxels.state[:] = rng.choice([FREE, OCCUPIED, UNKNOWN], size=voxels.state.shape, p=[.65, .05, .30])
        ground = GroundGrid(np.zeros((12, 12), dtype=np.int8), .25, clearance=0)
        starts, sensor = [(.375, .375), (2.625, 2.625)], SensorModel(.22, .1, 2, -1, 1)
        actual = evaluate_candidates(voxels, ground, starts, sensor, .5)
        expected = set()
        for cell in ground.candidate_cells(starts, .5):
            expected.update(map(int, voxels.visible(sensor.origin(ground.xy(cell)), voxels.unknown, sensor)))
        self.assertEqual(actual.actionable, expected)


if __name__ == "__main__":
    unittest.main()
