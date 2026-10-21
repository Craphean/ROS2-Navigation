"""Full-circle density, finite-beam visibility and heading-free task execution."""
from dataclasses import replace
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml

from classroom_voxel_scene import ROOT, DEFAULT_CONFIG
from continuous_voxel_replay import Motion
from voxel_replay_io import PlanarObservations, observe, parse_args
from lidar360_observation import Lidar360, FullCircleVisibility, evaluate_candidates_360, first_unknowns
from observation_visualization import ObservedPoints
from prepare_voxel360_sim import prepare
from voxel_exploration_core import FREE, OCCUPIED, UNKNOWN, GroundGrid, VoxelMap, solve_coverage_routes


class Lidar360Tests(unittest.TestCase):
    def test_profile_density_rate_full_circle_and_unique_seam(self):
        profile = Lidar360.load()
        vectors = profile.vectors()
        self.assertEqual(vectors.shape, (64*1024, 3))
        self.assertEqual(profile.scan_hz, 10.)
        self.assertEqual(profile.metadata()["rays_per_second"], 655360)
        azimuth = np.unwrap(np.arctan2(vectors[:1024, 1], vectors[:1024, 0]))
        np.testing.assert_allclose(np.diff(azimuth), 2*np.pi/1024, atol=1e-12)
        self.assertFalse(np.allclose(vectors[0], vectors[1023]))
        np.testing.assert_allclose(np.linalg.norm(vectors, axis=1), 5.)
        elevation = np.rad2deg(np.arctan2(vectors[::1024, 2], np.linalg.norm(vectors[::1024, :2], axis=1)))
        np.testing.assert_allclose(elevation, np.linspace(-45., 45., 64))

    def test_generated_gazebo_density_frame_and_nav_goal_checker_match(self):
        with tempfile.TemporaryDirectory() as folder:
            prepare(folder)
            document = xacro.process_file(str(Path(folder)/"robot.sdf.xacro"),
                                         mappings={"namespace": "robot1"})
            model = ET.fromstring(document.toxml())
            profile = Lidar360.load()
            for name in ("hls_lfcd_lds", "tb3_3d_lidar"):
                sensor = model.find(f".//sensor[@name='{name}']")
                self.assertEqual(float(sensor.findtext("update_rate")), profile.scan_hz)
                self.assertEqual(sensor.findtext("frame_id"), "lidar360")
                self.assertEqual(int(sensor.findtext("ray/scan/horizontal/samples")), 1024)
                low = float(sensor.findtext("ray/scan/horizontal/min_angle"))
                high = float(sensor.findtext("ray/scan/horizontal/max_angle"))
                self.assertAlmostEqual((high-low)/1023, 2*np.pi/1024)
                self.assertEqual(float(sensor.findtext("ray/range/max")), 5.)
                np.testing.assert_allclose(np.fromstring(sensor.findtext("pose"), sep=" ")[:2], 0.)
            self.assertEqual(model.findtext(".//sensor[@name='tb3_3d_lidar']/ray/scan/vertical/samples"), "64")
            np.testing.assert_allclose(np.fromstring((Path(folder)/"lidar_tf.xyz").read_text(), sep=" "), [.064, 0., 0.])
            config = yaml.safe_load((Path(folder)/"nav2.yaml").read_text())
            controller = config["controller_server"]["ros__parameters"]
            self.assertEqual(controller["general_goal_checker"]["plugin"], "nav2_controller::PositionGoalChecker")
            self.assertAlmostEqual(controller["general_goal_checker"]["xy_goal_tolerance"], .025)
            self.assertNotIn("yaw_goal_tolerance", controller["general_goal_checker"])
            self.assertFalse(controller["FollowPath"]["GoalAngleCritic"]["enabled"])
            self.assertTrue(config["planner_server"]["ros__parameters"]["GridBased"]["use_final_approach_orientation"])

    def test_stopping_at_observation_position_does_not_reset_heading(self):
        motion = Motion((0., 0.), .3, 1., route=[0], path=[(0., 0.), (0., 1.)],
                        length=1., observation_yaw=None)
        self.assertFalse(motion.step(0., 1., 1.))
        self.assertAlmostEqual(motion.yaw, np.pi/2)
        self.assertTrue(motion.step(1., 1., 1.))
        self.assertAlmostEqual(motion.yaw, np.pi/2)
        stopped = Motion((0., 0.), 1.23, 1., route=[0], path=[(0., 0.)], observation_yaw=None)
        stopped.step(0., .1, 1.)
        self.assertAlmostEqual(stopped.yaw, 1.23)

    def test_first_unknown_occludes_and_max_range_boundary_is_not_observed(self):
        voxels = VoxelMap([0, 5, 0, 1, 0, 1], 1.)
        voxels.state[:] = FREE
        voxels.state[0, 0, 1:] = UNKNOWN
        result = first_unknowns(voxels, [.5, .5, .5], np.array([[4., 0., 0.]]), .1)
        np.testing.assert_equal(result, [1])
        voxels.state[0, 0, 1] = OCCUPIED
        self.assertEqual(len(first_unknowns(voxels, [.5, .5, .5], np.array([[4., 0., 0.]]), .1)), 0)
        voxels.state[:] = FREE
        voxels.state[0, 0, 2] = UNKNOWN
        self.assertEqual(len(first_unknowns(voxels, [.5, .5, .5], np.array([[1.5, 0., 0.]]), .1)), 0)

    def test_sorted_boundary_filter_matches_set_intersection_including_empty_targets(self):
        voxels = VoxelMap([-2, 2, -2, 2, 0, 2], .1)
        voxels.state[1:5, 17:23, 17:23] = FREE
        profile = Lidar360.load()
        visibility = FullCircleVisibility(voxels, profile)
        origin = np.array([.05, .05, .22])
        for targets in (voxels.unknown[::3], np.array([], dtype=np.int64)):
            np.testing.assert_equal(visibility(origin, targets, profile.sensor(), sorted_targets=True),
                                    visibility(origin, targets, profile.sensor()))

    def test_blind_source_fast_path_cannot_remove_visible_part_of_large_cell(self):
        tiny = VoxelMap([0, .2, 0, .2, 0, .2], .1)
        rays = np.array([[2., 0., 0.]])
        self.assertEqual(len(first_unknowns(tiny, [.05, .05, .05], rays, .3)), 0)
        large = VoxelMap([0, 2, 0, 1, 0, 1], 1.)
        # The unknown source extends beyond the blind range and is observable.
        np.testing.assert_equal(first_unknowns(large, [.1, .5, .5], rays, .3), [0])

    def test_native_and_numpy_ray_coverage_agree(self):
        rng = np.random.default_rng(21)
        voxels = VoxelMap([-2, 2, -2, 2, 0, 2], .25)
        voxels.state[:] = rng.choice([FREE, UNKNOWN, OCCUPIED], size=voxels.state.shape, p=[.9, .08, .02])
        voxels.state[2:5, 6:10, 6:10] = FREE
        profile = replace(Lidar360.load(), horizontal_samples=64, vertical_samples=8, max_range=3.)
        origin = np.array([.1, .1, .8])
        accelerated = first_unknowns(voxels, origin, profile.vectors(), .3)
        with patch("lidar360_observation.library", return_value=None):
            reference = first_unknowns(voxels, origin, profile.vectors(), .3)
        np.testing.assert_equal(accelerated, reference)

    def test_beam_gap_is_not_geometric_unreachability(self):
        voxels = VoxelMap([-2, 2, -2, 2, 0, 1], .1)
        voxels.state[:] = FREE
        target, _ = voxels.indices([1.05, 1.05, .55])
        voxels.state.ravel()[target] = UNKNOWN
        profile = replace(Lidar360.load(), horizontal_samples=4, vertical_samples=2,
                          vertical_min_deg=-1., vertical_max_deg=1., height=.55, max_range=3.)
        visibility = FullCircleVisibility(voxels, profile)
        self.assertEqual(len(visibility([0., 0., .55], np.array([target]), profile.sensor())), 0)
        np.testing.assert_equal(visibility([0., 0., .55], np.array([target]), profile.sensor(), optimistic=True), [target])

    def test_full_scan_returns_surfaces_on_all_four_sides(self):
        truth = VoxelMap([-2, 2, -2, 2, 0, 1], .1)
        truth.state[:] = FREE
        truth.state[:, (0, -1), :] = OCCUPIED
        truth.state[:, :, (0, -1)] = OCCUPIED
        profile = replace(Lidar360.load(), height=.5)
        cloud = ObservedPoints()
        known = truth.snapshot(empty=True)
        observe(truth, known, (0., 0.), profile.sensor(), profile.vectors(), cloud, 0, .1, 0.)
        points = cloud.latest[0]
        for axis in (0, 1):
            self.assertTrue(np.any(points[:, axis] > 1.8))
            self.assertTrue(np.any(points[:, axis] < -1.8))
        self.assertLessEqual(np.linalg.norm(points-[0, 0, .5], axis=1).max(), 5.)
        planar = PlanarObservations(truth, profile.sensor(), 5., profile=profile)
        self.assertEqual(len(planar.vectors), 1024)

    def test_candidate_positions_are_unique_and_same_joint_solver_covers_tasks(self):
        voxels = VoxelMap([0, 3, 0, 3, 0, 1], .5)
        voxels.state[:] = FREE
        voxels.state[0, 3, 3] = UNKNOWN
        ground = GroundGrid(np.zeros((6, 6), dtype=np.int8), .5, clearance=0.)
        starts = [(.25, .25), (2.75, 2.75)]
        profile = replace(Lidar360.load(), horizontal_samples=64, vertical_samples=8,
                          height=.25, min_range=.1)
        coverage = evaluate_candidates_360(voxels, ground, starts, profile.sensor(), 1., profile)
        self.assertTrue(coverage.actionable)
        self.assertEqual(len(coverage.candidates), len({c.cell for c in coverage.candidates}))
        plan = solve_coverage_routes(coverage, ground, starts, exact_limit=10)
        covered = set().union(*(coverage.candidates[i].coverage for route in plan.routes for i in route))
        self.assertEqual(covered, coverage.actionable)

    def test_new_entry_retains_completion_driven_defaults(self):
        args = parse_args([], defaults={"scan_hz": 10., "config": ROOT/"config/voxel360_strong_baseline_params.yaml"})
        self.assertEqual(args.duration, 0.)
        self.assertIsNone(args.max_events)
        self.assertEqual(args.scan_hz, 10.)
        baseline = yaml.safe_load(DEFAULT_CONFIG.read_text())["rolling_observation_planner"]["ros__parameters"]
        current = yaml.safe_load(args.config.read_text())["rolling_observation_planner"]["ros__parameters"]
        for key in ("voxel_resolution", "candidate_position_stride", "robot_speeds", "observation_time",
                    "completion_stable_seconds", "exploration_bounds"):
            self.assertEqual(current[key], baseline[key])


if __name__ == "__main__":
    unittest.main()
