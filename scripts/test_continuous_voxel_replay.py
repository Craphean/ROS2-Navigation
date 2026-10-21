"""Finite sensor range, continuous motion and actual video-duration checks."""
from pathlib import Path
from concurrent.futures import Future
import contextlib
import io
import json
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from classroom_voxel_scene import ROOT
from continuous_voxel_replay import (
    Motion, VideoOutput, apply_plan, prune_observed_routes, run_continuous,
    step_until_arrival, stop_for_arrival_replan, wait_for_task_replan,
)
from voxel_replay_io import lidar_vectors, ray_returns, scan_pose
from merge_pointclouds import PointCloudMerger
from lidar360_observation import Lidar360
from observation_visualization import MapImage
from voxel_exploration_core import FREE, OCCUPIED, UNKNOWN, Candidate, Coverage, GroundGrid, RoutePlan, VoxelMap


class ContinuousReplayTests(unittest.TestCase):
    def test_first_arrival_advances_both_cars_to_same_time_without_dwell(self):
        motions = [
            Motion((0., 0.), 0., 1., route=[0], path=[(0., 0.), (.3, 0.)], length=.3),
            Motion((0., 1.), 0., 2., route=[1], path=[(0., 1.), (4., 1.)], length=4.),
        ]
        dt, arrived = step_until_arrival(motions, 0., 1.)
        self.assertAlmostEqual(dt, .3)
        self.assertEqual(arrived, [0])
        np.testing.assert_allclose([m.xy for m in motions], [[.3, 0.], [.6, 1.]])
        self.assertAlmostEqual(motions[0].finish_at, .3)
        stop_for_arrival_replan(motions, None, None)
        before = [m.xy for m in motions]
        self.assertEqual(step_until_arrival(motions, .3, 1.), (1., []))
        self.assertEqual([m.xy for m in motions], before)

    def test_arrival_cycle_scan_plan_joint_execution_integration(self):
        """Early observed requirements do not interrupt a committed physical visit."""
        profile = Lidar360.load()
        truth = VoxelMap([0, 8, 0, 3, 0, 1], 1.)
        known = truth.snapshot(empty=True)
        ground = GroundGrid(np.zeros((3, 8), dtype=np.int8), 1., clearance=0.)
        starts = [(.5, .5), (.5, 2.5)]
        config = dict(observation_model="lidar360", lidar_profile="unused", replan_policy="arrival_cycle",
                      sensor_max_range=5., min_clearance=0., robot_speeds=[1., 1.],
                      robot1_init_pose=[*starts[0], 0.], robot2_init_pose=[*starts[1], 0.],
                      observation_time=1., completion_stable_seconds=8., idle_replan_period=5.,
                      candidate_position_stride=1., exact_candidate_limit=10, failed_view_cooldown=30.)
        calculations, scans = [], []

        def observe(_truth, state, xy, _sensor, _vectors, cloud, robot, stamp, _yaw):
            cloud.add(robot, [], stamp)
            scans.append((robot, stamp, tuple(xy)))
            if stamp > 0. and state.state.ravel()[0] == UNKNOWN:
                state.state.ravel()[:2] = FREE
                state.revision += 1
                return 2
            return 0

        def compute(snapshot, _data, _resolution, _origin, _clearance, positions, *_rest):
            n = len(calculations)
            calculations.append((snapshot, positions))
            candidates = ([Candidate((0, 2), (2.5, .5), frozenset([0])),
                           Candidate((2, 5), (5.5, 2.5), frozenset([1]))] if n == 0 else
                          [Candidate((0, 4), (4.5, .5), frozenset([2])),
                           Candidate((2, 6), (6.5, 2.5), frozenset([3]))])
            ids = set().union(*(c.coverage for c in candidates))
            return (Coverage(candidates, ids, set(map(int, snapshot.unknown))-ids, set(), 2, 0, False, "nearest"),
                    RoutePlan([[0], [1]], (2., 5.), 5., "fixture", [[], []]), 7.25)

        def submit(fn, *args):
            # Deliberately unfinished until result() is awaited. Polling while
            # advancing simulation time would never accept this plan.
            future = Future()
            future.result = Mock(side_effect=lambda: fn(*args))
            return future

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            world = root/"world.sdf"
            world.write_text("<sdf/>")
            args = SimpleNamespace(output_dir=root, config=None, world=world, resolution=None,
                                   sensor_range=None, duration=10., max_events=2, scan_hz=10.,
                                   record_period=.5, playback_speed=1., simulation_rate=0.)
            planar = Mock(max_range=5.)
            planar.image.return_value = MapImage(ground.data, 1., (0., 0., 0.), 0.)
            worker, video = Mock(), Mock(frames_written=0)
            worker.submit.side_effect = submit
            with (patch("continuous_voxel_replay.scene", return_value=(truth, known, ground, starts, profile.sensor(), [], config)),
                  patch("continuous_voxel_replay.Lidar360.load", return_value=profile),
                  patch("continuous_voxel_replay.observe", side_effect=observe),
                  patch("continuous_voxel_replay.PlanarObservations", return_value=planar),
                  patch("continuous_voxel_replay.compute_plan", side_effect=compute),
                  patch("continuous_voxel_replay.ThreadPoolExecutor", return_value=worker),
                  patch("continuous_voxel_replay.VideoOutput", return_value=video),
                  patch("continuous_voxel_replay.save_frame", return_value=Image.new("RGB", (8, 8))),
                  contextlib.redirect_stdout(io.StringIO())):
                run_continuous(args)
            report = json.loads((root/"events.json").read_text())
        self.assertEqual(len(calculations), 2)
        first, second = report["events"]
        self.assertEqual(first["time"], 2.)
        self.assertEqual(first["arrived_robots"], [1])
        self.assertEqual(first["finished_robots"], [1])
        self.assertEqual(first["next_goals"], [None, None])
        self.assertGreater(first["post_stop_scan_time"], first["time"])
        request = report["planning_requests"][1]
        self.assertEqual(request["request_time"], first["post_stop_scan_time"])
        self.assertEqual(request["accepted_time"], request["request_time"])
        self.assertEqual(report["planning_time_model"], "instantaneous")
        self.assertEqual(report["planning_wall_seconds"], 14.5)
        self.assertEqual(report["replans"][1]["map_age_seconds"], 0.)
        self.assertEqual(report["scan_counts"], [42, 42])
        np.testing.assert_allclose(calculations[1][1], [[2.5, .5], [2.5, 2.5]])
        self.assertTrue((calculations[1][0].state.ravel()[:2] == FREE).all())
        for robot, stamp, xy in scans:
            if first["time"] <= stamp <= request["accepted_time"]:
                np.testing.assert_allclose(xy, first["robot_positions"][robot])
        self.assertAlmostEqual(second["time"]-request["accepted_time"], 2.)
        self.assertEqual(report["skipped_observed_views"], [0, 0])
        self.assertTrue(all(e["trigger"] == "arrival" for e in report["events"]))
        # Neither the renderer nor the scan clock sees a computation interval.
        self.assertTrue(all(r["accepted_time"] == r["request_time"]
                            for r in report["planning_requests"]))

    def test_task_completion_waits_for_new_joint_plan_without_starting_queued_visit(self):
        ground = GroundGrid(np.zeros((4, 8), dtype=np.int8), 1., clearance=0)
        coverage = SimpleNamespace(candidates=[
            SimpleNamespace(cell=(1, col), xy=(col+.5, 1.5), coverage={col})
            for col in (1, 3, 6)])
        motions = [Motion((1.5, 1.5), 0., 1., route=[1, 2]),
                   Motion((4.5, 1.5), 0., 1., route=[2, 0])]
        for motion in motions:
            motion.next_path(ground, coverage)
        active_path = motions[1].path
        wait_for_task_replan(motions, {0}, ground, coverage)
        self.assertEqual(motions[0].route, [])
        self.assertEqual(motions[1].route, [2])
        self.assertIs(motions[1].path, active_path)
        before = motions[0].xy
        motions[0].step(10., 1., 1.)
        self.assertEqual(motions[0].xy, before)
        # A new joint plan's ordering is respected, without restoring the old tour.
        known = VoxelMap([0, 8, 0, 4, 0, 1], 1.)
        apply_plan(motions, coverage, SimpleNamespace(routes=[[1, 0], [2]]),
                   ground, known, set(), [None, (1, 6)], task_event=True)
        self.assertEqual(motions[0].route, [1, 0])
        self.assertIs(motions[1].path, active_path)
        self.assertEqual(motions[0].path[0], before)

    def test_both_configured_sensors_have_five_metre_limit(self):
        model = ET.parse(ROOT/"config/gz_waffle_3d_lidar.sdf.xacro")
        for name in ("hls_lfcd_lds", "tb3_3d_lidar"):
            self.assertEqual(float(model.findtext(f".//sensor[@name='{name}']/ray/range/max")), 5.)
        with self.assertRaises(ValueError):
            lidar_vectors(6.)
        np.testing.assert_allclose(np.linalg.norm(lidar_vectors(3.), axis=1), 3.)

    def test_far_obstacle_has_no_return_or_free_evidence_past_range(self):
        truth = VoxelMap([0, 10, 0, 1, 0, 1], .1)
        truth.state[:] = FREE
        truth.state[:, :, 80] = OCCUPIED
        origin = np.array([1., .55, .55])
        endpoints, hit = ray_returns(truth, origin, np.array([[5., 0., 0.]]))
        self.assertFalse(hit[0])
        self.assertLessEqual(np.linalg.norm(endpoints[0]-origin), 5.+1e-9)
        known = truth.snapshot(empty=True)
        known.integrate(origin, endpoints, hit)
        self.assertTrue((known.state[:, :, 61:] == UNKNOWN).all())
        self.assertFalse((known.state == OCCUPIED).any())

    def test_moving_scan_rotates_beams_and_sensor_offset(self):
        origin, directions = scan_pose((2., 3.), (-.064, 0.), .22,
                                       np.array([[5., 0., 0.]]), np.pi/2)
        np.testing.assert_allclose(origin, [2., 2.936, .22])
        np.testing.assert_allclose(directions, [[0., 5., 0.]], atol=1e-12)

    def test_motion_uses_speed_and_dwell_not_video_frame_count(self):
        m = Motion((0., 0.), 0., .22, route=[0], path=[(0., 0.), (2.2, 0.)], length=2.2)
        for t in range(5):
            self.assertFalse(m.step(t, 1., 1.))
        np.testing.assert_allclose(m.xy, [1.1, 0.])
        for t in range(5, 10):
            self.assertFalse(m.step(t, 1., 1.))
        self.assertTrue(m.step(10., 1., 1.))
        self.assertAlmostEqual(m.distance, 2.2)

    def test_delayed_plan_starts_at_current_pose_and_skips_completed_views(self):
        ground = GroundGrid(np.zeros((4, 8), dtype=np.int8), 1., clearance=0)
        known = VoxelMap([0, 8, 0, 4, 0, 1], 1.)
        coverage = SimpleNamespace(candidates=[
            SimpleNamespace(cell=(1, 4), xy=(4.5, 1.5), coverage={0}),
            SimpleNamespace(cell=(1, 6), xy=(6.5, 1.5), coverage={1})])
        motion = Motion((3.5, 1.5), 0., .22)
        apply_plan([motion], coverage, SimpleNamespace(routes=[[0, 1]]),
                   ground, known, {(1, 4)})
        self.assertEqual(motion.xy, (3.5, 1.5))
        self.assertEqual(motion.path[0], (3.5, 1.5))
        self.assertEqual(motion.route, [1])

    def test_continuous_scan_skips_other_robots_completed_visits_without_waiting(self):
        ground = GroundGrid(np.zeros((4, 8), dtype=np.int8), 1., clearance=0)
        known = VoxelMap([0, 8, 0, 4, 0, 1], 1.)
        coverage = SimpleNamespace(candidates=[
            SimpleNamespace(cell=(1, 4), xy=(4.5, 1.5), coverage={0, 1}),
            SimpleNamespace(cell=(1, 6), xy=(6.5, 1.5), coverage={2})])
        motions = [Motion((1.5, 1.5), 0., .22, route=[0, 1]),
                   Motion((2.5, 1.5), 0., .22, route=[0])]
        for m in motions:
            m.next_path(ground, coverage)
        known.state.ravel()[0] = FREE
        self.assertEqual(prune_observed_routes(motions, coverage, ground, known), [0, 0])
        known.state.ravel()[1] = OCCUPIED
        before = known.state.copy()
        self.assertEqual(prune_observed_routes(motions, coverage, ground, known), [1, 1])
        self.assertEqual(motions[0].route, [1])
        self.assertEqual(motions[0].path[0], motions[0].xy)
        self.assertEqual(motions[1].route, [])
        self.assertEqual(motions[0].observations, 0)
        np.testing.assert_array_equal(known.state, before)

    def test_old_plan_does_not_turn_a_useful_active_visit_back_toward_spawn(self):
        ground = GroundGrid(np.zeros((4, 8), dtype=np.int8), 1., clearance=0)
        known = VoxelMap([0, 8, 0, 4, 0, 1], 1.)
        coverage = SimpleNamespace(candidates=[
            SimpleNamespace(cell=(1, 1), xy=(1.5, 1.5), coverage={0}),
            SimpleNamespace(cell=(1, 6), xy=(6.5, 1.5), coverage={1})])
        motion = Motion((3.7, 1.5), .2, .22, route=[1],
                        path=[(3.5, 1.5), (6.5, 1.5)], progress=.2, length=3.)
        old_path = motion.path
        apply_plan([motion], coverage, SimpleNamespace(routes=[[0, 1]]),
                   ground, known, set(), [(1, 6)])
        self.assertEqual(motion.route, [1, 0])
        self.assertIs(motion.path, old_path)
        self.assertEqual(motion.progress, .2)
        self.assertFalse(motion.step(10., .1, 1.))
        self.assertGreater(motion.xy[0], 3.7)
        # Reaccepting the same target during dwell must not restart its timer.
        motion.finish_at = 12.
        apply_plan([motion], coverage, SimpleNamespace(routes=[[0, 1]]),
                   ground, known, set(), [(1, 6)])
        self.assertEqual(motion.finish_at, 12.)

    def test_old_plan_without_active_goal_is_reordered_from_live_pose(self):
        ground = GroundGrid(np.zeros((4, 8), dtype=np.int8), 1., clearance=0)
        known = VoxelMap([0, 8, 0, 4, 0, 1], 1.)
        coverage = SimpleNamespace(candidates=[
            SimpleNamespace(cell=(1, col), xy=(col+.5, 1.5), coverage={col})
            for col in (1, 3, 5)])
        motion = Motion((5.5, 1.5), 0., .22)
        apply_plan([motion], coverage, SimpleNamespace(routes=[[0, 1, 2]]),
                   ground, known, set(), [None])
        self.assertEqual(motion.route, [2, 1, 0])
        self.assertEqual(motion.path[0], motion.xy)
        self.assertEqual(set(motion.route), {0, 1, 2})

    def test_video_duration_tracks_simulated_timestamps_without_padding(self):
        with tempfile.TemporaryDirectory() as directory, patch("continuous_voxel_replay.cv2.VideoWriter") as factory:
            writer = factory.return_value
            writer.isOpened.return_value = True
            video = VideoOutput(Path(directory), 1., 1.)
            image = Image.new("RGB", (16, 16))
            for stamp in (0., 60., 180.):
                video.append(image, stamp)
            video.close(image, 180.)
            self.assertEqual(video.frames_written, 1800)
            self.assertEqual(writer.write.call_count, 1800)

    def test_cloud_merger_uses_spherical_range_and_drops_max_range_shell(self):
        merger = object.__new__(PointCloudMerger)
        merger.min_range, merger.max_range = .1, 5.
        merger.min_z, merger.max_z = -10., 10.
        merger.footprint_x_min, merger.footprint_x_max = -.22, .22
        merger.footprint_y_min, merger.footprint_y_max = -.18, .18
        merger.footprint_z_max = .05
        merger.world_frame, merger.tf_timeout = "world", .01
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        merger.tf_buffer = Mock()
        merger.tf_buffer.lookup_transform.return_value = transform
        msg = point_cloud2.create_cloud_xyz32(Header(frame_id="base_scan"),
                                            [[4., 0., 4.], [5., 0., 0.], [3., 0., 3.]])
        points, _ = merger._transform_points("robot1", msg)
        np.testing.assert_allclose(points, [[3., 0., 3.]])


if __name__ == "__main__":
    unittest.main()
