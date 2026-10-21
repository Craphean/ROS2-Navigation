"""Asynchronous handoff, coalescing, measured clocks and Nav2 lifecycle tests."""
import contextlib
from concurrent.futures import Future
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image
from action_msgs.msg import GoalStatus

from continuous_voxel_replay import run_continuous
from lidar360_observation import Lidar360
from observation_visualization import MapImage
from voxel_ros_transport import ViewTask
import test_voxel_planner_runtime as runtime_tests
from test_voxel_planner_runtime import future
from voxel_async_execution import latency_deadline, rebase_plan
from voxel_exploration_core import Candidate, Coverage, FREE, OCCUPIED, UNKNOWN, GroundGrid, RoutePlan, VoxelMap


class AsyncReplayTests(unittest.TestCase):
    def test_live_rebase_filters_observed_cooldown_and_newly_unreachable_targets(self):
        known = VoxelMap([0, 8, 0, 3, 0, 1], 1.)
        known.state.ravel()[0] = FREE
        data = np.zeros((3, 8), np.int8)
        data[:, 4] = OCCUPIED
        ground = GroundGrid(data, 1., clearance=0.)
        candidates = [Candidate((1, c), (c+.5, 1.5), frozenset([i]))
                      for i, c in enumerate([0, 1, 6, 3])]
        coverage = Coverage(candidates, set(range(4)), set(), set(), 4, 4)
        old = RoutePlan([[0, 1, 2, 3], []], (9., 0.), 9., 'test', [[], []])
        before = known.state.copy()
        plan, detail = rebase_plan(coverage, old, ground, known, [(2.75, 1.5), (.5, .5)], {(1, 1)})
        self.assertEqual(plan.routes, [[3], []])
        self.assertEqual(plan.paths[0][0], (2.75, 1.5))
        self.assertEqual(detail['filtered_tasks'], 3)
        self.assertEqual(old.routes, [[0, 1, 2, 3], []])
        np.testing.assert_array_equal(before, known.state)
        with self.assertRaises(RuntimeError):
            rebase_plan(coverage, old, ground, known, [(4.5, .5), (.5, .5)])

    def test_latency_rejects_invalid_clock_values(self):
        self.assertEqual(latency_deadline(3., 4.5), 7.5)
        for value in (-1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                latency_deadline(0., value)

    def test_replay_drives_old_queue_coalesces_arrivals_and_counts_compute_time(self):
        self.check_delayed_replay('async_arrival')

    def test_v2_prediction_never_predicts_observations_or_discards_slow_result(self):
        self.check_delayed_replay('async_v2')

    def check_delayed_replay(self, mode):
        profile = Lidar360.load()
        truth = VoxelMap([0, 12, 0, 3, 0, 1], 1.)
        known = truth.snapshot(empty=True)
        ground = GroundGrid(np.zeros((3, 12), np.int8), 1., clearance=0.)
        starts = [(.5, .5), (.5, 2.5)]
        config = dict(observation_model='lidar360', lidar_profile='unused', replan_policy=mode,
                      sensor_max_range=5., min_clearance=0., robot_speeds=[1., 1.],
                      robot1_init_pose=[*starts[0], 0.], robot2_init_pose=[*starts[1], 0.],
                      observation_time=1., completion_stable_seconds=8., idle_replan_period=5.,
                      candidate_position_stride=1., exact_candidate_limit=10, failed_view_cooldown=30.)
        candidates = [Candidate((y, x), (x+.5, y+.5), frozenset([target]))
                      for x, y, target in [(1, 0, 10), (4, 0, 11), (3, 2, 12), (7, 2, 13),
                                           (8, 2, 2), (9, 0, 20), (9, 2, 21)]]
        calls, scanned, rendered = [], [], []

        def compute(snapshot, _data, _resolution, _origin, _clearance, positions, *_rest):
            calls.append((snapshot, list(positions)))
            routes = [[0, 1], [2, 3]] if len(calls) == 1 else [[1, 5], [4, 6]]
            coverage = Coverage(candidates, {10, 11, 12, 13, 2, 20, 21}, set(), set(), 7, 7, False)
            plan = RoutePlan(routes, (9., 9.), 9., 'test', [[], []])
            return coverage, plan, [.5, 4., 10.][min(len(calls)-1, 2)]

        def observe(_truth, measured, xy, _sensor, _vectors, _cloud, robot, stamp, _yaw):
            scanned.append((robot, stamp, tuple(xy)))
            _cloud.add(robot, np.empty((0, 3)), stamp)
            if stamp >= 3.:
                measured.state.ravel()[2] = FREE
            return 0

        def render(*args, **kwargs):
            rendered.append((args[9], [tuple(p[:2]) for p in args[4]], kwargs['cycle_phase']))
            return Image.new('RGB', (8, 8))

        def submit(fn, *args):
            return future(fn(*args))

        worker, video = Mock(), Mock(frames_written=62)
        worker.submit.side_effect = submit
        planar = SimpleNamespace(max_range=5., observe=lambda *_: None,
                                 image=lambda: MapImage(ground.data, 1., (0., 0., 0.), 0.))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            world = root/'world'
            world.write_text('fixture')
            args = SimpleNamespace(output_dir=root, config=world, world=world, resolution=None,
                                   sensor_range=None, scan_hz=10., duration=6.2, max_events=None,
                                   record_period=.1, playback_speed=1., simulation_rate=0.)
            with (patch('continuous_voxel_replay.scene', return_value=(
                    truth, known, ground, starts, profile.sensor(5.), [], config)),
                  patch('continuous_voxel_replay.Lidar360.load', return_value=profile),
                  patch('continuous_voxel_replay.PlanarObservations', return_value=planar),
                  patch('continuous_voxel_replay.observe', side_effect=observe),
                  patch('continuous_voxel_replay.compute_plan', side_effect=compute),
                  patch('continuous_voxel_replay.ThreadPoolExecutor', return_value=worker),
                  patch('continuous_voxel_replay.VideoOutput', return_value=video),
                  patch('continuous_voxel_replay.save_frame', side_effect=render),
                  contextlib.redirect_stdout(io.StringIO())):
                run_continuous(args)
            report = json.loads((root/'events.json').read_text())
        self.assertEqual(len(calls), 3)
        self.assertEqual([e['time'] for e in report['events']], [1.5, 3.5, 4.5])
        self.assertEqual(report['events'][0]['next_goals'], [[4.5, .5], [3.5, 2.5]])
        initial, delayed, followup = report['planning_requests']
        self.assertEqual(initial['accepted_time'], .5)
        self.assertEqual(delayed['request_time'], 1.5)
        self.assertEqual(delayed['accepted_time'], 5.5)
        self.assertEqual(delayed['accepted_event'], 3)
        self.assertEqual(delayed['handoff']['filtered_tasks'], 2)
        np.testing.assert_allclose(delayed['distance_during_planning'], [3., 4.])
        np.testing.assert_allclose(delayed['handoff']['live_positions'], [[4.5, .5], [5.5, 2.5]])
        self.assertEqual(followup['event'], 3)
        self.assertEqual(followup['request_time'], 5.5)
        self.assertEqual(followup['status'], 'computing')
        self.assertEqual(report['discarded_replans'], 0)
        self.assertEqual(report['planning_time_model'], 'measured_latency')
        self.assertFalse(report['complete'])
        self.assertTrue(report['pending_replan_at_stop'])
        self.assertTrue(all(e['trigger'] == 'arrival' for e in report['events']))
        self.assertTrue(any(1.5 < stamp < 3.5 and xy[0] > 1.5 for r, stamp, xy in scanned if r == 0))
        self.assertTrue(any(stamp > 5.5 and xy[0] > 4.5 for r, stamp, xy in scanned if r == 0))
        # The in-flight snapshot is isolated from later moving observations.
        self.assertEqual(calls[1][0].state.ravel()[2], UNKNOWN)
        self.assertEqual(calls[2][0].state.ravel()[2], FREE)
        for request in (initial, delayed):
            self.assertAlmostEqual(request['accepted_time']-request['request_time'], request['planning_seconds'])
        if mode == 'async_v2':
            np.testing.assert_allclose(calls[1][1], delayed['predicted_positions'])
            self.assertGreater(delayed['predicted_positions'][0][0], delayed['robot_positions'][0][0])
            self.assertEqual(delayed['commit_until'], 3.)


class AsyncRosTests(unittest.TestCase):
    setUp = runtime_tests.RuntimeTests.setUp
    completion_fixture = runtime_tests.RuntimeTests.completion_fixture

    def test_arrival_keeps_peer_and_dispatches_next_old_goal_during_worker(self):
        node = self.node
        node.async_arrival_mode = node.task_event_mode = True
        node.task_epoch, node.accepted_task_epoch = 0, 0
        queued = ViewTask(('v', 0, 2), 2.5, .5, .5, 0., frozenset([2]))
        self.robot.route = [self.task, queued]
        peer = SimpleNamespace(state='NAVIGATING', route=[self.task, queued], current_handle=Mock())
        node.robots = {'robot1': self.robot, 'robot2': peer}
        node.planning_future = Future()
        node.dispatch_task = Mock()
        before = node.voxels.state.copy()
        node.on_goal_result(future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), self.robot, 4)
        self.assertEqual(self.robot.route, [queued])
        self.assertEqual(peer.route, [self.task, queued])
        self.assertEqual(peer.state, 'NAVIGATING')
        peer.current_handle.cancel_goal_async.assert_not_called()
        node.dispatch_task.assert_called_once_with(self.robot, queued)
        self.assertTrue(node.plan_dispatch_ready())
        self.assertEqual(node.task_epoch, 1)
        self.assertTrue(node.replan_requested)
        np.testing.assert_array_equal(before, node.voxels.state)

    def test_new_arrival_does_not_starve_worker_result(self):
        node = self.node
        node.async_arrival_mode = node.task_event_mode = True
        node.task_epoch, node.planning_epoch = 8, 7
        node.discarded_replans = 0
        node.replan_requested = True
        node.robots = {}
        node.planning_future = future(('snapshot',))
        node.accept_plan = Mock()
        node.completed, node.latest_map = False, None
        node.control_timer()
        node.accept_plan.assert_called_once_with('snapshot')
        self.assertTrue(node.replan_requested)
        self.assertEqual(node.discarded_replans, 0)

    def test_failed_handoff_keeps_both_old_routes_and_old_localization_is_not_reused(self):
        snapshot, ground, starts, coverage, plan, _ = self.completion_fixture()
        node = self.node
        node.async_arrival_mode = node.task_event_mode = True
        node.task_epoch = node.planning_epoch = 0
        node.speeds = (1., 1.)
        for robot in node.robots.values():
            robot.route = [self.task]
        node.get_robot_pose = Mock(side_effect=RuntimeError('TF unavailable'))
        with self.assertRaises(RuntimeError):
            node.accept_plan(snapshot, ground, starts, coverage, plan, 1.)
        self.assertTrue(all(r.route == [self.task] for r in node.robots.values()))

    def test_old_empty_plan_does_not_complete_a_map_that_changed_during_computation(self):
        snapshot, ground, starts, coverage, plan, _ = self.completion_fixture()
        node = self.node
        node.async_arrival_mode = node.task_event_mode = True
        node.task_epoch = node.planning_epoch = 0
        node.speeds = (1., 1.)
        node.voxels.state.ravel()[1] = UNKNOWN
        node.voxels.revision += 1
        for now in [10., 20.]:
            node.now_sec = lambda: now
            node.accept_plan(snapshot, ground, starts, coverage, plan, 1.)
        self.assertFalse(node.completed)
        node.completed_pub.publish.assert_not_called()

    def test_handoff_commits_both_routes_before_cancelling_and_waits_for_terminal(self):
        _, ground, starts, _, _, _ = self.completion_fixture()
        node = self.node
        node.async_arrival_mode = node.task_event_mode = True
        node.task_epoch, node.planning_epoch = 2, 1
        node.speeds = (1., 1.)
        node.lidar360 = Lidar360.load()
        node.voxels.state[:] = UNKNOWN
        active = node.robots['robot1']
        active.current_task, active.state = self.task, 'NAVIGATING'
        active.current_handle, active.cancel_reason, active.sequence = Mock(), '', 8
        coverage = Coverage([Candidate((0, i), (i+.5, .5), frozenset([i])) for i in (0, 2)],
                            {0, 2}, {1}, set(), 2, 2, False)
        plan = RoutePlan([[0], [1]], (1., 1.), 1., 'test', [[], []])
        node.dispatch_task = Mock()
        node.accept_plan(node.voxels.snapshot(), ground, starts, coverage, plan, .7)
        self.assertEqual(active.state, 'CANCELLING')
        self.assertEqual(node.robots['robot2'].route[0].key, ('v', 0, 2))
        self.assertEqual(node.accepted_task_epoch, 1)
        active.current_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(node.dispatch_task.call_count, 1)  # peer only
        node.on_goal_result(future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), active, 8)
        node.dispatch_task.assert_called_with(active, active.route[0])
        self.assertEqual(node.task_epoch, 2)  # cancellation is not another arrival


if __name__ == '__main__':
    unittest.main()
