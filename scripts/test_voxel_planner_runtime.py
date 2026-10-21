#!/usr/bin/env python3
"""ROS adapter lifecycle tests without a DDS graph or running simulator."""
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from lidar360_observation import Lidar360

from voxel_ros_transport import ViewTask
from voxel_ros_transport import quaternion_to_yaw
from voxel_exploration_core import (
    FREE, OCCUPIED, UNKNOWN, Candidate, CompletionMonitor, Coverage, GroundGrid, RoutePlan, SensorModel, VoxelMap,
)
from voxel_exploration_planner import VoxelExplorationPlanner


def future(value):
    f = Future()
    f.set_result(value)
    return f


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(VoxelExplorationPlanner)
        self.node.voxels = VoxelMap([0, 3, 0, 1, 0, 1], 1)
        self.node.now_sec = lambda: 10.
        self.node.get_logger = lambda: Mock()
        self.node.failed_cells = {}
        self.node.failed_view_cooldown = 30.
        self.node.accepted_observations = 0
        self.node.rejected_observations = 0
        self.node.replan_requested = False
        self.node.replan_not_before = 0.
        self.node.completion = CompletionMonitor(8.)
        self.node.lidar360 = None
        self.task = ViewTask(("v", 0, 1, 0), 1.5, .5, .5, 0., frozenset([0, 1]))
        self.robot = SimpleNamespace(
            namespace="robot1", state="NAVIGATING", current_task=self.task,
            current_handle=None, sequence=4, cancel_reason="", arrival_time=0.,
            observation_verify_time=0., goal_start_time=0., task_deadline=180.)

    def test_stale_result_cannot_finish_replacement_goal(self):
        self.node.on_goal_result(
            future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), self.robot, 3)
        self.assertEqual(self.robot.state, "NAVIGATING")
        self.assertFalse(self.node.replan_requested)

    def arrival_fixture(self):
        node = self.node
        node.arrival_cycle_mode = node.task_event_mode = True
        node.task_epoch = node.accepted_task_epoch = 0
        node.cycle_phase, node.cycle_stop_time = "EXECUTING", None
        self.robot.route = [self.task]
        peer = SimpleNamespace(namespace="robot2", state="NAVIGATING", current_task=self.task,
                               current_handle=Mock(), sequence=9, cancel_reason="", route=[self.task])
        node.robots = {"robot1": self.robot, "robot2": peer}
        node.scan_times = {"robot1": 10., "robot2": 10.}
        return peer

    def test_arrival_mode_keeps_goal_when_all_requirements_are_observed_en_route(self):
        self.arrival_fixture()
        self.node.voxels.state[:] = FREE
        self.node.refresh_execution(self.robot)
        self.assertEqual(self.robot.state, "NAVIGATING")
        self.assertEqual(self.robot.route, [self.task])
        self.assertEqual(self.node.task_epoch, 0)
        self.assertFalse(self.node.replan_requested)

    def test_arrival_stops_peer_and_requires_post_stop_scans_from_both_robots(self):
        peer = self.arrival_fixture()
        node = self.node
        node.on_goal_result(future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), self.robot, 4)
        self.assertEqual(node.task_epoch, 1)
        self.assertEqual(self.robot.state, "IDLE")
        self.assertEqual(peer.state, "CANCELLING")
        self.assertEqual(peer.cancel_reason, "cycle_pause")
        peer.current_handle.cancel_goal_async.assert_called_once()
        self.assertFalse(node.arrival_scan_ready(10.))
        self.assertFalse(node.plan_dispatch_ready())
        node.on_goal_result(future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), peer, 9)
        self.assertEqual(node.task_epoch, 1)
        self.assertEqual(node.failed_cells, {})
        self.assertEqual(node.rejected_observations, 0)
        self.assertFalse(node.arrival_scan_ready(10.2))
        self.assertEqual(node.cycle_phase, "SCANNING")
        node.scan_times["robot1"] = 10.3
        self.assertFalse(node.arrival_scan_ready(10.3))
        node.scan_times["robot2"] = 10.3
        self.assertTrue(node.arrival_scan_ready(10.3))
        self.assertFalse(node.plan_dispatch_ready())

    def test_initial_arrival_cycle_requires_both_initial_scans(self):
        peer = self.arrival_fixture()
        for robot in self.node.robots.values():
            robot.state, robot.current_task, robot.current_handle = "IDLE", None, None
        self.node.cycle_phase, self.node.cycle_stop_time = "SCANNING", 0.
        self.node.scan_times = {"robot1": .1}
        self.assertFalse(self.node.arrival_scan_ready(.1))
        self.node.scan_times["robot2"] = .2
        self.assertTrue(self.node.arrival_scan_ready(.2))

    def test_control_submits_only_after_peer_stops_and_both_new_scans_are_integrated(self):
        peer = self.arrival_fixture()
        node = self.node
        node.planning_future, node.completed, node.latest_map = None, False, object()
        node.worker = Mock()
        node.worker.submit.return_value = Future()
        node.request_serial, node.started_at, node.last_replan_time = 0, 0., 0.
        node.idle_replan_period = 5.
        node.get_parameter = lambda _name: SimpleNamespace(value=15.)
        node.make_ground = lambda: GroundGrid(np.zeros((1, 3), dtype=np.int8), 1., clearance=0.)
        node.get_robot_pose = lambda robot: (.5 if robot.namespace == "robot1" else 2.5, .5, 0.)
        node.on_goal_result(future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), self.robot, 4)
        node.control_timer()
        node.worker.submit.assert_not_called()
        node.now_sec = lambda: 10.1
        node.on_goal_result(future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), peer, 9)
        node.control_timer()
        node.worker.submit.assert_not_called()
        node.voxels.state.ravel()[0] = FREE
        node.voxels.revision = 17
        node.now_sec = lambda: 10.2
        node.scan_times = {"robot1": 10.2, "robot2": 10.2}
        node.control_timer()
        node.worker.submit.assert_called_once()
        snapshot = node.worker.submit.call_args.args[1]
        self.assertEqual(snapshot.revision, 17)
        self.assertEqual(snapshot.state.ravel()[0], FREE)
        self.assertEqual(node.cycle_phase, "PLANNING")
        self.assertFalse(node.plan_dispatch_ready())

    def test_arrival_cycle_commits_both_new_routes_before_dispatch(self):
        _, ground, starts, _, _, _ = self.completion_fixture()
        node = self.node
        node.lidar360 = Lidar360.load()
        node.arrival_cycle_mode = node.task_event_mode = True
        node.task_epoch, node.accepted_task_epoch = 1, 0
        node.cycle_phase = "PLANNING"
        node.voxels.state[:] = UNKNOWN
        coverage = Coverage([Candidate((0, i), (i+.5, .5), frozenset([i])) for i in (0, 2)],
                            {0, 2}, {1}, set(), 2, 2, False, "nearest")
        plan = RoutePlan([[0], [1]], (1., 1.), 1., "two", [[], []])
        dispatched = []
        def dispatch(robot, task):
            self.assertTrue(node.plan_dispatch_ready())
            self.assertTrue(all(r.route for r in node.robots.values()))
            dispatched.append(robot.namespace)
        node.dispatch_task = dispatch
        node.accept_plan(node.voxels.snapshot(), ground, starts, coverage, plan, .1)
        self.assertEqual(dispatched, ["robot1", "robot2"])
        self.assertEqual(node.cycle_phase, "EXECUTING")

    def test_arrival_requires_scan_and_only_then_replans(self):
        self.node.observation_dwell = 1.
        self.node.on_goal_result(
            future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)), self.robot, 4)
        self.assertEqual(self.robot.state, "OBSERVING")
        self.assertEqual(self.robot.arrival_time, 10.)
        self.assertEqual(self.robot.observation_verify_time, 11.)
        self.assertFalse(self.node.replan_requested)
        self.node.voxels.state.ravel()[0] = FREE
        self.node.verify_observation(self.robot)
        self.assertTrue(self.node.replan_requested)
        self.assertEqual(self.node.accepted_observations, 1)
        self.assertIn(1, self.node.voxels.unknown)  # partial progress never erases work

    def test_missing_post_arrival_scan_keeps_observing(self):
        self.robot.state = "OBSERVING"
        self.robot.arrival_time = 8.
        self.robot.observation_verify_time = 9.
        self.node.scan_times = {"robot1": 7.}
        self.node.robots = {"robot1": self.robot}
        self.node.planning_future = None
        self.node.completed = False
        self.node.latest_map = None
        self.node.get_parameter = lambda name: SimpleNamespace(value=15.)
        self.node.control_timer()
        self.assertEqual(self.robot.state, "OBSERVING")
        self.assertEqual(self.node.accepted_observations, 0)
        self.node.scan_times["robot1"] = 9.
        self.node.voxels.state.ravel()[0] = FREE
        self.node.control_timer()
        self.assertEqual(self.robot.state, "IDLE")
        self.assertTrue(self.node.replan_requested)

    def test_cancel_while_send_pending_waits_for_terminal_result(self):
        self.robot.state = "SENDING"
        self.node.cancel_goal(self.robot, "replanned")
        self.assertEqual(self.robot.state, "CANCELLING")
        handle = Mock()
        handle.accepted = True
        handle.get_result_async.return_value = Future()
        self.node.on_goal_response(future(handle), self.robot, 4)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.robot.state, "CANCELLING")
        self.assertIs(self.robot.current_task, self.task)
        self.node.on_goal_result(
            future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), self.robot, 4)
        self.assertEqual(self.robot.state, "IDLE")
        self.assertTrue(self.node.replan_requested)

    def test_failure_cools_viewpoint_without_retiring_voxels(self):
        before = set(self.node.voxels.unknown)
        self.node.fail_task(self.robot)
        self.assertEqual(set(self.node.voxels.unknown), before)
        self.assertEqual(self.node.failed_cells[(0, 1)], 40.)
        self.assertEqual(self.node.rejected_observations, 1)

    def test_observed_in_transit_cancels_once_without_failure_or_early_replacement(self):
        self.robot.route = [self.task]
        self.robot.current_handle = Mock()
        self.node.voxels.state.ravel()[:2] = [FREE, OCCUPIED]
        before = self.node.voxels.state.copy()
        self.node.refresh_execution(self.robot)
        self.node.refresh_execution(self.robot)
        self.assertEqual(self.robot.state, "CANCELLING")
        self.assertEqual(self.robot.cancel_reason, "observed_en_route")
        self.assertEqual(self.robot.route, [])
        self.robot.current_handle.cancel_goal_async.assert_called_once()
        self.assertIs(self.robot.current_task, self.task)
        self.node.on_goal_result(
            future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), self.robot, 4)
        self.assertEqual(self.robot.state, "IDLE")
        self.assertEqual(self.node.failed_cells, {})
        self.assertEqual(self.node.rejected_observations, 0)
        np.testing.assert_array_equal(self.node.voxels.state, before)

    def test_partial_progress_retains_active_visit_and_prunes_known_queued_visit(self):
        known_task = ViewTask(("v", 0, 0, 0), .5, .5, .5, 0., frozenset([0]))
        self.robot.route = [self.task, known_task]
        self.node.voxels.state.ravel()[0] = FREE
        self.node.refresh_execution(self.robot)
        self.assertEqual(self.robot.route, [self.task])
        self.assertEqual(self.robot.state, "NAVIGATING")
        self.assertEqual(self.robot.cancel_reason, "")

    def test_stale_rejection_cannot_reset_current_goal(self):
        self.node.on_goal_response(future(SimpleNamespace(accepted=False)), self.robot, 3)
        self.assertEqual(self.robot.state, "NAVIGATING")
        self.assertIs(self.robot.current_task, self.task)

    def test_fine_voxel_display_is_packed_and_preserves_unknown_state(self):
        from builtin_interfaces.msg import Time
        self.node.voxels.state.ravel()[:2] = [FREE, OCCUPIED]
        self.node.world_frame = "world"
        self.node.robots = {}
        self.node.voxel_pub = Mock()
        self.node.marker_pub = Mock()
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        self.node.get_parameter = lambda name: SimpleNamespace(
            value=False if name == "voxel_display_all" else 1.2)
        coverage = SimpleNamespace(actionable={2}, deferred=set(), excluded=set(), candidates=[])
        plan = SimpleNamespace(routes=[[], []], paths=[[], []])
        self.node.publish_visualization(self.node.voxels, coverage, plan, [])
        cloud = self.node.voxel_pub.publish.call_args.args[0]
        self.assertEqual(cloud.point_step, 16)
        self.assertEqual(cloud.width, 2)
        self.assertEqual(len(cloud.data), 32)
        self.assertIn(2, self.node.voxels.unknown)

    def completion_fixture(self):
        node = self.node
        node.voxels.state[:] = FREE
        ground = GroundGrid(np.zeros((1, 3), dtype=np.int8), 1., clearance=0.)
        node.make_ground = lambda: ground
        node.get_robot_pose = lambda _robot: (.5, .5, 0.)
        node.sensor = SensorModel()
        node.robots = {
            name: SimpleNamespace(namespace=name, state="IDLE", current_task=None,
                                  current_handle=None, route=[])
            for name in ("robot1", "robot2")}
        node.scan_times = {"robot1": 10., "robot2": 10.}
        node.replan_count, node.completed = 0, False
        node.stats_pub, node.completed_pub, node.decision_pub = Mock(), Mock(), Mock()
        node.publish_visualization = Mock()
        node.get_parameter = lambda name: SimpleNamespace(value="" if name == "snapshot_output" else 15.)
        coverage = Coverage([], set(), set(), set(), 0, 0)
        plan = RoutePlan([[], []], (0., 0.), 0., "empty", [[], []])
        return node.voxels.snapshot(), ground, [], coverage, plan, 0.

    def test_ros_completion_needs_fresh_scans_idle_actions_and_stability(self):
        args = self.completion_fixture()
        node = self.node
        node.accept_plan(*args)
        node.completed_pub.publish.assert_not_called()
        node.now_sec = lambda: 18.
        node.scan_times = {"robot1": 18., "robot2": 18.}
        node.robots["robot1"].state = "CANCELLING"
        node.accept_plan(*args)
        self.assertFalse(node.completed)
        node.robots["robot1"].state = "IDLE"
        node.accept_plan(*args)
        node.now_sec = lambda: 26.
        node.scan_times = {"robot1": 26., "robot2": 26.}
        node.accept_plan(*args)
        self.assertTrue(node.completed)
        node.completed_pub.publish.assert_called_once()

    def test_ros_changed_ground_or_voxels_invalidate_empty_old_plan(self):
        args = self.completion_fixture()
        node = self.node
        node.accept_plan(*args)
        node.voxels.revision += 1
        node.now_sec = lambda: 20.
        node.scan_times = {"robot1": 20., "robot2": 20.}
        node.accept_plan(*args)
        self.assertFalse(node.completed)
        args = (node.voxels.snapshot(), *args[1:])
        node.accept_plan(*args)
        changed = GroundGrid(np.array([[0, 1, 0]], dtype=np.int8), 1., clearance=0.)
        node.make_ground = lambda: changed
        node.now_sec = lambda: 30.
        node.scan_times = {"robot1": 30., "robot2": 30.}
        node.accept_plan(*args)
        self.assertFalse(node.completed)
        self.assertIsNone(node.completion.since)

    def test_ros_sensor_silence_is_not_completion(self):
        args = self.completion_fixture()
        node = self.node
        node.accept_plan(*args)
        node.now_sec = lambda: 40.
        node.accept_plan(*args)  # The last scans are still from t=10.
        self.assertFalse(node.completed)
        node.completed_pub.publish.assert_not_called()

    def test_360_task_key_contains_position_without_angle(self):
        _, ground, starts, _, _, _ = self.completion_fixture()
        node = self.node
        node.lidar360 = Lidar360.load()
        node.voxels.state[0, 0, 1] = UNKNOWN
        node.dispatch_task = Mock()
        coverage = Coverage([Candidate((0, 1), (1.5, .5), frozenset([1]))], {1}, set(), set(), 1, 1)
        plan = RoutePlan([[0], []], (1., 0.), 1., "one", [[], []])
        node.accept_plan(node.voxels.snapshot(), ground, starts, coverage, plan, 0.)
        task = node.robots["robot1"].route[0]
        self.assertEqual(task.key, ("v", 0, 1))

    def test_delayed_ros_plan_keeps_active_goal_instead_of_cancelling_back_to_spawn(self):
        _, ground, starts, _, _, _ = self.completion_fixture()
        node = self.node
        node.lidar360 = Lidar360.load()
        node.voxels.state[:] = UNKNOWN
        robot = node.robots["robot1"]
        robot.current_task = ViewTask(("v", 0, 2), 2.5, .5, .22, 0., frozenset([2]))
        robot.state = "NAVIGATING"
        node.cancel_goal = Mock()
        coverage = Coverage([Candidate((0, i), (i+.5, .5), frozenset([i]))
                             for i in (0, 2)], {0, 2}, {1}, set(), 2, 2)
        plan = RoutePlan([[0, 1], []], (1., 0.), 1., "two", [[], []])
        node.accept_plan(node.voxels.snapshot(), ground, starts, coverage, plan, 0.)
        node.cancel_goal.assert_not_called()
        self.assertEqual([t.key for t in robot.route], [("v", 0, 2), ("v", 0, 0)])

    def test_next_valid_visit_can_run_while_joint_replan_is_pending(self):
        node = self.node
        self.robot.state, self.robot.current_task = "IDLE", None
        self.robot.route = [self.task]
        node.robots = {"robot1": self.robot}
        node.planning_future = Future()
        node.completed, node.latest_map = False, None
        node.dispatch_task = Mock()
        node.control_timer()
        node.dispatch_task.assert_called_once_with(self.robot, self.task)

    def test_task_event_invalidates_queued_tours_and_rejects_late_worker_result(self):
        node = self.node
        node.task_event_mode = True
        node.task_epoch = node.accepted_task_epoch = node.planning_epoch = 7
        node.discarded_replans = 0
        node.robots = {"robot1": self.robot}
        self.robot.route = [self.task]
        node.voxels.state.ravel()[0] = FREE
        node.verify_observation(self.robot)
        self.assertEqual(node.task_epoch, 8)
        self.assertEqual(self.robot.route, [])
        self.assertFalse(node.plan_dispatch_ready())
        node.planning_future = future(("old snapshot",))
        node.accept_plan = Mock()
        node.completed, node.latest_map = False, None
        node.control_timer()
        node.accept_plan.assert_not_called()
        self.assertEqual(node.discarded_replans, 1)
        self.assertTrue(node.replan_requested)
        self.assertEqual(node.replan_not_before, 10.)

    def test_event_wait_keeps_only_other_robots_active_goal(self):
        node = self.node
        node.task_event_mode = True
        node.task_epoch = node.accepted_task_epoch = 0
        queued = ViewTask(("v", 0, 2), 2.5, .5, .5, 0., frozenset([2]))
        self.robot.route = [self.task, queued]
        idle = SimpleNamespace(state="IDLE", current_task=None, route=[queued])
        node.robots = {"robot1": self.robot, "robot2": idle}
        node.request_task_replan()
        self.assertEqual(self.robot.route, [self.task])
        self.assertEqual(idle.route, [])
        self.assertEqual(self.robot.state, "NAVIGATING")
        # Even an accidentally retained stale task cannot be sent to Nav2.
        idle.route = [queued]
        node.dispatch_task(idle, queued)
        self.assertEqual(idle.state, "IDLE")

    def test_observed_en_route_event_is_not_repeated_while_cancelling(self):
        node = self.node
        node.task_event_mode = True
        node.task_epoch = node.accepted_task_epoch = 0
        node.robots = {"robot1": self.robot}
        self.robot.route = [self.task]
        node.voxels.state.ravel()[:2] = FREE
        node.refresh_execution(self.robot)
        node.refresh_execution(self.robot)
        self.assertEqual(node.task_epoch, 1)
        self.assertEqual(self.robot.state, "CANCELLING")
        node.on_goal_result(future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED)), self.robot, 4)
        self.assertEqual(node.task_epoch, 2)  # New plan must use the actual stopping pose.
        self.assertFalse(node.plan_dispatch_ready())

    def test_latest_event_plan_uses_new_joint_order_and_opens_dispatch_gate(self):
        _, ground, starts, _, _, _ = self.completion_fixture()
        node = self.node
        node.lidar360 = Lidar360.load()
        node.task_event_mode = True
        node.task_epoch, node.accepted_task_epoch = 3, 2
        node.voxels.state[:] = UNKNOWN
        node.dispatch_task = Mock()
        coverage = Coverage([Candidate((0, i), (i+.5, .5), frozenset([i]))
                             for i in (0, 2)], {0, 2}, {1}, set(), 2, 2, False, "nearest")
        plan = RoutePlan([[0, 1], []], (1., 0.), 1., "two", [[], []])
        node.get_robot_pose = lambda _robot: (2.5, .5, 0.)
        node.accept_plan(node.voxels.snapshot(), ground, starts, coverage, plan, 0.)
        self.assertTrue(node.plan_dispatch_ready())
        robot = node.robots["robot1"]
        self.assertEqual([t.key for t in robot.route], [("v", 0, 0), ("v", 0, 2)])
        node.dispatch_task.assert_called_once_with(robot, robot.route[0])

    def test_task_event_clears_displayed_future_routes_before_worker_finishes(self):
        _, ground, _, _, _, _ = self.completion_fixture()
        node = self.node
        node.task_event_mode, node.task_epoch = True, 0
        node.ground, node.world_frame = ground, "world"
        node.marker_pub, node.voxel_pub = Mock(), Mock()
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10)))
        for robot in node.robots.values():
            robot.route = [self.task]
            robot.route_pub = Mock()
        active = node.robots["robot1"]
        active.state, active.current_task = "NAVIGATING", self.task
        node.request_task_replan()
        self.assertGreater(len(active.route_pub.publish.call_args.args[0].poses), 0)
        self.assertEqual(node.robots["robot2"].route_pub.publish.call_args.args[0].poses, [])
        node.marker_pub.publish.assert_called_once()
        node.voxel_pub.publish.assert_not_called()

    def test_route_visualization_uses_live_pose_and_execution_order(self):
        snapshot, ground, _, coverage, plan, _ = self.completion_fixture()
        node = self.node
        node.world_frame, node.ground = "world", ground
        node.voxel_pub, node.marker_pub = Mock(), Mock()
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
        node.get_parameter = lambda name: SimpleNamespace(
            value=False if name == "voxel_display_all" else 1.2)
        node.get_robot_pose = lambda _robot: (1.5, .5, 0.)
        for robot in node.robots.values():
            robot.route_pub = Mock()
        robot = node.robots["robot1"]
        robot.route = [ViewTask(("v", 0, col), col+.5, .5, .22, 0., frozenset([col]))
                       for col in (2, 0)]
        plan.paths[0] = [(.5, .5), (2.5, .5)]  # Obsolete snapshot order.
        VoxelExplorationPlanner.publish_visualization(node, snapshot, coverage, plan, [])
        poses = robot.route_pub.publish.call_args.args[0].poses
        self.assertEqual(poses[0].pose.position.x, 1.5)
        self.assertEqual(poses[-1].pose.position.x, .5)
        self.assertTrue(any(p.pose.position.x == 2.5 for p in poses))

    def test_360_dispatch_uses_approach_tangent_instead_of_fixed_observation_yaw(self):
        self.completion_fixture()
        node = self.node
        node.lidar360 = Lidar360.load()
        node.dispatch_goals = True
        node.world_frame = "world"
        node.task_timeout, node.observation_dwell, node.speeds = 180., 1., (.22, .22)
        node.get_robot_pose = lambda _r: (.5, .5, .123)
        node.make_ground = lambda: GroundGrid(np.zeros((3, 3), dtype=np.int8), 1., clearance=0.)
        node.world_to_robot_map = lambda _r, x, y: (x, y, 0.)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10)))
        robot = node.robots["robot1"]
        robot.action_client, robot.goal_pub, robot.sequence = Mock(), Mock(), 0
        robot.action_client.server_is_ready.return_value = True
        robot.action_client.send_goal_async.return_value = Future()
        task = ViewTask(("v", 2, 0), .5, 2.5, .22, 0., frozenset([1]))
        node.dispatch_task(robot, task)
        goal = robot.action_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(quaternion_to_yaw(goal.pose.pose.orientation), np.pi/2)


if __name__ == "__main__":
    unittest.main()
