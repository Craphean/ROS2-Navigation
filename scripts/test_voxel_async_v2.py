"""V2 commitment, early refill, ROS lifecycle and incremental-map correctness."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

import test_voxel_planner_runtime as runtime_tests
from voxel_async_execution import AsyncV2Policy, AsyncV2Window, path_tail, queued_path
from voxel_exploration_core import FREE, OCCUPIED, UNKNOWN, GroundGrid, VoxelMap
from voxel_frontier_policy import FrontierPolicy, prepare_targets
from voxel_incremental_frontier import IncrementalFrontier


class IncrementalTargetsTests(unittest.TestCase):
    def check_reference(self, cache, voxels, policy):
        before = voxels.state.copy()
        ignored, support = cache.update(voxels, policy)
        expected, reference = prepare_targets(voxels, policy)
        self.assertEqual(ignored, expected)
        np.testing.assert_allclose(support, reference, atol=8e-7, rtol=0)
        np.testing.assert_array_equal(voxels.state, before)
        retained = voxels.state == UNKNOWN
        retained.ravel()[list(ignored)] = False
        near = np.stack([cv2.dilate(layer.astype(np.uint8), np.ones((3, 3), np.uint8))
                         for layer in (voxels.state == FREE)])
        adjacent = near.copy()
        adjacent[1:] |= near[:-1]
        adjacent[:-1] |= near[1:]
        np.testing.assert_array_equal(cache.boundary, retained & adjacent.astype(bool))

    def test_random_deletions_splits_new_unknowns_and_occupied_changes(self):
        rng = np.random.default_rng(831)
        for threshold in (1, 5, 40, 100):
            voxels = VoxelMap([0, 1.2, 0, 1.1, 0, 1], .1)
            voxels.state[:] = rng.choice([FREE, UNKNOWN], voxels.state.shape, p=[.65, .35])
            policy = FrontierPolicy(min_unknown_component_volume=threshold*.001, frontier_support_radius=.2)
            cache = IncrementalFrontier()
            for step in range(24):
                if step == 12:
                    voxels.state[2:5, 2:5, 2:5] = UNKNOWN  # Merge requires a full fallback.
                else:
                    ids = rng.choice(voxels.state.size, 45, replace=False)
                    voxels.state.ravel()[ids] = FREE if step%2 else OCCUPIED
                self.check_reference(cache, voxels, policy)
                if step == 12:
                    self.assertEqual(cache.stats['target_update'], 'full_components')
            self.check_reference(cache, voxels, replace(policy, frontier_support_radius=.3))

    def test_thin_bridge_split_diagonal_connectivity_and_exact_threshold(self):
        voxels = VoxelMap([0, 1.2, 0, 1.2, 0, 1.2], .1)
        voxels.state[:] = FREE
        voxels.state[2, 2, 1:8] = UNKNOWN
        voxels.state[3, 3, 8] = UNKNOWN  # Diagonal connects the last cell.
        policy = FrontierPolicy(min_unknown_component_volume=.005, frontier_support_radius=.1)
        cache = IncrementalFrontier()
        self.check_reference(cache, voxels, policy)
        self.assertEqual(len(cache.ignored), 0)
        voxels.state[2, 2, 3] = FREE  # Split into 2 cells and exactly 5 cells.
        self.check_reference(cache, voxels, policy)
        self.assertEqual(len(cache.ignored), 2)
        voxels.state[2, 2, 4] = FREE  # Five becomes four: all remaining cells ignored.
        self.check_reference(cache, voxels, policy)
        self.assertEqual(len(cache.ignored), 6)

    def test_local_support_halo_at_roi_edges_and_no_compiler_fallback(self):
        voxels = VoxelMap([0, 2, 0, 2, 0, 2], .1)
        policy = FrontierPolicy(frontier_support_radius=.2)
        cache = IncrementalFrontier()
        self.check_reference(cache, voxels, policy)
        for cell in ((0, 0, 0), (10, 10, 10), (19, 19, 19)):
            voxels.state[cell] = FREE
            self.check_reference(cache, voxels, policy)
            self.assertLess(cache.stats['support_updated_voxels'], voxels.state.size)
        voxels.state[2, 2, 2] = OCCUPIED
        with patch('voxel_incremental_frontier.library', return_value=None):
            self.check_reference(cache, voxels, policy)
        self.assertEqual(cache.stats['target_update'], 'full_components')
        self.check_reference(cache, voxels, policy)
        self.assertEqual(cache.stats['target_update'], 'unchanged')


class ExecutionWindowTests(unittest.TestCase):
    def test_prediction_follows_bends_and_never_moves_past_queue(self):
        window = AsyncV2Window()
        paths = [[(0., 0.), (1., 0.), (1., 3.)], [(3., 0.), (3., 1.)]]
        predicted, detail = window.begin(10., paths, (1., .5))
        self.assertEqual(detail['commit_until'], 11.5)
        np.testing.assert_allclose(predicted, [(1., .5), (3., .75)])
        window.observe_latency(8.)
        predicted, detail = window.begin(20., paths, (1., .5))
        self.assertEqual(predicted, [(1., 3.), (3., 1.)])
        self.assertEqual(detail['commit_until'], 21.5)  # No unbounded lock.
        self.assertEqual(path_tail(paths[0], 1.5), [(1., .5), (1., 3.)])

    def test_empty_queue_cannot_be_held_by_peer_commitment(self):
        window = AsyncV2Window()
        _, detail = window.begin(4., [[(0., 0.)], [(1., 0.), (9., 0.)]], (1., 1.))
        self.assertEqual(detail['commit_until'], 4.)

    def test_early_refill_is_edge_triggered_and_rate_limited(self):
        window = AsyncV2Window()
        self.assertFalse(window.needs_refill(0., [10., 10.]))
        self.assertTrue(window.needs_refill(1., [1.8, 10.]))
        self.assertFalse(window.needs_refill(1.1, [1.7, 1.8]))
        self.assertFalse(window.needs_refill(2., [0., 10.]))
        self.assertTrue(window.needs_refill(3., [0., 1.8]))
        self.assertFalse(window.needs_refill(8., [0., 0.]))
        self.assertFalse(window.needs_refill(9., [5., 5.]))
        self.assertTrue(window.needs_refill(10., [1., 5.]))
        window.observe_latency(3.)
        self.assertGreater(window.refill_threshold, 3.)

    def test_prediction_stops_at_unreachable_ground(self):
        data = np.zeros((3, 8), np.int8)
        data[:, 3] = OCCUPIED
        ground = GroundGrid(data, 1., clearance=0.)
        path = queued_path(ground, (.5, .5), [(2.5, .5), (6.5, .5)])
        self.assertEqual(path[-1], (2.5, .5))

    def test_idle_robot_retries_deferred_work_without_peer_arrival(self):
        window = AsyncV2Window()
        self.assertFalse(window.needs_idle_search(4.9, [30., 0.], True, 0.))
        self.assertTrue(window.needs_idle_search(5., [30., 0.], True, 0.))
        self.assertFalse(window.needs_idle_search(5.1, [30., 0.], True, 0.))
        self.assertFalse(window.needs_idle_search(10., [30., 0.], True, 8.))
        self.assertTrue(window.needs_idle_search(13., [30., 0.], True, 8.))
        self.assertFalse(window.needs_idle_search(20., [0., 0.], True, 0.))
        self.assertFalse(window.needs_idle_search(20., [30., 0.], False, 0.))
        self.assertFalse(window.needs_idle_search(20., [30., 30.], True, 0.))


class V2RosTests(unittest.TestCase):
    setUp = runtime_tests.RuntimeTests.setUp

    def fixture(self):
        node = self.node
        node.async_arrival_mode = node.task_event_mode = True
        node.async_v2 = AsyncV2Window()
        node.async_v2.commit_until = 11.5
        node.v2_next_check = 20.
        node.v2_latency_recorded = False
        node.task_epoch, node.planning_epoch = 8, 7
        node.replan_requested = True
        self.robot.route = [self.task]
        node.robots = {'robot1': self.robot}
        node.planning_future = runtime_tests.future(('snapshot', .2))
        node.accept_plan = Mock()
        node.completed, node.latest_map = False, None
        return node

    def test_ready_result_waits_for_commitment_without_cancelling_old_goal(self):
        node = self.fixture()
        node.control_timer()
        node.accept_plan.assert_not_called()
        self.assertEqual(self.robot.state, 'NAVIGATING')
        node.now_sec = lambda: 11.5
        node.control_timer()
        node.accept_plan.assert_called_once_with('snapshot', .2)
        self.assertEqual(len(node.async_v2.latencies), 1)
        self.assertTrue(node.replan_requested)  # Arrival during commitment coalesces.

    def test_empty_buffer_breaks_commitment(self):
        node = self.fixture()
        self.robot.route = []
        node.control_timer()
        node.accept_plan.assert_called_once()

    def test_new_obstacle_breaks_commitment_and_cancels_unsafe_route(self):
        node = self.fixture()
        node.latest_map = object()
        node.v2_next_check = 0.
        node.speeds = (1.,)
        node.get_robot_pose = lambda _: (.5, .5, 0.)
        ground = GroundGrid(np.array([[FREE, OCCUPIED, FREE]], np.int8), 1., clearance=0.)
        node.make_ground = lambda: ground
        self.robot.current_handle = Mock()
        node.check_v2_buffer(10.)
        self.assertEqual(node.async_v2.commit_until, 10.)
        self.assertFalse(self.robot.route)
        self.robot.current_handle.cancel_goal_async.assert_called_once()

    def test_ros_idle_search_keeps_peer_moving_and_does_not_invent_arrival(self):
        node = self.fixture()
        node.latest_map, node.v2_next_check = object(), 0.
        node.planning_future, node.replan_requested = None, False
        node.latest_result = (None, None, None, SimpleNamespace(deferred={1}))
        node.last_replan_time = 0.
        node.speeds = (.1, .1)
        node.get_robot_pose = lambda _: (.5, .5, 0.)
        ground = GroundGrid(np.zeros((1, 3), np.int8), 1., clearance=0.)
        node.make_ground = lambda: ground
        self.robot.current_handle = Mock()
        peer = SimpleNamespace(route=[], state='IDLE', current_task=None, current_handle=None)
        node.robots['robot2'] = peer
        node.check_v2_buffer(10.)
        self.assertTrue(node.replan_requested)
        self.assertEqual(node.task_epoch, 8)
        self.assertEqual(self.robot.state, 'NAVIGATING')
        self.robot.current_handle.cancel_goal_async.assert_not_called()
        self.assertEqual(node.async_v2.idle_search_count, 1)


if __name__ == '__main__':
    unittest.main()
