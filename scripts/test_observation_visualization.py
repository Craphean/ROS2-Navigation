"""Evidence and coordinate checks for the current-observation view."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

from voxel_replay_io import PlanarObservations, ray_returns
from observation_visualization import MapImage, ObservedPoints, ObservationRenderer, remaining_path
from record_voxel_visualization import ObservationView
from voxel_exploration_core import FREE, OCCUPIED, UNKNOWN, SensorModel, VoxelMap
from voxel_exploration_planner import VoxelExplorationPlanner


class ObservedViewTests(unittest.TestCase):
    def test_planar_map_keeps_unseen_cells_unknown_behind_wall(self):
        truth = VoxelMap([0, 3, -1, 1, 0, 1], .05)
        truth.state[:] = FREE
        truth.state[:, :, 30] = OCCUPIED
        planar = PlanarObservations(truth, SensorModel(offset_x=0))
        self.assertTrue(np.all(planar.image().data == UNKNOWN))
        planar.observe(truth, (.5, 0.), 3.)
        grid = planar.image()
        self.assertEqual(grid.stamp, 3.)
        self.assertEqual(grid.data[20, 20], 0)
        self.assertEqual(grid.data[20, 30], 100)
        self.assertTrue(np.all(grid.data[:, 31:] == UNKNOWN))

    def test_parallel_rays_on_voxel_face_produce_finite_surface_returns(self):
        truth = VoxelMap([0, 3, 0, 1, 0, 1], .05)
        truth.state[:] = FREE
        truth.state[:, :, 30] = OCCUPIED
        endpoint, hit = ray_returns(truth, [.5, .5, .25], np.array([[2., 0., 0.]]))
        self.assertTrue(hit[0])
        self.assertTrue(np.isfinite(endpoint).all())
        self.assertAlmostEqual(endpoint[0, 0], 1.5, places=6)

    def test_accumulation_preserves_measured_coordinates_and_scan_timestamps(self):
        cloud = ObservedPoints(.05)
        cloud.add(0, [[.011, .012, .013], [.02, .02, .02], [np.nan, 0, 0]], 1.)
        cloud.add(1, [[1., 2., 3.], [np.inf, 0, 0]], 2.)
        self.assertEqual(len(cloud.points), 2)
        self.assertEqual(len(cloud.current), 3)
        self.assertEqual(cloud.stamps, {0: 1., 1: 2.})
        self.assertTrue(any(np.allclose(p, [.011, .012, .013]) for p in cloud.points))
        # Re-scanning replaces the latest points, but preserves observed history.
        cloud.add(0, [], 3.)
        self.assertEqual(len(cloud.current), 1)
        self.assertEqual(len(cloud.points), 2)

    def test_map_rotation_and_unknown_outside_observed_grid(self):
        renderer = ObservationRenderer([-1, 1, -1, 1, 0, 1])
        grid = MapImage(np.array([[100, 0], [-1, 0]], dtype=np.int8), .5, (0, 0, np.pi/2), 2.)
        data = renderer.measured_map(grid)
        # A cell at local (.25,.25) rotates to world (-.25,.25).
        self.assertEqual(data[207, 276], 100)
        self.assertEqual(data[400, 500], UNKNOWN)

    def test_remaining_route_clips_motion_without_inventing_connection(self):
        path = remaining_path([(0, 0), (1, 0), (1, 1)], (.5, .2))
        np.testing.assert_allclose(path, [[.5, 0], [1, 0], [1, 1]])

    def test_pose_publication_uses_tf_time_during_planning_and_never_spawn(self):
        node = object.__new__(VoxelExplorationPlanner)
        node.world_frame = "world"
        node.planning_future = Mock()
        node.pose_pubs = {"robot1": Mock(), "robot2": Mock()}
        transform = TransformStamped()
        transform.header.stamp = Time(sec=7)
        transform.transform.translation.x = 2.
        transform.transform.rotation.w = 1.
        node.tf_buffer = Mock()
        node.tf_buffer.lookup_transform.side_effect = [
            transform, TransformException("missing"), TransformException("missing")]
        node.publish_current_poses()
        msg = node.pose_pubs["robot1"].publish.call_args.args[0]
        self.assertEqual(msg.header.stamp.sec, 7)
        self.assertEqual(msg.pose.position.x, 2.)
        node.pose_pubs["robot2"].publish.assert_not_called()

    def test_live_view_drops_stale_pose_instead_of_using_route_start(self):
        node = object.__new__(ObservationView)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
        node.poses, node.pose_stamps = [(1., 2., 0.), (3., 4., 0.)], [8., 9.5]
        node.routes = [[(5., 5.), (6., 6.)], []]
        node.route_stamps = [4., 4.]
        node.goals = [None, None]
        node.stats = {}
        node.grid = None
        node.cloud = ObservedPoints()
        node.accumulated, node.accumulated_stamp = None, None
        renderer = Mock()
        _, timing = node.frame(renderer)
        self.assertEqual(timing["poses"], [None, (3., 4., 0.)])
        self.assertEqual(renderer.render.call_args.args[2], timing["poses"])

    def test_measured_cloud_packs_real_xyz_with_acquisition_stamp(self):
        node = object.__new__(VoxelExplorationPlanner)
        node.world_frame = "world"
        points = np.array([[.123, .456, .789]])
        msg = node.measured_message(points, Time(sec=42))
        self.assertEqual(msg.header.stamp.sec, 42)
        self.assertEqual(msg.width, 1)
        self.assertEqual(msg.point_step, 12)
        np.testing.assert_allclose(np.frombuffer(msg.data, dtype="<f4").reshape(-1, 3), points)


if __name__ == "__main__":
    unittest.main()
