"""Exercise both supported ROS constructors without starting DDS or Gazebo."""
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from voxel_exploration_planner import VoxelExplorationPlanner
from voxel_ros_transport import Node


class TransportTests(unittest.TestCase):
    def test_supported_configs_initialize_complete_planners(self):
        root = Path(__file__).resolve().parents[1]
        for algorithm, filename in (
            ('arrival_cycle', 'voxel360_strong_baseline_params.yaml'),
            ('async_v2', 'voxel360_a1_params.yaml'),
        ):
            with self.subTest(algorithm=algorithm), ExitStack() as stack:
                params = yaml.safe_load((root/'config'/filename).read_text())['rolling_observation_planner']['ros__parameters']
                declared = {}
                def declare(_node, name, default):
                    self.assertNotIn(name, declared, 'Duplicate ROS parameter')
                    declared[name] = params.get(name, default)
                replacements = {
                    '__init__': lambda *_a, **_kw: None,
                    'declare_parameter': declare,
                    'has_parameter': lambda _node, name: name in declared,
                    'get_parameter': lambda _node, name: SimpleNamespace(value=declared[name]),
                    'get_logger': lambda _node: Mock(),
                    'get_clock': lambda _node: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
                    'create_publisher': lambda *_a, **_kw: Mock(),
                    'destroy_publisher': lambda *_a, **_kw: True,
                    'create_subscription': lambda *_a, **_kw: Mock(),
                    'create_timer': lambda *_a, **_kw: Mock(),
                }
                for name, replacement in replacements.items():
                    stack.enter_context(patch.object(Node, name, replacement))
                for name in ('Buffer', 'TransformListener', 'ActionClient'):
                    stack.enter_context(patch('voxel_ros_transport.'+name))
                node = VoxelExplorationPlanner()
                try:
                    self.assertEqual(set(node.robots), {'robot1', 'robot2'})
                    self.assertEqual(node.voxels.resolution, .05)
                    self.assertIsNotNone(node.lidar360)
                    self.assertEqual(node.sensor.max_range, 5.)
                    self.assertEqual(node.arrival_cycle_mode, algorithm == 'arrival_cycle')
                    self.assertEqual(node.async_v2 is not None, algorithm == 'async_v2')
                    self.assertEqual(node.fast_evaluator is not None, algorithm == 'async_v2')
                    self.assertFalse(hasattr(node, 'rl_selector'))
                    for robot in node.robots.values():
                        self.assertIsNotNone(robot.route_pub)
                        self.assertEqual(robot.state, 'IDLE')
                finally:
                    node.worker.shutdown(wait=True)


if __name__ == '__main__':
    unittest.main()
