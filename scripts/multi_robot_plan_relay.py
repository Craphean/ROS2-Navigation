#!/usr/bin/env python3
"""
Multi-Robot Nav2 Plan & Goal Relay for RViz Visualization.

Subscribes to each robot's local Nav2 plan (/<robot>/plan, header.frame_id='map')
and transforms the waypoints into the global 'world' coordinate frame using
TF (world -> <robot>/map) with an analytical fallback to initial spawn poses.
Publishes transformed paths to /<robot>/plan_world (header.frame_id='world')
so that RViz can display them with zero TF coordinate offset errors.
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotPlanRelay:
    def __init__(self, namespace: str, init_pose: List[float]):
        self.namespace = namespace.strip('/')
        self.init_x = float(init_pose[0])
        self.init_y = float(init_pose[1])
        self.init_yaw = float(init_pose[2])
        self.plan_pub = None
        self.plan_sub = None


class MultiRobotPlanRelay(Node):
    def __init__(self):
        super().__init__('multi_robot_plan_relay')

        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('robot1_init_pose', [-2.0, -0.5, 0.0])
        self.declare_parameter('robot2_init_pose', [0.0, 0.5, 0.0])

        namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        if not namespaces:
            namespaces = ['robot1', 'robot2']

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robots: Dict[str, RobotPlanRelay] = {}
        for ns in namespaces:
            clean_ns = ns.strip('/')
            init_param_name = f'{clean_ns}_init_pose'
            if self.has_parameter(init_param_name):
                init_pose = self.get_parameter(init_param_name).value
            elif clean_ns == 'robot1':
                init_pose = self.get_parameter('robot1_init_pose').value
            elif clean_ns == 'robot2':
                init_pose = self.get_parameter('robot2_init_pose').value
            else:
                init_pose = [0.0, 0.0, 0.0]

            relay = RobotPlanRelay(clean_ns, init_pose)

            # Global transformed plan publisher
            relay.plan_pub = self.create_publisher(
                Path,
                f'/{clean_ns}/plan_world',
                10
            )

            # Local plan subscriber from planner_server
            relay.plan_sub = self.create_subscription(
                Path,
                f'/{clean_ns}/plan',
                lambda msg, r=relay: self.plan_callback(msg, r),
                10
            )

            self.robots[clean_ns] = relay
            self.get_logger().info(
                f'Relaying plan for [{clean_ns}]: /{clean_ns}/plan -> /{clean_ns}/plan_world '
                f'(init_pose=[{relay.init_x:.2f}, {relay.init_y:.2f}, {relay.init_yaw:.2f}])'
            )

    def robot_local_to_world(self, robot: RobotPlanRelay, xl: float, yl: float, yawl: float = 0.0) -> Tuple[float, float, float]:
        """Convert robot local coordinates (xl, yl, yawl) to global 'world' frame."""
        for map_frame in [f'{robot.namespace}/map', 'map']:
            try:
                t = self.tf_buffer.lookup_transform('world', map_frame, Time())
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tyaw = quaternion_to_yaw(t.transform.rotation)
                c = math.cos(tyaw)
                s = math.sin(tyaw)
                xw = tx + xl * c - yl * s
                yw = ty + xl * s + yl * c
                yaww = tyaw + yawl
                return xw, yw, yaww
            except Exception:
                continue

        # Analytical forward transform: P_world = R(init_yaw) * P_local + T_init
        c = math.cos(robot.init_yaw)
        s = math.sin(robot.init_yaw)
        xw = robot.init_x + xl * c - yl * s
        yw = robot.init_y + xl * s + yl * c
        yaww = robot.init_yaw + yawl
        return xw, yw, yaww

    def plan_callback(self, msg: Path, robot: RobotPlanRelay):
        if not msg.poses or robot.plan_pub is None:
            return

        world_path = Path()
        world_path.header.stamp = self.get_clock().now().to_msg()
        world_path.header.frame_id = 'world'

        for p in msg.poses:
            xl = p.pose.position.x
            yl = p.pose.position.y
            yawl = quaternion_to_yaw(p.pose.orientation)
            xw, yw, yaww = self.robot_local_to_world(robot, xl, yl, yawl)

            pw = PoseStamped()
            pw.header.stamp = world_path.header.stamp
            pw.header.frame_id = 'world'
            pw.pose.position.x = xw
            pw.pose.position.y = yw
            pw.pose.position.z = p.pose.position.z
            pw.pose.orientation = yaw_to_quaternion(yaww)
            world_path.poses.append(pw)

        robot.plan_pub.publish(world_path)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotPlanRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

