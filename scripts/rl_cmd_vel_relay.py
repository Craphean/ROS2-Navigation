#!/usr/bin/env python3
"""Relay TwistStamped from cmd_vel_smoothed to cmd_vel for RL bringup."""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class CmdVelRelay(Node):
  def __init__(self) -> None:
    super().__init__('rl_cmd_vel_relay')
    qos = QoSProfile(
      history=HistoryPolicy.KEEP_LAST,
      depth=10,
      reliability=ReliabilityPolicy.RELIABLE,
      durability=DurabilityPolicy.VOLATILE,
    )
    self._pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)
    self.create_subscription(TwistStamped, 'cmd_vel_smoothed', self._cb, qos)

  def _cb(self, msg: TwistStamped) -> None:
    self._pub.publish(msg)


def main() -> None:
  rclpy.init()
  node = CmdVelRelay()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
