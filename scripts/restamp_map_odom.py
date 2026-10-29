#!/usr/bin/env python3
"""Republish map -> odom with the current simulation time.

slam_toolbox stamps that transform at the laser scan time. Odometry is often
a few tens of milliseconds ahead of the scan, so the controller looks up
map -> odom at the odom stamp, misses, and aborts the goal with error 102.
The behavior tree then spins, waits and backs up even when the corridor is
empty. Copying the latest slam sample forward to "now" makes that lookup hit.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


class MapOdomRestamp(Node):
    def __init__(self) -> None:
        super().__init__("restamp_map_odom")
        qos = QoSProfile(depth=100)
        self._latest: TFMessage | None = None
        self._pub = self.create_publisher(TFMessage, "tf", qos)
        self.create_subscription(TFMessage, "tf", self._on_tf, qos)
        self.create_timer(0.02, self._publish)
        self.get_logger().info("republishing map -> odom at the current sim time")

    def _on_tf(self, msg: TFMessage) -> None:
        now = self.get_clock().now()
        kept = []
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent != "map" or child != "odom":
                continue
            stamp = rclpy.time.Time.from_msg(transform.header.stamp)
            # Our own republish comes back on /tf a couple of milliseconds
            # old. Slam's sample is the one still stamped at the scan time.
            age = (now - stamp).nanoseconds * 1e-9
            if age < 0.008:
                continue
            kept.append(transform)
        if kept:
            self._latest = kept[-1]

    def _publish(self) -> None:
        if self._latest is None:
            return
        message = TFMessage()
        transform = self._latest
        transform.header.stamp = self.get_clock().now().to_msg()
        message.transforms = [transform]
        self._pub.publish(message)


def main() -> None:
    rclpy.init()
    node = MapOdomRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
