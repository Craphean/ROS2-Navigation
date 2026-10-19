#!/usr/bin/env python3
"""Save a periodic PointCloud2 topic to an ASCII PCD file.

Typical use after exploration:
  python3 scripts/save_cloud_pcd.py \
    --topic /octomap_point_cloud_centers \
    --output maps/tb3_3d_map.pcd

No Open3D/PCL dependency; only numpy and sensor_msgs_py are required.
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudSaver(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("save_cloud_pcd")
        # octomap_server uses latch=false to avoid serializing the full tree
        # on every update; its point-cloud topic is published periodically
        # while a subscriber is present.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.message = None
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.callback, qos
        )
        self.get_logger().info(f"Waiting for a cloud on {topic} ...")

    def callback(self, msg: PointCloud2) -> None:
        self.message = msg


def parse_points(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)


def write_pcd(path: str, points: np.ndarray) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    count = len(points)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z\n")
        handle.write("SIZE 4 4 4\n")
        handle.write("TYPE F F F\n")
        handle.write("COUNT 1 1 1\n")
        handle.write(f"WIDTH {count}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {count}\n")
        handle.write("DATA ascii\n")
        for x, y, z in points:
            handle.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic", default="/octomap_point_cloud_centers", help="PointCloud2 topic"
    )
    parser.add_argument("--output", required=True, help="Output .pcd path")
    parser.add_argument(
        "--timeout", type=float, default=10.0, help="Seconds to wait for a message"
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = CloudSaver(args.topic)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.message is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                break
        if node.message is None:
            node.get_logger().error(
                f"No cloud received on {args.topic} within {args.timeout:.1f}s"
            )
            return 1

        points = parse_points(node.message)
        write_pcd(args.output, points)
        node.get_logger().info(
            f"Wrote {len(points)} points to {args.output} "
            f"(frame_id={node.message.header.frame_id})"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
