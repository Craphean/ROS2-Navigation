#!/usr/bin/env python3
"""Merge the two robots' 3D LiDAR clouds into one world-frame cloud.

The multi-robot TF tree published by multirobot_map_merge uses prefixed
frames (robot1/base_scan, robot2/base_scan), while the point clouds coming
from Gazebo carry the unprefixed sensor frame (base_scan).  This node
normalizes the frame id, looks up world -> robotX/base_scan, transforms the
points, voxel-downsamples them, and publishes /points_merged for
octomap_server.

Usage:
  python3 scripts/merge_pointclouds.py --ros-args \
    -p use_sim_time:=true \
    -p robots:="['robot1', 'robot2']" \
    -p world_frame:=world \
    -p voxel_size:=0.02 \
    -p publish_rate:=3.0
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from cloud_stamp import pose_is_discontinuous


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a 3x3 rotation matrix from a quaternion."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


class PointCloudMerger(Node):
    def __init__(self) -> None:
        super().__init__("merge_pointclouds")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self.robots = list(
            self.declare_parameter("robots", ["robot1", "robot2"]).value
        )
        self.world_frame = str(
            self.declare_parameter("world_frame", "world").value
        )
        self.voxel_size = float(self.declare_parameter("voxel_size", 0.02).value)
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 3.0).value
        )
        self.min_z = float(self.declare_parameter("min_z", 0.05).value)
        # 3.0 m keeps ceilings and the top of tall walls for the PCD archive
        # (a house interior is routinely 2.4-2.8 m).  octomap_server applies
        # its own 2.0 m clip from config/octomap_params.yaml, so navigation and
        # the coordinator's voxel index are unaffected by this wider range.
        self.max_z = float(self.declare_parameter("max_z", 3.0).value)
        self.min_range = float(self.declare_parameter("min_range", 0.1).value)
        self.max_range = float(self.declare_parameter("max_range", 12.0).value)
        self.tf_timeout = float(self.declare_parameter("tf_timeout", 0.1).value)
        # Latest-TF fallback is only safe for a few milliseconds of stamp skew.
        # A stalled bridge otherwise paints the old scan at the new pose.
        self.max_tf_skew = float(self.declare_parameter("max_tf_skew", 0.15).value)
        self._accepted_pose: dict[str, tuple[float, float, float]] = {}

        # Vehicle self-reflection box filter in sensor frame (base_scan)
        # TurtleBot3 Waffle footprint: length 0.265m, width 0.265m, wheels at y=+-0.144m.
        # With base_scan at x=-0.064m in base_link, the chassis extends:
        #   x in [-0.14, +0.20]m, y in [-0.16, +0.16]m.
        # Points inside this bounding box and below the sensor (z <= footprint_z_max)
        # are robot self-reflections and should be rejected.
        self.footprint_x_min = float(
            self.declare_parameter("footprint_x_min", -0.22).value
        )
        self.footprint_x_max = float(
            self.declare_parameter("footprint_x_max", 0.22).value
        )
        self.footprint_y_min = float(
            self.declare_parameter("footprint_y_min", -0.18).value
        )
        self.footprint_y_max = float(
            self.declare_parameter("footprint_y_max", 0.18).value
        )
        self.footprint_z_max = float(
            self.declare_parameter("footprint_z_max", 0.05).value
        )

        if self.publish_rate <= 0.0:
            self.publish_rate = 3.0
        if self.voxel_size <= 0.0:
            self.voxel_size = 0.02

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest = {robot: None for robot in self.robots}
        self.cloud_subscriptions = []
        for robot in self.robots:
            topic = f"/{robot}/points"
            self.cloud_subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, r=robot: self.cloud_callback(r, msg),
                    qos_profile_sensor_data,
                )
            )
            self.get_logger().info(f"Subscribing to {topic}")

        self.publisher = self.create_publisher(PointCloud2, "/points_merged", 5)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_merged)

        self.get_logger().info(
            f"Merging {self.robots} into {self.world_frame}; "
            f"voxel={self.voxel_size:.3f} m, rate={self.publish_rate:.1f} Hz, "
            f"box_filter=[x: ({self.footprint_x_min}, {self.footprint_x_max}), "
            f"y: ({self.footprint_y_min}, {self.footprint_y_max}), "
            f"z<={self.footprint_z_max}]"
        )

    def cloud_callback(self, robot: str, msg: PointCloud2) -> None:
        self.latest[robot] = msg

    def _normalized_frame(self, robot: str, frame: str) -> str:
        frame = frame.lstrip("/")
        if frame == self.world_frame:
            return frame
        if frame.startswith(robot + "/"):
            return frame
        return f"{robot}/{frame}"

    def _transform_points(
        self, robot: str, msg: PointCloud2
    ) -> tuple[np.ndarray | None, str]:
        source_frame = self._normalized_frame(robot, msg.header.frame_id)

        try:
            points = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        except Exception as exc:  # noqa: BLE001 - log and skip malformed cloud
            self.get_logger().warning(
                f"{robot}: failed to read point cloud: {exc}", throttle_duration_sec=5.0
            )
            return None, source_frame

        if points.size == 0:
            return None, source_frame
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)

        # Range/crop in the sensor frame before the transform.
        ranges = np.linalg.norm(points[:, :2], axis=1)

        # Reject self-reflections on robot's own chassis / camera / wheels
        is_self_body = (
            (points[:, 0] >= self.footprint_x_min)
            & (points[:, 0] <= self.footprint_x_max)
            & (points[:, 1] >= self.footprint_y_min)
            & (points[:, 1] <= self.footprint_y_max)
            & (points[:, 2] <= self.footprint_z_max)
        )

        valid = (
            np.isfinite(points).all(axis=1)
            & (ranges >= self.min_range)
            & (ranges <= self.max_range)
            & (~is_self_body)
        )
        points = points[valid]
        if points.size == 0:
            return None, source_frame

        stamp = msg.header.stamp
        has_stamp = bool(stamp.sec or stamp.nanosec)
        lookup_stamp = Time.from_msg(stamp) if has_stamp else Time()
        transform = None
        last_error = None
        used_latest = False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                source_frame,
                lookup_stamp,
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            last_error = exc
        if transform is None and has_stamp:
            # GZ stamps are often a few milliseconds off the newest TF sample.
            # Fall back only when that sample is still the same instant.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    source_frame,
                    Time(clock_type=lookup_stamp.clock_type),
                    timeout=Duration(seconds=0.0),
                )
                used_latest = True
            except TransformException as exc:
                last_error = exc
        if transform is None:
            self.get_logger().warning(
                f"TF {self.world_frame} -> {source_frame} unavailable: {last_error}",
                throttle_duration_sec=5.0,
            )
            return None, source_frame

        cloud_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if used_latest and has_stamp:
            tf_stamp = transform.header.stamp
            tf_sec = float(tf_stamp.sec) + float(tf_stamp.nanosec) * 1e-9
            skew = abs(tf_sec - cloud_sec)
            if skew > self.max_tf_skew:
                self.get_logger().warning(
                    f"{robot}: dropping cloud, TF is {skew:.2f}s from the scan "
                    f"(clock stall). Inserting it would ghost the walls.",
                    throttle_duration_sec=2.0,
                )
                return None, source_frame

        t = transform.transform.translation
        if has_stamp:
            prev = self._accepted_pose.get(robot)
            if prev is not None:
                dist = float(math.hypot(t.x - prev[1], t.y - prev[2]))
                if pose_is_discontinuous(cloud_sec - prev[0], dist):
                    self.get_logger().warning(
                        f"{robot}: dropping cloud, pose jumped {dist:.2f} m "
                        f"in {cloud_sec - prev[0]:.2f}s",
                        throttle_duration_sec=2.0,
                    )
                    return None, source_frame
            self._accepted_pose[robot] = (cloud_sec, float(t.x), float(t.y))

        q = transform.transform.rotation
        rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
        points = points @ rotation.T + np.array([t.x, t.y, t.z])

        # Crop in the world frame so the floor and ceiling/outliers are removed.
        valid = (
            np.isfinite(points).all(axis=1)
            & (points[:, 2] >= self.min_z)
            & (points[:, 2] <= self.max_z)
        )
        points = points[valid]
        return points, source_frame

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        return points[indices]

    def publish_merged(self) -> None:
        merged = []
        for robot in self.robots:
            msg = self.latest.get(robot)
            if msg is None:
                continue
            points, _ = self._transform_points(robot, msg)
            if points is not None and points.size:
                merged.append(points)

        if not merged:
            return

        points = self._voxel_downsample(np.vstack(merged))
        if points.size == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.world_frame
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)

        self.get_logger().debug(
            f"Published {len(points)} points in {self.world_frame}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
