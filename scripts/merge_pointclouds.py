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
    -p voxel_size:=0.05 \
    -p publish_rate:=3.0
"""

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
        self.voxel_size = float(self.declare_parameter("voxel_size", 0.05).value)
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 3.0).value
        )
        self.min_z = float(self.declare_parameter("min_z", 0.05).value)
        self.accumulate = bool(self.declare_parameter("accumulate", True).value)
        self.accumulated_topic = str(
            self.declare_parameter("accumulated_topic", "/map_cloud").value
        )
        self.accumulated_rate = float(
            self.declare_parameter("accumulated_rate", 1.0).value
        )
        self.accumulated_voxel_size = float(
            self.declare_parameter("accumulated_voxel_size", 0.05).value
        )
        self.max_accumulated_points = int(
            self.declare_parameter("max_accumulated_points", 250000).value
        )
        self.max_z = float(self.declare_parameter("max_z", 2.0).value)
        self.min_range = float(self.declare_parameter("min_range", 0.1).value)
        self.max_range = float(self.declare_parameter("max_range", 5.0).value)
        self.tf_timeout = float(self.declare_parameter("tf_timeout", 0.1).value)

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
            self.voxel_size = 0.05

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest = {robot: None for robot in self.robots}
        self.accumulated = {}
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
        self.accumulated_publisher = None
        self.accumulated_timer = None
        if self.accumulate:
            self.accumulated_publisher = self.create_publisher(
                PointCloud2, self.accumulated_topic, 2
            )
            self.accumulated_timer = self.create_timer(
                1.0 / max(0.1, self.accumulated_rate),
                self.publish_accumulated,
            )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_merged)

        self.get_logger().info(
            f"Merging {self.robots} into {self.world_frame}; "
            f"voxel={self.voxel_size:.3f} m, rate={self.publish_rate:.1f} Hz, "
            f"box_filter=[x: ({self.footprint_x_min}, {self.footprint_x_max}), "
            f"y: ({self.footprint_y_min}, {self.footprint_y_max}), "
            f"z<={self.footprint_z_max}], accumulate={self.accumulate}"
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
        ranges = np.linalg.norm(points, axis=1)

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
            & (ranges < self.max_range - 1e-4)  # Finite max-range samples are misses.
            & (~is_self_body)
        )
        points = points[valid]
        if points.size == 0:
            return None, source_frame

        stamp = msg.header.stamp
        has_stamp = bool(stamp.sec or stamp.nanosec)
        lookup_stamp = Time.from_msg(stamp) if has_stamp else Time()
        # Prefer the cloud timestamp, but fall back to the latest available
        # transform.  GZ cloud stamps can be a few milliseconds ahead of /
        # behind the newest bridged TF sample, which otherwise drops frames
        # with "extrapolation into the future/past".
        lookup_stamps = [lookup_stamp]
        if has_stamp:
            lookup_stamps.append(Time(clock_type=lookup_stamp.clock_type))
        transform = None
        last_error = None
        for candidate in lookup_stamps:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    source_frame,
                    candidate,
                    timeout=Duration(seconds=self.tf_timeout),
                )
                break
            except TransformException as exc:
                last_error = exc
        if transform is None:
            self.get_logger().warning(
                f"TF {self.world_frame} -> {source_frame} unavailable: {last_error}",
                throttle_duration_sec=5.0,
            )
            return None, source_frame

        t = transform.transform.translation
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

        if self.accumulate:
            keys = np.floor(points / self.accumulated_voxel_size).astype(np.int64)
            for key, point in zip(keys, points):
                self.accumulated[(int(key[0]), int(key[1]), int(key[2]))] = point
            if len(self.accumulated) > self.max_accumulated_points:
                # Keep memory bounded; this only affects very large scenes.
                items = list(self.accumulated.items())[-self.max_accumulated_points :]
                self.accumulated = dict(items)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.world_frame
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)

        self.get_logger().debug(
            f"Published {len(points)} points in {self.world_frame}"
        )


    def publish_accumulated(self) -> None:
        if not self.accumulate or self.accumulated_publisher is None:
            return
        if not self.accumulated:
            return
        points = np.asarray(
            list(self.accumulated.values()), dtype=np.float64
        ).reshape(-1, 3)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.world_frame
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.accumulated_publisher.publish(cloud)
        self.get_logger().debug(
            f"Published {len(points)} accumulated points on "
            f"{self.accumulated_topic}"
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
