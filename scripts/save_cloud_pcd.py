#!/usr/bin/env python3
"""Save a periodic PointCloud2 topic to an ASCII PCD file.

Snapshot (a single frame):
  python3 scripts/save_cloud_pcd.py \
    --topic /octomap_point_cloud_centers \
    --output maps/tb3_3d_map.pcd

Whole-run archive from the raw merged LiDAR cloud.  /points_merged is published
per frame by merge_pointclouds.py, so it has to be accumulated; the output is
rewritten every --checkpoint-sec seconds so a killed run still leaves a usable
file.  Memory grows with unique voxels, not with runtime:
  python3 scripts/save_cloud_pcd.py \
    --topic /points_merged \
    --output maps/tb3_3d_raw.pcd \
    --accumulate 3600 --voxel-size 0.05 --binary

No Open3D/PCL dependency; only numpy and sensor_msgs_py are required.
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudSaver(Node):
    """Collects either one snapshot frame or an accumulated voxel set.

    ``/points_merged`` from merge_pointclouds.py is a *per-frame* merge of the
    two robots' latest 3D LiDAR scans, not a growing map: a single message only
    covers what both robots see at that instant.  Archiving a whole run from it
    therefore has to accumulate frames, and accumulating raw messages does not
    scale (~2 MB x 3 Hz = 21 GB/h).  In accumulate mode every frame is folded
    into a voxel hash instead, so memory is bounded by the number of unique
    voxels in the explored area rather than by runtime.
    """

    def __init__(
        self,
        topic: str,
        accumulate_sec: float = 0.0,
        voxel_size: float = 0.0,
    ) -> None:
        super().__init__("save_cloud_pcd")
        # octomap_server uses latch=false to avoid serializing the full tree
        # on every update; its point-cloud topic is published periodically
        # while a subscriber is present.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT if ("points" in topic and topic.startswith("/robot")) else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.message = None
        self.accumulate_sec = accumulate_sec
        self.voxel_size = voxel_size
        self.frames = 0
        self._voxels: dict = {}
        self._loose: list = []
        self._last_report = time.monotonic()
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.callback, qos
        )
        self.get_logger().info(f"Waiting for a cloud on {topic} ...")

    def callback(self, msg: PointCloud2) -> None:
        self.message = msg
        self.frames += 1
        if self.accumulate_sec <= 0.0:
            return

        points = parse_points(msg)
        if points.size == 0:
            return

        if self.voxel_size > 0.0:
            keys = np.floor(points / self.voxel_size).astype(np.int64)
            # Deduplicate inside the frame first so the Python-level dict only
            # sees one insert per occupied voxel.
            uniq_keys, first_idx = np.unique(
                keys, axis=0, return_index=True
            )
            reps = points[first_idx]
            store = self._voxels
            for key, point in zip(map(tuple, uniq_keys.tolist()), reps.tolist()):
                store[key] = point
        else:
            self._loose.append(points)

        now = time.monotonic()
        if now - self._last_report >= 10.0:
            self._last_report = now
            self.get_logger().info(
                f"Accumulating: {self.frames} frames, "
                f"{self.accumulated_count()} voxels"
            )

    def accumulated_count(self) -> int:
        return len(self._voxels) + sum(len(p) for p in self._loose)

    def collected_points(self) -> np.ndarray:
        """Accumulated points as an (N, 3) float32 array."""
        chunks = []
        if self._voxels:
            chunks.append(
                np.asarray(list(self._voxels.values()), dtype=np.float32)
            )
        if self._loose:
            chunks.append(np.vstack(self._loose).astype(np.float32))
        if not chunks:
            return np.empty((0, 3), dtype=np.float32)
        points = np.vstack(chunks)
        if self.voxel_size > 0.0 and self._loose:
            # Mixed mode (voxel size changed mid-run) - normalise at the end.
            points = voxel_downsample(points, self.voxel_size)
        return points


def parse_points(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.size == 0 or voxel_size <= 0.0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[indices]


def write_pcd(path: str, points: np.ndarray, binary: bool = False) -> None:
    """Write an (N, 3) float32 point array as PCD 0.7 (ascii or binary).

    Binary is ~2x smaller and ~50x faster to write, which matters once an
    accumulated archive reaches millions of points.
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    points = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 3)
    count = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        f"DATA {'binary' if binary else 'ascii'}\n"
    )
    if binary:
        with open(path, "wb") as handle:
            handle.write(header.encode("ascii"))
            handle.write(points.tobytes())
        return
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(header)
        for x, y, z in points:
            handle.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def _safe_log(node: Node, text: str) -> None:
    """Log without dying when the ROS context is already shut down."""
    try:
        node.get_logger().info(text)
    except Exception:
        print(text, file=sys.stderr)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic", default="/octomap_point_cloud_centers", help="PointCloud2 topic"
    )
    parser.add_argument("--output", required=True, help="Output .pcd path")
    parser.add_argument(
        "--timeout", type=float, default=10.0, help="Seconds to wait for a message"
    )
    parser.add_argument(
        "--accumulate",
        type=float,
        default=0.0,
        help="Seconds to accumulate incoming frames into a voxel set "
             "(0 saves a single snapshot).",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Voxel leaf size in meters for accumulation/snapshot downsampling "
             "(defaults to 0.05 when accumulating)",
    )
    parser.add_argument(
        "--binary",
        action="store_true",
        help="Write binary PCD instead of ascii (much faster and ~2x smaller; "
             "recommended for accumulated archives)",
    )
    parser.add_argument(
        "--checkpoint-sec",
        type=float,
        default=60.0,
        help="While accumulating, rewrite the output every N seconds so a "
             "killed run still leaves a usable file (0 disables)",
    )
    args, ros_args = parser.parse_known_args(argv)

    voxel_defaulted = False
    if args.accumulate > 0.0 and args.voxel_size <= 0.0:
        args.voxel_size = 0.05
        voxel_defaulted = True

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = CloudSaver(
        args.topic,
        accumulate_sec=args.accumulate,
        voxel_size=args.voxel_size,
    )
    if voxel_defaulted:
        node.get_logger().info(
            "--voxel-size not given; using 0.05 m for accumulation."
        )
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

        if args.accumulate > 0.0:
            node.get_logger().info(
                f"Accumulating frames from {args.topic} for "
                f"{args.accumulate:.1f}s; checkpointing every "
                f"{args.checkpoint_sec:.0f}s to {args.output} ..."
            )
            accumulate_deadline = time.monotonic() + args.accumulate
            next_checkpoint = time.monotonic() + args.checkpoint_sec
            try:
                while rclpy.ok() and time.monotonic() < accumulate_deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if (
                        args.checkpoint_sec > 0.0
                        and time.monotonic() >= next_checkpoint
                    ):
                        next_checkpoint = time.monotonic() + args.checkpoint_sec
                        snapshot = node.collected_points()
                        if snapshot.size:
                            write_pcd(args.output, snapshot, args.binary)
                            _safe_log(
                                node,
                                f"Checkpoint: {len(snapshot)} points, "
                                f"{node.frames} frames -> {args.output}",
                            )
            except BaseException as exc:  # noqa: BLE001 - keep what we have
                # Ctrl-C on a manual spin_once loop can surface as
                # KeyboardInterrupt, SystemExit or ExternalShutdownException -
                # whatever it is, the accumulated cloud is still worth saving.
                _safe_log(
                    node,
                    f"Stopped ({type(exc).__name__}); saving the points "
                    f"accumulated so far.",
                )

            points = node.collected_points()
            if points.size == 0:
                node.get_logger().error(
                    "No valid points parsed from accumulated frames."
                )
                return 1
        else:
            points = parse_points(node.message)
            if args.voxel_size > 0.0:
                points = voxel_downsample(points, args.voxel_size)

        write_pcd(args.output, points, args.binary)
        frame_id = (
            node.message.header.frame_id if node.message else "world"
        )
        node.get_logger().info(
            f"Wrote {len(points)} points to {args.output} (frame_id={frame_id})"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
