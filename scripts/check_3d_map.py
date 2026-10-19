#!/usr/bin/env python3
"""Headless checks for the dual-robot 3D mapping stack.

Modes:
  cloud    Subscribe to /points_merged and report point count, bbox,
           z histogram and occupied voxel count.
  growth   Sample /octomap_point_cloud_centers over a time window and report
           voxel growth. Used as the 3D completion check.
  costmap  Read /robotX/global_costmap/costmap, transform a world point into
           the costmap frame and check whether its cell is lethal.
  pcd      Read an ASCII PCD file offline and report the same statistics.

All modes print JSON. No RViz / Gazebo GUI is required.

Examples:
  python3 scripts/check_3d_map.py cloud --topic /points_merged --duration 10 --json
  python3 scripts/check_3d_map.py growth --window 30 --json
  python3 scripts/check_3d_map.py costmap --robot robot1 \
    --world-point 1.2,-0.3 --expect-lethal --json
  python3 scripts/check_3d_map.py pcd --input maps/tb3_3d_map.pcd --json
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

VOXEL_SIZE = 0.05
Z_BINS = (0.05, 0.3, 0.6, 1.2, 2.0)


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
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


def points_from_msg(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)


def cloud_stats(points: np.ndarray) -> dict:
    if points.size == 0:
        return {
            "points": 0,
            "bbox": None,
            "z_histogram": {f"{Z_BINS[i]:.2f}-{Z_BINS[i + 1]:.2f}": 0
                            for i in range(len(Z_BINS) - 1)},
            "voxels": 0,
        }

    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    if points.size == 0:
        return cloud_stats(np.empty((0, 3), dtype=np.float32))

    bbox = {
        "min": [float(v) for v in points.min(axis=0)],
        "max": [float(v) for v in points.max(axis=0)],
    }
    histogram = {}
    for i in range(len(Z_BINS) - 1):
        lo, hi = Z_BINS[i], Z_BINS[i + 1]
        histogram[f"{lo:.2f}-{hi:.2f}"] = int(
            np.count_nonzero((points[:, 2] >= lo) & (points[:, 2] < hi))
        )
    keys = np.floor(points / VOXEL_SIZE).astype(np.int64)
    voxels = int(np.unique(keys, axis=0).shape[0])
    return {
        "points": int(len(points)),
        "bbox": bbox,
        "z_histogram": histogram,
        "voxels": voxels,
    }


def read_ascii_pcd(path: str) -> np.ndarray:
    header = {}
    with open(path, "r", encoding="utf-8") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("PCD ended before DATA line")
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            key, _, value = stripped.partition(" ")
            header[key.upper()] = value
            if key.upper() == "DATA":
                break
        if header.get("DATA", "").lower() != "ascii":
            raise ValueError("Only ASCII PCD is supported")
        data = np.loadtxt(handle, dtype=np.float64)
    if data.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(data, dtype=np.float32).reshape(-1, 3)


def emit(result: dict, as_json: bool) -> None:
    if as_json:
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))


def init_ros(ros_args) -> None:
    rclpy.init(args=[sys.argv[0], *ros_args])


def run_cloud(args, ros_args) -> int:
    topic = args.topic or "/points_merged"
    init_ros(ros_args)
    node = Node("check_3d_cloud")
    collected = []

    def callback(msg: PointCloud2) -> None:
        collected.append(points_from_msg(msg))

    node.create_subscription(PointCloud2, topic, callback, qos_profile_sensor_data)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    points = (
        np.vstack(collected)
        if collected
        else np.empty((0, 3), dtype=np.float32)
    )
    result = {"mode": "cloud", "topic": topic, **cloud_stats(points)}
    emit(result, args.json)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["points"] > 0 else 1


def run_growth(args, ros_args) -> int:
    topic = args.topic or "/octomap_point_cloud_centers"
    init_ros(ros_args)
    node = Node("check_3d_growth")
    latest = {"msg": None}
    # octomap_server is configured with latch=false so it only publishes
    # while there are subscribers; use a volatile reliable subscription.
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    def callback(msg: PointCloud2) -> None:
        latest["msg"] = msg

    node.create_subscription(PointCloud2, topic, callback, qos)

    # Wait for the first message before starting the growth window, so DDS
    # discovery latency does not eat into the 30 s measurement.
    wait_start = time.monotonic()
    while (
        rclpy.ok()
        and latest["msg"] is None
        and (time.monotonic() - wait_start) <= args.timeout
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    if latest["msg"] is None:
        result = {
            "mode": "growth",
            "topic": topic,
            "pass": False,
            "error": "no point cloud received",
        }
        emit(result, args.json)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    start = time.monotonic()
    next_sample = start
    samples = []
    while rclpy.ok() and (time.monotonic() - start) <= args.window:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now >= next_sample and latest["msg"] is not None:
            stats = cloud_stats(points_from_msg(latest["msg"]))
            samples.append(
                {
                    "t": round(now - start, 2),
                    "voxels": stats["voxels"],
                }
            )
            next_sample = now + max(args.interval, 0.1)

    if not samples:
        result = {
            "mode": "growth",
            "topic": topic,
            "pass": False,
            "error": "no point cloud received",
        }
        emit(result, args.json)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    first = samples[0]["voxels"]
    last = samples[-1]["voxels"]
    growth = 0.0 if first == 0 else (last - first) / first * 100.0
    growth_voxels = last - first
    # OctoMap's occupied-voxel count fluctuates by a few voxels even on a
    # static, fully observed scene (tree pruning / sensor noise).  A pure
    # percentage threshold would reject a small but saturated map, so accept
    # a small absolute change as well.  In a house-sized map the percentage
    # threshold dominates.
    converged = growth <= args.max_growth or growth_voxels <= args.max_growth_voxels
    result = {
        "mode": "growth",
        "topic": topic,
        "window": args.window,
        "samples": samples,
        "first_voxels": first,
        "last_voxels": last,
        "growth_voxels": growth_voxels,
        "growth_percent": round(growth, 3),
        "max_growth_percent": args.max_growth,
        "max_growth_voxels": args.max_growth_voxels,
        "pass": first > 0 and converged,
    }
    emit(result, args.json)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["pass"] else 1


def lookup_transform_spinning(
    node: Node,
    buffer: Buffer,
    target_frame: str,
    source_frame: str,
    timeout: float,
):
    """Look up TF while spinning the node so /tf_static callbacks are processed."""
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            return buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            last_error = exc
            rclpy.spin_once(node, timeout_sec=0.05)
    raise TransformException(str(last_error))


def run_costmap(args, ros_args) -> int:
    init_ros(ros_args)
    node = Node("check_3d_costmap")
    latest = {"msg": None}
    # Nav2 costmap publishers are periodic; reliable volatile is compatible.
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    topic = args.topic or f"/{args.robot}/global_costmap/costmap"

    def callback(msg: OccupancyGrid) -> None:
        latest["msg"] = msg

    node.create_subscription(OccupancyGrid, topic, callback, qos)
    buffer = Buffer(cache_time=Duration(seconds=30.0))
    TransformListener(buffer, node)

    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and latest["msg"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if latest["msg"] is None:
        result = {
            "mode": "costmap",
            "topic": topic,
            "pass": False,
            "error": "no costmap received",
        }
        emit(result, args.json)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = latest["msg"]
    costmap_frame = msg.header.frame_id or "map"
    # Nav2 runs inside /robotX with an unprefixed local map frame (`map`).
    # In the global TF tree that same frame is named `robotX/map`, published
    # by multirobot_map_merge together with the known initial pose.  Looking
    # up plain `map` would hit map_merge's identity world->map static frame
    # and give the wrong coordinates.
    if costmap_frame == args.world_frame:
        tf_target = costmap_frame
    elif costmap_frame.startswith(args.robot + "/"):
        tf_target = costmap_frame
    else:
        tf_target = f"{args.robot}/{costmap_frame}"
    wx, wy = args.world_point
    world_point = np.array([wx, wy, 0.0], dtype=np.float64)

    try:
        if tf_target == args.world_frame:
            transformed = world_point
        else:
            tf = lookup_transform_spinning(
                node,
                buffer,
                tf_target,
                args.world_frame,
                args.timeout,
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
            transformed = rotation @ world_point + np.array([t.x, t.y, t.z])
    except TransformException as exc:
        result = {
            "mode": "costmap",
            "topic": topic,
            "world_point": [wx, wy],
            "costmap_frame": costmap_frame,
            "tf_target": tf_target,
            "pass": False,
            "error": f"TF failed: {exc}",
        }
        emit(result, args.json)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    origin = msg.info.origin
    q = origin.orientation
    origin_rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
    local = origin_rotation.T @ (
        transformed - np.array([origin.position.x, origin.position.y, origin.position.z])
    )
    resolution = msg.info.resolution
    # Add a tiny epsilon so a point exactly on a cell boundary does not fall
    # into the previous cell because of floating-point round-off.
    mx = int(math.floor(local[0] / resolution + 1e-6))
    my = int(math.floor(local[1] / resolution + 1e-6))
    width = msg.info.width
    height = msg.info.height
    in_bounds = 0 <= mx < width and 0 <= my < height

    # Small tolerance for TF/origin round-off; the cell itself is reported,
    # and the highest valid cost in the neighbourhood is used for the verdict.
    best_cell = None
    best_cost = None
    for dy in range(-args.cell_tolerance, args.cell_tolerance + 1):
        for dx in range(-args.cell_tolerance, args.cell_tolerance + 1):
            cx, cy = mx + dx, my + dy
            if not (0 <= cx < width and 0 <= cy < height):
                continue
            value = int(msg.data[cy * width + cx]) & 0xFF
            if value == 255:  # unknown is not an obstacle
                continue
            if best_cost is None or value > best_cost:
                best_cost = value
                best_cell = [cx, cy]
    lethal = best_cost is not None and best_cost >= args.lethal_threshold

    result = {
        "mode": "costmap",
        "topic": topic,
        "robot": args.robot,
        "world_point": [wx, wy],
        "costmap_frame": costmap_frame,
        "tf_target": tf_target,
        "cell": [mx, my],
        "best_cell": best_cell,
        "in_bounds": in_bounds,
        "cost": best_cost,
        "cell_tolerance": args.cell_tolerance,
        "lethal_threshold": args.lethal_threshold,
        "lethal": lethal,
        "expect_lethal": args.expect_lethal,
        "pass": lethal if args.expect_lethal else True,
    }
    emit(result, args.json)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["pass"] else 1


def run_pcd(args, ros_args) -> int:
    del ros_args
    try:
        points = read_ascii_pcd(args.input)
    except Exception as exc:  # noqa: BLE001 - report the parse error as JSON
        result = {"mode": "pcd", "input": args.input, "error": str(exc)}
        emit(result, args.json)
        return 1
    result = {"mode": "pcd", "input": args.input, **cloud_stats(points)}
    emit(result, args.json)
    return 0 if result["points"] > 0 else 1


def parse_world_point(value: str) -> tuple[float, float]:
    parts = value.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected 'x,y'")
    return float(parts[0]), float(parts[1])


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["cloud", "growth", "costmap", "pcd"])
    parser.add_argument("--json", action="store_true", help="compact JSON output")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--topic", default=None, help="PointCloud2 or costmap topic")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--window", type=float, default=30.0)
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--max-growth", type=float, default=1.0)
    parser.add_argument(
        "--max-growth-voxels",
        type=int,
        default=30,
        help="Absolute voxel change also accepted as converged (small-map noise)",
    )
    parser.add_argument("--robot", default="robot1")
    parser.add_argument("--world-point", type=parse_world_point, default=None)
    parser.add_argument("--expect-lethal", action="store_true")
    parser.add_argument("--lethal-threshold", type=int, default=50)
    parser.add_argument("--cell-tolerance", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--input", default=None, help="ASCII PCD path")
    args, ros_args = parser.parse_known_args(argv)

    if args.mode == "cloud":
        return run_cloud(args, ros_args)
    if args.mode == "growth":
        return run_growth(args, ros_args)
    if args.mode == "costmap":
        if args.world_point is None:
            parser.error("costmap mode requires --world-point x,y")
        return run_costmap(args, ros_args)
    if args.input is None:
        parser.error("pcd mode requires --input path")
    return run_pcd(args, ros_args)


if __name__ == "__main__":
    sys.exit(main())
