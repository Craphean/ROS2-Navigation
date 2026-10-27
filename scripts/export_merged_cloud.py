#!/usr/bin/env python3
"""Accumulate a PointCloud2 stream into one high-detail PCD archive.

Unlike ``save_cloud_pcd.py --topic /octomap_point_cloud_centers``, this reads
the *raw* merged cloud, so the archive is not limited by the OctoMap voxel
grid (0.10 m) nor by octomap's sensor-range gate.

It also stops on its own: once no new voxel has appeared for ``--idle-stop``
seconds the exploration is over and the file is written.

Usage:
  python3 scripts/export_merged_cloud.py \
    --topic /points_merged --output maps/merged_full.pcd \
    --voxel-size 0.02 --min-z 0.03 --max-z 3.0 --idle-stop 25
"""

import argparse
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

FIELDS = ("x", "y", "z")


def read_xyz(msg):
    """PointCloud2 -> (N, 3) float32 array."""
    pts = point_cloud2.read_points_numpy(msg, field_names=FIELDS, skip_nans=True)
    pts = np.asarray(pts, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return np.empty((0, 3), dtype=np.float32)
    return pts[:, :3]


def write_pcd(path, points, binary=True):
    """Write an xyz PCD (binary float32 or ascii)."""
    n = int(len(points))
    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {n}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {n}",
        "DATA binary" if binary else "DATA ascii",
    ]
    with open(path, "wb") as fh:
        fh.write(("\n".join(header) + "\n").encode("ascii"))
        if binary:
            fh.write(np.ascontiguousarray(points, dtype=np.float32).tobytes())
        else:
            np.savetxt(fh, points, fmt="%.4f")


def voxel_centers(keys, voxel):
    """Sorted int64 voxel keys -> (N, 3) float32 voxel centres."""
    if not keys:
        return np.empty((0, 3), dtype=np.float32)
    arr = np.fromiter(keys, dtype=np.int64, count=len(keys))
    mask = (1 << 21) - 1
    ix = (arr >> 42) - (1 << 20)
    iy = ((arr >> 21) & mask) - (1 << 20)
    iz = (arr & mask) - (1 << 20)
    out = np.stack([ix, iy, iz], axis=1).astype(np.float32)
    return (out + 0.5) * float(voxel)


def key_of(pts, voxel):
    """(N, 3) points -> int64 keys on the voxel grid."""
    idx = np.floor(pts / voxel).astype(np.int64)
    idx += 1 << 20  # keep positive for the bit packing below
    idx = np.clip(idx, 0, (1 << 21) - 1)
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


class CloudAccumulator(Node):
    def __init__(self, topic, voxel, zmin, zmax):
        super().__init__("export_merged_cloud")
        self.voxel = float(voxel)
        self.zmin = float(zmin)
        self.zmax = float(zmax)
        self.keys = set()
        self.frames = 0
        self.dropped_z = 0
        self.last_growth = time.monotonic()
        self.started = time.monotonic()
        self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"accumulating {topic} at {self.voxel:g} m voxels, "
            f"z in [{self.zmin:g}, {self.zmax:g}]"
        )

    def on_cloud(self, msg):
        pts = read_xyz(msg)
        self.frames += 1
        if pts.size == 0:
            return
        keep = (pts[:, 2] >= self.zmin) & (pts[:, 2] <= self.zmax)
        self.dropped_z += int((~keep).sum())
        pts = pts[keep]
        if pts.size == 0:
            return
        before = len(self.keys)
        self.keys.update(key_of(pts, self.voxel).tolist())
        if len(self.keys) > before:
            self.last_growth = time.monotonic()

    def idle_for(self):
        return time.monotonic() - self.last_growth

    def elapsed(self):
        return time.monotonic() - self.started

    def points(self):
        return voxel_centers(self.keys, self.voxel)


def save(node, path, binary, report_path):
    pts = node.points()
    write_pcd(path, pts, binary=binary)
    if pts.size:
        lo = pts.min(axis=0).tolist()
        hi = pts.max(axis=0).tolist()
    else:
        lo = hi = [0.0, 0.0, 0.0]
    report = {
        "output": path,
        "points": int(len(pts)),
        "voxel_size_m": node.voxel,
        "frames": node.frames,
        "dropped_by_z": node.dropped_z,
        "duration_s": round(node.elapsed(), 1),
        "bounds": {"min": [round(v, 3) for v in lo], "max": [round(v, 3) for v in hi]},
    }
    if report_path:
        with open(report_path, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2)
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/points_merged")
    ap.add_argument("--output", required=True)
    ap.add_argument("--voxel-size", type=float, default=0.02)
    ap.add_argument("--min-z", type=float, default=0.03)
    ap.add_argument("--max-z", type=float, default=3.0)
    ap.add_argument("--idle-stop", type=float, default=25.0,
                    help="stop after N s without a new voxel (0 = never)")
    ap.add_argument("--max-duration", type=float, default=3600.0)
    ap.add_argument("--checkpoint-sec", type=float, default=60.0)
    ap.add_argument("--ascii", action="store_true")
    ap.add_argument("--report", default="")
    args = ap.parse_args(argv)

    rclpy.init()
    node = CloudAccumulator(args.topic, args.voxel_size, args.min_z, args.max_z)
    next_ckpt = time.monotonic() + args.checkpoint_sec
    last_log = time.monotonic()
    reason = "max-duration"
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if args.max_duration > 0 and node.elapsed() > args.max_duration:
                reason = "max-duration"
                break
            if (
                args.idle_stop > 0
                and node.frames > 0
                and node.idle_for() > args.idle_stop
            ):
                reason = "idle"
                break
            if args.checkpoint_sec > 0 and now > next_ckpt:
                next_ckpt = now + args.checkpoint_sec
                save(node, args.output, not args.ascii, args.report)
                node.get_logger().info(
                    f"checkpoint: {len(node.keys)} voxels, {node.frames} frames"
                )
            if now - last_log > 10.0:
                last_log = now
                node.get_logger().info(
                    f"{node.frames} frames, {len(node.keys)} voxels, "
                    f"idle {node.idle_for():.0f}s"
                )
    except (KeyboardInterrupt, BaseException) as exc:  # noqa: BLE001
        reason = f"stopped ({type(exc).__name__})"
    finally:
        report = save(node, args.output, not args.ascii, args.report)
        node.get_logger().info(f"done ({reason}): {json.dumps(report)}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report["points"] else 1


if __name__ == "__main__":
    sys.exit(main())
