#!/usr/bin/env python3
"""ROS 1 Noetic per-robot telemetry & 2D/3D map recorder for official CMU M-TARE benchmarks."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


def _parse_pc2_xyz(msg: PointCloud2) -> np.ndarray:
    """Fast vectorized extraction of (N, 3) float32 XYZ from ROS1 PointCloud2."""
    n_pts = int(msg.width) * int(msg.height)
    if n_pts <= 0 or msg.point_step < 12:
        return np.empty((0, 3), dtype=np.float32)
    off = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z"):
            off[f.name] = f.offset
    if len(off) < 3:
        return np.empty((0, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = n_pts * msg.point_step
    if raw.size < expected:
        n_pts = raw.size // msg.point_step
    raw = raw[: n_pts * msg.point_step].reshape(n_pts, msg.point_step)
    x = np.frombuffer(raw[:, off["x"] : off["x"] + 4].tobytes(), dtype=np.float32)
    y = np.frombuffer(raw[:, off["y"] : off["y"] + 4].tobytes(), dtype=np.float32)
    z = np.frombuffer(raw[:, off["z"] : off["z"] + 4].tobytes(), dtype=np.float32)
    pts = np.column_stack((x, y, z))
    valid = np.all(np.isfinite(pts), axis=1)
    return pts[valid]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", type=int, required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--gt-2d", required=True)
    ap.add_argument("--max-sim-sec", type=float, default=180.0)
    ap.add_argument("--voxel-size", type=float, default=0.05)
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rid = int(args.robot_id)

    gt_data = np.load(args.gt_2d)
    grid_h, grid_w = gt_data["grid"].shape
    origin_x = float(gt_data["origin_x"])
    origin_y = float(gt_data["origin_y"])
    res = float(gt_data["resolution"])

    free_map = np.zeros((grid_h, grid_w), dtype=np.uint8)
    occ_map = np.zeros((grid_h, grid_w), dtype=np.uint8)
    first_seen = np.full((grid_h, grid_w), 1e9, dtype=np.float32)
    cloud_voxels: Dict[int, Tuple[float, float, float]] = {}

    rospy.init_node(f"mtare_bench_rec_{rid}", anonymous=True)

    state = {
        "t0_sim": None,
        "last_scan_sim": -1e9,
        "pose": None,
        "comms_bytes": 0,
        "done": False,
        "wall_start": time.monotonic(),
    }
    traj: List[List[float]] = []
    runtime_ms: List[float] = []
    area_series: List[List[float]] = []

    def on_odom(msg: Odometry) -> None:
        p = msg.pose.pose.position
        state["pose"] = (float(p.x), float(p.y), float(p.z))

    def on_runtime(msg: Float32) -> None:
        v = float(msg.data)
        if math.isfinite(v) and v >= 0.0:
            # M-TARE /runtime is reported in seconds
            runtime_ms.append(v * 1000.0 if v < 20.0 else v)

    def on_comms_raw(msg) -> None:
        buf = getattr(msg, "_buff", None)
        if buf is not None:
            state["comms_bytes"] += len(buf)
        else:
            state["comms_bytes"] += 64

    def on_scan(msg: PointCloud2) -> None:
        now_sim = rospy.Time.now().to_sec()
        if now_sim <= 0.0:
            return
        if state["t0_sim"] is None:
            state["t0_sim"] = now_sim
        if now_sim - state["last_scan_sim"] < 0.45:
            return
        state["last_scan_sim"] = now_sim

        pts = _parse_pc2_xyz(msg)
        if pts.size == 0:
            return

        # 3D voxel accumulation (0.05 m)
        vs = args.voxel_size
        keys = np.floor(pts / vs).astype(np.int64) + (1 << 20)
        np.clip(keys, 0, (1 << 21) - 1, out=keys)
        packed = (keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]
        _, uniq_idx = np.unique(packed, return_index=True)
        for k, p in zip(packed[uniq_idx].tolist(), pts[uniq_idx].tolist()):
            if k not in cloud_voxels:
                cloud_voxels[k] = (p[0], p[1], p[2])

        # 2D occupancy grid projection via 360-bin raycasting from robot pose
        pose = state["pose"]
        if pose is None:
            return
        rx, ry, _ = pose
        rc = int(math.floor((rx - origin_x) / res))
        rr = int(math.floor((ry - origin_y) / res))
        if not (0 <= rc < grid_w and 0 <= rr < grid_h):
            return

        dx = pts[:, 0] - rx
        dy = pts[:, 1] - ry
        rng = np.hypot(dx, dy)
        in_rng = (rng > 0.25) & (rng <= 14.0)
        if not np.any(in_rng):
            return

        pts_r = pts[in_rng]
        dx_r = dx[in_rng]
        dy_r = dy[in_rng]
        rng_r = rng[in_rng]
        ang_bin = np.floor((np.arctan2(dy_r, dx_r) + math.pi) / (2.0 * math.pi) * 360.0).astype(np.int32)
        ang_bin = np.clip(ang_bin, 0, 359)

        # Free rays to furthest return per 1-degree bin
        max_r = np.zeros(360, dtype=np.float32)
        np.maximum.at(max_r, ang_bin, rng_r)
        # Wall hits (0.20 <= z <= 1.85 m)
        wall_mask = (pts_r[:, 2] >= 0.20) & (pts_r[:, 2] <= 1.85)
        wall_r = np.full(360, 1e6, dtype=np.float32)
        if np.any(wall_mask):
            np.minimum.at(wall_r, ang_bin[wall_mask], rng_r[wall_mask])
            wc = np.floor((pts_r[wall_mask, 0] - origin_x) / res).astype(np.int32)
            wr = np.floor((pts_r[wall_mask, 1] - origin_y) / res).astype(np.int32)
            ok = (wc >= 0) & (wc < grid_w) & (wr >= 0) & (wr < grid_h)
            occ_map[wr[ok], wc[ok]] = 1

        for b in range(360):
            dist = min(float(max_r[b]), float(wall_r[b]))
            if dist <= 0.3 or not math.isfinite(dist):
                continue
            ang = -math.pi + (b + 0.5) * (2.0 * math.pi / 360.0)
            ex = rx + max(0.0, dist - res) * math.cos(ang)
            ey = ry + max(0.0, dist - res) * math.sin(ang)
            ec = max(0, min(grid_w - 1, int(math.floor((ex - origin_x) / res))))
            er = max(0, min(grid_h - 1, int(math.floor((ey - origin_y) / res))))
            cv2.line(free_map, (rc, rr), (ec, er), 1, thickness=1)

    rospy.Subscriber("/state_estimation_at_scan", Odometry, on_odom, queue_size=5)
    rospy.Subscriber("/registered_scan", PointCloud2, on_scan, queue_size=2)
    rospy.Subscriber("/runtime", Float32, on_runtime, queue_size=10)
    rospy.Subscriber(f"/wheeled{rid}/position", PointStamped, on_comms_raw, queue_size=10)
    rospy.Subscriber(f"/wheeled{rid}/exploration_info", rospy.AnyMsg, on_comms_raw, queue_size=10)

    last_tick_sim = -1.0
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        now_sim = rospy.Time.now().to_sec()
        if state["t0_sim"] is None:
            if time.monotonic() - state["wall_start"] > 120.0:
                break
            rate.sleep()
            continue

        elapsed = now_sim - state["t0_sim"]
        if elapsed - last_tick_sim >= 1.0:
            last_tick_sim = elapsed
            pose = state["pose"]
            if pose is not None:
                traj.append([round(pose[0], 4), round(pose[1], 4), round(elapsed, 2)])
            free_only = (free_map > 0) & (occ_map == 0)
            newly_seen = free_only & (first_seen > elapsed)
            first_seen[newly_seen] = float(elapsed)
            area_m2 = float(np.sum(free_only)) * (res ** 2)
            area_series.append([round(elapsed, 2), round(area_m2, 3), len(cloud_voxels)])

        if elapsed >= args.max_sim_sec or (time.monotonic() - state["wall_start"]) > args.max_sim_sec * 3.5 + 120.0:
            break
        rate.sleep()

    # Save per-robot artifacts
    (out_dir / f"robot_{rid}_meta.json").write_text(
        json.dumps(
            {
                "robot_id": rid,
                "duration_sim_s": round(max(0.0, last_tick_sim), 2),
                "traj": traj,
                "runtime_ms": runtime_ms,
                "area_series": area_series,
                "comms_bytes": int(state["comms_bytes"]),
            }
        ),
        encoding="utf-8",
    )
    pts_arr = (
        np.asarray(list(cloud_voxels.values()), dtype=np.float32)
        if cloud_voxels
        else np.empty((0, 3), dtype=np.float32)
    )
    np.savez_compressed(
        out_dir / f"robot_{rid}_maps.npz",
        free_map=free_map,
        occ_map=occ_map,
        first_seen=first_seen,
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=res,
        points=pts_arr,
    )


if __name__ == "__main__":
    main()
