#!/usr/bin/env python3
"""Lightweight ROS 2 telemetry & resource recorder for headless benchmarks.

Records during each exploration run:
  - ``timeseries.csv``: sim_time, explored_area_m2, occupied_voxels_3d, cycle_time_ms,
    cpu_percent, ram_mb, comms_kbps, converged
  - ``trajectories.json``: per-robot estimated world poses and optional GT poses
  - ``resources.json``: summary CPU mean/peak, RAM peak, planning latency P50/P95/Max,
    min inter-robot distance & collision count, total comms KB/s
  - ``map_final.npz`` & ``cloud_final.pcd``: final 2D OccupancyGrid and accumulated 3D cloud
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import psutil

from .pcd_io import voxel_downsample, write_pcd


_SIM_PROC_MARKERS = (
    "gz sim",
    "gz-sim",
    "ros_gz",
    "parameter_bridge",
    "robot_state_publisher",
    "ekf_node",
    "slam_toolbox",
    "nav2_",
    "bt_navigator",
    "controller_server",
    "planner_server",
    "behavior_server",
    "smoother_server",
    "velocity_smoother",
    "collision_monitor",
    "lifecycle_manager",
    "se2_map_merge_node",
    "map_merge",
    "cloud_merger_node",
    "merge_pointclouds.py",
    "octomap_server",
    "multi_robot_coordinator",
    "coordinator_node",
    "explore_lite",
)


class _ProcessResourceMonitor:
    """Track per-role and aggregate simulation CPU% and RSS using persistent psutil handles."""

    def __init__(self) -> None:
        self._self_pid = os.getpid()
        self._procs: Dict[int, Tuple[psutil.Process, str]] = {}
        self.refresh()

    @staticmethod
    def _cmdline_str(proc: psutil.Process) -> str:
        try:
            cmd = proc.cmdline()
            if cmd:
                return " ".join(cmd)
            return proc.name()
        except psutil.Error:
            return ""

    def refresh(self) -> None:
        alive: Dict[int, Tuple[psutil.Process, str]] = {}
        for p in psutil.process_iter(["pid"]):
            pid = p.pid
            if pid == self._self_pid:
                continue
            if pid in self._procs:
                alive[pid] = self._procs[pid]
                continue
            cmd = self._cmdline_str(p)
            if not cmd or "bench_recorder" in cmd or "run_benchmark" in cmd:
                continue
            if any(m in cmd for m in _SIM_PROC_MARKERS):
                try:
                    p.cpu_percent(interval=None)
                    alive[pid] = (p, cmd)
                except psutil.Error:
                    continue
        self._procs = alive

    def sample(self) -> Dict[str, float]:
        self.refresh()
        total_cpu = 0.0
        total_rss = 0.0
        cloud_cpu = 0.0
        merge_cpu = 0.0
        ekf_cpu = 0.0
        coord_cpu = 0.0
        matched = 0
        for proc, cmd in list(self._procs.values()):
            try:
                c = float(proc.cpu_percent(interval=None))
                rss = float(proc.memory_info().rss) / (1024.0 * 1024.0)
            except psutil.Error:
                continue
            matched += 1
            total_cpu += c
            total_rss += rss
            if "cloud_merger_node" in cmd or "merge_pointclouds.py" in cmd:
                cloud_cpu += c
            elif "se2_map_merge_node" in cmd or "/multirobot_map_merge/map_merge" in cmd:
                merge_cpu += c
            elif "ekf_node" in cmd:
                ekf_cpu += c
            elif "multi_robot_coordinator" in cmd or "coordinator_node" in cmd:
                coord_cpu += c

        if matched == 0:
            total_cpu = float(psutil.cpu_percent(interval=None))
            total_rss = float(psutil.virtual_memory().used) / (1024.0 * 1024.0)

        return {
            "total_cpu": total_cpu,
            "total_rss": total_rss,
            "cloud_merger_cpu": cloud_cpu,
            "map_merge_cpu": merge_cpu,
            "ekf_cpu": ekf_cpu,
            "coordinator_cpu": coord_cpu,
        }


def _sample_system_resources(watch_pids: Optional[List[int]] = None) -> Tuple[float, float]:
    """Return (total_cpu_percent, total_rss_mb) for the target process tree or container."""
    if not watch_pids:
        cpu = psutil.cpu_percent(interval=None)
        ram_mb = psutil.virtual_memory().used / (1024.0 * 1024.0)
        return float(cpu), float(ram_mb)

    total_cpu = 0.0
    total_rss = 0.0
    seen = set()
    for pid in watch_pids:
        try:
            root = psutil.Process(pid)
            procs = [root] + root.children(recursive=True)
        except psutil.Error:
            continue
        for p in procs:
            if p.pid in seen:
                continue
            seen.add(p.pid)
            try:
                total_cpu += p.cpu_percent(interval=None)
                total_rss += p.memory_info().rss / (1024.0 * 1024.0)
            except psutil.Error:
                continue
    return float(total_cpu), float(total_rss)


def run_ros_recorder(
    out_dir: Path,
    robots: List[str],
    max_sim_sec: float = 300.0,
    voxel_size: float = 0.02,
    stop_on_converged: bool = True,
    collision_dist_m: float = 0.34,
) -> None:
    """Spin an rclpy recorder node until convergence or max_sim_sec."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Bool, Float64, String
    import tf2_ros

    out_dir.mkdir(parents=True, exist_ok=True)

    class BenchRecorderNode(Node):
        def __init__(self) -> None:
            super().__init__(
                "bench_recorder",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            self.robots = robots
            self.t0_sim: Optional[float] = None
            self.wall_start = time.monotonic()
            self.converged = False
            self.converged_since: Optional[float] = None
            self.done = False

            self.latest_map: Optional[OccupancyGrid] = None
            self.explored_area = 0.0
            self.voxel_count = 0
            self.cloud_voxels: Dict[int, Tuple[float, float, float]] = {}
            self.cycle_ms_samples: List[float] = []
            self.cpu_samples: List[float] = []
            self.ram_samples: List[float] = []
            self.cloud_merger_cpu_samples: List[float] = []
            self.map_merge_cpu_samples: List[float] = []
            self.ekf_cpu_samples: List[float] = []
            self.coordinator_cpu_samples: List[float] = []
            self.comms_bytes = 0
            self.last_comms_bytes = 0
            self.last_sample_sim = 0.0
            self.se2_accept_count = 0
            self.se2_degenerate_count = 0
            self.se2_elapsed_ms: List[float] = []
            self.proc_mon = _ProcessResourceMonitor()

            self.est_trajs: Dict[str, List[List[float]]] = {ns: [] for ns in robots}
            self.gt_trajs: Dict[str, List[List[float]]] = {ns: [] for ns in robots}
            self.latest_gt: Dict[str, Optional[Tuple[float, float]]] = {ns: None for ns in robots}
            self.min_inter_robot_dist = float("inf")
            self.inter_robot_collisions = 0
            self._in_collision = False

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            cloud_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
            self.create_subscription(PointCloud2, "/octomap_point_cloud_centers", self._on_octo, cloud_qos)
            self.create_subscription(PointCloud2, "/points_merged", self._on_merged_cloud, cloud_qos)
            self.create_subscription(Float64, "/exploration/cycle_time_ms", self._on_cycle_ms, 10)
            self.create_subscription(Bool, "/exploration/converged", self._on_converged, 10)
            self.create_subscription(
                String, "/map_merge/alignment_diagnostics", self._on_align_diag, 10
            )
            for ns in robots:
                self.create_subscription(
                    Odometry, f"/{ns}/gt_pose", lambda m, r=ns: self._on_gt_pose(r, m), 10
                )

            self.diag_file = (out_dir / "alignment_diagnostics.jsonl").open("w", encoding="utf-8")
            self.ts_file = (out_dir / "timeseries.csv").open("w", newline="", encoding="utf-8")
            self.ts_writer = csv.writer(self.ts_file)
            self.ts_writer.writerow(
                [
                    "sim_time_s",
                    "explored_area_m2",
                    "occupied_voxels_3d",
                    "cycle_time_ms",
                    "cpu_percent",
                    "ram_mb",
                    "comms_kbps",
                    "converged",
                ]
            )
            psutil.cpu_percent(interval=None)
            self.create_timer(1.0, self._on_tick)

        def _sim_sec(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _on_map(self, msg: OccupancyGrid) -> None:
            self.latest_map = msg
            arr = np.asarray(msg.data, dtype=np.int8)
            free_cells = int(np.sum((arr >= 0) & (arr < 20)))
            self.explored_area = free_cells * (float(msg.info.resolution) ** 2)
            self.comms_bytes += len(msg.data)

        def _on_octo(self, msg: PointCloud2) -> None:
            self.voxel_count = int(msg.width) * int(msg.height)
            self.comms_bytes += len(msg.data)

        def _on_merged_cloud(self, msg: PointCloud2) -> None:
            self.comms_bytes += len(msg.data)
            try:
                pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
                pts = voxel_downsample(np.asarray(pts, dtype=np.float32).reshape(-1, 3), voxel_size)
            except Exception:
                return
            if pts.size == 0:
                return
            keys = np.floor(pts / voxel_size).astype(np.int64) + (1 << 20)
            np.clip(keys, 0, (1 << 21) - 1, out=keys)
            packed = (keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]
            for k, p in zip(packed.tolist(), pts.tolist()):
                if k not in self.cloud_voxels:
                    self.cloud_voxels[k] = (p[0], p[1], p[2])

        def _on_cycle_ms(self, msg: Float64) -> None:
            self.cycle_ms_samples.append(float(msg.data))

        def _on_converged(self, msg: Bool) -> None:
            self.converged = bool(msg.data)

        def _on_align_diag(self, msg: String) -> None:
            raw = msg.data.strip()
            if not raw:
                return
            self.diag_file.write(raw + "\n")
            self.diag_file.flush()
            try:
                rec = json.loads(raw)
                if rec.get("accepted"):
                    self.se2_accept_count += 1
                if rec.get("degenerate_trans"):
                    self.se2_degenerate_count += 1
                if "elapsed_ms" in rec:
                    self.se2_elapsed_ms.append(float(rec["elapsed_ms"]))
            except Exception:
                pass

        def _on_gt_pose(self, ns: str, msg: Odometry) -> None:
            self.latest_gt[ns] = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            )

        def _on_tick(self) -> None:
            now = self._sim_sec()
            if now <= 0.0:
                if time.monotonic() - self.wall_start > 180.0:
                    self.done = True
                return
            if self.t0_sim is None:
                if self.latest_map is None:
                    if time.monotonic() - self.wall_start > 180.0:
                        self.done = True
                    return
                self.t0_sim = now
                self.last_sample_sim = now
                self.proc_mon.sample()  # prime CPU counters at exploration start
            elapsed = now - self.t0_sim

            poses_now: Dict[str, Tuple[float, float]] = {}
            for ns in self.robots:
                for frame in (f"{ns}/base_footprint", f"{ns}/base_link"):
                    try:
                        t = self.tf_buffer.lookup_transform("world", frame, Time())
                        px, py = float(t.transform.translation.x), float(t.transform.translation.y)
                        self.est_trajs[ns].append([round(px, 4), round(py, 4), round(elapsed, 2)])
                        poses_now[ns] = (px, py)
                        break
                    except Exception:
                        continue
                gt = self.latest_gt.get(ns)
                if gt is not None:
                    self.gt_trajs[ns].append([round(gt[0], 4), round(gt[1], 4), round(elapsed, 2)])

            # Inter-robot collision monitoring
            ns_list = list(poses_now.keys())
            min_d = float("inf")
            for i in range(len(ns_list)):
                for j in range(i + 1, len(ns_list)):
                    p1, p2 = poses_now[ns_list[i]], poses_now[ns_list[j]]
                    d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                    min_d = min(min_d, d)
            if math.isfinite(min_d):
                self.min_inter_robot_dist = min(self.min_inter_robot_dist, min_d)
                if min_d < collision_dist_m:
                    if not self._in_collision:
                        self.inter_robot_collisions += 1
                        self._in_collision = True
                else:
                    self._in_collision = False

            res_sample = self.proc_mon.sample()
            cpu = res_sample["total_cpu"]
            ram = res_sample["total_rss"]
            self.cpu_samples.append(cpu)
            self.ram_samples.append(ram)
            self.cloud_merger_cpu_samples.append(res_sample["cloud_merger_cpu"])
            self.map_merge_cpu_samples.append(res_sample["map_merge_cpu"])
            self.ekf_cpu_samples.append(res_sample["ekf_cpu"])
            self.coordinator_cpu_samples.append(res_sample["coordinator_cpu"])
            dt = max(1e-3, now - self.last_sample_sim)
            kbps = (self.comms_bytes - self.last_comms_bytes) / 1024.0 / dt
            self.last_comms_bytes = self.comms_bytes
            self.last_sample_sim = now

            last_cycle = self.cycle_ms_samples[-1] if self.cycle_ms_samples else 0.0
            self.ts_writer.writerow(
                [
                    round(elapsed, 2),
                    round(self.explored_area, 3),
                    self.voxel_count,
                    round(last_cycle, 2),
                    round(cpu, 2),
                    round(ram, 2),
                    round(kbps, 2),
                    int(self.converged),
                ]
            )
            self.ts_file.flush()

            if stop_on_converged and self.converged:
                if self.converged_since is None:
                    self.converged_since = elapsed
                elif elapsed - self.converged_since >= 5.0:
                    self.done = True
            else:
                self.converged_since = None

            if elapsed >= max_sim_sec:
                self.done = True

        def finalize(self) -> None:
            self.ts_file.close()
            self.diag_file.close()
            if self.latest_map is not None:
                m = self.latest_map
                grid = np.asarray(m.data, dtype=np.int8).reshape((int(m.info.height), int(m.info.width)))
                np.savez_compressed(
                    out_dir / "map_final.npz",
                    grid=grid,
                    origin_x=float(m.info.origin.position.x),
                    origin_y=float(m.info.origin.position.y),
                    resolution=float(m.info.resolution),
                )
            if self.cloud_voxels:
                pts = np.asarray(list(self.cloud_voxels.values()), dtype=np.float32)
                write_pcd(out_dir / "cloud_final.pcd", pts, binary=True)

            (out_dir / "trajectories.json").write_text(
                json.dumps({"estimated": self.est_trajs, "ground_truth": self.gt_trajs}),
                encoding="utf-8",
            )
            cyc = np.asarray(self.cycle_ms_samples, dtype=np.float64)
            se2_ms = np.asarray(self.se2_elapsed_ms, dtype=np.float64)
            cpu = np.asarray(self.cpu_samples, dtype=np.float64)
            ram = np.asarray(self.ram_samples, dtype=np.float64)
            cm_cpu = np.asarray(self.cloud_merger_cpu_samples, dtype=np.float64)
            mm_cpu = np.asarray(self.map_merge_cpu_samples, dtype=np.float64)
            ekf_cpu = np.asarray(self.ekf_cpu_samples, dtype=np.float64)
            coord_cpu = np.asarray(self.coordinator_cpu_samples, dtype=np.float64)
            total_sim = (self.last_sample_sim - (self.t0_sim or self.last_sample_sim)) or 1.0
            resources = {
                "converged": bool(self.converged),
                "duration_sim_s": round(float(total_sim), 2),
                "cpu_mean_pct": float(np.mean(cpu)) if len(cpu) else 0.0,
                "cpu_peak_pct": float(np.max(cpu)) if len(cpu) else 0.0,
                "ram_peak_mb": float(np.max(ram)) if len(ram) else 0.0,
                "cloud_merger_cpu_mean_pct": float(np.mean(cm_cpu)) if len(cm_cpu) else 0.0,
                "cloud_merger_cpu_peak_pct": float(np.max(cm_cpu)) if len(cm_cpu) else 0.0,
                "map_merge_cpu_mean_pct": float(np.mean(mm_cpu)) if len(mm_cpu) else 0.0,
                "ekf_cpu_mean_pct": float(np.mean(ekf_cpu)) if len(ekf_cpu) else 0.0,
                "coordinator_cpu_mean_pct": float(np.mean(coord_cpu)) if len(coord_cpu) else 0.0,
                "cycle_ms_p50": float(np.percentile(cyc, 50)) if len(cyc) else float("nan"),
                "cycle_ms_p95": float(np.percentile(cyc, 95)) if len(cyc) else float("nan"),
                "cycle_ms_max": float(np.max(cyc)) if len(cyc) else float("nan"),
                "se2_accept_count": int(self.se2_accept_count),
                "se2_degenerate_count": int(self.se2_degenerate_count),
                "se2_elapsed_ms_p95": float(np.percentile(se2_ms, 95)) if len(se2_ms) else float("nan"),
                "comms_avg_kbps": float(self.comms_bytes / 1024.0 / total_sim),
                "min_inter_robot_dist_m": (
                    float(self.min_inter_robot_dist)
                    if math.isfinite(self.min_inter_robot_dist)
                    else float("nan")
                ),
                "inter_robot_collisions": int(self.inter_robot_collisions),
            }
            (out_dir / "resources.json").write_text(json.dumps(resources, indent=2), encoding="utf-8")

    rclpy.init()
    node = BenchRecorderNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.t0_sim is None and (time.monotonic() - node.wall_start) > 180.0:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(description="Record ROS 2 exploration benchmark telemetry.")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--robots", nargs="+", default=["robot1", "robot2"])
    ap.add_argument("--max-sim-sec", type=float, default=300.0)
    ap.add_argument("--voxel-size", type=float, default=0.02)
    ap.add_argument("--no-stop-converged", action="store_true")
    args = ap.parse_args()
    run_ros_recorder(
        Path(args.out_dir),
        robots=args.robots,
        max_sim_sec=args.max_sim_sec,
        voxel_size=args.voxel_size,
        stop_on_converged=not args.no_stop_converged,
    )


if __name__ == "__main__":
    main()
