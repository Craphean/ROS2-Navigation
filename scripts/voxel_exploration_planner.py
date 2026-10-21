#!/usr/bin/env python3
"""ROS adapter for bounded unknown-voxel coverage exploration.

The strong baseline and async V2 share subscriptions, TF and Nav2 action clients.
Planning, completion and action lifecycles are implemented below.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from observation_visualization import ObservedPoints
from lidar360_observation import DEFAULT_PROFILE, Lidar360, evaluate_candidates_360
from voxel_frontier_policy import FrontierPolicy
from voxel_fast_policy import FastFrontierEvaluator, FastPolicy
from voxel_async_execution import AsyncV2Policy, AsyncV2Window, path_length, queued_path, rebase_plan
from voxel_ros_transport import ViewTask
from voxel_ros_transport import (
    VoxelRosTransport, quaternion_to_yaw, stamp_to_sec,
    transform_to_matrix, yaw_to_quaternion,
)
from voxel_exploration_core import (
    FREE, OCCUPIED, UNKNOWN, CompletionMonitor, Coverage, GroundDistanceCache, GroundGrid, SensorModel, VoxelMap,
    continue_route, coverage_exhausted, evaluate_candidates, pending_route, solve_coverage_routes,
)


class VoxelExplorationPlanner(VoxelRosTransport):
    def __init__(self):
        super().__init__()
        defaults = {
            "exploration_bounds": [-8.0, 8.0, -6.0, 6.0, 0.05, 2.85],
            "robot_speeds": [0.22, 0.22],
            "robot_body_z_min": 0.06,
            "robot_body_z_max": 0.40,
            "exact_candidate_limit": 10,
            "scan_wait_timeout": 15.0,
            "voxel_snapshot": "",
            "snapshot_output": "",
            "sensor_offset_x": -0.064,
            "sensor_offset_y": 0.0,
            "voxel_display_z_max": 1.2,
            "voxel_display_all": False,
            "observation_model": "ideal_visibility",
            "lidar_profile": str(DEFAULT_PROFILE),
            "candidate_ownership": "nearest",
            "audit_when_idle": True,
            "replan_policy": "arrival_cycle",
            "target_policy": "exhaustive",
            "candidate_evaluation": "exhaustive",
        }
        defaults.update(vars(FrontierPolicy()))
        defaults.update(vars(FastPolicy()))
        defaults.update(vars(AsyncV2Policy()))
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        strategy = self.get_parameter("candidate_evaluation").value
        if strategy not in ("exhaustive", "incremental_frontier"):
            raise ValueError("Unknown candidate_evaluation strategy")
        self.fast_evaluator = FastFrontierEvaluator() if strategy == "incremental_frontier" else None
        self.fast_policy = FastPolicy.from_config({name: self.get_parameter(name).value
                                                  for name in FastPolicy.__dataclass_fields__})
        if len(self.robots) != 2:
            raise ValueError("Voxel exploration requires exactly two robots")
        snapshot = str(self.get_parameter("voxel_snapshot").value)
        self.voxels = VoxelMap.load(snapshot) if snapshot else VoxelMap(
            self.get_parameter("exploration_bounds").value,
            float(self.get_parameter("voxel_resolution").value))
        self.sensor = SensorModel(
            self.config.observation_z,
            float(self.get_parameter("sensor_min_range").value),
            self.config.max_obs_range,
            self.config.sensor_vfov_min, self.config.sensor_vfov_max,
            float(self.get_parameter("sensor_offset_x").value),
            float(self.get_parameter("sensor_offset_y").value))
        self.lidar360 = (Lidar360.load(self.get_parameter("lidar_profile").value)
                         if self.get_parameter("observation_model").value == "lidar360" else None)
        if self.lidar360 is not None:
            expected = self.lidar360.sensor(self.sensor.max_range)
            if self.sensor != expected:
                raise ValueError("360 planner parameters must match lidar_profile range, FOV and centered mounting")
        if not self.voxels.bounds[4] <= self.sensor.height < self.voxels.bounds[5]:
            raise ValueError("Sensor height must lie inside exploration_bounds")
        if self.sensor.max_range > float(self.get_parameter("sensor_max_range").value):
            raise ValueError("max_obs_range cannot exceed sensor_max_range")
        self.speeds = tuple(map(float, self.get_parameter("robot_speeds").value))
        if len(self.speeds) != 2 or min(self.speeds) <= 0:
            raise ValueError("robot_speeds must contain two positive m/s values")
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.planning_future = None
        replan_policy = self.get_parameter("replan_policy").value
        if replan_policy not in ("arrival_cycle", "async_arrival", "async_v2", "task_event", "continue_queue"):
            raise ValueError("Unknown replan_policy")
        self.async_arrival_mode = self.lidar360 is not None and replan_policy in ("async_arrival", "async_v2")
        self.async_v2 = (AsyncV2Window(AsyncV2Policy.from_config({
            name: self.get_parameter(name).value for name in AsyncV2Policy.__dataclass_fields__}))
            if self.lidar360 is not None and replan_policy == "async_v2" else None)
        self.v2_next_check = 0.
        self.v2_window = None
        self.v2_latency_recorded = False
        self.arrival_cycle_mode = (self.lidar360 is not None
                                  and self.get_parameter("replan_policy").value == "arrival_cycle")
        self.task_event_mode = (self.lidar360 is not None
                               and replan_policy in ("task_event", "arrival_cycle", "async_arrival", "async_v2"))
        self.cycle_phase = "SCANNING"
        self.cycle_stop_time = self.now_sec()
        self.task_epoch = 0
        self.accepted_task_epoch = -1
        self.planning_epoch = 0
        self.discarded_replans = 0
        self.ground = None
        self.distance_cache = GroundDistanceCache()
        self.latest_result = None
        self.failed_cells = {}
        self.scan_times = {}
        self.completed_revision = -1
        self.completion = CompletionMonitor(self.completion_stable_seconds)
        self.completed_ground = None
        self.request_serial = 0
        display_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stats_pub = self.create_publisher(String, "/observation/voxel_stats", display_qos)
        self.voxel_pub = self.create_publisher(PointCloud2, "/observation/voxels", 1)
        self.observed_cloud = ObservedPoints(self.voxels.resolution)
        self.cloud_dirty = False
        self.measured_cloud_pub = self.create_publisher(PointCloud2, "/observation/measured_cloud", display_qos)
        self.pose_pubs = {}
        self.scan_pubs = {}
        for robot in self.robots.values():
            robot.sequence = 0
            robot.arrival_time = 0.0
            robot.last_path = []
            robot.task_deadline = float("inf")
            self.pose_pubs[robot.namespace] = self.create_publisher(
                PoseStamped, f"/{robot.namespace}/pose_world", 1)
            self.scan_pubs[robot.namespace] = self.create_publisher(
                PointCloud2, f"/{robot.namespace}/observed_points_world", display_qos)
            # A viewer opened mid-run must receive the current plan immediately.
            self.destroy_publisher(robot.route_pub)
            self.destroy_publisher(robot.goal_pub)
            robot.route_pub = self.create_publisher(Path, f"/{robot.namespace}/route_world", display_qos)
            robot.goal_pub = self.create_publisher(
                PoseStamped, f"/{robot.namespace}/observation_goal_world", display_qos)
        # Current poses and measurements keep updating while a plan is computing.
        self.create_timer(.2, self.publish_current_poses)
        self.create_timer(1., self.publish_measured_cloud)
        self.get_logger().info(
            f"Unknown-voxel mode: ROI={self.voxels.bounds.tolist()}, "
            f"{self.voxels.state.size} voxels; exhaustive candidate visibility, "
            "joint makespan objective; failed scans never retire voxels.")

    def get_robot_pose(self, robot):
        # A lost TF must not silently teleport a moving robot to its spawn pose.
        for frame in (f"{robot.namespace}/base_footprint", f"{robot.namespace}/base_link"):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame, frame, Time(), timeout=Duration(seconds=self.tf_timeout))
                t = transform.transform
                return t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation)
            except TransformException:
                pass
        raise RuntimeError(f"No current world pose for {robot.namespace}")

    def integration_timer(self):
        if not hasattr(self, "voxels"):
            return
        for namespace in self.robots:
            msg = self.latest_clouds.get(namespace)
            if msg is None:
                continue
            stamp = stamp_to_sec(msg.header.stamp)
            if stamp <= self.scan_times.get(namespace, -1.0):
                continue
            try:
                # Only the transform at acquisition time is valid evidence.
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame, self.normalized_frame(namespace, msg.header.frame_id),
                    Time.from_msg(msg.header.stamp), timeout=Duration(seconds=self.tf_timeout))
                points = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=True)
                points = np.asarray(points, dtype=float).reshape(-1, 3)
                # Remove own chassis returns in sensor coordinates.
                own = ((np.abs(points[:, 0]) < .22) & (np.abs(points[:, 1]) < .18)
                       & (points[:, 2] < .05))
                points = points[~own]
                matrix = transform_to_matrix(transform)
                world_points = points @ matrix[:3, :3].T + matrix[:3, 3]
                # Robot bodies are dynamic, not permanent map obstacles. Drop
                # those returns (do not clear through the occluding vehicle).
                for peer in self.robots:
                    pose = self.tf_buffer.lookup_transform(
                        self.world_frame, f"{peer}/base_footprint",
                        Time.from_msg(msg.header.stamp), timeout=Duration(seconds=self.tf_timeout))
                    p = pose.transform.translation
                    body = ((world_points[:, 0] - p.x) ** 2 + (world_points[:, 1] - p.y) ** 2 < .28 ** 2)
                    body &= (world_points[:, 2] >= p.z - .05) & (world_points[:, 2] <= p.z + .45)
                    points, world_points = points[~body], world_points[~body]
                count = self.voxels.integrate_cloud(
                    points, matrix, stamp,
                    float(self.get_parameter("sensor_min_range").value),
                    float(self.get_parameter("sensor_max_range").value))
                if count:
                    self.scan_times[namespace] = stamp
                    self.integrated_clouds += 1
                    ranges = np.linalg.norm(points, axis=1)
                    valid = (np.isfinite(world_points).all(axis=1)
                             & (ranges >= float(self.get_parameter("sensor_min_range").value))
                             & (ranges < float(self.get_parameter("sensor_max_range").value)-1e-4))
                    returns = world_points[valid]
                    _, inside = self.voxels.indices(returns)
                    returns = returns[inside]
                    self.observed_cloud.add(namespace, returns, stamp)
                    self.cloud_dirty = True
                    self.scan_pubs[namespace].publish(self.measured_message(returns, msg.header.stamp))
            except (TransformException, ValueError) as exc:
                self.get_logger().warning(f"{namespace}: scan waiting for valid TF/data: {exc}",
                                          throttle_duration_sec=5.0)

    def measured_message(self, points, stamp):
        """Actual world-frame hit coordinates, excluding max-range/no-return rays."""
        packed = np.ascontiguousarray(points, dtype="<f4").reshape(-1, 3)
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = self.world_frame, stamp
        cloud.height, cloud.width = 1, len(packed)
        cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(("x", "y", "z"))]
        cloud.is_dense, cloud.is_bigendian = True, False
        cloud.point_step, cloud.row_step = 12, 12*len(packed)
        cloud.data = packed.tobytes()
        return cloud

    def publish_measured_cloud(self):
        if not self.cloud_dirty:
            return
        stamp = Time(seconds=max(self.observed_cloud.stamps.values())).to_msg()
        self.measured_cloud_pub.publish(self.measured_message(self.observed_cloud.points, stamp))
        self.cloud_dirty = False

    def publish_current_poses(self):
        for namespace, publisher in self.pose_pubs.items():
            for frame in (f"{namespace}/base_footprint", f"{namespace}/base_link"):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.world_frame, frame, Time(), timeout=Duration(seconds=.01))
                    msg = PoseStamped()
                    msg.header.frame_id, msg.header.stamp = self.world_frame, transform.header.stamp
                    p = transform.transform.translation
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x, p.y, p.z
                    msg.pose.orientation = transform.transform.rotation
                    publisher.publish(msg)
                    break
                except TransformException:
                    continue

    def make_ground(self):
        """Resample merged 2D occupancy into the finite ROI, add 3D body obstacles."""
        grid = super().build_grid()
        if grid is None or self.latest_map.header.frame_id.lstrip("/") != self.world_frame:
            raise RuntimeError("Merged map must be available in world_frame")
        nx, ny, _ = self.voxels.shape_xyz
        x = self.voxels.origin[0] + (np.arange(nx) + .5) * self.voxels.resolution
        y = self.voxels.origin[1] + (np.arange(ny) + .5) * self.voxels.resolution
        xx, yy = np.meshgrid(x, y)
        dx, dy = xx - grid.origin_x, yy - grid.origin_y
        rows = np.floor((-grid.sin_yaw * dx + grid.cos_yaw * dy) / grid.resolution).astype(int)
        cols = np.floor((grid.cos_yaw * dx + grid.sin_yaw * dy) / grid.resolution).astype(int)
        inside = (rows >= 0) & (rows < grid.height) & (cols >= 0) & (cols < grid.width)
        data = np.full((ny, nx), UNKNOWN, dtype=np.int8)
        values = grid.data[rows[inside], cols[inside]]
        data[inside] = np.where(values < 0, UNKNOWN, np.where(values >= 50, OCCUPIED, FREE))
        # Preserve even thin obstacles lost by center resampling.
        rr, cc = np.nonzero(grid.data >= 50)
        wx = grid.origin_x + grid.cos_yaw * (cc + .5) * grid.resolution - grid.sin_yaw * (rr + .5) * grid.resolution
        wy = grid.origin_y + grid.sin_yaw * (cc + .5) * grid.resolution + grid.cos_yaw * (rr + .5) * grid.resolution
        vc = np.floor((wx - self.voxels.origin[0]) / self.voxels.resolution).astype(int)
        vr = np.floor((wy - self.voxels.origin[1]) / self.voxels.resolution).astype(int)
        valid = (vc >= 0) & (vc < nx) & (vr >= 0) & (vr < ny)
        data[vr[valid], vc[valid]] = OCCUPIED
        z_low = self.voxels.origin[2] + np.arange(self.voxels.state.shape[0]) * self.voxels.resolution
        body = ((z_low + self.voxels.resolution > float(self.get_parameter("robot_body_z_min").value))
                & (z_low < float(self.get_parameter("robot_body_z_max").value)))
        data[np.any(self.voxels.state[body] == OCCUPIED, axis=0)] = OCCUPIED
        return GroundGrid(data, self.voxels.resolution, self.voxels.origin[:2], self.config.min_clearance,
                          distance_cache=getattr(self, "distance_cache", None))

    def request_task_replan(self):
        """Invalidate the joint plan and, in arrival mode, stop both cars first."""
        if getattr(self, "async_arrival_mode", False):
            # One in-flight snapshot, one coalesced request for the latest
            # state. Preserve both executing and queued goals until handoff.
            self.task_epoch += 1
            self.replan_requested = True
            self.replan_not_before = self.now_sec()
            return
        if getattr(self, "task_event_mode", False):
            self.task_epoch += 1
            arrival_cycle = getattr(self, "arrival_cycle_mode", False)
            if arrival_cycle:
                self.cycle_phase, self.cycle_stop_time = "STOPPING", None
            for robot in self.robots.values():
                if arrival_cycle:
                    robot.route = []
                    if robot.state in ("SENDING", "NAVIGATING"):
                        self.cancel_goal(robot, "cycle_pause")
                else:
                    robot.route = ([robot.current_task] if robot.current_task is not None
                                   and robot.state in ("SENDING", "NAVIGATING", "OBSERVING") else [])
            # Clear obsolete future paths immediately, without republishing
            # the large voxel cloud while the next solution is computing.
            if getattr(self, "ground", None) is not None:
                markers = MarkerArray()
                markers.markers = self.execution_route_markers()
                markers.markers.append(Marker(ns="selected_tasks", id=31, action=Marker.DELETE))
                self.marker_pub.publish(markers)
        self.replan_requested = True
        self.replan_not_before = self.now_sec()

    def plan_dispatch_ready(self):
        return (getattr(self, "async_arrival_mode", False)
                or not getattr(self, "task_event_mode", False)
                or (self.accepted_task_epoch == self.task_epoch
                    and (not getattr(self, "arrival_cycle_mode", False) or self.cycle_phase == "EXECUTING")))

    def arrival_scan_ready(self, now):
        """A completed Nav2 cancellation and a fresh scan from EACH car form the barrier."""
        if self.cycle_phase == "STOPPING":
            if any(r.state != "IDLE" or r.current_task is not None or r.current_handle is not None
                   for r in self.robots.values()):
                return False
            self.cycle_phase, self.cycle_stop_time = "SCANNING", now
        if self.cycle_phase == "SCANNING":
            return all(self.scan_times.get(r.namespace, -math.inf) > self.cycle_stop_time
                       for r in self.robots.values())
        return True

    def v2_execution_paths(self, ground, starts):
        return [queued_path(ground, start, [(t.x, t.y) for t in robot.route])
                for start, robot in zip(starts, self.robots.values())]

    def check_v2_buffer(self, now):
        """Bound timer overhead; an invalid route can always break commitment."""
        v2 = getattr(self, 'async_v2', None)
        if v2 is None or now < self.v2_next_check or self.latest_map is None:
            return
        self.v2_next_check = now+.25
        try:
            starts = [self.get_robot_pose(robot)[:2] for robot in self.robots.values()]
            ground = self.make_ground()
            paths = self.v2_execution_paths(ground, starts)
            for robot, path in zip(self.robots.values(), paths):
                if robot.route and (not path or tuple(path[-1]) != (robot.route[-1].x, robot.route[-1].y)):
                    # New obstacles outrank the temporal commitment. Nav2's
                    # controller remains responsible for immediate braking.
                    robot.route = []
                    self.cancel_goal(robot, 'replanned')
                    self.replan_requested = True
                    v2.commit_until = now
            if self.planning_future is None and not self.replan_requested:
                remaining = [path_length(p)/s for p, s in zip(paths, self.speeds)]
                result = getattr(self, 'latest_result', None)
                if (v2.needs_refill(now, remaining)
                        or v2.needs_idle_search(now, remaining, bool(result and result[3].deferred),
                                               self.last_replan_time)):
                    self.replan_requested = True
        except RuntimeError:
            # Do not predict or replace goals from stale localization.
            return

    def control_timer(self):
        if not hasattr(self, "voxels") or self.now_sec() <= 0:
            return
        now = self.now_sec()
        self.check_v2_buffer(now)
        for robot in self.robots.values():
            self.refresh_execution(robot)
            if robot.state == "OBSERVING":
                fresh = self.scan_times.get(robot.namespace, -1) > robot.arrival_time
                if fresh and now >= robot.observation_verify_time:
                    self.verify_observation(robot)
                elif now - robot.arrival_time > float(self.get_parameter("scan_wait_timeout").value):
                    self.get_logger().warning(f"{robot.namespace}: no fresh post-arrival scan; retaining unknowns")
                    self.fail_task(robot)
            elif robot.state in ("NAVIGATING", "SENDING") and now > robot.task_deadline:
                self.cancel_goal(robot, "timeout")
            if robot.state == "IDLE" and robot.route and self.plan_dispatch_ready():
                self.dispatch_task(robot, robot.route[0])
        v2 = getattr(self, 'async_v2', None)
        done = self.planning_future is not None and self.planning_future.done()
        if done and v2 and not self.v2_latency_recorded and self.planning_future.exception() is None:
            v2.observe_latency(self.planning_future.result()[-1])
            self.v2_latency_recorded = True
        # Empty old buffers and worker failures must not wait for a prefix.
        handoff_ready = (not v2 or now >= v2.commit_until
                         or any(not robot.route for robot in self.robots.values())
                         or (done and self.planning_future.exception() is not None))
        if done and handoff_ready:
            try:
                result = self.planning_future.result()
                if (getattr(self, "task_event_mode", False)
                        and not getattr(self, "async_arrival_mode", False)
                        and self.planning_epoch != self.task_epoch):
                    self.discarded_replans += 1
                    self.replan_requested = True
                    self.replan_not_before = now
                else:
                    self.accept_plan(*result)
            except Exception as exc:
                self.get_logger().error(f"Voxel planning failed: {exc}")
                self.replan_requested = True
                self.replan_not_before = now + self.idle_replan_period
            finally:
                self.planning_future = None
        if self.completed:
            if (self.voxels.revision == self.completed_revision
                    and np.array_equal(self.make_ground().data, self.completed_ground)):
                return
            self.completed = False
            self.completion.reset()
            self.replan_requested = True
        if self.completion.since is not None:
            if (self.voxels.revision != self.completion.version[0]
                    or any(robot.state != "IDLE" for robot in self.robots.values())
                    or self.make_ground().data.tobytes() != self.completion.version[1]
                    or any(now-self.scan_times.get(r.namespace, -math.inf)
                           > float(self.get_parameter("scan_wait_timeout").value)
                           for r in self.robots.values())):
                self.completion.reset()
        if self.latest_map is None or len(self.scan_times) < 2 or self.planning_future is not None:
            return
        if getattr(self, "async_arrival_mode", False) and any(
                not 0 <= now-self.scan_times.get(r.namespace, -math.inf)
                <= float(self.get_parameter("scan_wait_timeout").value) for r in self.robots.values()):
            return
        if getattr(self, "arrival_cycle_mode", False) and not self.arrival_scan_ready(now):
            return
        idle = all(robot.state == "IDLE" for robot in self.robots.values())
        result = getattr(self, "latest_result", None)
        retry_period = (min(self.idle_replan_period, .1)
                        if result and result[3].planning_stats.get("search_pending") else self.idle_replan_period)
        if not self.replan_requested and not (idle and now - self.last_replan_time >= retry_period):
            return
        if now < self.replan_not_before:
            return
        try:
            starts = [self.get_robot_pose(robot)[:2] for robot in self.robots.values()]
            ground = self.make_ground()
            if any(not ground.valid(ground.cell(start), ground.safe) for start in starts):
                raise RuntimeError("Robot pose is not on known safe ground; waiting for map/TF")
            snapshot = self.voxels.snapshot()
            blocked = {cell for cell, until in self.failed_cells.items() if until > now}
            self.request_serial += 1
            self.planning_epoch = getattr(self, "task_epoch", 0)
            if v2:
                starts, self.v2_window = v2.begin(now, self.v2_execution_paths(ground, starts), self.speeds)
                self.v2_latency_recorded = False
            if getattr(self, "arrival_cycle_mode", False) or getattr(self, "async_arrival_mode", False):
                self.cycle_phase = "PLANNING"
            self.planning_future = self.worker.submit(self.compute_plan, snapshot, ground, starts, blocked)
            self.replan_requested = False
            self.last_replan_time = now
            if not self.started_at:
                self.started_at = now
        except RuntimeError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            self.replan_not_before = now + 2.0

    def compute_plan(self, snapshot, ground, starts, blocked):
        begin = time.monotonic()
        if getattr(self, "fast_evaluator", None) is not None:
            if self.lidar360 is None or self.get_parameter("target_policy").value != "frontier":
                raise ValueError("Incremental frontier requires lidar360 and frontier targets")
            coverage = self.fast_evaluator.evaluate(
                snapshot, ground, starts, self.sensor, self.config.candidate_position_stride,
                self.lidar360, blocked=blocked, fast=self.fast_policy,
                policy=FrontierPolicy.from_config({name: self.get_parameter(name).value
                                                   for name in FrontierPolicy.__dataclass_fields__}),
                speeds=self.speeds, observation_time=self.config.observation_time)
        elif self.lidar360 is not None:
            coverage = evaluate_candidates_360(snapshot, ground, starts, self.sensor,
                                               self.config.candidate_position_stride, self.lidar360,
                                               ownership=self.get_parameter("candidate_ownership").value,
                                               audit_when_idle=self.get_parameter("audit_when_idle").value,
                                               blocked=blocked,
                                               target_policy=self.get_parameter("target_policy").value,
                                               frontier_policy=FrontierPolicy.from_config({
                                                   name: self.get_parameter(name).value
                                                   for name in FrontierPolicy.__dataclass_fields__}),
                                               speeds=self.speeds, observation_time=self.config.observation_time)
        else:
            coverage = evaluate_candidates(snapshot, ground, starts, self.sensor, self.config.candidate_position_stride)
        available = [c for c in coverage.candidates if c.cell not in blocked]
        actionable = set().union(*(c.coverage for c in available))
        # Cooldowns temporarily defer work; they cannot prove unreachability.
        coverage = replace(coverage, candidates=available, actionable=actionable,
                           deferred=coverage.deferred | (coverage.actionable-actionable))
        solve_begin = time.monotonic()
        plan = solve_coverage_routes(
            coverage, ground, starts, self.speeds, self.config.observation_time,
            int(self.get_parameter("exact_candidate_limit").value))
        if coverage.planning_stats:
            coverage.planning_stats.update(route_solver_seconds=time.monotonic()-solve_begin)
        return snapshot, ground, starts, coverage, plan, time.monotonic() - begin

    def accept_plan(self, snapshot, ground, starts, coverage, plan, elapsed):
        current_ground = self.make_ground()
        asynchronous = getattr(self, "async_arrival_mode", False)
        handoff = None
        if asynchronous:
            live = [self.get_robot_pose(robot)[:2] for robot in self.robots.values()]
            plan, handoff = rebase_plan(
                coverage, plan, current_ground, self.voxels, live,
                {cell for cell, until in self.failed_cells.items() if until > self.now_sec()}, self.speeds)
            if handoff['filtered_tasks'] and not any(plan.routes):
                self.replan_requested = True
        else:
            # Legacy modes reject invalid snapshot paths. Async mode rebuilds
            # connections from live poses instead of demanding old paths survive.
            for path in plan.paths:
                if any(not current_ground.valid(current_ground.cell(p), current_ground.safe) for p in path):
                    self.replan_requested = True
                    return
        self.ground = current_ground
        self.latest_result = snapshot, ground, starts, coverage, plan
        self.accepted_task_epoch = (getattr(self, "planning_epoch", self.task_epoch) if asynchronous
                                    else getattr(self, "task_epoch", 0))
        if getattr(self, "arrival_cycle_mode", False) or asynchronous:
            self.cycle_phase = "EXECUTING"
        now = self.now_sec()
        for r, robot in enumerate(self.robots.values()):
            tasks = []
            route = pending_route(plan.routes[r], coverage.candidates, self.voxels,
                                  {cell for cell, until in self.failed_cells.items() if until > now})
            active_cell = tuple(robot.current_task.key[1:3]) if robot.current_task else None
            try:
                current = self.get_robot_pose(robot)[:2]
            except RuntimeError:
                current = None
            if not getattr(self, "task_event_mode", False):
                route = continue_route(route, coverage.candidates, active_cell, current_ground, current)
            for index in route:
                c = coverage.candidates[index]
                requirements = frozenset(int(k) for k in c.coverage if self.voxels.state.ravel()[k] == UNKNOWN)
                if not requirements:
                    continue
                key = ("v", *c.cell) if self.lidar360 is not None else ("v", *c.cell, 0)
                tasks.append(ViewTask(key, *c.xy, self.sensor.height, 0.0, requirements))
            robot.route = tasks
        # Commit both routes before either action client sees the new cycle.
        for r, robot in enumerate(self.robots.values()):
            tasks = robot.route
            if robot.current_task is not None:
                if robot.state == "OBSERVING":
                    continue
                if not tasks or tasks[0].key != robot.current_task.key:
                    self.cancel_goal(robot, "replanned")
            elif tasks:
                self.dispatch_task(robot, tasks[0])
            elif plan.routes[r]:
                self.replan_requested = True
        stats = {
            "revision": snapshot.revision, "current_revision": self.voxels.revision,
            "classification_fresh": snapshot.revision == self.voxels.revision
                                    and np.array_equal(ground.data, current_ground.data)
                                    and coverage.classification_complete,
            "classification_complete": coverage.classification_complete,
            "candidate_ownership": coverage.ownership,
            "target_policy": coverage.target_policy, "selection_phase": coverage.selection_phase,
            "planning_stats": coverage.planning_stats,
            "ignored_unknown": len(coverage.ignored),
            "stopping_policy": ("all_retained_observable_unknowns_resolved"
                                if coverage.target_policy == "frontier" else "all_observable_unknowns_resolved"),
            "frontier_candidates": coverage.frontier_candidates,
            "available_candidates": coverage.available_candidates,
            "task_epoch": getattr(self, "task_epoch", 0),
            "planning_epoch": getattr(self, "planning_epoch", 0),
            "handoff": handoff,
            "async_v2_window": getattr(self, 'v2_window', None),
            "early_refill_count": getattr(getattr(self, 'async_v2', None), 'refill_count', 0),
            "idle_search_count": getattr(getattr(self, 'async_v2', None), 'idle_search_count', 0),
            "motion_during_planning": asynchronous,
            "cycle_phase": getattr(self, "cycle_phase", "EXECUTING"),
            "replan_trigger": "arrival" if getattr(self, "arrival_cycle_mode", False) or asynchronous else "task_completion",
            "discarded_replans": getattr(self, "discarded_replans", 0),
            "total": int(snapshot.state.size), "free": int(np.sum(snapshot.state == FREE)),
            "occupied": int(np.sum(snapshot.state == OCCUPIED)), "unknown": len(snapshot.unknown),
            "actionable": len(coverage.actionable), "deferred": len(coverage.deferred),
            "excluded": len(coverage.excluded), "candidate_positions": coverage.evaluated_positions,
            "candidate_views": len(coverage.candidates), "solver": plan.method,
            "route_seconds": plan.times, "makespan_seconds": plan.makespan,
            "planning_seconds": elapsed, "replan": self.replan_count,
            "sensor_range": self.sensor.max_range,
            "sensor_offset": [self.sensor.offset_x, self.sensor.offset_y],
            "sensor_height": self.sensor.height,
            "observation_heading_required": self.lidar360 is None,
            "status": "planning" if coverage.actionable else ("waiting_for_information" if coverage.deferred else "complete_candidate"),
        }
        self.replan_count += 1
        if self.lidar360 is not None:
            stats["lidar_profile"] = self.lidar360.metadata()
        complete_candidate = coverage_exhausted(snapshot, coverage)
        idle = all(robot.state == "IDLE" and robot.current_task is None
                   and robot.current_handle is None and not robot.route
                   for robot in self.robots.values())
        fresh_scans = all(0 <= now-self.scan_times.get(r.namespace, -math.inf)
                          <= float(self.get_parameter("scan_wait_timeout").value)
                          for r in self.robots.values())
        if self.completion.update(
                now, exhausted=complete_candidate, version=(snapshot.revision, ground.data.tobytes()),
                fresh=stats["classification_fresh"] and fresh_scans,
                idle=idle, planning=self.replan_requested):
            self.completed = True
            self.completed_revision = self.voxels.revision
            self.completed_ground = current_ground.data.copy()
            stats["status"] = ("completed_with_ignored_unknowns" if coverage.ignored else
                               "completed_with_exclusions" if coverage.excluded else "completed")
            stats["complete"], stats["stop_reason"] = True, "complete"
            self.completed_pub.publish(String(data=json.dumps(stats)))
        elif self.completion.since is not None:
            stats["status"] = "verifying_completion"
        payload = json.dumps(stats)
        self.stats_pub.publish(String(data=payload))
        self.decision_pub.publish(String(data=payload))
        self.get_logger().info(payload)
        self.publish_visualization(snapshot, coverage, plan, starts)
        output = str(self.get_parameter("snapshot_output").value)
        if output:
            self.voxels.save(output)

    def dispatch_task(self, robot, task):
        if not self.plan_dispatch_ready():
            return
        if not self.dispatch_goals or not robot.action_client.server_is_ready():
            return
        if robot.state != "IDLE":
            return
        current = self.get_robot_pose(robot)
        live_ground = self.make_ground()
        cell = live_ground.cell((task.x, task.y))
        if not live_ground.valid(cell, live_ground.safe):
            self.replan_requested = True
            return
        distance = live_ground.distances(current[:2])[cell]
        if not np.isfinite(distance):
            self.replan_requested = True
            return
        xl, yl, rotation = self.world_to_robot_map(robot, task.x, task.y)
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = xl, yl
        # In 360 mode Nav2's PositionGoalChecker ignores terminal heading.
        # Use the approach tangent as a neutral path hint, never a scan angle.
        goal_yaw = task.yaw
        if self.lidar360 is not None:
            path = live_ground.path(current[:2], (task.x, task.y))
            goal_yaw = current[2]
            for a, b in zip(path, path[1:]):
                if math.dist(a, b) > 1e-8:
                    goal_yaw = math.atan2(b[1]-a[1], b[0]-a[0])
        msg.pose.pose.orientation = yaw_to_quaternion(goal_yaw + rotation)
        robot.sequence += 1
        token = robot.sequence
        robot.current_task, robot.current_handle = task, None
        robot.state, robot.cancel_reason = "SENDING", ""
        robot.goal_start_time = self.now_sec()
        index = list(self.robots).index(robot.namespace)
        robot.task_deadline = robot.goal_start_time + max(
            self.task_timeout, 1.75 * distance / self.speeds[index] + self.observation_dwell + 15.0)
        world = PoseStamped()
        world.header.frame_id = self.world_frame
        world.header.stamp = msg.pose.header.stamp
        world.pose.position.x, world.pose.position.y = task.x, task.y
        world.pose.orientation = yaw_to_quaternion(goal_yaw)
        robot.goal_pub.publish(world)
        future = robot.action_client.send_goal_async(msg)
        future.add_done_callback(lambda f: self.on_goal_response(f, robot, token))

    def on_goal_response(self, future, robot, token):
        try:
            handle = future.result()
            if token != robot.sequence:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.fail_task(robot)
                return
            robot.current_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.on_goal_result(f, robot, token))
            if robot.cancel_reason:
                robot.state = "CANCELLING"
                handle.cancel_goal_async()
            else:
                robot.state = "NAVIGATING"
        except Exception as exc:
            self.get_logger().error(f"Action response: {exc}")
            if token == robot.sequence:
                self.fail_task(robot)

    def on_goal_result(self, future, robot, token):
        if token != robot.sequence:
            return
        try:
            status = future.result().status
            robot.current_handle = None
            if status == GoalStatus.STATUS_SUCCEEDED and not robot.cancel_reason:
                if getattr(self, "async_arrival_mode", False):
                    robot.arrival_time = self.now_sec()
                    # Revisit cooldown is not an observation or exclusion.
                    if robot.current_task is not None:
                        self.failed_cells[tuple(robot.current_task.key[1:3])] = (
                            robot.arrival_time + self.failed_view_cooldown)
                    self.remove_current_visit(robot)
                    robot.state, robot.current_task = "IDLE", None
                    self.accepted_observations += 1
                    self.last_accepted_time = robot.arrival_time
                    self.request_task_replan()
                    if robot.route:
                        self.dispatch_task(robot, robot.route[0])
                    return
                if getattr(self, "arrival_cycle_mode", False):
                    robot.arrival_time = self.now_sec()
                    self.remove_current_visit(robot)
                    robot.state, robot.current_task = "IDLE", None
                    self.accepted_observations += 1
                    self.last_accepted_time = robot.arrival_time
                    self.request_task_replan()
                    self.get_logger().info(
                        f"{robot.namespace}: arrived; stopping both robots, then scanning and replanning")
                    return
                robot.state = "OBSERVING"
                robot.arrival_time = self.now_sec()
                robot.observation_verify_time = robot.arrival_time + self.observation_dwell
                return
            if robot.cancel_reason == "cycle_pause":
                # A peer's arrival stops this goal without declaring it a
                # failed viewpoint or a second arrival/replanning event.
                robot.state, robot.current_task, robot.cancel_reason = "IDLE", None, ""
                robot.route = []
                return
            if robot.cancel_reason in ("replanned", "observed_en_route"):
                robot.state, robot.current_task, robot.cancel_reason = "IDLE", None, ""
                if getattr(self, "async_arrival_mode", False):
                    # A replacement is already committed. Waited for the old
                    # action's terminal result; now dispatch from actual TF.
                    if robot.route:
                        self.dispatch_task(robot, robot.route[0])
                    return
                # Recompute from the actual stopping pose before dispatching.
                self.request_task_replan()
            else:
                self.fail_task(robot)
        except Exception as exc:
            self.get_logger().error(f"Action result: {exc}")
            self.fail_task(robot)

    def cancel_goal(self, robot, reason):
        if robot.current_task is None or robot.state == "CANCELLING":
            return
        robot.cancel_reason = reason
        robot.state = "CANCELLING"
        if robot.current_handle is not None:
            robot.current_handle.cancel_goal_async()
        # When send is pending, on_goal_response sends the cancellation.
        # Never send a replacement until a terminal result is received.

    def fail_task(self, robot):
        if robot.current_task is not None:
            cell = tuple(robot.current_task.key[1:3])
            self.failed_cells[cell] = self.now_sec() + self.failed_view_cooldown
            self.remove_current_visit(robot)
        robot.state, robot.current_task, robot.current_handle = "IDLE", None, None
        robot.cancel_reason = ""
        self.rejected_observations += 1
        self.request_task_replan()

    def remove_current_visit(self, robot):
        if robot.current_task is not None:
            robot.route = [task for task in getattr(robot, "route", [])
                           if task.key != robot.current_task.key]

    def refresh_execution(self, robot):
        """Both robots' scans can make a queued or active visit unnecessary."""
        if getattr(self, "arrival_cycle_mode", False) or getattr(self, "async_arrival_mode", False):
            # The current joint plan is committed until an actual goal arrival.
            # Streaming observations alone are not a navigation completion.
            return
        state = self.voxels.state.ravel()

        def needed(task):
            return np.any(state[np.fromiter(task.requirements, dtype=np.int64)] == UNKNOWN)

        previous = getattr(robot, "route", [])
        robot.route = [task for task in previous if needed(task)]
        if len(robot.route) != len(previous) and not getattr(self, "task_event_mode", False):
            self.request_task_replan()
        if (robot.current_task is not None and robot.state in ("SENDING", "NAVIGATING")
                and not needed(robot.current_task)):
            # Still wait for Nav2's terminal result before sending another goal.
            # Observed work is a success, not a failed viewpoint or an exclusion.
            self.cancel_goal(robot, "observed_en_route")
            self.request_task_replan()

    def verify_observation(self, robot):
        ids = list(robot.current_task.requirements) if robot.current_task else []
        newly_known = sum(self.voxels.state.ravel()[i] != UNKNOWN for i in ids)
        if newly_known:
            self.accepted_observations += 1
            self.last_accepted_time = self.now_sec()
            self.remove_current_visit(robot)
            robot.state, robot.current_task, robot.current_handle = "IDLE", None, None
            self.request_task_replan()
        else:
            self.fail_task(robot)
        self.get_logger().info(
            f"{robot.namespace}: fresh scan verified {newly_known}/{len(ids)} voxels; "
            "replanning both routes from current poses")
        # Partial success is only progress. Remaining unknowns stay in the map.
        self.replan_requested = True
        self.replan_not_before = self.now_sec()

    def publish_visualization(self, snapshot, coverage, plan, starts):
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        # Packed PointCloud2 avoids millions of Python Point/Marker objects.
        # Surface/cutaway filtering affects rendering only, never the task map.
        labels = np.zeros(snapshot.state.size, dtype=np.uint8)
        labels[snapshot.state.ravel() == OCCUPIED] = 1
        labels[list(coverage.actionable)] = 2
        labels[coverage.deferred.mask if hasattr(coverage.deferred, 'mask') else list(coverage.deferred)] = 3
        labels[list(coverage.excluded)] = 4
        labels[list(getattr(coverage, "ignored", ()))] = 5
        display = snapshot.state != FREE
        if not bool(self.get_parameter("voxel_display_all").value):
            boundary = np.zeros(snapshot.state.shape, dtype=bool)
            for axis in range(3):
                left = [slice(None)]*3
                right = [slice(None)]*3
                left[axis], right[axis] = slice(None, -1), slice(1, None)
                different = snapshot.state[tuple(left)] != snapshot.state[tuple(right)]
                boundary[tuple(left)] |= different
                boundary[tuple(right)] |= different
            display &= boundary
            z = snapshot.origin[2] + (np.arange(display.shape[0])+.5)*snapshot.resolution
            display &= (z <= float(self.get_parameter("voxel_display_z_max").value))[:, None, None]
        ids = np.flatnonzero(display)
        packed = np.empty(len(ids), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgba", "<u4")])
        for axis, name in enumerate(("x", "y", "z")):
            packed[name] = snapshot.centers[ids, axis]
        palette = np.array([0xff66bfd9, 0xff596b80, 0xffffb31a, 0xffa659ff, 0xfff2404d,
                            0xff888888], dtype=np.uint32)
        packed["rgba"] = palette[labels[ids]]
        cloud = PointCloud2()
        cloud.header.frame_id = self.world_frame
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height, cloud.width = 1, len(ids)
        cloud.fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                        for name, offset, datatype in (("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32),
                                                       ("z", 8, PointField.FLOAT32), ("rgba", 12, PointField.UINT32))]
        cloud.is_bigendian, cloud.is_dense = False, True
        cloud.point_step, cloud.row_step = 16, 16*len(ids)
        cloud.data = packed.tobytes()
        self.voxel_pub.publish(cloud)
        markers.markers.extend(self.execution_route_markers())
        selected_cells = {tuple(t.key[1:3]) for robot in self.robots.values() for t in robot.route}
        for name, indices, scale, color in (
            ("candidate_views", range(len(coverage.candidates)), .07, (.6, .65, .7, .7)),
            ("selected_tasks", [i for i, c in enumerate(coverage.candidates)
                                if c.cell in selected_cells], .22, (.2, 1., .5, 1.)),
        ):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type = name, 30 if name == "candidate_views" else 31, Marker.SPHERE_LIST
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.points = [Point(x=float(coverage.candidates[i].xy[0]),
                              y=float(coverage.candidates[i].xy[1]), z=.12) for i in indices]
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def execution_route_markers(self):
        markers = []
        for r, robot in enumerate(self.robots.values()):
            path = Path()
            path.header.frame_id = self.world_frame
            path.header.stamp = self.get_clock().now().to_msg()
            # Display the accepted execution order, which may have been pruned
            # and resumed at an active goal since this snapshot was planned.
            points = []
            try:
                previous = self.get_robot_pose(robot)[:2]
                for task in robot.route:
                    segment = self.ground.path(previous, (task.x, task.y))
                    if not segment:
                        break
                    points.extend(segment if not points else segment[1:])
                    previous = (task.x, task.y)
            except RuntimeError:
                pass  # Lost localization is not a return to the snapshot start.
            for x, y in points:
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), .08
                p.pose.orientation.w = 1.0
                path.poses.append(p)
            robot.route_pub.publish(path)
            m = Marker()
            m.header = path.header
            m.ns, m.id, m.type = f"{robot.namespace}_route", 20 + r, Marker.LINE_STRIP
            m.pose.orientation.w = 1.0
            m.scale.x = .05
            m.color.r, m.color.g, m.color.b, m.color.a = ((1., .3, .25, 1.) if r == 0 else (.2, .6, 1., 1.))
            m.points = [p.pose.position for p in path.poses]
            markers.append(m)
        return markers

    def shutdown_robots(self):
        super().shutdown_robots()
        self.worker.shutdown(wait=True, cancel_futures=True)
        if getattr(self, "fast_evaluator", None) is not None:
            self.fast_evaluator.close()


def main():
    rclpy.init()
    node = VoxelExplorationPlanner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_robots()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
