#!/usr/bin/env python3
"""Show/record observed 2D map, measured 3D returns, poses and voxel-plan routes."""
import argparse
import json
import math
from pathlib import Path as FilePath
import time
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import yaml

from observation_visualization import MapImage, ObservedPoints, ObservationRenderer, remaining_path


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class ObservationView(Node):
    def __init__(self, config):
        super().__init__("voxel_observation_view")
        self.names = config["robot_namespaces"]
        self.world = config["world_frame"]
        self.grid = None
        self.cloud = ObservedPoints(config["voxel_resolution"])
        self.accumulated = None
        self.accumulated_stamp = None
        self.poses, self.pose_stamps = [None, None], [None, None]
        self.routes, self.route_stamps = [[], []], [None, None]
        self.goals = [None, None]
        self.stats = {}
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, config["map_topic"], self.on_map, map_qos)
        self.create_subscription(String, "/observation/voxel_stats", self.on_stats, map_qos)
        self.create_subscription(PointCloud2, "/observation/measured_cloud", self.on_accumulated, map_qos)
        for r, name in enumerate(self.names):
            self.create_subscription(PointCloud2, f"/{name}/observed_points_world",
                                     lambda msg, i=r: self.on_cloud(i, msg), map_qos)
            self.create_subscription(PoseStamped, f"/{name}/pose_world",
                                     lambda msg, i=r: self.on_pose(i, msg), 1)
            self.create_subscription(Path, f"/{name}/route_world",
                                     lambda msg, i=r: self.on_route(i, msg), map_qos)
            self.create_subscription(PoseStamped, f"/{name}/observation_goal_world",
                                     lambda msg, i=r: self.on_goal(i, msg), map_qos)

    def valid_frame(self, msg):
        return msg.header.frame_id.lstrip("/") == self.world.lstrip("/")

    def on_map(self, msg):
        if not self.valid_frame(msg):
            return
        p = msg.info.origin
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width).copy()
        self.grid = MapImage(data, msg.info.resolution, (p.position.x, p.position.y, yaw(p.orientation)),
                             seconds(msg.header.stamp))

    def on_cloud(self, robot, msg):
        if self.valid_frame(msg):
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            self.cloud.add(robot, np.asarray(points).reshape(-1, 3), seconds(msg.header.stamp))

    def on_accumulated(self, msg):
        if self.valid_frame(msg):
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            self.accumulated = np.asarray(points, dtype=np.float32).reshape(-1, 3).copy()
            self.accumulated_stamp = seconds(msg.header.stamp)

    def on_pose(self, robot, msg):
        if self.valid_frame(msg):
            p = msg.pose
            self.poses[robot] = (p.position.x, p.position.y, yaw(p.orientation))
            self.pose_stamps[robot] = seconds(msg.header.stamp)

    def on_route(self, robot, msg):
        if self.valid_frame(msg):
            self.routes[robot] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.route_stamps[robot] = seconds(msg.header.stamp)

    def on_goal(self, robot, msg):
        if self.valid_frame(msg):
            self.goals[robot] = (msg.pose.position.x, msg.pose.position.y)

    def on_stats(self, msg):
        try:
            self.stats = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def frame(self, renderer):
        now = self.get_clock().now().nanoseconds*1e-9
        # Never use route starts or configured spawn poses as current localization.
        poses = [p if stamp is not None and 0 <= now-stamp <= 1.0 else None
                 for p, stamp in zip(self.poses, self.pose_stamps)]
        active = [[], []]
        for r in range(2):
            path = remaining_path(self.routes[r], poses[r])
            if len(path) and self.goals[r] is not None:
                distance = np.linalg.norm(path-np.asarray(self.goals[r]), axis=1)
                i = int(np.argmin(distance))
                if distance[i] < .1:
                    active[r] = path[:i+1]
        route_stamp = min((s for s in self.route_stamps if s is not None), default=None)
        status = self.stats.get("status", "Waiting for planner")
        cloud = SimpleNamespace(
            points=self.accumulated if self.accumulated is not None else self.cloud.points,
            current=self.cloud.current, stamps=self.cloud.stamps)
        frame = renderer.render(
            self.grid, cloud, poses, self.routes, now,
            event=self.stats.get("replan", 0), stats=self.stats, route_stamp=route_stamp,
            pose_stamps=self.pose_stamps, active_paths=active, status=status,
            source="LIVE ROS OBSERVATIONS", map_source="merged SLAM /map")
        timing = dict(time=now, map_stamp=self.grid.stamp if self.grid else None,
                      cloud_stamps=self.cloud.stamps, accumulated_stamp=self.accumulated_stamp,
                      pose_stamps=self.pose_stamps,
                      route_stamps=self.route_stamps, poses=poses)
        return frame, timing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = FilePath(__file__).resolve().parents[1]
    parser.add_argument("--config", type=FilePath, default=root/"config/voxel360_planner_params.yaml")
    parser.add_argument("--output", type=FilePath, default=root/"maps/classroom_observed_live.mp4")
    parser.add_argument("--duration", type=float, default=0.,
                        help="Optional wall-time cap; 0 records until exploration completes or is interrupted")
    parser.add_argument("--fps", type=float, default=5.)
    parser.add_argument("--show", action="store_true", help="Open a live OpenCV window; Q exits")
    args, ros_args = parser.parse_known_args()
    if args.fps <= 0 or args.duration < 0:
        parser.error("fps must be positive and duration nonnegative")
    config = yaml.safe_load(args.config.read_text())["rolling_observation_planner"]["ros__parameters"]
    if len(config["robot_namespaces"]) != 2:
        parser.error("This view requires two robot namespaces")
    rclpy.init(args=ros_args)
    node = ObservationView(config)
    renderer = ObservationRenderer(config["exploration_bounds"], config["voxel_resolution"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"avc1"),
                             args.fps, (renderer.width, renderer.height))
    if not writer.isOpened():
        node.destroy_node()
        rclpy.try_shutdown()
        raise RuntimeError("Could not open H.264 output")
    start = next_frame = time.monotonic()
    frames = 0
    try:
        with args.output.with_suffix(".jsonl").open("w") as log:
            while rclpy.ok() and (not args.duration or time.monotonic()-start < args.duration):
                rclpy.spin_once(node, timeout_sec=.02)
                if time.monotonic() < next_frame:
                    continue
                frame, timing = node.frame(renderer)
                complete = node.stats.get("complete", False) and node.stats.get("status") in (
                    "completed", "completed_with_exclusions", "completed_with_ignored_unknowns")
                timing.update(complete=complete, planner_status=node.stats.get("status"))
                writer.write(frame)
                log.write(json.dumps(timing)+"\n")
                if complete or frames % max(1, round(args.fps)) == 0:
                    cv2.imwrite(str(args.output.with_suffix(".png")), frame)
                    log.flush()
                frames += 1
                next_frame = time.monotonic()+1/args.fps
                if complete:
                    break
                if args.show:
                    cv2.imshow("Classroom / current observations", frame)
                    if cv2.waitKey(1) & 0xff in (ord("q"), 27):
                        break
    except KeyboardInterrupt:
        pass
    finally:
        writer.release()
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"Saved {frames} frames: {args.output}")


if __name__ == "__main__":
    main()
