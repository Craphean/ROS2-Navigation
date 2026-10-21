#!/usr/bin/env python3
"""Shared ROS transport for Strong Baseline and A1 Algorithm.

The historical ROS node name is retained for parameter-file compatibility.
Task selection and execution policies live in voxel_exploration_planner.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

@dataclass(frozen=True)
class ViewTask:
    key: tuple
    x: float
    y: float
    z: float
    yaw: float
    requirements: frozenset
    frontier_coverage: int = 0
    surface_coverage: int = 0

    def to_dict(self) -> dict:
        return {
            "key": self.key,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
            "requirements": len(self.requirements),
            "frontier_coverage": self.frontier_coverage,
            "surface_coverage": self.surface_coverage,
        }


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    quaternion = Quaternion()
    quaternion.z = math.sin(half)
    quaternion.w = math.cos(half)
    return quaternion


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def transform_to_matrix(transform) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (yy + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = np.array([translation.x, translation.y, translation.z])
    return matrix


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


@dataclass
class RobotRuntime:
    namespace: str
    init_pose: Tuple[float, float, float]
    state: str = "IDLE"
    current_task: Optional[ViewTask] = None
    current_handle=None
    route: List[ViewTask] = field(default_factory=list)
    goal_start_time: float = 0.0
    observation_verify_time: float = 0.0
    task_attempts: int = 0
    cancel_reason: str = ""
    action_client: Optional[ActionClient] = None
    goal_pub = None
    route_pub = None

    @property
    def navigating(self) -> bool:
        return self.state == "NAVIGATING"

    @property
    def busy(self) -> bool:
        return self.state in ("NAVIGATING", "OBSERVING")


class VoxelRosTransport(Node):
    def __init__(self):
        super().__init__("rolling_observation_planner")
        defaults = {
            "robot_namespaces": ["robot1", "robot2"],
            "robot1_init_pose": [-6.5, 4.8, 3.14],
            "robot2_init_pose": [6.5, 4.8, 3.14],
            "world_frame": "world", "map_topic": "/map", "points_suffix": "points",
            "integration_rate": 10.0, "control_rate": 10.0,
            "dispatch_goals": True, "decision_mode": "baseline",
            "observation_dwell": 1.0, "completion_stable_seconds": 8.0,
            "failed_view_cooldown": 30.0, "task_timeout": 180.0,
            "idle_replan_period": 5.0, "tf_timeout": 0.15,
            "voxel_resolution": 0.05, "sensor_min_range": 0.30,
            "sensor_max_range": 5.0, "observation_z": 0.22,
            "max_obs_range": 5.0, "sensor_vfov_min": -math.pi/4,
            "sensor_vfov_max": math.pi/4, "candidate_position_stride": 0.25,
            "min_clearance": 0.25, "observation_time": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        for name in ("world_frame", "dispatch_goals", "observation_dwell",
                     "completion_stable_seconds", "failed_view_cooldown",
                     "task_timeout", "idle_replan_period", "tf_timeout"):
            setattr(self, name, self.get_parameter(name).value)
        self.config = SimpleNamespace(**{
            name: float(self.get_parameter(name).value)
            for name in ("observation_z", "max_obs_range", "sensor_vfov_min",
                         "sensor_vfov_max", "candidate_position_stride",
                         "min_clearance", "observation_time")})
        namespaces = list(self.get_parameter("robot_namespaces").value)
        if len(namespaces) != 2 or len(set(ns.strip("/") for ns in namespaces)) != 2:
            raise ValueError("Voxel exploration requires two distinct robot namespaces")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.callback_group = ReentrantCallbackGroup()

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_clouds: Dict[str, PointCloud2] = {}
        self.last_integrated_stamp: Dict[str, Tuple[int, int]] = {}
        self.integrated_clouds = 0

        self.robots: Dict[str, RobotRuntime] = {}
        for index, namespace in enumerate(namespaces):
            clean = namespace.strip("/")
            parameter_name = f"{clean}_init_pose"
            if self.has_parameter(parameter_name):
                init_pose = self.get_parameter(parameter_name).value
            elif index == 0:
                init_pose = self.get_parameter("robot1_init_pose").value
            else:
                init_pose = self.get_parameter("robot2_init_pose").value
            robot = RobotRuntime(
                namespace=clean,
                init_pose=(float(init_pose[0]), float(init_pose[1]), float(init_pose[2])),
            )
            robot.action_client = ActionClient(
                self,
                NavigateToPose,
                f"/{clean}/navigate_to_pose",
                callback_group=self.callback_group,
            )
            robot.goal_pub = self.create_publisher(PoseStamped, f"/{clean}/observation_goal_world", 10)
            robot.route_pub = self.create_publisher(Path, f"/{clean}/route_world", 10)
            self.robots[clean] = robot

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            map_qos,
            callback_group=self.callback_group,
        )

        points_suffix = str(self.get_parameter("points_suffix").value).strip("/")
        for namespace in self.robots:
            topic = f"/{namespace}/{points_suffix}"
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, robot=namespace: self.cloud_callback(robot, msg),
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
            self.get_logger().info(f"Subscribing to {topic}")

        self.marker_pub = self.create_publisher(MarkerArray, "/observation/markers", 10)
        self.task_status_pub = self.create_publisher(
            MarkerArray, "/observation/task_status", 10
        )
        self.decision_pub = self.create_publisher(String, "/observation/decision", 10)
        self.completed_pub = self.create_publisher(String, "/observation/completed", 10)

        self.replan_requested = True
        self.replan_not_before = 0.0
        self.last_replan_time = 0.0
        self.replan_count = 0
        self.started_at = 0.0
        self.accepted_observations = 0
        self.rejected_observations = 0
        self.last_accepted_time = 0.0
        self.completed = False
        for name, callback in (("integration_rate", self.integration_timer),
                               ("control_rate", self.control_timer)):
            rate = max(0.2, float(self.get_parameter(name).value))
            self.create_timer(1.0/rate, callback, callback_group=self.callback_group)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9


    def map_callback(self, msg: OccupancyGrid) -> None:
        # Map updates are consumed by the next event-driven replan.  They must
        # not trigger replanning on their own, otherwise the node would replan
        # at the map-merge publish rate.
        self.latest_map = msg


    def cloud_callback(self, namespace: str, msg: PointCloud2) -> None:
        self.latest_clouds[namespace] = msg


    def normalized_frame(self, namespace: str, frame: str) -> str:
        frame = frame.lstrip("/")
        if not frame:
            return f"{namespace}/base_scan"
        if frame == self.world_frame or frame.startswith(f"{namespace}/"):
            return frame
        return f"{namespace}/{frame}"


    def world_to_robot_map(
        self, robot: RobotRuntime, x: float, y: float
    ) -> Tuple[float, float, float]:
        for map_frame in (f"{robot.namespace}/map", "map"):
            try:
                transform = self.tf_buffer.lookup_transform(
                    map_frame,
                    self.world_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                )
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tyaw = quaternion_to_yaw(transform.transform.rotation)
                c = math.cos(tyaw)
                s = math.sin(tyaw)
                return (
                    tx + x * c - y * s,
                    ty + x * s + y * c,
                    tyaw,
                )
            except TransformException:
                continue
        dx = x - robot.init_pose[0]
        dy = y - robot.init_pose[1]
        c = math.cos(robot.init_pose[2])
        s = math.sin(robot.init_pose[2])
        return (
            dx * c + dy * s,
            -dx * s + dy * c,
            -robot.init_pose[2],
        )


    def shutdown_robots(self) -> None:
        for robot in self.robots.values():
            self.cancel_goal(robot, "shutdown")
            robot.state = "IDLE"
            robot.current_task = None
            robot.route = []
