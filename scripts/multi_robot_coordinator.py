#!/usr/bin/env python3
"""
Multi-Robot Auction Exploration Coordinator for Nav2 + slam_toolbox + multirobot_map_merge.

Subscribes to the merged map (/map in 'world' frame) and the global OctoMap
occupied-voxel cloud, extracts 2D frontiers, scores them with a 3D structural
information gain, schedules near-field Local tours (TSP) and far-field Global
targets (ROI auction), and dispatches Nav2 actions.

The legacy 2D behaviour is preserved when the new feature switches are off.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - happens only outside a ROS install
    point_cloud2 = None

from exploration_3d import (
    AgentState,
    ConvergenceParams,
    ConvergenceTracker,
    InfoGainParams,
    OccupancyView,
    ResidualParams,
    RoiParams,
    TourParams,
    VoxelColumnIndex,
    VoxelParams,
    assign_ownership,
    build_local_tour,
    detect_residuals,
    exclude_near_waypoints,
    global_bid,
    path_length,
    wrap_angle,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw (radians) to ROS geometry_msgs/Quaternion."""
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """Convert ROS geometry_msgs/Quaternion to yaw (radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FrontierCluster:
    def __init__(
        self,
        world_x: float,
        world_y: float,
        size: int,
        grid_r: int,
        grid_c: int,
        yaw_world: float = 0.0,
        gain_2d: float = 0.0,
        gain_3d: float = 0.0,
        total_gain: float = 0.0,
        kind: str = 'frontier',
        component_label: int = -1,
    ):
        self.world_x = world_x
        self.world_y = world_y
        self.size = size
        self.grid_r = grid_r
        self.grid_c = grid_c
        self.yaw_world = yaw_world
        self.gain_2d = gain_2d
        self.gain_3d = gain_3d
        self.total_gain = total_gain
        self.kind = kind
        self.component_label = component_label

    @property
    def x(self) -> float:
        """Alias matching exploration_3d's candidate interface."""
        return self.world_x

    @property
    def y(self) -> float:
        return self.world_y


class RobotTracker:
    def __init__(self, namespace: str, init_pose: List[float]):
        self.namespace = namespace.strip('/')
        self.init_x = float(init_pose[0])
        self.init_y = float(init_pose[1])
        self.init_yaw = float(init_pose[2])

        # 'IDLE', 'DISPATCHING', 'NAVIGATING'
        self.state = 'IDLE'
        self.current_goal_world: Optional[Tuple[float, float]] = None
        self.current_goal_handle = None
        self.goal_start_time: float = 0.0
        self.goal_token: int = 0

        # Current estimated pose in 'world' frame
        self.world_x = self.init_x
        self.world_y = self.init_y
        self.world_yaw = self.init_yaw

        self.action_client: Optional[ActionClient] = None
        self.through_client: Optional[ActionClient] = None
        self.failed_goals_count: Dict[Tuple[int, int], int] = {}
        self.goal_pub = None
        self.plan_pub = None
        self.plan_sub = None

        # Active route / tour state
        self.active_waypoints_world: List[Tuple[float, float]] = []
        self.tour_candidates: List[object] = []
        self.tour_poses_world: List[Tuple[float, float, float]] = []
        self.current_waypoint_index: int = 0
        self.tour_start_time: float = 0.0
        self.last_progress_time: float = 0.0
        self.last_distance_remaining: float = float('nan')
        self.stall_distance_ref: float = float('inf')
        self.stall_ref_x: float = self.init_x
        self.stall_ref_y: float = self.init_y
        self.through_poses_failures: int = 0
        self.use_through_poses: bool = True
        self.tour_is_active: bool = False
        self.tour_uses_through_poses: bool = False
        self.rewrite_count: int = 0
        self.current_residual_key: Optional[Tuple[float, float]] = None

        # No-progress livelock breaker: repeated instant successes on the
        # same goal mean the robot is already standing at a frontier pocket
        # that never clears (stale map / 2D-invisible cells); after a few
        # repeats the spot is blacklisted so exploration can move on.
        self.no_progress_key: Optional[Tuple[float, float]] = None
        self.no_progress_streak: int = 0

        # Relocalization jump detection
        self.prev_world_x: Optional[float] = None
        self.prev_world_y: Optional[float] = None


class MultiRobotCoordinator(Node):
    def __init__(self):
        super().__init__('multi_robot_coordinator')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('robot1_init_pose', [-2.0, -0.5, 0.0])
        self.declare_parameter('robot2_init_pose', [0.0, 0.5, 0.0])
        self.declare_parameter('rate_hz', 0.5)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('potential_scale', 2.0)
        self.declare_parameter('orientation_scale', 0.5)
        self.declare_parameter('exclusion_radius', 1.5)
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('blacklist_retry_limit', 2)
        self.declare_parameter('blacklist_radius', 0.9)
        self.declare_parameter('min_clearance', 0.35)

        # No-progress livelock breaker
        self.declare_parameter('no_progress_time', 2.0)
        self.declare_parameter('no_progress_distance', 0.3)
        self.declare_parameter('no_progress_limit', 2)

        # 3D information gain
        self.declare_parameter('use_3d_gain', True)
        self.declare_parameter('octomap_cloud_topic', '/octomap_point_cloud_centers')
        self.declare_parameter('voxel_index_resolution', 0.10)
        self.declare_parameter('voxel_min_z', 0.08)
        self.declare_parameter('voxel_max_z', 2.0)
        self.declare_parameter('voxel_min_points_per_column', 3)
        self.declare_parameter('voxel_max_points', 400000)
        self.declare_parameter('voxel_update_period', 1.0)
        self.declare_parameter('gain_3d_window_radius', 1.5)
        self.declare_parameter('gain_3d_scale', 40.0)
        self.declare_parameter('gain_3d_density_weight', 0.30)
        self.declare_parameter('gain_3d_unknown_weight', 0.35)
        self.declare_parameter('gain_3d_floating_weight', 0.15)
        self.declare_parameter('gain_3d_height_weight', 0.10)
        self.declare_parameter('gain_3d_occupancy_weight', 0.10)
        self.declare_parameter('gain_3d_density_ref', 0.18)
        self.declare_parameter('gain_3d_occupancy_ref', 8.0)
        self.declare_parameter('gain_3d_height_ref', 0.80)
        self.declare_parameter('gain_3d_wall_zmin', 0.25)
        self.declare_parameter('gain_3d_wall_zmax', 1.30)
        self.declare_parameter('voxel_cloud_warn_timeout', 15.0)
        self.declare_parameter('cycle_time_budget_ms', 150.0)
        self.declare_parameter('map_loss_timeout', 5.0)

        # Local / Global scheduling
        self.declare_parameter('local_scheduling_enable', True)
        self.declare_parameter('local_radius', 6.0)
        self.declare_parameter('local_max_waypoints', 6)
        self.declare_parameter('local_min_spacing', 1.2)
        self.declare_parameter('local_max_tour_length', 10.0)
        self.declare_parameter('local_tour_score_dist_weight', 1.0)
        self.declare_parameter('local_release_unselected', False)
        self.declare_parameter('local_release_cooldown', 5.0)
        self.declare_parameter('local_release_score_penalty', 0.7)
        self.declare_parameter('tsp_2opt_passes', 2)
        self.declare_parameter('global_roi_enable', True)
        self.declare_parameter('global_roi_distance_offset', 1.0)
        self.declare_parameter('global_roi_min', 0.05)
        self.declare_parameter('global_roi_scale', 5.0)
        self.declare_parameter('global_max_distance', 0.0)
        self.declare_parameter('global_max_candidates', 8)

        # Tour dispatch
        self.declare_parameter('use_through_poses', True)
        self.declare_parameter('through_poses_fail_limit', 2)
        self.declare_parameter('action_wait_sec', 2.0)
        self.declare_parameter('tf_wait_sec', 0.1)
        self.declare_parameter('tour_timeout_per_waypoint', 25.0)
        self.declare_parameter('tour_stall_window', 20.0)
        self.declare_parameter('tour_stall_min_translation', 0.10)
        self.declare_parameter('tour_stall_min_progress', 0.20)
        self.declare_parameter('tour_rewrite_on_invalidation', True)
        self.declare_parameter('tour_max_rewrites', 2)

        # Residual blind spots
        self.declare_parameter('residual_enable', True)
        self.declare_parameter('residual_activation_cycles', 3)
        self.declare_parameter('residual_min_cluster_cells', 25)
        self.declare_parameter('residual_min_density', 0.08)
        self.declare_parameter('residual_min_unknown_frac', 0.20)
        self.declare_parameter('residual_structure_radius', 2.5)
        self.declare_parameter('residual_min_structure_columns', 3)
        self.declare_parameter('residual_ring_radius', 1.0)
        self.declare_parameter('residual_ring_min_empty_sectors', 3)
        self.declare_parameter('residual_viewpoint_search_radius', 2.5)
        self.declare_parameter('residual_max_candidates', 4)
        self.declare_parameter('residual_dedupe_radius', 1.5)
        self.declare_parameter('residual_score_scale', 10.0)
        self.declare_parameter('residual_blacklist_radius', 1.0)
        self.declare_parameter('residual_max_attempts', 2)
        self.declare_parameter('residual_coverage_hole_enable', False)

        # Convergence
        self.declare_parameter('convergence_enable', True)
        self.declare_parameter('convergence_cycles_required', 6)
        self.declare_parameter('convergence_residual_gain_threshold', 0.50)
        self.declare_parameter('convergence_voxel_growth_epsilon', 25)
        self.declare_parameter('convergence_voxel_growth_rate', 0.005)
        self.declare_parameter('convergence_require_3d', True)
        self.declare_parameter('fallback_convergence_timeout', 120.0)
        self.declare_parameter('convergence_stop_dispatch', True)

        namespaces = (
            self.get_parameter('robot_namespaces')
            .get_parameter_value()
            .string_array_value
        )
        if not namespaces:
            namespaces = ['robot1', 'robot2']

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.gain_scale = float(self.get_parameter('gain_scale').value)
        self.potential_scale = float(self.get_parameter('potential_scale').value)
        self.orientation_scale = float(
            self.get_parameter('orientation_scale').value
        )
        self.exclusion_radius = float(self.get_parameter('exclusion_radius').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.blacklist_retry_limit = int(
            self.get_parameter('blacklist_retry_limit').value
        )
        self.blacklist_radius = float(
            self.get_parameter('blacklist_radius').value
        )
        self.min_clearance = float(self.get_parameter('min_clearance').value)
        self.no_progress_time = float(
            self.get_parameter('no_progress_time').value
        )
        self.no_progress_distance = float(
            self.get_parameter('no_progress_distance').value
        )
        self.no_progress_limit = int(
            self.get_parameter('no_progress_limit').value
        )

        self.use_3d_gain = bool(self.get_parameter('use_3d_gain').value)
        self.octomap_cloud_topic = str(
            self.get_parameter('octomap_cloud_topic').value
        )
        self.voxel_update_period = float(
            self.get_parameter('voxel_update_period').value
        )
        self.gain_3d_window_radius = float(
            self.get_parameter('gain_3d_window_radius').value
        )
        self.gain_3d_scale = float(self.get_parameter('gain_3d_scale').value)
        self.voxel_cloud_warn_timeout = float(
            self.get_parameter('voxel_cloud_warn_timeout').value
        )
        self.cycle_time_budget_ms = float(
            self.get_parameter('cycle_time_budget_ms').value
        )
        self.map_loss_timeout = float(
            self.get_parameter('map_loss_timeout').value
        )
        self.voxel_params = VoxelParams(
            resolution=float(
                self.get_parameter('voxel_index_resolution').value
            ),
            min_z=float(self.get_parameter('voxel_min_z').value),
            max_z=float(self.get_parameter('voxel_max_z').value),
            min_points_per_column=int(
                self.get_parameter('voxel_min_points_per_column').value
            ),
            max_points=int(self.get_parameter('voxel_max_points').value),
        )
        self.info_gain_params = InfoGainParams(
            window_radius=self.gain_3d_window_radius,
            density_ref=float(
                self.get_parameter('gain_3d_density_ref').value
            ),
            occupancy_ref=float(
                self.get_parameter('gain_3d_occupancy_ref').value
            ),
            height_ref=float(self.get_parameter('gain_3d_height_ref').value),
            wall_zmin=float(self.get_parameter('gain_3d_wall_zmin').value),
            wall_zmax=float(self.get_parameter('gain_3d_wall_zmax').value),
            w_density=float(
                self.get_parameter('gain_3d_density_weight').value
            ),
            w_unknown=float(
                self.get_parameter('gain_3d_unknown_weight').value
            ),
            w_floating=float(
                self.get_parameter('gain_3d_floating_weight').value
            ),
            w_height=float(self.get_parameter('gain_3d_height_weight').value),
            w_occupancy=float(
                self.get_parameter('gain_3d_occupancy_weight').value
            ),
        )

        self.local_scheduling_enable = bool(
            self.get_parameter('local_scheduling_enable').value
        )
        self.local_radius = float(self.get_parameter('local_radius').value)
        self.local_release_unselected = bool(
            self.get_parameter('local_release_unselected').value
        )
        self.local_release_cooldown = float(
            self.get_parameter('local_release_cooldown').value
        )
        self.local_release_score_penalty = float(
            self.get_parameter('local_release_score_penalty').value
        )
        self.tour_params = TourParams(
            max_waypoints=int(
                self.get_parameter('local_max_waypoints').value
            ),
            min_spacing=float(self.get_parameter('local_min_spacing').value),
            max_tour_length=float(
                self.get_parameter('local_max_tour_length').value
            ),
            score_dist_weight=float(
                self.get_parameter('local_tour_score_dist_weight').value
            ),
            two_opt_passes=int(self.get_parameter('tsp_2opt_passes').value),
        )
        self.global_roi_enable = bool(
            self.get_parameter('global_roi_enable').value
        )
        self.roi_params = RoiParams(
            distance_offset=float(
                self.get_parameter('global_roi_distance_offset').value
            ),
            roi_scale=float(self.get_parameter('global_roi_scale').value),
            min_roi=float(self.get_parameter('global_roi_min').value),
            max_distance=float(
                self.get_parameter('global_max_distance').value
            ),
            orientation_scale=self.orientation_scale,
        )
        self.global_max_candidates = int(
            self.get_parameter('global_max_candidates').value
        )

        self.use_through_poses = bool(
            self.get_parameter('use_through_poses').value
        )
        self.through_poses_fail_limit = int(
            self.get_parameter('through_poses_fail_limit').value
        )
        self.action_wait_sec = float(
            self.get_parameter('action_wait_sec').value
        )
        self.tf_wait_sec = max(
            0.0, float(self.get_parameter('tf_wait_sec').value)
        )
        self.tour_timeout_per_waypoint = float(
            self.get_parameter('tour_timeout_per_waypoint').value
        )
        self.tour_stall_window = float(
            self.get_parameter('tour_stall_window').value
        )
        self.tour_stall_min_translation = float(
            self.get_parameter('tour_stall_min_translation').value
        )
        self.tour_stall_min_progress = float(
            self.get_parameter('tour_stall_min_progress').value
        )
        self.tour_rewrite_on_invalidation = bool(
            self.get_parameter('tour_rewrite_on_invalidation').value
        )
        self.tour_max_rewrites = int(
            self.get_parameter('tour_max_rewrites').value
        )

        self.residual_enable = bool(
            self.get_parameter('residual_enable').value
        )
        self.residual_activation_cycles = int(
            self.get_parameter('residual_activation_cycles').value
        )
        self.residual_params = ResidualParams(
            activation_cycles=self.residual_activation_cycles,
            min_cluster_cells=int(
                self.get_parameter('residual_min_cluster_cells').value
            ),
            min_density=float(
                self.get_parameter('residual_min_density').value
            ),
            min_unknown_frac=float(
                self.get_parameter('residual_min_unknown_frac').value
            ),
            structure_radius=float(
                self.get_parameter('residual_structure_radius').value
            ),
            min_structure_columns=int(
                self.get_parameter('residual_min_structure_columns').value
            ),
            ring_radius=float(
                self.get_parameter('residual_ring_radius').value
            ),
            ring_min_empty_sectors=int(
                self.get_parameter('residual_ring_min_empty_sectors').value
            ),
            viewpoint_search_radius=float(
                self.get_parameter('residual_viewpoint_search_radius').value
            ),
            min_clearance=self.min_clearance,
            max_candidates=int(
                self.get_parameter('residual_max_candidates').value
            ),
            dedupe_radius=float(
                self.get_parameter('residual_dedupe_radius').value
            ),
            score_scale=float(
                self.get_parameter('residual_score_scale').value
            ),
            blacklist_radius=float(
                self.get_parameter('residual_blacklist_radius').value
            ),
            coverage_hole_enable=bool(
                self.get_parameter('residual_coverage_hole_enable').value
            ),
        )
        self.residual_max_attempts = int(
            self.get_parameter('residual_max_attempts').value
        )

        self.convergence_params = ConvergenceParams(
            enabled=bool(self.get_parameter('convergence_enable').value),
            cycles_required=int(
                self.get_parameter('convergence_cycles_required').value
            ),
            residual_gain_threshold=float(
                self.get_parameter('convergence_residual_gain_threshold').value
            ),
            voxel_growth_epsilon=int(
                self.get_parameter('convergence_voxel_growth_epsilon').value
            ),
            voxel_growth_rate=float(
                self.get_parameter('convergence_voxel_growth_rate').value
            ),
            require_3d=bool(
                self.get_parameter('convergence_require_3d').value
            ),
            fallback_timeout=float(
                self.get_parameter('fallback_convergence_timeout').value
            ),
        )
        self.convergence_stop_dispatch = bool(
            self.get_parameter('convergence_stop_dispatch').value
        )
        self.convergence = ConvergenceTracker(self.convergence_params)

        # Blacklists / failure bookkeeping
        self.blacklisted_goals: List[Tuple[float, float]] = []
        self.global_failed_goals: Dict[Tuple[float, float], int] = {}
        self.residual_blacklist: List[Tuple[float, float]] = []
        self.residual_attempts: Dict[Tuple[float, float], int] = {}
        self._local_seen: Dict[Tuple[float, float], float] = {}

        # 3D index / cloud state
        self.latest_map: Optional[OccupancyGrid] = None
        self._latest_cloud: Optional[PointCloud2] = None
        self._last_map_time: float = -1e9
        self._last_cloud_time: float = -1e9
        self._last_voxel_build_time: float = -1e9
        self.voxel_index: Optional[VoxelColumnIndex] = None
        self._voxel_prev_keys: Optional[set] = None
        self._voxel_new_columns: int = 0
        self._voxel_total_columns: int = 0
        self._voxel_build_count: int = 0
        self._cloud_warned: bool = False
        self._cloud_frame_warned: bool = False
        self._voxel_error_log_time: float = -1e9

        self._prev_map_extent: Optional[Tuple[float, float, float, float]] = None
        self._skip_invalidation_once: bool = False
        self._map_loss_warned: bool = False
        self.frontier_empty_cycles: int = 0
        self._last_residual_count: int = -1
        self._last_residual_log_time: float = -1e9
        self._converged_logged: bool = False
        self._last_no_frontier_log_time: float = -1e9
        self._last_cycle_time: float = -1e9
        self._through_unready_warned: set = set()
        self._tf_checked: set = set()
        self.cycle_lock = threading.Lock()
        # Serializes robot goal/tracker state mutations between the timer
        # thread and action callbacks.  Reentrant because dispatch/reset and
        # the callbacks call each other.
        self._state_lock = threading.RLock()

        # ------------------------------------------------------------------
        # Robot trackers and ROS interfaces
        # ------------------------------------------------------------------
        self.robots: Dict[str, RobotTracker] = {}
        for ns in namespaces:
            clean_ns = ns.strip('/')
            init_param_name = f'{clean_ns}_init_pose'
            if self.has_parameter(init_param_name):
                init_pose = self.get_parameter(init_param_name).value
            elif clean_ns == 'robot1':
                init_pose = self.get_parameter('robot1_init_pose').value
            elif clean_ns == 'robot2':
                init_pose = self.get_parameter('robot2_init_pose').value
            else:
                init_pose = [0.0, 0.0, 0.0]
            robot = RobotTracker(clean_ns, init_pose)
            robot.use_through_poses = self.use_through_poses
            self.robots[clean_ns] = robot

        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        for ns, robot in self.robots.items():
            action_topic = f'/{ns}/navigate_to_pose'
            self.get_logger().info(f'Connecting ActionClient to {action_topic} ...')
            robot.action_client = ActionClient(
                self,
                NavigateToPose,
                action_topic,
                callback_group=self.cb_group,
            )
            robot.through_client = ActionClient(
                self,
                NavigateThroughPoses,
                f'/{ns}/navigate_through_poses',
                callback_group=self.cb_group,
            )

            # One-shot startup readiness probe.  Nav2 is normally already up
            # when the coordinator starts; if not, degrade this robot to
            # single-point goals instead of repeatedly burning wait time.
            if self.use_through_poses and self.action_wait_sec > 0.0:
                if not robot.through_client.wait_for_server(
                    timeout_sec=self.action_wait_sec
                ):
                    robot.use_through_poses = False
                    self.get_logger().warning(
                        f'[{ns}] navigate_through_poses not available after '
                        f'{self.action_wait_sec:.1f}s; using NavigateToPose.'
                    )

            # Do NOT publish on `goal_pose`: Nav2's NavigateToPoseNavigator
            # subscribes to that topic (the RViz 2D-goal interface) and would
            # treat this world-frame visualization message as a new navigation
            # request, causing TF_ERROR 9002 or rejecting goals.
            robot.goal_pub = self.create_publisher(
                PoseStamped, f'/{ns}/exploration_goal', 10
            )
            robot.plan_pub = self.create_publisher(
                Path, f'/{ns}/plan_world', 10
            )
            robot.plan_sub = self.create_subscription(
                Path,
                f'/{ns}/plan',
                lambda msg, r=robot: self.plan_callback(msg, r),
                10,
                callback_group=self.cb_group,
            )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1,
            callback_group=self.cb_group,
        )

        self.cloud_sub = None
        if self.use_3d_gain:
            cloud_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.cloud_sub = self.create_subscription(
                PointCloud2,
                self.octomap_cloud_topic,
                self.cloud_callback,
                cloud_qos,
                callback_group=self.cb_group,
            )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/exploration/markers', 10
        )

        timer_period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(
            timer_period, self.coordination_cycle, callback_group=self.cb_group
        )

        self.get_logger().info(
            f'MultiRobotCoordinator initialized for robots: '
            f'{list(self.robots.keys())} at {self.rate_hz} Hz. '
            f'3D gain={"on" if self.use_3d_gain else "off"} '
            f'local_scheduling={"on" if self.local_scheduling_enable else "off"}'
        )

    # ------------------------------------------------------------------
    # Time / map / cloud inputs
    # ------------------------------------------------------------------
    def get_sim_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self._last_map_time = self.get_sim_time_sec()

    def cloud_callback(self, msg: PointCloud2):
        self._latest_cloud = msg
        now = self.get_sim_time_sec()
        self._last_cloud_time = now if now > 0.0 else 0.0
        self._cloud_warned = False

    def _map_valid(self, now: float) -> bool:
        if self.latest_map is None:
            return False
        if self._last_map_time <= 0.0:
            return True
        return (now - self._last_map_time) <= self.map_loss_timeout

    def _cloud_age(self, now: float) -> float:
        if self._latest_cloud is None:
            return float('inf')
        if self._last_cloud_time <= 0.0:
            return 0.0
        return max(0.0, now - self._last_cloud_time)

    def _build_occupancy_view(self) -> Optional[OccupancyView]:
        if self.latest_map is None:
            return None
        try:
            return OccupancyView.from_grid(self.latest_map)
        except Exception as exc:  # malformed map should not kill the node
            self.get_logger().error(f'Failed to build occupancy view: {exc}')
            return None

    def _check_map_jump(self, view: OccupancyView) -> bool:
        extent = (
            view.origin_x,
            view.origin_y,
            view.width * view.resolution,
            view.height * view.resolution,
        )
        jump = False
        if self._prev_map_extent is not None:
            px, py, pw, ph = self._prev_map_extent
            x, y, w, h = extent
            jump = (
                abs(x - px) > 0.30 * max(pw, 0.5)
                or abs(y - py) > 0.30 * max(ph, 0.5)
                or abs(w - pw) > 0.30 * max(pw, 0.5)
                or abs(h - ph) > 0.30 * max(ph, 0.5)
            )
        self._prev_map_extent = extent
        if jump:
            self.convergence.reset('map jump')
            self._skip_invalidation_once = True
            self.voxel_index = None
            self._voxel_prev_keys = None
            self._voxel_new_columns = 0
            self._voxel_total_columns = 0
            self._last_voxel_build_time = -1e9
            self.get_logger().warning(
                'Merged map extent changed by >30%; resetting 3D index and convergence.'
            )
        return jump

    # ------------------------------------------------------------------
    # Pose / TF
    # ------------------------------------------------------------------
    def update_robot_poses(self):
        """Update each robot's pose in 'world' using prefixed TF only."""
        for ns, robot in self.robots.items():
            found_tf = False
            for base_frame in [f'{ns}/base_footprint', f'{ns}/base_link']:
                try:
                    t = self.tf_buffer.lookup_transform('world', base_frame, Time())
                    nx = t.transform.translation.x
                    ny = t.transform.translation.y
                    nyaw = quaternion_to_yaw(t.transform.rotation)
                    self._track_pose_jump(robot, nx, ny)
                    robot.world_x = nx
                    robot.world_y = ny
                    robot.world_yaw = nyaw
                    found_tf = True
                    break
                except Exception:
                    continue

            if found_tf:
                continue

            # Fallback: local map -> base, projected through the analytical
            # initial-pose transform.  Never use the unprefixed global 'map'
            # frame here: in the coordinator's tree it is an identity alias of
            # world and would produce world coordinates.
            for base_frame in [f'{ns}/base_footprint', f'{ns}/base_link']:
                try:
                    t = self.tf_buffer.lookup_transform(
                        f'{ns}/map', base_frame, Time()
                    )
                    local_x = t.transform.translation.x
                    local_y = t.transform.translation.y
                    local_yaw = quaternion_to_yaw(t.transform.rotation)
                    c = math.cos(robot.init_yaw)
                    s = math.sin(robot.init_yaw)
                    nx = robot.init_x + local_x * c - local_y * s
                    ny = robot.init_y + local_x * s + local_y * c
                    nyaw = robot.init_yaw + local_yaw
                    self._track_pose_jump(robot, nx, ny)
                    robot.world_x = nx
                    robot.world_y = ny
                    robot.world_yaw = nyaw
                    break
                except Exception:
                    continue

    def _track_pose_jump(self, robot: RobotTracker, x: float, y: float):
        if robot.prev_world_x is not None:
            dt = max(1e-3, 1.0 / max(self.rate_hz, 0.1))
            jump = math.hypot(x - robot.prev_world_x, y - robot.prev_world_y)
            # Normal motion at <=0.6 m/s is well below this threshold; a
            # relocalization jump is typically several metres in one cycle.
            if jump > 0.5 + 0.8 * dt:
                self.convergence.reset('pose jump')
        robot.prev_world_x = x
        robot.prev_world_y = y

    def _world_to_local_transform(
        self, robot: RobotTracker
    ) -> Tuple[float, float, float]:
        """Return (tx, ty, tyaw) with p_local = R(tyaw) p_world + t."""
        try:
            t = self.tf_buffer.lookup_transform(
                f'{robot.namespace}/map',
                'world',
                Time(),
                timeout=Duration(seconds=self.tf_wait_sec),
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                quaternion_to_yaw(t.transform.rotation),
            )
        except Exception:
            c = math.cos(robot.init_yaw)
            s = math.sin(robot.init_yaw)
            tx = -(c * robot.init_x + s * robot.init_y)
            ty = s * robot.init_x - c * robot.init_y
            return tx, ty, -robot.init_yaw

    def _local_to_world_transform(
        self, robot: RobotTracker
    ) -> Tuple[float, float, float]:
        """Return (tx, ty, tyaw) with p_world = R(tyaw) p_local + t."""
        try:
            t = self.tf_buffer.lookup_transform(
                'world', f'{robot.namespace}/map', Time()
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                quaternion_to_yaw(t.transform.rotation),
            )
        except Exception:
            return robot.init_x, robot.init_y, robot.init_yaw

    def world_to_robot_local(
        self, robot: RobotTracker, xw: float, yw: float
    ) -> Tuple[float, float]:
        tx, ty, tyaw = self._world_to_local_transform(robot)
        c = math.cos(tyaw)
        s = math.sin(tyaw)
        return tx + xw * c - yw * s, ty + xw * s + yw * c

    def robot_local_to_world(
        self,
        robot: RobotTracker,
        xl: float,
        yl: float,
        yawl: float = 0.0,
        transform: Optional[Tuple[float, float, float]] = None,
    ) -> Tuple[float, float, float]:
        if transform is None:
            transform = self._local_to_world_transform(robot)
        tx, ty, tyaw = transform
        c = math.cos(tyaw)
        s = math.sin(tyaw)
        return (
            tx + xl * c - yl * s,
            ty + xl * s + yl * c,
            wrap_angle(tyaw + yawl),
        )

    def _check_tf_consistency(self):
        """One-shot consistency check between world TF and the analytical init pose."""
        if self.tf_buffer is None:
            return
        for ns, robot in self.robots.items():
            if ns in self._tf_checked:
                continue
            try:
                tw = self.tf_buffer.lookup_transform(
                    'world', f'{ns}/base_footprint', Time()
                )
                tl = self.tf_buffer.lookup_transform(
                    f'{ns}/map', f'{ns}/base_footprint', Time()
                )
            except Exception:
                continue
            local_x = tl.transform.translation.x
            local_y = tl.transform.translation.y
            c = math.cos(robot.init_yaw)
            s = math.sin(robot.init_yaw)
            px = robot.init_x + local_x * c - local_y * s
            py = robot.init_y + local_x * s + local_y * c
            err = math.hypot(px - tw.transform.translation.x,
                             py - tw.transform.translation.y)
            self._tf_checked.add(ns)
            if err > 0.5:
                self.get_logger().warning(
                    f'[{ns}] TF inconsistency: analytical init pose vs world TF '
                    f'differs by {err:.2f}m. Disabling through-poses for this robot.'
                )
                robot.use_through_poses = False

    def plan_callback(self, msg: Path, robot: RobotTracker):
        """Transform a robot-local path to world for RViz."""
        if not msg.poses or robot.plan_pub is None:
            return

        transform = self._local_to_world_transform(robot)
        world_path = Path()
        world_path.header.stamp = self.get_clock().now().to_msg()
        world_path.header.frame_id = 'world'

        for p in msg.poses:
            xw, yw, yaww = self.robot_local_to_world(
                robot,
                p.pose.position.x,
                p.pose.position.y,
                quaternion_to_yaw(p.pose.orientation),
                transform=transform,
            )
            pw = PoseStamped()
            pw.header.stamp = world_path.header.stamp
            pw.header.frame_id = 'world'
            pw.pose.position.x = xw
            pw.pose.position.y = yw
            pw.pose.position.z = p.pose.position.z
            pw.pose.orientation = yaw_to_quaternion(yaww)
            world_path.poses.append(pw)

        robot.plan_pub.publish(world_path)

    # ------------------------------------------------------------------
    # 2D frontier extraction
    # ------------------------------------------------------------------
    def split_cluster_if_large(
        self,
        cluster_cells: List[Tuple[int, int]],
        res: float,
        min_size: int = 8,
    ) -> List[List[Tuple[int, int]]]:
        if len(cluster_cells) < min_size * 2:
            return [cluster_cells]

        coords = np.array(cluster_cells)
        span = np.ptp(coords, axis=0)
        spread_m = math.hypot(span[0], span[1]) * res

        if spread_m < 2.0:
            return [cluster_cells]

        var = np.var(coords, axis=0)
        split_dim = 1 if var[1] > var[0] else 0
        median_val = np.median(coords[:, split_dim])

        part1 = [c for c in cluster_cells if c[split_dim] <= median_val]
        part2 = [c for c in cluster_cells if c[split_dim] > median_val]

        if len(part1) < min_size or len(part2) < min_size:
            return [cluster_cells]

        results = []
        for part in (part1, part2):
            p_coords = np.array(part)
            p_span = np.ptp(p_coords, axis=0)
            p_spread = math.hypot(p_span[0], p_span[1]) * res
            if p_spread >= 2.5 and len(part) >= min_size * 2:
                results.extend(
                    self.split_cluster_if_large(part, res, min_size)
                )
            else:
                results.append(part)
        return results

    def find_safe_goal(
        self,
        cluster_cells: List[Tuple[int, int]],
        is_free: np.ndarray,
        obstacle_dt: np.ndarray,
        res: float,
    ) -> Optional[Tuple[int, int]]:
        """Retract a frontier centroid to a cell with sufficient clearance."""
        height, width = is_free.shape
        min_clearance_cells = self.min_clearance / res
        max_retract_cells = int(1.5 / res)

        mean_r = sum(c[0] for c in cluster_cells) / len(cluster_cells)
        mean_c = sum(c[1] for c in cluster_cells) / len(cluster_cells)

        safe_cells = [
            (r, c)
            for (r, c) in cluster_cells
            if obstacle_dt[r, c] >= min_clearance_cells
        ]
        if safe_cells:
            return min(
                safe_cells,
                key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2,
            )

        sorted_cluster = sorted(
            cluster_cells,
            key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2,
        )
        q = deque()
        visited = set()
        for r, c in sorted_cluster:
            q.append((r, c, 0))
            visited.add((r, c))

        while q:
            r, c, steps = q.popleft()
            if steps > max_retract_cells:
                break
            if obstacle_dt[r, c] >= min_clearance_cells:
                return (r, c)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if (
                    0 <= nr < height
                    and 0 <= nc < width
                    and (nr, nc) not in visited
                ):
                    visited.add((nr, nc))
                    if is_free[nr, nc]:
                        q.append((nr, nc, steps + 1))
        return None

    def _frontier_yaw(
        self, occ: OccupancyView, safe_r: int, safe_c: int
    ) -> float:
        r0 = max(0, safe_r - 2)
        r1 = min(occ.height, safe_r + 3)
        c0 = max(0, safe_c - 2)
        c1 = min(occ.width, safe_c + 3)
        sub = occ.is_unknown[r0:r1, c0:c1]
        if not sub.any():
            return 0.0
        rr, cc = np.nonzero(sub)
        gr = rr + r0
        gc = cc + c0
        k = int(np.argmin((gr - safe_r) ** 2 + (gc - safe_c) ** 2))
        ux = occ.origin_x + (gc[k] + 0.5) * occ.resolution
        uy = occ.origin_y + (gr[k] + 0.5) * occ.resolution
        wx = occ.origin_x + (safe_c + 0.5) * occ.resolution
        wy = occ.origin_y + (safe_r + 0.5) * occ.resolution
        return math.atan2(uy - wy, ux - wx)

    def extract_frontiers(self, occ: OccupancyView) -> List[FrontierCluster]:
        """Vectorized 2D frontier extraction with safe-goal retraction."""
        height, width = occ.height, occ.width
        res = occ.resolution

        unknown_padded = np.pad(
            occ.is_unknown, 1, mode='constant', constant_values=False
        )
        lethal_padded = np.pad(
            occ.is_lethal, 1, mode='constant', constant_values=False
        )

        has_unknown_neighbor = np.zeros((height, width), dtype=bool)
        has_lethal_neighbor = np.zeros((height, width), dtype=bool)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                has_unknown_neighbor |= unknown_padded[
                    1 + dr : height + 1 + dr, 1 + dc : width + 1 + dc
                ]
                has_lethal_neighbor |= lethal_padded[
                    1 + dr : height + 1 + dr, 1 + dc : width + 1 + dc
                ]

        frontier_mask = (
            occ.is_free & has_unknown_neighbor & (~has_lethal_neighbor)
        )

        frontier_coords = np.argwhere(frontier_mask)
        visited = np.zeros((height, width), dtype=bool)
        raw_clusters: List[List[Tuple[int, int]]] = []

        for r, c in frontier_coords:
            if visited[r, c]:
                continue
            cluster_cells = []
            q = deque([(r, c)])
            visited[r, c] = True
            while q:
                curr_r, curr_c = q.popleft()
                cluster_cells.append((curr_r, curr_c))
                r_start = max(0, curr_r - 1)
                r_end = min(height, curr_r + 2)
                c_start = max(0, curr_c - 1)
                c_end = min(width, curr_c + 2)
                for nr in range(r_start, r_end):
                    for nc in range(c_start, c_end):
                        if frontier_mask[nr, nc] and not visited[nr, nc]:
                            visited[nr, nc] = True
                            q.append((nr, nc))

            if len(cluster_cells) >= self.min_frontier_size:
                raw_clusters.extend(
                    self.split_cluster_if_large(
                        cluster_cells, res, self.min_frontier_size
                    )
                )

        clusters: List[FrontierCluster] = []
        for sub_cells in raw_clusters:
            safe_cell = self.find_safe_goal(
                sub_cells, occ.is_free, occ.obstacle_dt, res
            )
            if safe_cell is None:
                continue
            safe_r, safe_c = safe_cell
            wx = occ.origin_x + (safe_c + 0.5) * res
            wy = occ.origin_y + (safe_r + 0.5) * res

            if any(
                math.hypot(wx - bx, wy - by) < self.blacklist_radius
                for bx, by in self.blacklisted_goals
            ):
                continue
            if any(
                math.hypot(wx - c.world_x, wy - c.world_y) < 0.4
                for c in clusters
            ):
                continue

            clusters.append(
                FrontierCluster(
                    wx,
                    wy,
                    len(sub_cells),
                    safe_r,
                    safe_c,
                    yaw_world=self._frontier_yaw(occ, safe_r, safe_c),
                    gain_2d=min(len(sub_cells), 1500) * res,
                    component_label=int(occ.free_labels[safe_r, safe_c]),
                )
            )
        return clusters

    # ------------------------------------------------------------------
    # 3D index and scoring
    # ------------------------------------------------------------------
    def maybe_update_voxel_index(self, now: float, occ: OccupancyView):
        if not self.use_3d_gain or self._latest_cloud is None:
            return
        if self._cloud_age(now) > self.voxel_cloud_warn_timeout:
            # Never rebuild from a stale cloud: doing so would reset the
            # column-diff baseline every cycle and block convergence.
            return
        if now - self._last_voxel_build_time < self.voxel_update_period:
            return
        if point_cloud2 is None:
            if not self._cloud_warned:
                self.get_logger().warning(
                    'sensor_msgs_py.point_cloud2 unavailable; 3D gain disabled.'
                )
                self._cloud_warned = True
            return

        frame = (self._latest_cloud.header.frame_id or '').lstrip('/')
        if frame not in ('world', ''):
            if not self._cloud_frame_warned:
                self.get_logger().warning(
                    f'OctoMap cloud frame "{frame}" is not "world"; ignoring 3D gain.'
                )
                self._cloud_frame_warned = True
            return

        t0 = time.perf_counter()
        try:
            pts = point_cloud2.read_points_numpy(
                self._latest_cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
            pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        except Exception as exc:
            if now - self._voxel_error_log_time > 10.0:
                self.get_logger().warning(f'Failed to decode OctoMap cloud: {exc}')
                self._voxel_error_log_time = now
            return

        try:
            new_index = VoxelColumnIndex.build(pts, occ, self.voxel_params)
        except Exception as exc:
            if now - self._voxel_error_log_time > 10.0:
                self.get_logger().warning(f'Failed to build voxel index: {exc}')
                self._voxel_error_log_time = now
            return

        new_columns = new_index.new_column_count(self._voxel_prev_keys)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        # Atomic swap: readers only ever see a complete index object.
        self.voxel_index = new_index
        self._voxel_prev_keys = new_index.column_keys
        self._voxel_new_columns = new_columns
        self._voxel_total_columns = new_index.total_columns
        self._last_voxel_build_time = now
        self._voxel_build_count += 1

        if (
            self._voxel_build_count == 1
            or self._voxel_build_count % 5 == 0
            or new_columns > self.convergence_params.voxel_growth_epsilon
        ):
            self.get_logger().info(
                f'[3D] voxel index: {new_index.total_columns} columns '
                f'(+{new_columns}), {len(pts)} points, {elapsed_ms:.1f} ms'
            )
        if elapsed_ms > self.cycle_time_budget_ms:
            self.get_logger().warning(
                f'[3D] voxel index build {elapsed_ms:.1f} ms exceeds '
                f'cycle_time_budget_ms={self.cycle_time_budget_ms:.1f} ms; '
                f'skipping 3D gain scoring this cycle.'
            )

    def _check_cloud_staleness(self, now: float):
        if not self.use_3d_gain:
            return
        age = self._cloud_age(now)
        if age > self.voxel_cloud_warn_timeout:
            if not self._cloud_warned:
                age_text = (
                    f'for {age:.1f}s'
                    if math.isfinite(age)
                    else '(no message received yet)'
                )
                self.get_logger().warning(
                    f'[3D] No OctoMap cloud on {self.octomap_cloud_topic} '
                    f'{age_text}; degrading to 2D gain.'
                )
                self._cloud_warned = True
            self.voxel_index = None
            self._voxel_prev_keys = None
            self._voxel_new_columns = 0
            self._voxel_total_columns = 0
            self._last_voxel_build_time = -1e9

    def score_candidates(
        self, clusters: List[FrontierCluster], skip_3d: bool
    ):
        for c in clusters:
            g3d = 0.0
            if (
                not skip_3d
                and self.use_3d_gain
                and self.voxel_index is not None
            ):
                stats = self.voxel_index.window_stats(
                    c.world_x,
                    c.world_y,
                    self.gain_3d_window_radius,
                    self.info_gain_params,
                )
                g3d = stats.g3d
            c.gain_2d = min(c.size, 1500) * (
                self.latest_map.info.resolution
                if self.latest_map is not None
                else 0.05
            )
            c.gain_3d = g3d
            c.total_gain = self.gain_scale * c.gain_2d + self.gain_3d_scale * g3d

    def _index_available(self, now: float) -> bool:
        return (
            self.use_3d_gain
            and self.voxel_index is not None
            and self._last_voxel_build_time > 0.0
            and (now - self._last_voxel_build_time)
            <= max(3.0, 3.0 * self.voxel_update_period)
        )

    # ------------------------------------------------------------------
    # Main cycle
    # ------------------------------------------------------------------
    def coordination_cycle(self):
        if not self.cycle_lock.acquire(blocking=False):
            return
        try:
            self._coordination_cycle_impl()
        finally:
            self.cycle_lock.release()

    def _coordination_cycle_impl(self):
        if self.latest_map is None:
            return

        now = self.get_sim_time_sec()
        if now == 0.0:
            # Sim time has not started yet; nothing is safe to time out.
            return

        cycle_start = time.perf_counter()
        self._last_cycle_time = now

        self.update_robot_poses()
        self._check_tf_consistency()

        occ = self._build_occupancy_view()
        if occ is None:
            return
        self._check_map_jump(occ)
        self._check_cloud_staleness(now)
        self.maybe_update_voxel_index(now, occ)

        self.check_action_timeouts(now)
        self.check_stalls(now)
        self.check_goal_invalidation(occ, now)

        # 2D frontiers + 3D gain.
        clusters = self.extract_frontiers(occ)
        if not clusters:
            self.frontier_empty_cycles += 1
        else:
            self.frontier_empty_cycles = 0
        skip_3d = (
            (time.perf_counter() - cycle_start) * 1000.0
            > self.cycle_time_budget_ms
        )
        self.score_candidates(clusters, skip_3d)

        # Residual blind spots once the 2D frontier is quiet.
        residual: List[object] = []
        residual_raw_gain = 0.0
        if (
            self.residual_enable
            and self.voxel_index is not None
            and self.frontier_empty_cycles >= self.residual_activation_cycles
        ):
            residual, residual_raw_gain = detect_residuals(
                occ,
                self.voxel_index,
                self.residual_params,
                blacklist=self.residual_blacklist,
                attempts=self.residual_attempts,
                max_attempts=self.residual_max_attempts,
            )
            for cand in residual:
                cand.yaw_world = cand.yaw
                if not hasattr(cand, 'kind'):
                    cand.kind = 'residual'
            if residual:
                if (
                    len(residual) != self._last_residual_count
                    or now - self._last_residual_log_time > 20.0
                ):
                    self.get_logger().info(
                        f'[Residual] {len(residual)} blind-spot candidates '
                        f'(raw gain {residual_raw_gain:.2f})'
                    )
                    self._last_residual_count = len(residual)
                    self._last_residual_log_time = now
            elif self._last_residual_count > 0:
                self._last_residual_count = 0

        candidates = list(clusters) + list(residual)

        if self.local_scheduling_enable:
            # Exclude anything near an active waypoint (all remaining tour
            # points, not just the current one).
            active_waypoints = self._all_active_waypoints()
            available = exclude_near_waypoints(
                candidates, active_waypoints, self.exclusion_radius
            )
            if not available and candidates and active_waypoints:
                relaxed = exclude_near_waypoints(
                    candidates, active_waypoints, 0.8
                )
                if relaxed:
                    available = relaxed
                    self.get_logger().info(
                        f'Anti-starvation: relaxed exclusion_radius to 0.8m, '
                        f'found {len(available)} candidates.'
                    )
        else:
            # Legacy path: preserve the original 2D auction behaviour exactly.
            # HEAD filtered candidates within exclusion_radius of each
            # NAVIGATING robot's current goal (one goal per robot), with the
            # same 0.8 m anti-starvation fallback.
            active_goals = [
                r.current_goal_world
                for r in self.robots.values()
                if r.state == 'NAVIGATING' and r.current_goal_world is not None
            ]
            available = [
                c for c in candidates
                if not any(
                    math.hypot(c.world_x - gx, c.world_y - gy)
                    < self.exclusion_radius
                    for gx, gy in active_goals
                )
            ]
            if not available and candidates and active_goals:
                relaxed_radius = 0.8
                available = [
                    c for c in candidates
                    if not any(
                        math.hypot(c.world_x - gx, c.world_y - gy)
                        < relaxed_radius
                        for gx, gy in active_goals
                    )
                ]
                if available:
                    self.get_logger().info(
                        f'Anti-starvation: relaxed exclusion_radius to '
                        f'{relaxed_radius}m, found {len(available)} candidates.'
                    )

        idle_robots = [
            r for r in self.robots.values() if r.state == 'IDLE'
        ]
        dispatch_allowed = not (
            self.convergence.converged and self.convergence_stop_dispatch
        )

        ownership = []
        dispatched = set()
        if dispatch_allowed and available:
            if self.local_scheduling_enable:
                agents = [
                    self._agent_state(r, occ) for r in self.robots.values()
                ]
                ownership = assign_ownership(
                    agents, available, self.local_radius
                )

                local_by_robot: Dict[str, List[object]] = {}
                for cand, own in zip(available, ownership):
                    if own.pool == 'local' and own.owner is not None:
                        local_by_robot.setdefault(own.owner, []).append(cand)

                for robot in idle_robots:
                    pool = local_by_robot.get(robot.namespace, [])
                    if not pool:
                        continue
                    robot_component = occ.component_at_world(
                        robot.world_x, robot.world_y
                    )
                    if robot_component <= 0:
                        continue
                    pool = [
                        c
                        for c in pool
                        if getattr(c, 'component_label', -1)
                        in (-1, robot_component)
                    ]
                    tour = build_local_tour(
                        pool, (robot.world_x, robot.world_y), self.tour_params
                    )
                    if not tour:
                        continue
                    if self.dispatch_tour(robot, tour):
                        dispatched.add(robot.namespace)

                global_pool = self._build_global_pool(
                    available, ownership, dispatched, now
                )

                remaining_idle = [
                    r for r in idle_robots if r.namespace not in dispatched
                ]
                if remaining_idle and global_pool:
                    raw_global_pool = list(global_pool)
                    global_pool = exclude_near_waypoints(
                        global_pool,
                        self._all_active_waypoints(),
                        self.exclusion_radius,
                    )
                    if not global_pool:
                        # Anti-starvation: retry only the Global pool at the
                        # relaxed radius; never leak another robot's Local pool.
                        global_pool = exclude_near_waypoints(
                            raw_global_pool,
                            self._all_active_waypoints(),
                            0.8,
                        )
                    if global_pool:
                        if self.global_roi_enable:
                            self.run_global_auction(
                                remaining_idle, global_pool, occ
                            )
                        else:
                            self.run_auction(
                                remaining_idle,
                                global_pool,
                                occ.resolution,
                            )
            else:
                self.run_auction(
                    idle_robots, available, occ.resolution
                )

        # Convergence tracking.
        map_valid = self._map_valid(now)
        if not map_valid and self.convergence.converged:
            self.convergence.reset('map lost')
            if not self._map_loss_warned:
                self.get_logger().warning(
                    f'No /map for more than {self.map_loss_timeout:.1f}s; '
                    f'resetting convergence.'
                )
                self._map_loss_warned = True
        elif map_valid:
            self._map_loss_warned = False

        if self.convergence.converged and clusters:
            self.convergence.reset('new frontier')

        index_available = self._index_available(now)
        state = self.convergence.update(
            now,
            map_valid=map_valid,
            frontier_count=len(clusters),
            all_idle=all(r.state == 'IDLE' for r in self.robots.values()),
            residual_count=len(residual),
            residual_raw_gain=residual_raw_gain,
            index_available=index_available,
            new_columns=self._voxel_new_columns if index_available else 0,
            total_columns=self._voxel_total_columns if index_available else 0,
        )
        if state.converged and not self._converged_logged:
            self.get_logger().info(
                f'Exploration converged: 2D frontiers empty, residual gain '
                f'{residual_raw_gain:.2f} < '
                f'{self.convergence_params.residual_gain_threshold:.2f}, '
                f'voxel growth {self._voxel_new_columns} < '
                f'{self.convergence_params.voxel_growth_epsilon} for '
                f'{self.convergence_params.cycles_required} cycles'
            )
            self._converged_logged = True
        elif not state.converged:
            self._converged_logged = False

        # Markers (also during navigation, so the dashboard stays live).
        self.publish_markers(
            occ, candidates, available, ownership, residual, dispatched
        )

        if (
            not available
            and not dispatched
            and all(r.state == 'IDLE' for r in self.robots.values())
            and now - self._last_no_frontier_log_time > 10.0
        ):
            self.get_logger().info(
                'No remaining reachable frontiers. Exploration complete or paused.'
            )
            self._last_no_frontier_log_time = now

    # ------------------------------------------------------------------
    # Candidate ownership / scheduling
    # ------------------------------------------------------------------
    def _agent_state(
        self, robot: RobotTracker, occ: OccupancyView
    ) -> AgentState:
        comp = occ.component_at_world(robot.world_x, robot.world_y)
        return AgentState(
            namespace=robot.namespace,
            x=robot.world_x,
            y=robot.world_y,
            yaw=robot.world_yaw,
            idle=robot.state == 'IDLE',
            component_label=comp,
        )

    def _component_ok(
        self, robot: RobotTracker, cand, occ: OccupancyView
    ) -> bool:
        robot_comp = occ.component_at_world(robot.world_x, robot.world_y)
        cand_comp = int(getattr(cand, 'component_label', -1))
        if robot_comp > 0 and cand_comp > 0 and robot_comp != cand_comp:
            return False
        return True

    def _build_global_pool(
        self,
        available: List[object],
        ownership: List,
        dispatched: set,
        now: float,
    ) -> List[object]:
        pool: List[object] = []
        selected_keys = set()
        for robot in self.robots.values():
            for cand in robot.tour_candidates:
                selected_keys.add(
                    (round(cand.world_x, 1), round(cand.world_y, 1))
                )

        for cand, own in zip(available, ownership):
            if own.pool == 'global':
                pool.append(cand)
                continue
            if not self.local_release_unselected:
                continue
            if own.owner in dispatched:
                continue
            key = (round(cand.world_x, 1), round(cand.world_y, 1))
            if key in selected_keys:
                continue
            first_seen = self._local_seen.setdefault(key, now)
            if now - first_seen < self.local_release_cooldown:
                continue
            # Lower the bid for released local leftovers so the owner keeps
            # priority if it becomes idle again.
            try:
                cand.bid_gain = (
                    cand.total_gain * self.local_release_score_penalty
                )
            except Exception:
                pass
            pool.append(cand)

        pool.sort(key=lambda c: float(getattr(c, 'total_gain', 0.0)), reverse=True)
        if self.global_max_candidates > 0:
            pool = pool[: self.global_max_candidates]
        return pool

    def _all_active_waypoints(self) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []
        for robot in self.robots.values():
            if robot.state not in ('NAVIGATING', 'DISPATCHING'):
                continue
            if robot.active_waypoints_world:
                start = max(0, robot.current_waypoint_index)
                points.extend(robot.active_waypoints_world[start:])
            elif robot.current_goal_world is not None:
                points.append(robot.current_goal_world)
        return points

    def calculate_utility(
        self, robot: RobotTracker, cluster, res: float
    ) -> float:
        """Legacy 2D utility used when local scheduling / ROI is disabled."""
        dx = cluster.world_x - robot.world_x
        dy = cluster.world_y - robot.world_y
        dist = math.hypot(dx, dy)

        if any(
            math.hypot(
                cluster.world_x - r.world_x, cluster.world_y - r.world_y
            )
            < 0.35
            for r in self.robots.values()
        ):
            return -float('inf')

        heading_to_goal = math.atan2(dy, dx)
        angle_diff = abs(wrap_angle(heading_to_goal - robot.world_yaw))

        gain = float(getattr(cluster, 'total_gain', 0.0))
        if gain <= 0.0:
            capped_size = min(cluster.size, 1500)
            gain = self.gain_scale * capped_size * res

        dist_cost = dist
        if dist > 5.0:
            dist_cost += (dist - 5.0) * 2.5

        return (
            gain
            - self.potential_scale * dist_cost
            - self.orientation_scale * angle_diff
        )

    def run_auction(
        self,
        idle_robots: List[RobotTracker],
        clusters: List[object],
        res: float,
    ):
        """Legacy sequential single-item greedy auction."""
        remaining_robots = list(idle_robots)
        remaining_clusters = list(clusters)

        while remaining_robots and remaining_clusters:
            best_bid = -float('inf')
            best_match: Optional[Tuple[RobotTracker, object]] = None
            for robot in remaining_robots:
                for cluster in remaining_clusters:
                    util = self.calculate_utility(robot, cluster, res)
                    if util > best_bid:
                        best_bid = util
                        best_match = (robot, cluster)

            if best_match is None:
                break

            winner_robot, won_cluster = best_match
            self.get_logger().info(
                f'★ [Auction Winner] {winner_robot.namespace} won frontier at '
                f'({won_cluster.world_x:.2f}, {won_cluster.world_y:.2f}) '
                f'(size={won_cluster.size}, bid={best_bid:.2f})'
            )
            if not self.dispatch_goal(winner_robot, won_cluster):
                # This robot's action server is unavailable; let the other
                # idle robots bid for the same pool instead of aborting.
                remaining_robots.remove(winner_robot)
                continue
            remaining_robots.remove(winner_robot)

            before = list(remaining_clusters)
            next_clusters = [
                c
                for c in remaining_clusters
                if math.hypot(
                    c.world_x - won_cluster.world_x,
                    c.world_y - won_cluster.world_y,
                )
                >= self.exclusion_radius
            ]
            if not next_clusters and remaining_robots:
                next_clusters = [
                    c
                    for c in before
                    if math.hypot(
                        c.world_x - won_cluster.world_x,
                        c.world_y - won_cluster.world_y,
                    )
                    >= 0.8
                ]
            remaining_clusters = next_clusters

    def run_global_auction(
        self,
        idle_robots: List[RobotTracker],
        clusters: List[object],
        occ: OccupancyView,
    ):
        """ROI-based auction for strategic Global transfers."""
        remaining_robots = list(idle_robots)
        remaining_clusters = list(clusters)

        while remaining_robots and remaining_clusters:
            best_bid = -float('inf')
            best_roi = 0.0
            best_dist = 0.0
            best_match: Optional[Tuple[RobotTracker, object]] = None

            for robot in remaining_robots:
                agent = self._agent_state(robot, occ)
                for cand in remaining_clusters:
                    if not self._component_ok(robot, cand, occ):
                        continue
                    bid, roi, dist = global_bid(agent, cand, self.roi_params)
                    if roi < self.roi_params.min_roi:
                        continue
                    if (
                        self.roi_params.max_distance > 0.0
                        and dist > self.roi_params.max_distance
                    ):
                        continue
                    if bid > best_bid:
                        best_bid = bid
                        best_roi = roi
                        best_dist = dist
                        best_match = (robot, cand)

            if best_match is None:
                break

            winner_robot, won_cluster = best_match
            self.get_logger().info(
                f'★ [Auction Winner] {winner_robot.namespace} won frontier at '
                f'({won_cluster.world_x:.2f}, {won_cluster.world_y:.2f}) '
                f'(size={won_cluster.size}, bid={best_bid:.2f}) '
                f'[Global ROI {best_roi:.3f} @ {best_dist:.2f}m]'
            )
            if not self.dispatch_goal(winner_robot, won_cluster):
                # This robot's action server is unavailable; let the other
                # idle robots bid for the same pool instead of aborting.
                remaining_robots.remove(winner_robot)
                continue
            remaining_robots.remove(winner_robot)

            before = list(remaining_clusters)
            next_clusters = [
                c
                for c in remaining_clusters
                if math.hypot(
                    c.world_x - won_cluster.world_x,
                    c.world_y - won_cluster.world_y,
                )
                >= self.exclusion_radius
            ]
            if not next_clusters and remaining_robots:
                next_clusters = [
                    c
                    for c in before
                    if math.hypot(
                        c.world_x - won_cluster.world_x,
                        c.world_y - won_cluster.world_y,
                    )
                    >= 0.8
                ]
            remaining_clusters = next_clusters

    # ------------------------------------------------------------------
    # Action dispatch
    # ------------------------------------------------------------------
    def _action_server_ready(self, client: Optional[ActionClient]) -> bool:
        if client is None:
            return False
        try:
            if client.server_is_ready():
                return True
            return bool(client.wait_for_server(timeout_sec=0.0))
        except Exception:
            return False

    def _candidate_yaw(self, cand, robot: RobotTracker) -> float:
        yaw = getattr(cand, 'yaw_world', None)
        if yaw is None:
            yaw = getattr(cand, 'yaw', None)
        if yaw is None:
            yaw = math.atan2(
                cand.world_y - robot.world_y,
                cand.world_x - robot.world_x,
            )
        return float(yaw)

    def _residual_key_for(self, candidate) -> Optional[Tuple[float, float]]:
        if getattr(candidate, 'kind', 'frontier') != 'residual':
            return None
        # Key on the blind-spot cluster, matching exploration_3d's detector.
        cx = float(getattr(candidate, 'cluster_x', candidate.world_x))
        cy = float(getattr(candidate, 'cluster_y', candidate.world_y))
        return (round(cx, 1), round(cy, 1))

    def _publish_goal_pose(
        self,
        robot: RobotTracker,
        gx: float,
        gy: float,
        yaw_world: float,
    ):
        if robot.goal_pub is None:
            return
        gp = PoseStamped()
        gp.header.frame_id = 'world'
        gp.header.stamp = self.get_clock().now().to_msg()
        gp.pose.position.x = gx
        gp.pose.position.y = gy
        gp.pose.position.z = 0.05
        gp.pose.orientation = yaw_to_quaternion(yaw_world)
        robot.goal_pub.publish(gp)

    def dispatch_goal(self, robot: RobotTracker, cluster) -> bool:
        with self._state_lock:
            return self._dispatch_goal_locked(robot, cluster)

    def _dispatch_goal_locked(self, robot: RobotTracker, cluster) -> bool:
        """Send a single-point NavigateToPose action."""
        if not self._action_server_ready(robot.action_client):
            self.get_logger().warning(
                f'[{robot.namespace}] Action server not ready!'
            )
            return False

        xl, yl = self.world_to_robot_local(
            robot, cluster.world_x, cluster.world_y
        )
        yaw_world = self._candidate_yaw(cluster, robot)
        # Final orientation faces the frontier/unknown side (or the blind-spot
        # cluster for residual candidates), converted into the local map frame.
        heading_local = wrap_angle(
            yaw_world + self._world_to_local_transform(robot)[2]
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = xl
        goal_msg.pose.pose.position.y = yl
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(heading_local)

        now = self.get_sim_time_sec()
        # Invalidate the previous goal's callbacks before overwriting state.
        robot.goal_token += 1
        token = robot.goal_token
        robot.state = 'DISPATCHING'
        robot.active_waypoints_world = [
            (cluster.world_x, cluster.world_y)
        ]
        robot.tour_candidates = [cluster]
        robot.tour_poses_world = [
            (cluster.world_x, cluster.world_y, yaw_world)
        ]
        robot.current_waypoint_index = 0
        robot.current_goal_world = (cluster.world_x, cluster.world_y)
        robot.goal_start_time = now
        robot.tour_start_time = now
        robot.last_progress_time = now
        robot.last_distance_remaining = float('nan')
        robot.stall_distance_ref = float('inf')
        robot.stall_ref_x = robot.world_x
        robot.stall_ref_y = robot.world_y
        robot.tour_is_active = False
        robot.tour_uses_through_poses = False
        robot.rewrite_count = 0
        robot.current_residual_key = self._residual_key_for(cluster)

        self._publish_goal_pose(
            robot, cluster.world_x, cluster.world_y, yaw_world
        )
        self.get_logger().info(
            f'[{robot.namespace}] Sending Nav2 goal: '
            f'world=({cluster.world_x:.2f}, {cluster.world_y:.2f}) -> '
            f'local=({xl:.2f}, {yl:.2f})'
        )

        try:
            send_future = robot.action_client.send_goal_async(
                goal_msg, feedback_callback=None
            )
        except Exception as exc:
            self.get_logger().error(
                f'[{robot.namespace}] Failed to send goal: {exc}'
            )
            self.handle_goal_failure(
                robot, robot.current_goal_world, robot.current_residual_key
            )
            self._reset_robot_nav(robot)
            return False

        send_future.add_done_callback(
            lambda future, r=robot, t=token: self.goal_response_callback(
                future, r, t, 'to_pose'
            )
        )
        return True

    def dispatch_tour(self, robot: RobotTracker, tour: List[object]) -> bool:
        with self._state_lock:
            return self._dispatch_tour_locked(robot, tour)

    def _dispatch_tour_locked(self, robot: RobotTracker, tour: List[object]) -> bool:
        """Send one NavigateThroughPoses action for a near-field tour.

        Degrades to the first waypoint via NavigateToPose when through-poses
        is disabled, unavailable, or the tour has a single point.
        """
        if not tour:
            return False

        through_ready = self._action_server_ready(robot.through_client)
        use_through = (
            len(tour) >= 2
            and self.use_through_poses
            and robot.use_through_poses
            and through_ready
        )
        if len(tour) >= 2 and not use_through:
            if not through_ready and robot.namespace not in self._through_unready_warned:
                self.get_logger().warning(
                    f'[{robot.namespace}] navigate_through_poses server not ready; '
                    f'falling back to single-point goals.'
                )
                self._through_unready_warned.add(robot.namespace)
            return self.dispatch_goal(robot, tour[0])

        if not use_through:
            return self.dispatch_goal(robot, tour[0])

        tx, ty, tyaw = self._world_to_local_transform(robot)
        c = math.cos(tyaw)
        s = math.sin(tyaw)

        local_xy = []
        for cand in tour:
            local_xy.append(
                (
                    tx + cand.world_x * c - cand.world_y * s,
                    ty + cand.world_x * s + cand.world_y * c,
                )
            )

        yaws_local = []
        for i, cand in enumerate(tour):
            if i + 1 < len(tour):
                nx, ny = local_xy[i + 1]
                cx, cy = local_xy[i]
                yaws_local.append(math.atan2(ny - cy, nx - cx))
            else:
                yaws_local.append(
                    wrap_angle(self._candidate_yaw(cand, robot) + tyaw)
                )

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses.header.frame_id = 'map'
        goal_msg.poses.header.stamp = self.get_clock().now().to_msg()
        for i, (xl, yl) in enumerate(local_xy):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = goal_msg.poses.header.stamp
            ps.pose.position.x = xl
            ps.pose.position.y = yl
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quaternion(yaws_local[i])
            goal_msg.poses.goals.append(ps)

        now = self.get_sim_time_sec()
        # Invalidate the previous goal's callbacks before overwriting state.
        robot.goal_token += 1
        token = robot.goal_token
        robot.state = 'DISPATCHING'
        robot.active_waypoints_world = [
            (cand.world_x, cand.world_y) for cand in tour
        ]
        robot.tour_candidates = list(tour)
        robot.tour_poses_world = [
            (
                cand.world_x,
                cand.world_y,
                self._candidate_yaw(cand, robot),
            )
            for cand in tour
        ]
        robot.current_waypoint_index = 0
        robot.current_goal_world = robot.active_waypoints_world[0]
        robot.goal_start_time = now
        robot.tour_start_time = now
        robot.last_progress_time = now
        robot.last_distance_remaining = float('nan')
        robot.stall_distance_ref = float('inf')
        robot.stall_ref_x = robot.world_x
        robot.stall_ref_y = robot.world_y
        robot.tour_is_active = True
        robot.tour_uses_through_poses = True
        robot.rewrite_count = 0
        robot.current_residual_key = self._residual_key_for(tour[0])

        self._publish_goal_pose(
            robot, tour[0].world_x, tour[0].world_y,
            self._candidate_yaw(tour[0], robot),
        )

        points = np.array(
            [[cand.world_x, cand.world_y] for cand in tour], dtype=np.float64
        )
        length = path_length(
            list(range(len(tour))),
            points,
            np.array([robot.world_x, robot.world_y], dtype=np.float64),
        )
        self.get_logger().info(
            f'[Local TSP] {robot.namespace}: {len(tour)} waypoints '
            f'(length={length:.2f}m) -> NavigateThroughPoses'
        )
        self.get_logger().info(
            f'Dispatched {len(tour)}-point NavigateThroughPoses tour for '
            f'{robot.namespace}.'
        )
        self.get_logger().info(
            f'[{robot.namespace}] Sending Nav2 goal: '
            f'world=({tour[0].world_x:.2f}, {tour[0].world_y:.2f}) -> '
            f'local=({local_xy[0][0]:.2f}, {local_xy[0][1]:.2f})'
        )

        try:
            send_future = robot.through_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda msg, r=robot, t=token: (
                    self.tour_feedback_callback(msg, r, t)
                ),
            )
        except Exception as exc:
            self.get_logger().error(
                f'[{robot.namespace}] Failed to send through-poses goal: {exc}'
            )
            self.handle_goal_failure(
                robot, robot.current_goal_world, robot.current_residual_key
            )
            self._reset_robot_nav(robot)
            return False

        send_future.add_done_callback(
            lambda future, r=robot, t=token: self.goal_response_callback(
                future, r, t, 'through_poses'
            )
        )
        return True

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------
    def tour_feedback_callback(self, msg, robot: RobotTracker, token: int):
        with self._state_lock:
            self._tour_feedback_callback_locked(msg, robot, token)

    def _tour_feedback_callback_locked(
        self, msg, robot: RobotTracker, token: int
    ):
        if token != robot.goal_token:
            return
        try:
            fb = msg.feedback
            n = len(robot.active_waypoints_world)
            if n == 0:
                return

            idx = None
            statuses = getattr(fb, 'waypoint_statuses', None)
            if statuses:
                completed = [
                    int(st.waypoint_index)
                    for st in statuses
                    if int(st.waypoint_status) in (1, 2, 3)
                ]
                if completed:
                    idx = min(n - 1, max(completed) + 1)
            if idx is None:
                remaining = int(fb.number_of_poses_remaining)
                idx = max(0, min(n - 1, n - remaining))

            if idx != robot.current_waypoint_index:
                self.get_logger().info(
                    f'[{robot.namespace}] Tour progress: waypoint '
                    f'{idx + 1}/{n}'
                )
                if idx < len(robot.tour_candidates):
                    wx, wy = robot.active_waypoints_world[idx]
                    self._publish_goal_pose(
                        robot,
                        wx,
                        wy,
                        self._candidate_yaw(robot.tour_candidates[idx], robot),
                    )
            robot.current_waypoint_index = idx
            robot.current_goal_world = robot.active_waypoints_world[idx]
            if idx < len(robot.tour_candidates):
                robot.current_residual_key = self._residual_key_for(
                    robot.tour_candidates[idx]
                )
            robot.last_distance_remaining = float(fb.distance_remaining)
        except Exception as exc:
            self.get_logger().warning(
                f'[{robot.namespace}] Bad tour feedback: {exc}'
            )

    def goal_response_callback(
        self,
        future,
        robot: RobotTracker,
        token: int,
        action_kind: str,
    ):
        with self._state_lock:
            self._goal_response_callback_locked(
                future, robot, token, action_kind
            )

    def _goal_response_callback_locked(
        self,
        future,
        robot: RobotTracker,
        token: int,
        action_kind: str,
    ):
        if token != robot.goal_token:
            try:
                gh = future.result()
                if gh is not None and gh.accepted:
                    gh.cancel_goal_async()
            except Exception:
                pass
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'[{robot.namespace}] Exception in goal response: {exc}'
            )
            target = robot.current_goal_world
            key = robot.current_residual_key
            self._increment_through_failure(robot)
            self._reset_robot_nav(robot)
            self.handle_goal_failure(robot, target, key)
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                f'[{robot.namespace}] Goal rejected by server!'
            )
            target = robot.current_goal_world
            key = robot.current_residual_key
            self._increment_through_failure(robot)
            self._reset_robot_nav(robot)
            self.handle_goal_failure(robot, target, key)
            return

        if token != robot.goal_token:
            self.get_logger().info(
                f'[{robot.namespace}] Goal accepted, but target was '
                f'cancelled/superseded. Cancelling handle.'
            )
            goal_handle.cancel_goal_async()
            return

        self.get_logger().info(f'[{robot.namespace}] Goal accepted.')
        robot.current_goal_handle = goal_handle
        robot.state = 'NAVIGATING'

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda res_future, r=robot, t=token: self.goal_result_callback(
                res_future, r, t
            )
        )

    def goal_result_callback(
        self, future, robot: RobotTracker, token: int
    ):
        with self._state_lock:
            self._goal_result_callback_locked(future, robot, token)

    def _goal_result_callback_locked(
        self, future, robot: RobotTracker, token: int
    ):
        if token != robot.goal_token:
            return

        try:
            wrapped = future.result()
            status = wrapped.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                target = robot.current_goal_world
                key = robot.current_residual_key
                self.get_logger().info(
                    f'[{robot.namespace}] Goal reached successfully! '
                    f'Target: {target}'
                )
                self._check_no_progress_success(robot, target)
                self._mark_tour_residuals_resolved(robot)
                robot.through_poses_failures = 0
                self._reset_robot_nav(robot)
                if target is not None and key is not None:
                    self._resolve_residual(key)
                return

            if status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(
                    f'[{robot.namespace}] Goal canceled cleanly. '
                    f'Target: {robot.current_goal_world}'
                )
                self._reset_robot_nav(robot)
                return

            # Aborted / failed.
            failed_idx = self._failed_waypoint_index(wrapped.result)
            target = robot.current_goal_world
            key = robot.current_residual_key
            if (
                failed_idx is not None
                and 0 <= failed_idx < len(robot.active_waypoints_world)
            ):
                target = robot.active_waypoints_world[failed_idx]
                if failed_idx < len(robot.tour_candidates):
                    key = self._residual_key_for(
                        robot.tour_candidates[failed_idx]
                    )
            error_code = int(getattr(wrapped.result, 'error_code', -1))
            error_msg = getattr(wrapped.result, 'error_msg', '')
            self.get_logger().warning(
                f'[{robot.namespace}] Navigation failed with status code '
                f'{status} (error_code={error_code}, msg="{error_msg}").'
            )
            self._increment_through_failure(robot)
            self._reset_robot_nav(robot)
            self.handle_goal_failure(robot, target, key)
        except Exception as exc:
            self.get_logger().error(
                f'[{robot.namespace}] Exception in goal result: {exc}'
            )
            target = robot.current_goal_world
            key = robot.current_residual_key
            self._reset_robot_nav(robot)
            self.handle_goal_failure(robot, target, key)

    def _check_no_progress_success(
        self, robot: RobotTracker, target: Optional[Tuple[float, float]]
    ):
        """Break instant-success livelocks on persistent frontier pockets.

        When the robot is already standing at a goal when it is dispatched
        (within Nav2's goal tolerance), the action succeeds in well under a
        second and no new observations happen.  If the frontier pocket never
        clears (stale merged map, cells a 2D lidar cannot see), the same
        candidate is re-extracted and won every cycle: dispatch -> instant
        success -> dispatch ... forever.  That loop also keeps the frontier
        count above zero, which blocks both the residual stage and
        convergence.  After a few consecutive repeats the spot is added to
        the regular blacklist so the pocket stops being re-won.

        Called from the success branch of the goal result callback, before
        the tracker is reset (goal_start_time / stall_ref are still valid).
        """
        if target is None:
            return
        elapsed = self.get_sim_time_sec() - robot.goal_start_time
        moved = math.hypot(
            robot.world_x - robot.stall_ref_x,
            robot.world_y - robot.stall_ref_y,
        )
        if (
            elapsed >= self.no_progress_time
            or moved >= self.no_progress_distance
        ):
            # Real travel happened; any pocket suspicion is stale.
            robot.no_progress_streak = 0
            robot.no_progress_key = None
            return

        tx, ty = float(target[0]), float(target[1])
        if (
            robot.no_progress_key is not None
            and math.hypot(
                tx - robot.no_progress_key[0],
                ty - robot.no_progress_key[1],
            )
            <= self.blacklist_radius
        ):
            robot.no_progress_streak += 1
        else:
            robot.no_progress_streak = 1
        robot.no_progress_key = (tx, ty)

        if robot.no_progress_streak >= self.no_progress_limit:
            self.get_logger().warning(
                f'[{robot.namespace}] Goal at ({tx:.2f}, {ty:.2f}) '
                f'succeeded instantly {robot.no_progress_streak} times '
                f'without the frontier clearing. Adding to blacklisted '
                f'goals (radius {self.blacklist_radius}m).'
            )
            if robot.no_progress_key not in self.blacklisted_goals:
                self.blacklisted_goals.append(robot.no_progress_key)
            robot.no_progress_key = None
            robot.no_progress_streak = 0
        else:
            self.get_logger().warning(
                f'[{robot.namespace}] Instant success at ({tx:.2f}, '
                f'{ty:.2f}): goal was already reached, frontier pocket may '
                f'not clear (no-progress {robot.no_progress_streak}/'
                f'{self.no_progress_limit}).'
            )

    def _failed_waypoint_index(self, result) -> Optional[int]:
        statuses = getattr(result, 'waypoint_statuses', None)
        if not statuses:
            return None
        for st in statuses:
            if int(st.waypoint_status) in (2, 3):  # SKIPPED / FAILED
                return int(st.waypoint_index)
        return None

    def _increment_through_failure(self, robot: RobotTracker):
        if not robot.tour_uses_through_poses:
            return
        robot.through_poses_failures += 1
        if (
            robot.use_through_poses
            and robot.through_poses_failures >= self.through_poses_fail_limit
        ):
            robot.use_through_poses = False
            self.get_logger().warning(
                f'[{robot.namespace}] NavigateThroughPoses failed '
                f'{robot.through_poses_failures} times; falling back to '
                f'NavigateToPose.'
            )

    def _mark_tour_residuals_resolved(self, robot: RobotTracker):
        for cand in robot.tour_candidates:
            key = self._residual_key_for(cand)
            if key is not None:
                self._resolve_residual(key)

    def _nearest_residual_key(
        self, key: Tuple[float, float], radius: float
    ) -> Optional[Tuple[float, float]]:
        """Return an existing residual key within radius (keys may jitter)."""
        best = None
        best_dist = radius
        for known in list(self.residual_attempts) + list(self.residual_blacklist):
            dist = math.hypot(key[0] - known[0], key[1] - known[1])
            if dist <= best_dist:
                best = known
                best_dist = dist
        return best

    def _resolve_residual(self, key: Tuple[float, float]):
        """Blacklist a resolved blind spot; idempotent.

        Single-point dispatch stores its candidate in ``tour_candidates`` too,
        so success handling may report the same key twice (once from the tour
        scan, once from ``current_residual_key``).
        """
        radius = self.residual_params.blacklist_radius
        already = any(
            math.hypot(key[0] - bx, key[1] - by) <= radius
            for bx, by in self.residual_blacklist
        )
        if already:
            return
        self.residual_blacklist.append(key)
        for known in list(self.residual_attempts):
            if math.hypot(key[0] - known[0], key[1] - known[1]) <= radius:
                self.residual_attempts.pop(known, None)
        self.get_logger().info(
            f'[Residual] Blind spot near {key} resolved; added to residual blacklist.'
        )

    def _reset_robot_nav(self, robot: RobotTracker):
        with self._state_lock:
            self._reset_robot_nav_locked(robot)

    def _reset_robot_nav_locked(self, robot: RobotTracker):
        # Invalidate stale callbacks first, then clear all state.  Flip the
        # state to IDLE only at the very end: the coordination cycle dispatches
        # only IDLE robots, so this keeps a concurrent dispatch from seeing a
        # half-cleared tracker (callbacks run on other executor threads).
        robot.goal_token += 1
        robot.current_goal_world = None
        robot.current_goal_handle = None
        robot.active_waypoints_world = []
        robot.tour_candidates = []
        robot.tour_poses_world = []
        robot.current_waypoint_index = 0
        robot.tour_is_active = False
        robot.tour_uses_through_poses = False
        robot.rewrite_count = 0
        robot.last_distance_remaining = float('nan')
        robot.stall_distance_ref = float('inf')
        robot.current_residual_key = None
        if robot.plan_pub is not None:
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'world'
            robot.plan_pub.publish(empty_path)
        robot.state = 'IDLE'

    def cancel_robot_goal(self, robot: RobotTracker):
        with self._state_lock:
            self._cancel_robot_goal_locked(robot)

    def _cancel_robot_goal_locked(self, robot: RobotTracker):
        if robot.current_goal_handle is not None:
            try:
                robot.current_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(
                    f'[{robot.namespace}] Error cancelling goal: {exc}'
                )
        self._reset_robot_nav(robot)

    def handle_goal_failure(
        self,
        robot: RobotTracker,
        target: Optional[Tuple[float, float]],
        residual_key: Optional[Tuple[float, float]] = None,
    ):
        if target is None:
            return

        if residual_key is not None:
            # Centroids can move by one grid cell between rebuilds; merge
            # attempts onto an existing nearby key instead of restarting the
            # retry counter from zero on every jitter.
            radius = self.residual_params.blacklist_radius
            store_key = self._nearest_residual_key(residual_key, radius)
            if store_key is None:
                store_key = residual_key
            count = self.residual_attempts.get(store_key, 0) + 1
            self.residual_attempts[store_key] = count
            if count >= self.residual_max_attempts:
                if not any(
                    math.hypot(store_key[0] - bx, store_key[1] - by) <= radius
                    for bx, by in self.residual_blacklist
                ):
                    self.residual_blacklist.append(store_key)
                self.get_logger().warning(
                    f'[Residual] Blind spot near {store_key} failed '
                    f'{count} times; adding to residual blacklist.'
                )
            return

        tx, ty = target
        found_key = None
        for fx, fy in self.global_failed_goals.keys():
            if math.hypot(tx - fx, ty - fy) < 0.6:
                found_key = (fx, fy)
                break
        if found_key is None:
            found_key = (round(tx, 2), round(ty, 2))
            self.global_failed_goals[found_key] = 0

        self.global_failed_goals[found_key] += 1
        current_count = self.global_failed_goals[found_key]
        if current_count >= self.blacklist_retry_limit:
            self.get_logger().warning(
                f'Goal area near {found_key} failed {current_count} times! '
                f'Adding to blacklisted goals (radius {self.blacklist_radius}m).'
            )
            if found_key not in self.blacklisted_goals:
                self.blacklisted_goals.append(found_key)

    # ------------------------------------------------------------------
    # Supervision: timeouts, stalls, invalidation
    # ------------------------------------------------------------------
    def check_action_timeouts(self, now: float):
        with self._state_lock:
            self._check_action_timeouts_locked(now)

    def _check_action_timeouts_locked(self, now: float):
        for ns, robot in self.robots.items():
            if robot.state == 'DISPATCHING':
                if now - robot.goal_start_time > 10.0:
                    target = robot.current_goal_world
                    key = robot.current_residual_key
                    self.get_logger().warning(
                        f'[{ns}] Goal timed out after 10.0s waiting for '
                        f'action acceptance! Cancelling...'
                    )
                    self.cancel_robot_goal(robot)
                    self.handle_goal_failure(robot, target, key)
                continue

            if robot.state != 'NAVIGATING':
                continue

            n = max(1, len(robot.active_waypoints_world))
            limit = self.goal_timeout
            if robot.tour_is_active:
                limit += self.tour_timeout_per_waypoint * (n - 1)
            if now - robot.goal_start_time > limit:
                target = robot.current_goal_world
                key = robot.current_residual_key
                self.get_logger().warning(
                    f'[{ns}] Goal timed out after {limit:.1f}s sim time! '
                    f'Cancelling...'
                )
                self._increment_through_failure(robot)
                self.cancel_robot_goal(robot)
                self.handle_goal_failure(robot, target, key)

    def check_stalls(self, now: float):
        with self._state_lock:
            self._check_stalls_locked(now)

    def _check_stalls_locked(self, now: float):
        for ns, robot in self.robots.items():
            if robot.state != 'NAVIGATING' or not robot.tour_is_active:
                continue
            n = len(robot.active_waypoints_world)
            if n < 2 or robot.current_waypoint_index >= n - 1:
                continue
            if now - robot.goal_start_time < 5.0:
                continue

            moved = math.hypot(
                robot.world_x - robot.stall_ref_x,
                robot.world_y - robot.stall_ref_y,
            )
            current_d = robot.last_distance_remaining
            dist_progress = False
            if math.isfinite(current_d):
                if not math.isfinite(robot.stall_distance_ref):
                    dist_progress = True
                elif (
                    robot.stall_distance_ref - current_d
                    >= self.tour_stall_min_progress
                ):
                    dist_progress = True

            if moved >= self.tour_stall_min_translation or dist_progress:
                robot.stall_ref_x = robot.world_x
                robot.stall_ref_y = robot.world_y
                robot.stall_distance_ref = (
                    current_d if math.isfinite(current_d) else float('inf')
                )
                robot.last_progress_time = now
                continue

            if now - robot.last_progress_time >= self.tour_stall_window:
                target = robot.current_goal_world
                key = robot.current_residual_key
                self.get_logger().warning(
                    f'[{ns}] Navigation failed: tour stalled for '
                    f'{self.tour_stall_window:.1f}s at waypoint '
                    f'{robot.current_waypoint_index + 1}/{n}. Cancelling...'
                )
                self._increment_through_failure(robot)
                self.cancel_robot_goal(robot)
                self.handle_goal_failure(robot, target, key)

    def _waypoint_invalid(
        self,
        occ: OccupancyView,
        robot: RobotTracker,
        gx: float,
        gy: float,
        residual_key: Optional[Tuple[float, float]] = None,
    ) -> Tuple[bool, str]:
        gr, gc = occ.world_to_cell(gx, gy)
        if not occ.contains_cell(gr, gc):
            return True, 'cleared by exploration'
        if int(occ.data[gr, gc]) > 50:
            return True, 'blocked by obstacle'
        if any(
            math.hypot(gx - bx, gy - by) < self.blacklist_radius
            for bx, by in self.blacklisted_goals
        ):
            return True, 'blocked by obstacle'
        if residual_key is not None:
            # Residual waypoints are viewpoints, while the blacklist stores
            # blind-spot centroids; compare centroids, not the viewpoint.
            if any(
                math.hypot(residual_key[0] - bx, residual_key[1] - by)
                < self.residual_params.blacklist_radius
                for bx, by in self.residual_blacklist
            ):
                return True, 'cleared by exploration'
        elif any(
            math.hypot(gx - bx, gy - by) < self.residual_params.blacklist_radius
            for bx, by in self.residual_blacklist
        ):
            return True, 'cleared by exploration'

        check_radius_cells = max(3, int(1.5 / occ.resolution))
        r_min = max(0, gr - check_radius_cells)
        r_max = min(occ.height, gr + check_radius_cells + 1)
        c_min = max(0, gc - check_radius_cells)
        c_max = min(occ.width, gc + check_radius_cells + 1)
        unknown_count = int(
            np.sum(occ.data[r_min:r_max, c_min:c_max] == -1)
        )
        if unknown_count < 5:
            return True, 'cleared by exploration'

        if occ.is_free[gr, gc]:
            goal_comp = int(occ.free_labels[gr, gc])
            robot_comp = occ.component_at_world(robot.world_x, robot.world_y)
            if goal_comp > 0 and robot_comp > 0 and goal_comp != robot_comp:
                return True, 'blocked by obstacle'
        return False, ''

    def check_goal_invalidation(
        self, occ: OccupancyView, now: float
    ):
        with self._state_lock:
            self._check_goal_invalidation_locked(occ, now)

    def _check_goal_invalidation_locked(
        self, occ: OccupancyView, now: float
    ):
        if self._skip_invalidation_once:
            self._skip_invalidation_once = False
            return

        for ns, robot in self.robots.items():
            if robot.state not in ('NAVIGATING', 'DISPATCHING'):
                continue
            if not robot.active_waypoints_world:
                continue
            if now - robot.goal_start_time < 8.0:
                continue

            start = max(0, robot.current_waypoint_index)
            for idx in range(start, len(robot.active_waypoints_world)):
                gx, gy = robot.active_waypoints_world[idx]
                # Ignore points the robot has already reached/passed.
                if math.hypot(gx - robot.world_x, gy - robot.world_y) < 1.0:
                    continue

                cand = (
                    robot.tour_candidates[idx]
                    if idx < len(robot.tour_candidates)
                    else None
                )
                residual_key = (
                    self._residual_key_for(cand) if cand is not None else None
                )
                invalid, reason = self._waypoint_invalid(
                    occ, robot, gx, gy, residual_key
                )
                if not invalid:
                    continue

                if idx == robot.current_waypoint_index:
                    self.get_logger().info(
                        f'[{ns}] Active goal ({gx:.2f}, {gy:.2f}) has been '
                        f'{reason}! Cancelling goal...'
                    )
                    self.cancel_robot_goal(robot)
                    break

                if (
                    self.tour_rewrite_on_invalidation
                    and robot.rewrite_count < self.tour_max_rewrites
                ):
                    old_count = len(robot.tour_candidates)
                    old_rewrites = robot.rewrite_count
                    suffix = []
                    for j, cand in enumerate(robot.tour_candidates):
                        if j < robot.current_waypoint_index or j == idx:
                            continue
                        cjx, cjy = cand.world_x, cand.world_y
                        # Do not re-add a waypoint the robot already reached.
                        if math.hypot(
                            cjx - robot.world_x, cjy - robot.world_y
                        ) < 1.0:
                            continue
                        cjr, cjc = occ.world_to_cell(cjx, cjy)
                        if not occ.contains_cell(cjr, cjc):
                            continue
                        if not occ.is_free[cjr, cjc]:
                            continue
                        suffix.append(cand)

                    self.get_logger().info(
                        f'[{ns}] Active goal ({gx:.2f}, {gy:.2f}) has been '
                        f'{reason}! Rewriting tour suffix...'
                    )
                    if len(suffix) >= 2:
                        # Send the replacement directly.  Nav2 preempts the
                        # active through-poses goal atomically; cancelling
                        # first races with the new goal and can abort the
                        # replacement instead of the old one.
                        self.dispatch_tour(robot, suffix)
                        robot.rewrite_count = old_rewrites + 1
                        self.get_logger().info(
                            f'Tour rewritten for {ns}: {old_count} -> '
                            f'{len(suffix)} points'
                        )
                    else:
                        # A single remaining point cannot preempt through-poses
                        # with a NavigateToPose request.  Drop the tour and let
                        # the next cycle re-dispatch it as a normal goal.
                        self.cancel_robot_goal(robot)
                        self.get_logger().info(
                            f'Tour dropped for {ns}: no multi-point suffix '
                            f'remains; re-planning next cycle.'
                        )
                    break

                self.get_logger().info(
                    f'[{ns}] Active goal ({gx:.2f}, {gy:.2f}) has been '
                    f'{reason}! Cancelling goal...'
                )
                self.cancel_robot_goal(robot)
                break

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def publish_markers(
        self,
        occ: OccupancyView,
        candidates: List[object],
        available: List[object],
        ownership: List,
        residual: List[object],
        dispatched: set,
    ):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. Candidate frontier points (dashboard counts this namespace).
        frontier_marker = Marker()
        frontier_marker.header.frame_id = 'world'
        frontier_marker.header.stamp = stamp
        frontier_marker.ns = 'candidate_frontiers'
        frontier_marker.id = 0
        frontier_marker.type = Marker.SPHERE_LIST
        frontier_marker.action = Marker.ADD
        frontier_marker.scale.x = 0.2
        frontier_marker.scale.y = 0.2
        frontier_marker.scale.z = 0.2
        frontier_marker.color.r = 0.0
        frontier_marker.color.g = 1.0
        frontier_marker.color.b = 0.2
        frontier_marker.color.a = 0.8
        for c in available:
            if getattr(c, 'kind', 'frontier') != 'frontier':
                continue
            p = Point()
            p.x = c.world_x
            p.y = c.world_y
            p.z = 0.05
            frontier_marker.points.append(p)
        ma.markers.append(frontier_marker)

        # 2. Pool-colored candidate overlays.
        pool_map = {}
        for cand, own in zip(available, ownership):
            pool_map[(round(cand.world_x, 2), round(cand.world_y, 2))] = own

        colors = {
            'robot1': (1.0, 0.2, 0.2),
            'robot2': (0.2, 0.5, 1.0),
        }
        for ns in self.robots:
            local_marker = Marker()
            local_marker.header.frame_id = 'world'
            local_marker.header.stamp = stamp
            local_marker.ns = f'local_pool_{ns}'
            local_marker.id = 0
            local_marker.type = Marker.SPHERE_LIST
            local_marker.action = Marker.ADD
            local_marker.scale.x = 0.14
            local_marker.scale.y = 0.14
            local_marker.scale.z = 0.14
            cr, cg, cb = colors.get(ns, (1.0, 1.0, 0.0))
            local_marker.color.r = cr
            local_marker.color.g = cg
            local_marker.color.b = cb
            local_marker.color.a = 0.9
            for cand in available:
                own = pool_map.get(
                    (round(cand.world_x, 2), round(cand.world_y, 2))
                )
                if own is not None and own.pool == 'local' and own.owner == ns:
                    p = Point()
                    p.x = cand.world_x
                    p.y = cand.world_y
                    p.z = 0.08
                    local_marker.points.append(p)
            ma.markers.append(local_marker)

        global_marker = Marker()
        global_marker.header.frame_id = 'world'
        global_marker.header.stamp = stamp
        global_marker.ns = 'global_pool'
        global_marker.id = 0
        global_marker.type = Marker.SPHERE_LIST
        global_marker.action = Marker.ADD
        global_marker.scale.x = 0.16
        global_marker.scale.y = 0.16
        global_marker.scale.z = 0.16
        global_marker.color.r = 1.0
        global_marker.color.g = 0.9
        global_marker.color.b = 0.1
        global_marker.color.a = 0.85
        for cand in available:
            own = pool_map.get(
                (round(cand.world_x, 2), round(cand.world_y, 2))
            )
            if own is not None and own.pool == 'global':
                p = Point()
                p.x = cand.world_x
                p.y = cand.world_y
                p.z = 0.08
                global_marker.points.append(p)
        ma.markers.append(global_marker)

        # 3. Residual blind spots.
        residual_marker = Marker()
        residual_marker.header.frame_id = 'world'
        residual_marker.header.stamp = stamp
        residual_marker.ns = 'residual_blindspots'
        residual_marker.id = 0
        residual_marker.type = Marker.CUBE_LIST
        residual_marker.action = Marker.ADD
        residual_marker.scale.x = 0.18
        residual_marker.scale.y = 0.18
        residual_marker.scale.z = 0.18
        residual_marker.color.r = 1.0
        residual_marker.color.g = 0.5
        residual_marker.color.b = 0.0
        residual_marker.color.a = 0.9
        for c in residual:
            p = Point()
            p.x = c.world_x
            p.y = c.world_y
            p.z = 0.12
            residual_marker.points.append(p)
        ma.markers.append(residual_marker)

        # 4. Assigned goals (dashboard counts ADD markers in this namespace).
        for idx, (ns, robot) in enumerate(self.robots.items(), start=1):
            goal_sphere = Marker()
            goal_sphere.header.frame_id = 'world'
            goal_sphere.header.stamp = stamp
            goal_sphere.ns = 'assigned_goals'
            goal_sphere.id = idx

            text_marker = Marker()
            text_marker.header.frame_id = 'world'
            text_marker.header.stamp = stamp
            text_marker.ns = 'assigned_labels'
            text_marker.id = idx

            active = (
                robot.state in ('DISPATCHING', 'NAVIGATING')
                and robot.current_goal_world is not None
            )
            if active:
                gx, gy = robot.current_goal_world
                cr, cg, cb = colors.get(ns, (1.0, 1.0, 0.0))
                goal_sphere.type = Marker.SPHERE
                goal_sphere.action = Marker.ADD
                goal_sphere.pose.position.x = gx
                goal_sphere.pose.position.y = gy
                goal_sphere.pose.position.z = 0.15
                goal_sphere.scale.x = 0.35
                goal_sphere.scale.y = 0.35
                goal_sphere.scale.z = 0.35
                goal_sphere.color.r = cr
                goal_sphere.color.g = cg
                goal_sphere.color.b = cb
                goal_sphere.color.a = 0.95

                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = gx
                text_marker.pose.position.y = gy
                text_marker.pose.position.z = 0.4
                text_marker.scale.z = 0.2
                text_marker.text = f'{ns} target'
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
            else:
                goal_sphere.action = Marker.DELETE
                text_marker.action = Marker.DELETE

            ma.markers.append(goal_sphere)
            ma.markers.append(text_marker)

        # 5. Tour path + remaining waypoints.
        for ns, robot in self.robots.items():
            path_marker = Marker()
            path_marker.header.frame_id = 'world'
            path_marker.header.stamp = stamp
            path_marker.ns = f'tour_path_{ns}'
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.04
            cr, cg, cb = colors.get(ns, (1.0, 1.0, 0.0))
            path_marker.color.r = cr
            path_marker.color.g = cg
            path_marker.color.b = cb
            path_marker.color.a = 0.9

            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = 'world'
            waypoint_marker.header.stamp = stamp
            waypoint_marker.ns = f'tour_waypoints_{ns}'
            waypoint_marker.id = 0
            waypoint_marker.type = Marker.SPHERE_LIST
            waypoint_marker.action = Marker.ADD
            waypoint_marker.scale.x = 0.12
            waypoint_marker.scale.y = 0.12
            waypoint_marker.scale.z = 0.12
            waypoint_marker.color.r = cr
            waypoint_marker.color.g = cg
            waypoint_marker.color.b = cb
            waypoint_marker.color.a = 1.0

            if (
                robot.state in ('DISPATCHING', 'NAVIGATING')
                and robot.active_waypoints_world
            ):
                p0 = Point()
                p0.x = robot.world_x
                p0.y = robot.world_y
                p0.z = 0.1
                path_marker.points.append(p0)
                start = max(0, robot.current_waypoint_index)
                for wx, wy in robot.active_waypoints_world[start:]:
                    p = Point()
                    p.x = wx
                    p.y = wy
                    p.z = 0.1
                    path_marker.points.append(p)
                    wp = Point()
                    wp.x = wx
                    wp.y = wy
                    wp.z = 0.15
                    waypoint_marker.points.append(wp)
            else:
                path_marker.action = Marker.DELETE
                waypoint_marker.action = Marker.DELETE

            ma.markers.append(path_marker)
            ma.markers.append(waypoint_marker)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
