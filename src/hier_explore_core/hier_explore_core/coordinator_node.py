#!/usr/bin/env python3
"""ROS 2 coordinator node orchestrating 2.5D-3D hierarchical multi-robot exploration.

Supports arbitrary N robots (N = 1, 2, 3, 4, ...), feature-flag ablation profiles,
and clean separation between ROS I/O and pure-Python/C++ planning modules.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav2_msgs.action import BackUp, NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64
from visualization_msgs.msg import MarkerArray
import tf2_ros

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover
    point_cloud2 = None

from .convergence import ConvergenceParams, ConvergenceTracker
from .deconfliction import (
    CorridorMutexManager,
    CorridorMutexParams,
    TeammateCloudParams,
    build_teammate_cloud_points,
    pack_xyz32_cloud,
)
from .frontier import FrontierParams, extract_frontiers, score_frontiers
from .geom_kernel import GeodesicCache, has_cpp_kernel
from .geometry import se2_apply, se2_inverse, wrap_angle
from .occupancy_view import OccupancyView
from .residual_nbv import ResidualParams, detect_residuals
from .robot_tracker import RobotTracker
from .scheduler import (
    AgentState,
    RoiParams,
    TourParams,
    UtilityParams,
    assign_ownership,
    build_local_tour,
    calculate_legacy_utility,
    euclidean_distance,
    exclude_near_waypoints,
    filter_corridor_safe_candidates,
    global_bid,
    path_length,
)
from .supervisor import (
    FailureBookkeeper,
    check_no_progress_success,
    check_waypoint_invalid,
    is_out_of_bounds_failure,
)
from .visualizer import build_exploration_markers
from .voxel_index import InfoGainParams, VoxelColumnIndex, VoxelParams


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MultiRobotCoordinator(Node):
    """Modular N-robot 2.5D-3D hierarchical exploration coordinator."""

    def __init__(self) -> None:
        super().__init__("multi_robot_coordinator")
        self._declare_and_load_params()

        self.bookkeeper = FailureBookkeeper(
            blacklist_retry_limit=self.blacklist_retry_limit,
            blacklist_radius=self.blacklist_radius,
            residual_blacklist_radius=self.residual_params.blacklist_radius,
            residual_max_attempts=self.residual_max_attempts,
        )
        self.convergence = ConvergenceTracker(self.convergence_params)
        self.corridor_mgr = CorridorMutexManager(self.corridor_params)

        # Runtime state
        self._local_seen: Dict[Tuple[float, float], float] = {}
        self.latest_map: Optional[OccupancyGrid] = None
        self._cached_map_id: int = -1
        self._cached_occ: Optional[OccupancyView] = None
        self._cached_clusters_key: Optional[Tuple[int, int]] = None
        self._cached_clusters: List[object] = []
        self._last_corridor_update_time: float = -1e9
        self._latest_cloud: Optional[PointCloud2] = None
        self._last_map_time: float = -1e9
        self._last_cloud_time: float = -1e9
        self._last_voxel_build_time: float = -1e9
        self.voxel_index: Optional[VoxelColumnIndex] = None
        self._voxel_prev_keys: Optional[object] = None
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
        self._nav_outage_all_logged: bool = False
        self._last_starved_log_time: float = -1e9
        self._global_reject_info: Optional[Tuple[int, float]] = None
        self._exploration_halted: bool = False

        self.cycle_lock = threading.Lock()
        self._state_lock = threading.RLock()

        # ROS interfaces
        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._init_robots()
        self._init_ros_io()

        timer_period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(
            timer_period, self.coordination_cycle, callback_group=self.cb_group
        )
        self.teammates_timer = None
        if self.teammate_deconfliction_enable and len(self.robots) >= 2:
            tm_period = 1.0 / max(1.0, self.teammate_cloud_rate_hz)
            self.teammates_timer = self.create_timer(
                tm_period, self._publish_teammates_clouds, callback_group=self.cb_group
            )
        self.get_logger().info(
            f"MultiRobotCoordinator ready for {list(self.robots.keys())} @ {self.rate_hz} Hz "
            f"(cpp_kernel={has_cpp_kernel()}, 3d_gain={self.use_3d_gain}, "
            f"geodesic={self.use_geodesic_distance}, los_fov={self.residual_params.use_los_fov_optimizer}, "
            f"deconflict={self.teammate_deconfliction_enable}, corridor_mutex={self.corridor_mutex_enable})"
        )

    # ------------------------------------------------------------------
    # Parameter loading & ROS initialization
    # ------------------------------------------------------------------
    def _declare_and_load_params(self) -> None:
        p = self.declare_parameter
        p("robot_namespaces", ["robot1", "robot2"])
        p("rate_hz", 0.5)
        p("min_frontier_size", 5)
        p("gain_scale", 1.0)
        p("potential_scale", 2.0)
        p("orientation_scale", 0.5)
        p("exclusion_radius", 1.5)
        p("goal_timeout", 90.0)
        p("blacklist_retry_limit", 2)
        p("blacklist_radius", 0.9)
        p("min_clearance", 0.55)
        p("no_progress_time", 2.0)
        p("no_progress_distance", 0.3)
        p("no_progress_limit", 2)

        # 3D information gain
        p("use_3d_gain", True)
        p("octomap_cloud_topic", "/octomap_point_cloud_centers")
        p("voxel_index_resolution", 0.10)
        p("voxel_min_z", 0.08)
        p("voxel_max_z", 2.0)
        p("voxel_min_points_per_column", 3)
        p("voxel_max_points", 400000)
        p("voxel_update_period", 2.0)
        p("gain_3d_window_radius", 1.5)
        p("gain_3d_scale", 40.0)
        p("gain_3d_density_weight", 0.30)
        p("gain_3d_unknown_weight", 0.35)
        p("gain_3d_floating_weight", 0.15)
        p("gain_3d_height_weight", 0.10)
        p("gain_3d_occupancy_weight", 0.10)
        p("gain_3d_density_ref", 0.18)
        p("gain_3d_occupancy_ref", 8.0)
        p("gain_3d_height_ref", 0.80)
        p("gain_3d_wall_zmin", 0.25)
        p("gain_3d_wall_zmax", 1.30)
        p("voxel_cloud_warn_timeout", 15.0)
        p("cycle_time_budget_ms", 150.0)
        p("map_loss_timeout", 5.0)

        # Local / Global scheduling
        p("local_scheduling_enable", True)
        p("local_radius", 6.0)
        p("local_max_waypoints", 6)
        p("local_min_spacing", 1.2)
        p("local_max_tour_length", 10.0)
        p("local_tour_score_dist_weight", 1.0)
        p("local_release_unselected", False)
        p("local_release_cooldown", 5.0)
        p("local_release_score_penalty", 0.7)
        p("tsp_2opt_passes", 2)
        p("global_roi_enable", True)
        p("global_roi_distance_offset", 1.0)
        p("global_roi_min", 0.05)
        p("global_roi_scale", 5.0)
        p("global_max_distance", 0.0)
        p("global_max_candidates", 8)

        # Tour supervision & recovery
        p("use_through_poses", True)
        p("through_poses_fail_limit", 2)
        p("action_wait_sec", 2.0)
        p("tf_wait_sec", 0.1)
        p("tour_timeout_per_waypoint", 25.0)
        p("tour_stall_window", 12.0)
        p("tour_stall_min_translation", 0.10)
        p("tour_stall_min_progress", 0.20)
        p("tour_rewrite_on_invalidation", True)
        p("tour_max_rewrites", 2)
        p("backup_recovery_enable", True)
        p("backup_distance", 0.40)
        p("backup_speed", 0.06)
        p("backup_cooldown", 20.0)
        p("nav_outage_warn_period", 15.0)
        p("exploration_halt_topic", "/exploration/halt")

        # Residual blind spots
        p("residual_enable", True)
        p("residual_activation_cycles", 3)
        p("residual_min_cluster_cells", 25)
        p("residual_min_density", 0.08)
        p("residual_min_unknown_frac", 0.20)
        p("residual_structure_radius", 2.5)
        p("residual_min_structure_columns", 3)
        p("residual_ring_radius", 1.0)
        p("residual_ring_min_empty_sectors", 3)
        p("residual_viewpoint_search_radius", 2.5)
        p("residual_max_candidates", 4)
        p("residual_dedupe_radius", 1.5)
        p("residual_score_scale", 10.0)
        p("residual_blacklist_radius", 1.0)
        p("residual_max_attempts", 2)
        p("residual_coverage_hole_enable", False)
        p("use_los_fov_optimizer", True)
        p("residual_viewpoint_r_min", 0.8)
        p("residual_viewpoint_r_max", 3.5)
        p("residual_viewpoint_samples", 24)
        p("lidar_sensor_z", 0.25)
        p("lidar_vfov_min_deg", -30.0)
        p("lidar_vfov_max_deg", 55.0)
        p("residual_max_eff_range", 5.0)

        # M3: Geodesic distance, teammate local-costmap deconfliction & corridor mutex
        p("use_geodesic_distance", True)
        p("geodesic_min_clearance", 0.22)
        p("geodesic_max_dist", 40.0)
        p("teammate_deconfliction_enable", True)
        p("teammate_cloud_rate_hz", 5.0)
        p("teammate_footprint_radius", 0.26)
        p("teammate_predict_horizon_s", 2.0)
        p("corridor_mutex_enable", True)
        p("corridor_narrow_margin", 0.35)
        p("corridor_robot_radius", 0.22)

        # Convergence
        p("convergence_enable", True)
        p("convergence_cycles_required", 6)
        p("convergence_residual_gain_threshold", 0.50)
        p("convergence_voxel_growth_epsilon", 25)
        p("convergence_voxel_growth_rate", 0.005)
        p("convergence_require_3d", True)
        p("fallback_convergence_timeout", 120.0)
        p("convergence_stop_dispatch", True)

        g = lambda name: self.get_parameter(name).value
        self.namespaces = [
            ns.strip("/")
            for ns in (
                self.get_parameter("robot_namespaces")
                .get_parameter_value()
                .string_array_value
                or ["robot1", "robot2"]
            )
        ]
        # Dynamically declare <ns>_init_pose for arbitrary N robots
        default_inits = {
            "robot1": [-2.0, -0.5, 0.0],
            "robot2": [0.0, 0.5, 0.0],
        }
        for ns in self.namespaces:
            param_name = f"{ns}_init_pose"
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, default_inits.get(ns, [0.0, 0.0, 0.0]))

        self.rate_hz = float(g("rate_hz"))
        self.gain_scale = float(g("gain_scale"))
        self.potential_scale = float(g("potential_scale"))
        self.orientation_scale = float(g("orientation_scale"))
        self.exclusion_radius = float(g("exclusion_radius"))
        self.goal_timeout = float(g("goal_timeout"))
        self.blacklist_retry_limit = int(g("blacklist_retry_limit"))
        self.blacklist_radius = float(g("blacklist_radius"))
        self.min_clearance = float(g("min_clearance"))
        self.no_progress_time = float(g("no_progress_time"))
        self.no_progress_distance = float(g("no_progress_distance"))
        self.no_progress_limit = int(g("no_progress_limit"))

        self.frontier_params = FrontierParams(
            min_frontier_size=int(g("min_frontier_size")),
            min_clearance=self.min_clearance,
            blacklist_radius=self.blacklist_radius,
        )
        self.utility_params = UtilityParams(
            gain_scale=self.gain_scale,
            potential_scale=self.potential_scale,
            orientation_scale=self.orientation_scale,
        )

        self.use_3d_gain = bool(g("use_3d_gain"))
        self.octomap_cloud_topic = str(g("octomap_cloud_topic"))
        self.voxel_update_period = float(g("voxel_update_period"))
        self.gain_3d_scale = float(g("gain_3d_scale"))
        self.voxel_cloud_warn_timeout = float(g("voxel_cloud_warn_timeout"))
        self.cycle_time_budget_ms = float(g("cycle_time_budget_ms"))
        self.map_loss_timeout = float(g("map_loss_timeout"))
        self.voxel_params = VoxelParams(
            resolution=float(g("voxel_index_resolution")),
            min_z=float(g("voxel_min_z")),
            max_z=float(g("voxel_max_z")),
            min_points_per_column=int(g("voxel_min_points_per_column")),
            max_points=int(g("voxel_max_points")),
        )
        self.info_gain_params = InfoGainParams(
            window_radius=float(g("gain_3d_window_radius")),
            density_ref=float(g("gain_3d_density_ref")),
            occupancy_ref=float(g("gain_3d_occupancy_ref")),
            height_ref=float(g("gain_3d_height_ref")),
            wall_zmin=float(g("gain_3d_wall_zmin")),
            wall_zmax=float(g("gain_3d_wall_zmax")),
            w_density=float(g("gain_3d_density_weight")),
            w_unknown=float(g("gain_3d_unknown_weight")),
            w_floating=float(g("gain_3d_floating_weight")),
            w_height=float(g("gain_3d_height_weight")),
            w_occupancy=float(g("gain_3d_occupancy_weight")),
        )

        self.local_scheduling_enable = bool(g("local_scheduling_enable"))
        self.local_radius = float(g("local_radius"))
        self.local_release_unselected = bool(g("local_release_unselected"))
        self.local_release_cooldown = float(g("local_release_cooldown"))
        self.local_release_score_penalty = float(g("local_release_score_penalty"))
        self.tour_params = TourParams(
            max_waypoints=int(g("local_max_waypoints")),
            min_spacing=float(g("local_min_spacing")),
            max_tour_length=float(g("local_max_tour_length")),
            score_dist_weight=float(g("local_tour_score_dist_weight")),
            two_opt_passes=int(g("tsp_2opt_passes")),
        )
        self.global_roi_enable = bool(g("global_roi_enable"))
        self.roi_params = RoiParams(
            distance_offset=float(g("global_roi_distance_offset")),
            roi_scale=float(g("global_roi_scale")),
            min_roi=float(g("global_roi_min")),
            max_distance=float(g("global_max_distance")),
            orientation_scale=self.orientation_scale,
        )
        self.global_max_candidates = int(g("global_max_candidates"))

        self.use_through_poses = bool(g("use_through_poses"))
        self.through_poses_fail_limit = int(g("through_poses_fail_limit"))
        self.action_wait_sec = float(g("action_wait_sec"))
        self.tf_wait_sec = max(0.0, float(g("tf_wait_sec")))
        self.tour_timeout_per_waypoint = float(g("tour_timeout_per_waypoint"))
        self.tour_stall_window = float(g("tour_stall_window"))
        self.tour_stall_min_translation = float(g("tour_stall_min_translation"))
        self.tour_rewrite_on_invalidation = bool(g("tour_rewrite_on_invalidation"))
        self.tour_max_rewrites = int(g("tour_max_rewrites"))
        self.backup_recovery_enable = bool(g("backup_recovery_enable"))
        self.backup_distance = float(g("backup_distance"))
        self.backup_speed = float(g("backup_speed"))
        self.backup_cooldown = float(g("backup_cooldown"))
        self.nav_outage_warn_period = float(g("nav_outage_warn_period"))
        self.halt_topic = str(g("exploration_halt_topic"))

        self.residual_enable = bool(g("residual_enable"))
        self.residual_activation_cycles = int(g("residual_activation_cycles"))
        self.residual_max_attempts = int(g("residual_max_attempts"))
        self.residual_params = ResidualParams(
            activation_cycles=self.residual_activation_cycles,
            min_cluster_cells=int(g("residual_min_cluster_cells")),
            min_density=float(g("residual_min_density")),
            min_unknown_frac=float(g("residual_min_unknown_frac")),
            structure_radius=float(g("residual_structure_radius")),
            min_structure_columns=int(g("residual_min_structure_columns")),
            ring_radius=float(g("residual_ring_radius")),
            ring_min_empty_sectors=int(g("residual_ring_min_empty_sectors")),
            viewpoint_search_radius=float(g("residual_viewpoint_search_radius")),
            min_clearance=self.min_clearance,
            max_candidates=int(g("residual_max_candidates")),
            dedupe_radius=float(g("residual_dedupe_radius")),
            score_scale=float(g("residual_score_scale")),
            blacklist_radius=float(g("residual_blacklist_radius")),
            coverage_hole_enable=bool(g("residual_coverage_hole_enable")),
            use_los_fov_optimizer=bool(g("use_los_fov_optimizer")),
            viewpoint_r_min=float(g("residual_viewpoint_r_min")),
            viewpoint_r_max=float(g("residual_viewpoint_r_max")),
            viewpoint_samples=int(g("residual_viewpoint_samples")),
            sensor_z=float(g("lidar_sensor_z")),
            vfov_min_deg=float(g("lidar_vfov_min_deg")),
            vfov_max_deg=float(g("lidar_vfov_max_deg")),
            max_eff_range=float(g("residual_max_eff_range")),
        )
        self.use_geodesic_distance = bool(g("use_geodesic_distance"))
        self.geodesic_min_clearance = float(g("geodesic_min_clearance"))
        self.geodesic_max_dist = float(g("geodesic_max_dist"))
        self.teammate_deconfliction_enable = bool(g("teammate_deconfliction_enable"))
        self.teammate_cloud_rate_hz = float(g("teammate_cloud_rate_hz"))
        self.teammate_cloud_params = TeammateCloudParams(
            footprint_radius=float(g("teammate_footprint_radius")),
            predict_horizon_s=float(g("teammate_predict_horizon_s")),
        )
        self.corridor_mutex_enable = bool(g("corridor_mutex_enable"))
        self.corridor_params = CorridorMutexParams(
            robot_radius=float(g("corridor_robot_radius")),
            narrow_margin_m=float(g("corridor_narrow_margin")),
        )
        self.convergence_params = ConvergenceParams(
            enabled=bool(g("convergence_enable")),
            cycles_required=int(g("convergence_cycles_required")),
            residual_gain_threshold=float(g("convergence_residual_gain_threshold")),
            voxel_growth_epsilon=int(g("convergence_voxel_growth_epsilon")),
            voxel_growth_rate=float(g("convergence_voxel_growth_rate")),
            require_3d=bool(g("convergence_require_3d")),
            fallback_timeout=float(g("fallback_convergence_timeout")),
        )
        self.convergence_stop_dispatch = bool(g("convergence_stop_dispatch"))

    def _init_robots(self) -> None:
        self.robots: Dict[str, RobotTracker] = {}
        for ns in self.namespaces:
            init_pose = self.get_parameter(f"{ns}_init_pose").value
            robot = RobotTracker(ns, init_pose)
            robot.use_through_poses = self.use_through_poses
            robot.action_client = ActionClient(
                self, NavigateToPose, f"/{ns}/navigate_to_pose", callback_group=self.cb_group
            )
            robot.through_client = ActionClient(
                self, NavigateThroughPoses, f"/{ns}/navigate_through_poses", callback_group=self.cb_group
            )
            robot.backup_client = ActionClient(
                self, BackUp, f"/{ns}/backup", callback_group=self.cb_group
            )
            if self.use_through_poses and self.action_wait_sec > 0.0:
                if not robot.through_client.wait_for_server(timeout_sec=self.action_wait_sec):
                    robot.use_through_poses = False
                    self.get_logger().warning(
                        f"[{ns}] navigate_through_poses unavailable after {self.action_wait_sec:.1f}s; using NavigateToPose."
                    )
            robot.goal_pub = self.create_publisher(PoseStamped, f"/{ns}/exploration_goal", 10)
            robot.plan_pub = self.create_publisher(Path, f"/{ns}/plan_world", 10)
            if self.teammate_deconfliction_enable:
                robot.teammates_pub = self.create_publisher(
                    PointCloud2, f"/{ns}/teammates_cloud", 5
                )
            robot.plan_sub = self.create_subscription(
                Path,
                f"/{ns}/plan",
                lambda msg, r=robot: self.plan_callback(msg, r),
                10,
                callback_group=self.cb_group,
            )
            self.robots[ns] = robot

    def _init_ros_io(self) -> None:
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 1, callback_group=self.cb_group
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

        self.marker_pub = self.create_publisher(MarkerArray, "/exploration/markers", 10)
        # Benchmark telemetry topics: per-cycle planning latency (ms) and convergence state
        self.cycle_ms_pub = self.create_publisher(Float64, "/exploration/cycle_time_ms", 10)
        self.converged_pub = self.create_publisher(Bool, "/exploration/converged", 10)

        halt_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, self.halt_topic, self._on_exploration_halt, halt_qos, callback_group=self.cb_group
        )

    # ------------------------------------------------------------------
    # Time, map, cloud, and TF helpers
    # ------------------------------------------------------------------
    def get_sim_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        now = self.get_sim_time_sec()
        self._last_map_time = now
        try:
            occ = OccupancyView.from_grid(msg)
            self._cached_occ = occ
            self._cached_map_id = id(msg)
            bl_len = len(self.bookkeeper.blacklisted_goals)
            self._cached_clusters = extract_frontiers(
                occ, self.frontier_params, self.bookkeeper.blacklisted_goals
            )
            self._cached_clusters_key = (id(occ), bl_len)
            if (
                self.corridor_mutex_enable
                and len(self.robots) >= 2
                and (now - self._last_corridor_update_time) >= 6.0
            ):
                self.corridor_mgr.update_map(occ)
                self._last_corridor_update_time = now
        except Exception:
            pass

    def cloud_callback(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        now = self.get_sim_time_sec()
        self._last_cloud_time = now if now > 0.0 else 0.0
        self._cloud_warned = False
        if self._cached_occ is not None and now > 0.0:
            self._maybe_update_voxel_index(now, self._cached_occ)

    def _map_valid(self, now: float) -> bool:
        if self.latest_map is None:
            return False
        if self._last_map_time <= 0.0:
            return True
        return (now - self._last_map_time) <= self.map_loss_timeout

    def _cloud_age(self, now: float) -> float:
        if self._latest_cloud is None:
            return float("inf")
        if self._last_cloud_time <= 0.0:
            return 0.0
        return max(0.0, now - self._last_cloud_time)

    def _index_available(self, now: float) -> bool:
        return (
            self.use_3d_gain
            and self.voxel_index is not None
            and self._last_voxel_build_time > 0.0
            and (now - self._last_voxel_build_time) <= max(3.0, 3.0 * self.voxel_update_period)
        )

    def _world_to_local_transform(self, robot: RobotTracker) -> Tuple[float, float, float]:
        try:
            t = self.tf_buffer.lookup_transform(f"{robot.namespace}/map", "world", Time())
            return (t.transform.translation.x, t.transform.translation.y, quaternion_to_yaw(t.transform.rotation))
        except Exception:
            return se2_inverse((robot.init_x, robot.init_y, robot.init_yaw))

    def _local_to_world_transform(self, robot: RobotTracker) -> Tuple[float, float, float]:
        try:
            t = self.tf_buffer.lookup_transform("world", f"{robot.namespace}/map", Time())
            return (t.transform.translation.x, t.transform.translation.y, quaternion_to_yaw(t.transform.rotation))
        except Exception:
            return (robot.init_x, robot.init_y, robot.init_yaw)

    def update_robot_poses(self) -> None:
        for ns, robot in self.robots.items():
            found = False
            for base_frame in (f"{ns}/base_footprint", f"{ns}/base_link"):
                try:
                    t = self.tf_buffer.lookup_transform("world", base_frame, Time())
                    nx, ny = t.transform.translation.x, t.transform.translation.y
                    nyaw = quaternion_to_yaw(t.transform.rotation)
                    self._track_pose_jump(robot, nx, ny)
                    robot.world_x, robot.world_y, robot.world_yaw = nx, ny, nyaw
                    found = True
                    break
                except Exception:
                    continue
            if found:
                continue
            for base_frame in (f"{ns}/base_footprint", f"{ns}/base_link"):
                try:
                    t = self.tf_buffer.lookup_transform(f"{ns}/map", base_frame, Time())
                    nx, ny, nyaw = se2_apply(
                        (robot.init_x, robot.init_y, robot.init_yaw),
                        t.transform.translation.x,
                        t.transform.translation.y,
                        quaternion_to_yaw(t.transform.rotation),
                    )
                    self._track_pose_jump(robot, nx, ny)
                    robot.world_x, robot.world_y, robot.world_yaw = nx, ny, nyaw
                    break
                except Exception:
                    continue

    def _track_pose_jump(self, robot: RobotTracker, x: float, y: float) -> None:
        if robot.prev_world_x is not None:
            dt = max(1e-3, 1.0 / max(self.rate_hz, 0.1))
            if math.hypot(x - robot.prev_world_x, y - robot.prev_world_y) > 0.5 + 0.8 * dt:
                self.convergence.reset("pose jump")
        robot.prev_world_x, robot.prev_world_y = x, y

    def plan_callback(self, msg: Path, robot: RobotTracker) -> None:
        if not msg.poses:
            robot.latest_plan_world = []
            return
        transform = self._local_to_world_transform(robot)
        world_path = Path()
        world_path.header.stamp = self.get_clock().now().to_msg()
        world_path.header.frame_id = "world"
        plan_xy: List[Tuple[float, float]] = []
        for p in msg.poses:
            xw, yw, yaww = se2_apply(
                transform,
                p.pose.position.x,
                p.pose.position.y,
                quaternion_to_yaw(p.pose.orientation),
            )
            plan_xy.append((xw, yw))
            pw = PoseStamped()
            pw.header.stamp = world_path.header.stamp
            pw.header.frame_id = "world"
            pw.pose.position.x = xw
            pw.pose.position.y = yw
            pw.pose.position.z = p.pose.position.z
            pw.pose.orientation = yaw_to_quaternion(yaww)
            world_path.poses.append(pw)
        robot.latest_plan_world = plan_xy
        robot.latest_plan_time = self.get_sim_time_sec()
        if robot.plan_pub is not None:
            robot.plan_pub.publish(world_path)

    def _publish_teammates_clouds(self) -> None:
        """5 Hz Layer-1 kinematic deconfliction: inject teammate footprints + 2s plans."""
        if not self.teammate_deconfliction_enable or len(self.robots) < 2:
            return
        now = self.get_sim_time_sec()
        if now <= 0.0:
            return
        self.update_robot_poses()
        stamp_msg = self.get_clock().now().to_msg()
        poses_world: Dict[str, Tuple[float, float, float]] = {}
        plans_world: Dict[str, List[Tuple[float, float]]] = {}
        for ns, robot in self.robots.items():
            poses_world[ns] = (robot.world_x, robot.world_y, robot.world_yaw)
            # Only inject recent plans (<= 5s old) while robot is actively moving
            if (
                robot.state in ("NAVIGATING", "DISPATCHING")
                and robot.latest_plan_world
                and (now - robot.latest_plan_time) <= 5.0
            ):
                plans_world[ns] = robot.latest_plan_world
            elif robot.state in ("NAVIGATING", "DISPATCHING") and robot.current_goal_world is not None:
                plans_world[ns] = [(robot.world_x, robot.world_y), robot.current_goal_world]
            else:
                plans_world[ns] = []

        for ns, robot in self.robots.items():
            if robot.teammates_pub is None:
                continue
            # Emit in ego's base_footprint frame so Nav2 VoxelLayer's raytrace
            # sensor origin stays centered inside the rolling local_costmap
            # regardless of how far the robot travels from the map origin.
            tf_w2b = se2_inverse((robot.world_x, robot.world_y, robot.world_yaw))
            pts = build_teammate_cloud_points(
                ns, poses_world, plans_world, tf_w2b, self.teammate_cloud_params
            )
            if len(pts) > 0:
                robot.teammates_pub.publish(pack_xyz32_cloud(pts, "base_footprint", stamp_msg))

    # ------------------------------------------------------------------
    # Main coordination cycle
    # ------------------------------------------------------------------
    def coordination_cycle(self) -> None:
        if not self.cycle_lock.acquire(blocking=False):
            return
        t0 = time.perf_counter()
        try:
            self._coordination_cycle_impl()
        finally:
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self.cycle_ms_pub.publish(Float64(data=float(elapsed_ms)))
            self.cycle_lock.release()

    def _coordination_cycle_impl(self) -> None:
        if self._exploration_halted:
            self._stop_all_robots()
            return
        if self.latest_map is None:
            return
        now = self.get_sim_time_sec()
        if now == 0.0:
            return

        self._check_nav_stack(now)
        self._check_backup_timeout(now)
        self._last_cycle_time = now
        self.update_robot_poses()

        map_msg = self.latest_map
        map_changed = id(map_msg) != self._cached_map_id or self._cached_occ is None
        if map_changed:
            try:
                occ = OccupancyView.from_grid(map_msg)
            except Exception as exc:
                self.get_logger().error(f"Failed to build occupancy view: {exc}")
                return
            self._cached_occ = occ
            self._cached_map_id = id(map_msg)
        else:
            occ = self._cached_occ

        self._check_map_jump(occ)
        self._check_cloud_staleness(now)

        self._check_action_timeouts(now)
        self._check_stalls(now)
        self._check_goal_invalidation(occ, now)

        # Layer-2 Topological Corridor Mutex: update active reservations and
        # pre-empt head-on conflicts before dispatching new goals.
        if (
            self.corridor_mutex_enable
            and len(self.robots) >= 2
            and not self.corridor_mgr.segments
            and (now - self._last_corridor_update_time) >= 6.0
        ):
            self.corridor_mgr.update_map(occ)
            self._last_corridor_update_time = now
        if self.corridor_mutex_enable and len(self.robots) >= 2 and self.corridor_mgr.segments:
            self._update_corridor_reservations_and_yield(occ, None, now)

        if self.voxel_index is None and self._latest_cloud is not None:
            self._maybe_update_voxel_index(now, occ)

        idle_robots = [
            r for r in self.robots.values()
            if r.state == "IDLE" and now >= r.yield_until_time
        ]

        geo: Optional[GeodesicCache] = None
        if idle_robots and self.use_geodesic_distance and has_cpp_kernel():
            geo = GeodesicCache(
                occ,
                min_clearance_m=self.geodesic_min_clearance,
                max_dist_m=self.geodesic_max_dist,
            )

        ck = (id(occ), len(self.bookkeeper.blacklisted_goals))
        if self._cached_clusters_key == ck:
            clusters = list(self._cached_clusters)
        else:
            clusters = extract_frontiers(occ, self.frontier_params, self.bookkeeper.blacklisted_goals)
            self._cached_clusters = list(clusters)
            self._cached_clusters_key = ck
        self.frontier_empty_cycles = self.frontier_empty_cycles + 1 if not clusters else 0

        skip_3d = (not idle_robots) or (not self._index_available(now))
        score_frontiers(
            clusters,
            resolution=occ.resolution,
            gain_scale=self.gain_scale,
            gain_3d_scale=self.gain_3d_scale,
            voxel_index=self.voxel_index,
            info_gain_params=self.info_gain_params,
            use_3d_gain=self.use_3d_gain,
            skip_3d=skip_3d,
        )

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
                blacklist=self.bookkeeper.residual_blacklist,
                attempts=self.bookkeeper.residual_attempts,
                max_attempts=self.residual_max_attempts,
            )

        candidates: List[object] = list(clusters) + list(residual)
        available = self._filter_available(candidates)
        dispatch_allowed = not (self.convergence.converged and self.convergence_stop_dispatch)

        ownership = []
        dispatched = set()
        if dispatch_allowed and available:
            if geo is not None and idle_robots:
                robot_xy = [(r.world_x, r.world_y) for r in self.robots.values()]
                cand_xy = [(float(c.world_x), float(c.world_y)) for c in available]
                geo.precompute_distances(robot_xy, cand_xy)
            if self.local_scheduling_enable:
                ownership = self._schedule_hierarchical(
                    occ, available, idle_robots, dispatched, now, geo
                )
            elif idle_robots:
                self._run_legacy_auction(idle_robots, available, occ.resolution, dispatched)

        if dispatch_allowed and available and idle_robots and not dispatched:
            still_idle = [
                r for r in self.robots.values()
                if r.state == "IDLE" and now >= r.yield_until_time
            ]
            if still_idle:
                self._warn_dispatch_starved(
                    f"{len(still_idle)} idle robot(s) and {len(available)} candidate(s) undispatched; falling back to utility auction."
                )
                self._run_legacy_auction(still_idle, available, occ.resolution, dispatched)

        self._update_convergence(now, len(clusters), len(residual), residual_raw_gain)
        markers = build_exploration_markers(
            self.get_clock().now().to_msg(), self.robots, available, ownership, residual
        )
        self.marker_pub.publish(markers)

    # ------------------------------------------------------------------
    # Corridor mutex & yield staging helpers
    # ------------------------------------------------------------------
    def _geodesic_path_fn(self, geo: Optional[GeodesicCache], occ: OccupancyView):
        if geo is not None:
            return geo.shortest_path

        def _fallback(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, np.ndarray]:
            return math.hypot(b[0] - a[0], b[1] - a[1]), np.array([a, b], dtype=np.float64)

        return _fallback

    def _update_corridor_reservations_and_yield(
        self, occ: OccupancyView, geo: Optional[GeodesicCache], now: float
    ) -> None:
        poses = {ns: (r.world_x, r.world_y, r.world_yaw) for ns, r in self.robots.items()}
        active_paths: Dict[str, List[Tuple[float, float]]] = {}
        path_fn = self._geodesic_path_fn(geo, occ)

        for ns, robot in self.robots.items():
            if robot.state not in ("NAVIGATING", "DISPATCHING"):
                continue
            if robot.latest_plan_world and (now - robot.latest_plan_time) <= 25.0:
                active_paths[ns] = robot.latest_plan_world
            elif robot.current_goal_world is not None:
                gx, gy = robot.current_goal_world
                active_paths[ns] = [
                    (
                        robot.world_x + (k / 12.0) * (gx - robot.world_x),
                        robot.world_y + (k / 12.0) * (gy - robot.world_y),
                    )
                    for k in range(13)
                ]

        self.corridor_mgr.update_reservations(poses, active_paths, now)

        # Interrupt low-priority robots approaching a narrow bottleneck head-on
        for ns, cid, owner_ns, yield_pose in self.corridor_mgr.detect_head_on_yielders(
            poses, active_paths
        ):
            robot = self.robots.get(ns)
            if robot is None or robot.yielding_corridor_id == cid:
                continue
            self.get_logger().info(
                f"[CorridorMutex] {ns} yielding narrow segment #{cid} to {owner_ns}"
            )
            self._cancel_robot_goal(robot)
            robot.yielding_corridor_id = cid
            if yield_pose is not None:
                yx, yy, yyaw = yield_pose
                if math.hypot(yx - robot.world_x, yy - robot.world_y) > 0.45:
                    self._dispatch_yield_pose(robot, yx, yy, yyaw)
                else:
                    robot.yield_until_time = now + 2.5
            else:
                robot.yield_until_time = now + 2.5

        # Release yield holds once corridor is no longer reserved by another robot
        for ns, robot in self.robots.items():
            if robot.yielding_corridor_id is not None:
                holder = self.corridor_mgr.reservations.get(robot.yielding_corridor_id)
                if holder is None or holder[0] == ns:
                    robot.yielding_corridor_id = None
                    robot.yield_until_time = -1e9

    def _dispatch_yield_pose(
        self, robot: RobotTracker, yx: float, yy: float, yyaw: float
    ) -> bool:
        from types import SimpleNamespace

        yield_cand = SimpleNamespace(
            x=yx, y=yy, world_x=yx, world_y=yy, yaw_world=yyaw, kind="yield"
        )
        return self.dispatch_goal(robot, yield_cand)

    # ------------------------------------------------------------------
    # Scheduling helpers
    # ------------------------------------------------------------------
    def _filter_available(self, candidates: List[object]) -> List[object]:
        if self.local_scheduling_enable:
            active_wps = self._all_active_waypoints()
            available = exclude_near_waypoints(candidates, active_wps, self.exclusion_radius)
            if not available and candidates and active_wps:
                available = exclude_near_waypoints(candidates, active_wps, 0.8)
            return available

        active_goals = [
            r.current_goal_world
            for r in self.robots.values()
            if r.state == "NAVIGATING" and r.current_goal_world is not None
        ]
        available = exclude_near_waypoints(candidates, active_goals, self.exclusion_radius)
        if not available and candidates and active_goals:
            available = exclude_near_waypoints(candidates, active_goals, 0.8)
        return available

    def _schedule_hierarchical(
        self,
        occ: OccupancyView,
        available: List[object],
        idle_robots: List[RobotTracker],
        dispatched: set,
        now: float,
        geo: Optional[GeodesicCache] = None,
    ) -> List:
        dist_fn = geo.distance if geo is not None else euclidean_distance
        path_fn = self._geodesic_path_fn(geo, occ)
        use_mutex = self.corridor_mutex_enable and len(self.robots) >= 2 and bool(self.corridor_mgr.segments)

        agents = [self._agent_state(r, occ) for r in self.robots.values()]
        ownership = assign_ownership(agents, available, self.local_radius, distance_fn=dist_fn)

        local_by_robot: Dict[str, List[object]] = {}
        for cand, own in zip(available, ownership):
            if own.pool == "local" and own.owner is not None:
                local_by_robot.setdefault(own.owner, []).append(cand)

        for robot in idle_robots:
            pool = local_by_robot.get(robot.namespace, [])
            if not pool:
                continue
            robot_comp = occ.component_at_world(robot.world_x, robot.world_y)
            if robot_comp <= 0:
                continue
            pool = [c for c in pool if getattr(c, "component_label", -1) in (-1, robot_comp)]
            if not pool:
                continue

            if use_mutex:
                safe_pool, conflict = filter_corridor_safe_candidates(
                    robot.namespace, (robot.world_x, robot.world_y), pool, self.corridor_mgr, path_fn
                )
                if safe_pool:
                    pool = safe_pool
                elif conflict is not None:
                    # Leave robot for global pool to check alternate branches before yielding
                    continue

            tour = build_local_tour(
                pool, (robot.world_x, robot.world_y), self.tour_params, distance_fn=dist_fn
            )
            if tour and self.dispatch_tour(robot, tour):
                dispatched.add(robot.namespace)
                if use_mutex:
                    _d, pxy = path_fn((robot.world_x, robot.world_y), (tour[0].world_x, tour[0].world_y))
                    self.corridor_mgr.reserve_for_robot(robot.namespace, pxy, now)

        global_pool = self._build_global_pool(available, ownership, dispatched, now)
        remaining_idle = [r for r in idle_robots if r.namespace not in dispatched]
        if remaining_idle and global_pool:
            raw_pool = list(global_pool)
            global_pool = exclude_near_waypoints(
                global_pool, self._all_active_waypoints(), self.exclusion_radius
            ) or exclude_near_waypoints(raw_pool, self._all_active_waypoints(), 0.8)
            if global_pool:
                if self.global_roi_enable:
                    n = self._run_global_auction(
                        remaining_idle, global_pool, occ, dispatched, now, geo
                    )
                    if n == 0 and self._global_reject_info is not None:
                        pool_n, best_roi = self._global_reject_info
                        self._warn_dispatch_starved(
                            f"{pool_n} global candidate(s) rejected by ROI gate (best {best_roi:.3f} < {self.roi_params.min_roi:.3f}); falling back."
                        )
                        self._run_legacy_auction(remaining_idle, global_pool, occ.resolution, dispatched)
                else:
                    self._run_legacy_auction(remaining_idle, global_pool, occ.resolution, dispatched)
        return ownership

    def _agent_state(self, robot: RobotTracker, occ: OccupancyView) -> AgentState:
        return AgentState(
            namespace=robot.namespace,
            x=robot.world_x,
            y=robot.world_y,
            yaw=robot.world_yaw,
            idle=robot.state == "IDLE",
            component_label=occ.component_at_world(robot.world_x, robot.world_y),
        )

    def _all_active_waypoints(self) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []
        for robot in self.robots.values():
            if robot.state not in ("NAVIGATING", "DISPATCHING"):
                continue
            if robot.active_waypoints_world:
                points.extend(robot.active_waypoints_world[max(0, robot.current_waypoint_index) :])
            elif robot.current_goal_world is not None:
                points.append(robot.current_goal_world)
        return points

    def _build_global_pool(
        self, available: List[object], ownership: List, dispatched: set, now: float
    ) -> List[object]:
        pool: List[object] = []
        selected_keys = {
            (round(c.world_x, 1), round(c.world_y, 1))
            for r in self.robots.values()
            for c in r.tour_candidates
        }
        for cand, own in zip(available, ownership):
            if own.pool == "global":
                pool.append(cand)
                continue
            if not self.local_release_unselected or own.owner in dispatched:
                continue
            key = (round(cand.world_x, 1), round(cand.world_y, 1))
            if key in selected_keys:
                continue
            first_seen = self._local_seen.setdefault(key, now)
            if now - first_seen < self.local_release_cooldown:
                continue
            try:
                cand.bid_gain = cand.total_gain * self.local_release_score_penalty
            except Exception:
                pass
            pool.append(cand)
        pool.sort(key=lambda c: float(getattr(c, "total_gain", 0.0)), reverse=True)
        return pool[: self.global_max_candidates] if self.global_max_candidates > 0 else pool

    def _run_legacy_auction(
        self, idle_robots: List[RobotTracker], clusters: List[object], res: float, dispatched: set
    ) -> int:
        remaining_robots = list(idle_robots)
        remaining_clusters = list(clusters)
        all_agents = [
            AgentState(r.namespace, r.world_x, r.world_y, r.world_yaw, r.state == "IDLE")
            for r in self.robots.values()
        ]
        count = 0
        while remaining_robots and remaining_clusters:
            best_bid = -float("inf")
            best_match = None
            for robot in remaining_robots:
                agent = AgentState(robot.namespace, robot.world_x, robot.world_y, robot.world_yaw, True)
                for cluster in remaining_clusters:
                    util = calculate_legacy_utility(agent, all_agents, cluster, res, self.utility_params)
                    if util > best_bid:
                        best_bid = util
                        best_match = (robot, cluster)
            if best_match is None:
                break
            winner, won = best_match
            if not self.dispatch_goal(winner, won):
                remaining_robots.remove(winner)
                continue
            count += 1
            dispatched.add(winner.namespace)
            remaining_robots.remove(winner)
            remaining_clusters = self._prune_after_win(remaining_clusters, won, bool(remaining_robots))
        return count

    def _run_global_auction(
        self,
        idle_robots: List[RobotTracker],
        clusters: List[object],
        occ: OccupancyView,
        dispatched: set,
        now: float = 0.0,
        geo: Optional[GeodesicCache] = None,
    ) -> int:
        dist_fn = geo.distance if geo is not None else euclidean_distance
        path_fn = self._geodesic_path_fn(geo, occ)
        use_mutex = self.corridor_mutex_enable and len(self.robots) >= 2 and bool(self.corridor_mgr.segments)

        remaining_robots = list(idle_robots)
        remaining_clusters = list(clusters)
        count = 0
        best_rejected_roi = -float("inf")
        self._global_reject_info = None
        blocked_robots: Dict[str, Tuple[int, str]] = {}

        while remaining_robots and remaining_clusters:
            best_bid = -float("inf")
            best_match = None
            for robot in remaining_robots:
                agent = self._agent_state(robot, occ)
                cands_for_robot = remaining_clusters
                if use_mutex:
                    safe_cands, conflict = filter_corridor_safe_candidates(
                        robot.namespace,
                        (robot.world_x, robot.world_y),
                        remaining_clusters,
                        self.corridor_mgr,
                        path_fn,
                    )
                    if safe_cands:
                        cands_for_robot = safe_cands
                    elif conflict is not None:
                        blocked_robots[robot.namespace] = conflict
                        continue
                for cand in cands_for_robot:
                    cand_comp = int(getattr(cand, "component_label", -1))
                    if agent.component_label > 0 and cand_comp > 0 and agent.component_label != cand_comp:
                        continue
                    bid, roi, dist = global_bid(agent, cand, self.roi_params, distance_fn=dist_fn)
                    if not math.isfinite(dist) or roi < self.roi_params.min_roi or (
                        self.roi_params.max_distance > 0.0 and dist > self.roi_params.max_distance
                    ):
                        best_rejected_roi = max(best_rejected_roi, roi)
                        continue
                    if bid > best_bid:
                        best_bid = bid
                        best_match = (robot, cand)
            if best_match is None:
                break
            winner, won = best_match
            if not self.dispatch_goal(winner, won):
                remaining_robots.remove(winner)
                continue
            if use_mutex:
                _d, pxy = path_fn((winner.world_x, winner.world_y), (won.world_x, won.world_y))
                self.corridor_mgr.reserve_for_robot(winner.namespace, pxy, now)
            count += 1
            dispatched.add(winner.namespace)
            blocked_robots.pop(winner.namespace, None)
            remaining_robots.remove(winner)
            remaining_clusters = self._prune_after_win(remaining_clusters, won, bool(remaining_robots))

        # Any robot whose ONLY reachable candidates cross an occupied bottleneck
        # stages cleanly at the bottleneck's spacious Yield Point instead of deadlocking.
        for robot in remaining_robots:
            conflict = blocked_robots.get(robot.namespace)
            if conflict is None:
                continue
            cid, owner_ns = conflict
            owner = self.robots.get(owner_ns)
            blocker_xy = (owner.world_x, owner.world_y) if owner is not None else None
            yp = self.corridor_mgr.select_yield_point(cid, (robot.world_x, robot.world_y), blocker_xy)
            robot.yielding_corridor_id = cid
            if yp is not None and math.hypot(yp[0] - robot.world_x, yp[1] - robot.world_y) > 0.45:
                if self._dispatch_yield_pose(robot, yp[0], yp[1], yp[2]):
                    dispatched.add(robot.namespace)
                    count += 1
            else:
                robot.yield_until_time = now + 2.0
                dispatched.add(robot.namespace)

        if count == 0 and clusters and best_rejected_roi > -float("inf"):
            self._global_reject_info = (len(clusters), best_rejected_roi)
        return count

    def _prune_after_win(
        self, clusters: List[object], won: object, more_robots: bool
    ) -> List[object]:
        nxt = [
            c for c in clusters
            if math.hypot(c.world_x - won.world_x, c.world_y - won.world_y) >= self.exclusion_radius
        ]
        if not nxt and more_robots:
            nxt = [
                c for c in clusters
                if math.hypot(c.world_x - won.world_x, c.world_y - won.world_y) >= 0.8
            ]
        return nxt

    # ------------------------------------------------------------------
    # 3D voxel index & convergence
    # ------------------------------------------------------------------
    def _check_map_jump(self, view: OccupancyView) -> None:
        extent = (view.origin_x, view.origin_y, view.width * view.resolution, view.height * view.resolution)
        if self._prev_map_extent is not None:
            px, py, pw, ph = self._prev_map_extent
            x, y, w, h = extent
            if (
                abs(x - px) > 0.30 * max(pw, 0.5)
                or abs(y - py) > 0.30 * max(ph, 0.5)
                or abs(w - pw) > 0.30 * max(pw, 0.5)
                or abs(h - ph) > 0.30 * max(ph, 0.5)
            ):
                self.convergence.reset("map jump")
                self._skip_invalidation_once = True
                self.voxel_index = None
                self._voxel_prev_keys = None
                self._last_voxel_build_time = -1e9
        self._prev_map_extent = extent

    def _check_cloud_staleness(self, now: float) -> None:
        if not self.use_3d_gain:
            return
        if self._cloud_age(now) > self.voxel_cloud_warn_timeout:
            if not self._cloud_warned:
                self.get_logger().warning(
                    f"[3D] No OctoMap cloud on {self.octomap_cloud_topic}; degrading to 2D gain."
                )
                self._cloud_warned = True
            self.voxel_index = None
            self._voxel_prev_keys = None
            self._last_voxel_build_time = -1e9

    @staticmethod
    def _fast_read_xyz(cloud_msg: PointCloud2) -> np.ndarray:
        """Zero-copy fast path for float32 XYZ PointCloud2 (12/16/32-byte point_step)."""
        if not cloud_msg.is_bigendian and cloud_msg.point_step >= 12:
            offs = {}
            for f in cloud_msg.fields:
                if f.datatype == 7 and f.name in ("x", "y", "z"):
                    offs[f.name] = int(f.offset)
            if len(offs) == 3:
                n = int(cloud_msg.width) * int(cloud_msg.height)
                if n == 0:
                    return np.empty((0, 3), dtype=np.float64)
                step = int(cloud_msg.point_step)
                stride = max(1, n // 30000)
                if step == 12 and offs["x"] == 0 and offs["y"] == 4 and offs["z"] == 8:
                    arr = np.frombuffer(cloud_msg.data, dtype="<f4", count=n * 3).reshape(n, 3)
                    return arr[::stride].astype(np.float64, copy=False)
                dt = np.dtype(
                    {
                        "names": ["x", "y", "z"],
                        "formats": ["<f4", "<f4", "<f4"],
                        "offsets": [offs["x"], offs["y"], offs["z"]],
                        "itemsize": step,
                    }
                )
                rec = np.frombuffer(cloud_msg.data, dtype=dt, count=n)[::stride]
                return np.column_stack((rec["x"], rec["y"], rec["z"])).astype(np.float64, copy=False)
        pts = point_cloud2.read_points_numpy(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        return np.asarray(pts, dtype=np.float64).reshape(-1, 3)

    def _maybe_update_voxel_index(self, now: float, occ: OccupancyView) -> None:
        if not self.use_3d_gain or self._latest_cloud is None or point_cloud2 is None:
            return
        if self._cloud_age(now) > self.voxel_cloud_warn_timeout:
            return
        if now - self._last_voxel_build_time < self.voxel_update_period:
            return
        frame = (self._latest_cloud.header.frame_id or "").lstrip("/")
        if frame not in ("world", ""):
            if not self._cloud_frame_warned:
                self.get_logger().warning(f'OctoMap cloud frame "{frame}" is not "world"; ignoring.')
                self._cloud_frame_warned = True
            return
        try:
            pts = self._fast_read_xyz(self._latest_cloud)
            new_index = VoxelColumnIndex.build(pts, occ, self.voxel_params)
        except Exception as exc:
            if now - self._voxel_error_log_time > 10.0:
                self.get_logger().warning(f"Failed to update voxel index: {exc}")
                self._voxel_error_log_time = now
            return
        new_cols = new_index.new_column_count(self._voxel_prev_keys)
        self.voxel_index = new_index
        self._voxel_prev_keys = new_index.column_keys
        self._voxel_new_columns = new_cols
        self._voxel_total_columns = new_index.total_columns
        self._last_voxel_build_time = now
        self._voxel_build_count += 1

    def _update_convergence(
        self, now: float, frontier_count: int, residual_count: int, residual_raw_gain: float
    ) -> None:
        map_valid = self._map_valid(now)
        if not map_valid and self.convergence.converged:
            self.convergence.reset("map lost")
        if self.convergence.converged and frontier_count > 0:
            self.convergence.reset("new frontier")
        idx_ok = self._index_available(now)
        state = self.convergence.update(
            now,
            map_valid=map_valid,
            frontier_count=frontier_count,
            all_idle=all(r.state == "IDLE" for r in self.robots.values()),
            residual_count=residual_count,
            residual_raw_gain=residual_raw_gain,
            index_available=idx_ok,
            new_columns=self._voxel_new_columns if idx_ok else 0,
            total_columns=self._voxel_total_columns if idx_ok else 0,
        )
        self.converged_pub.publish(Bool(data=bool(state.converged)))
        if state.converged and not self._converged_logged:
            self.get_logger().info(f"Exploration converged ({state.reason}).")
            self._converged_logged = True
        elif not state.converged:
            self._converged_logged = False

    # ------------------------------------------------------------------
    # Action dispatch & callbacks
    # ------------------------------------------------------------------
    @staticmethod
    def _action_ready(client: Optional[ActionClient]) -> bool:
        if client is None:
            return False
        try:
            return client.server_is_ready() or bool(client.wait_for_server(timeout_sec=0.0))
        except Exception:
            return False

    @staticmethod
    def _candidate_yaw(cand, robot: RobotTracker) -> float:
        yaw = getattr(cand, "yaw_world", None)
        if yaw is None:
            yaw = getattr(cand, "yaw", None)
        if yaw is None:
            yaw = math.atan2(cand.world_y - robot.world_y, cand.world_x - robot.world_x)
        return float(yaw)

    @staticmethod
    def _residual_key_for(cand) -> Optional[Tuple[float, float]]:
        if getattr(cand, "kind", "frontier") != "residual":
            return None
        return (
            round(float(getattr(cand, "cluster_x", cand.world_x)), 1),
            round(float(getattr(cand, "cluster_y", cand.world_y)), 1),
        )

    def dispatch_goal(self, robot: RobotTracker, cluster) -> bool:
        with self._state_lock:
            if self._exploration_halted or not self._action_ready(robot.action_client):
                return False
            tf_w2l = self._world_to_local_transform(robot)
            yaw_world = self._candidate_yaw(cluster, robot)
            xl, yl, heading_local = se2_apply(tf_w2l, cluster.world_x, cluster.world_y, yaw_world)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = xl
            goal_msg.pose.pose.position.y = yl
            goal_msg.pose.pose.orientation = yaw_to_quaternion(heading_local)

            now = self.get_sim_time_sec()
            robot.goal_token += 1
            token = robot.goal_token
            robot.state = "DISPATCHING"
            robot.active_waypoints_world = [(cluster.world_x, cluster.world_y)]
            robot.tour_candidates = [cluster]
            robot.tour_poses_world = [(cluster.world_x, cluster.world_y, yaw_world)]
            robot.current_waypoint_index = 0
            robot.current_goal_world = (cluster.world_x, cluster.world_y)
            robot.goal_start_time = now
            robot.tour_start_time = now
            robot.last_progress_time = now
            robot.stall_ref_x, robot.stall_ref_y = robot.world_x, robot.world_y
            robot.tour_is_active = False
            robot.tour_uses_through_poses = False
            robot.rewrite_count = 0
            robot.current_residual_key = self._residual_key_for(cluster)

            self._publish_goal_pose(robot, cluster.world_x, cluster.world_y, yaw_world)
            try:
                fut = robot.action_client.send_goal_async(goal_msg)
            except Exception as exc:
                self.get_logger().error(f"[{robot.namespace}] send_goal_async error: {exc}")
                self._record_failure(robot.current_goal_world, robot.current_residual_key)
                self._reset_robot_nav(robot)
                return False
            fut.add_done_callback(lambda f, r=robot, t=token: self._on_goal_response(f, r, t))
            return True

    def dispatch_tour(self, robot: RobotTracker, tour: List[object]) -> bool:
        with self._state_lock:
            if not tour or self._exploration_halted:
                return False
            use_through = (
                len(tour) >= 2
                and self.use_through_poses
                and robot.use_through_poses
                and self._action_ready(robot.through_client)
            )
            if not use_through:
                return self.dispatch_goal(robot, tour[0])

            tf_w2l = self._world_to_local_transform(robot)
            local_pts = [se2_apply(tf_w2l, c.world_x, c.world_y, self._candidate_yaw(c, robot)) for c in tour]
            goal_msg = NavigateThroughPoses.Goal()
            stamp = self.get_clock().now().to_msg()
            goal_msg.poses.header.frame_id = "map"
            goal_msg.poses.header.stamp = stamp
            for i, (xl, yl, yawl) in enumerate(local_pts):
                if i + 1 < len(local_pts):
                    yawl = math.atan2(local_pts[i + 1][1] - yl, local_pts[i + 1][0] - xl)
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.header.stamp = stamp
                ps.pose.position.x = xl
                ps.pose.position.y = yl
                ps.pose.orientation = yaw_to_quaternion(yawl)
                goal_msg.poses.goals.append(ps)

            now = self.get_sim_time_sec()
            robot.goal_token += 1
            token = robot.goal_token
            robot.state = "DISPATCHING"
            robot.active_waypoints_world = [(c.world_x, c.world_y) for c in tour]
            robot.tour_candidates = list(tour)
            robot.tour_poses_world = [(c.world_x, c.world_y, self._candidate_yaw(c, robot)) for c in tour]
            robot.current_waypoint_index = 0
            robot.current_goal_world = robot.active_waypoints_world[0]
            robot.goal_start_time = now
            robot.tour_start_time = now
            robot.last_progress_time = now
            robot.stall_ref_x, robot.stall_ref_y = robot.world_x, robot.world_y
            robot.tour_is_active = True
            robot.tour_uses_through_poses = True
            robot.rewrite_count = 0
            robot.current_residual_key = self._residual_key_for(tour[0])
            self._publish_goal_pose(robot, tour[0].world_x, tour[0].world_y, self._candidate_yaw(tour[0], robot))

            try:
                fut = robot.through_client.send_goal_async(
                    goal_msg,
                    feedback_callback=lambda m, r=robot, t=token: self._on_tour_feedback(m, r, t),
                )
            except Exception as exc:
                self.get_logger().error(f"[{robot.namespace}] through_poses send error: {exc}")
                self._record_failure(robot.current_goal_world, robot.current_residual_key)
                self._reset_robot_nav(robot)
                return False
            fut.add_done_callback(lambda f, r=robot, t=token: self._on_goal_response(f, r, t))
            return True

    def _publish_goal_pose(self, robot: RobotTracker, gx: float, gy: float, yaw: float) -> None:
        if robot.goal_pub is None:
            return
        gp = PoseStamped()
        gp.header.frame_id = "world"
        gp.header.stamp = self.get_clock().now().to_msg()
        gp.pose.position.x = gx
        gp.pose.position.y = gy
        gp.pose.position.z = 0.05
        gp.pose.orientation = yaw_to_quaternion(yaw)
        robot.goal_pub.publish(gp)

    def _on_tour_feedback(self, msg, robot: RobotTracker, token: int) -> None:
        with self._state_lock:
            if token != robot.goal_token or not robot.active_waypoints_world:
                return
            fb = msg.feedback
            n = len(robot.active_waypoints_world)
            remaining = int(getattr(fb, "number_of_poses_remaining", n))
            idx = max(0, min(n - 1, n - remaining))
            robot.current_waypoint_index = idx
            robot.current_goal_world = robot.active_waypoints_world[idx]
            if idx < len(robot.tour_candidates):
                robot.current_residual_key = self._residual_key_for(robot.tour_candidates[idx])
            robot.last_distance_remaining = float(getattr(fb, "distance_remaining", float("nan")))

    def _on_goal_response(self, future, robot: RobotTracker, token: int) -> None:
        with self._state_lock:
            if token != robot.goal_token:
                try:
                    gh = future.result()
                    if gh is not None and gh.accepted:
                        gh.cancel_goal_async()
                except Exception:
                    pass
                return
            try:
                gh = future.result()
            except Exception:
                self._increment_through_failure(robot)
                target, key = robot.current_goal_world, robot.current_residual_key
                self._reset_robot_nav(robot)
                self._record_failure(target, key)
                return
            if not gh.accepted:
                self._increment_through_failure(robot)
                target, key = robot.current_goal_world, robot.current_residual_key
                self._reset_robot_nav(robot)
                self._record_failure(target, key)
                return
            robot.current_goal_handle = gh
            robot.state = "NAVIGATING"
            gh.get_result_async().add_done_callback(
                lambda rf, r=robot, t=token: self._on_goal_result(rf, r, t)
            )

    def _on_goal_result(self, future, robot: RobotTracker, token: int) -> None:
        with self._state_lock:
            if token != robot.goal_token:
                return
            try:
                wrapped = future.result()
                status = wrapped.status
            except Exception:
                target, key = robot.current_goal_world, robot.current_residual_key
                self._reset_robot_nav(robot)
                self._record_failure(target, key)
                return

            if status == GoalStatus.STATUS_SUCCEEDED:
                msg = check_no_progress_success(
                    robot,
                    robot.current_goal_world,
                    self.get_sim_time_sec(),
                    self.bookkeeper,
                    self.no_progress_time,
                    self.no_progress_distance,
                    self.no_progress_limit,
                )
                if msg:
                    self.get_logger().warning(msg)
                for cand in robot.tour_candidates:
                    rkey = self._residual_key_for(cand)
                    if rkey is not None:
                        self.bookkeeper.resolve_residual(rkey)
                robot.through_poses_failures = 0
                self._reset_robot_nav(robot)
                return

            if status == GoalStatus.STATUS_CANCELED:
                self._reset_robot_nav(robot)
                return

            error_msg = getattr(wrapped.result, "error_msg", "")
            target, key = robot.current_goal_world, robot.current_residual_key
            self._increment_through_failure(robot)
            self._reset_robot_nav(robot)
            self._record_failure(target, key)
            if is_out_of_bounds_failure(error_msg):
                self._start_backup_recovery(robot)

    # ------------------------------------------------------------------
    # Supervision, recovery & halt
    # ------------------------------------------------------------------
    def _record_failure(
        self, target: Optional[Tuple[float, float]], key: Optional[Tuple[float, float]]
    ) -> None:
        msg = self.bookkeeper.record_failure(target, key)
        if msg:
            self.get_logger().warning(msg)

    def _increment_through_failure(self, robot: RobotTracker) -> None:
        if not robot.tour_uses_through_poses:
            return
        robot.through_poses_failures += 1
        if robot.use_through_poses and robot.through_poses_failures >= self.through_poses_fail_limit:
            robot.use_through_poses = False
            self.get_logger().warning(
                f"[{robot.namespace}] NavigateThroughPoses failed {robot.through_poses_failures}x; degrading to NavigateToPose."
            )

    def _reset_robot_nav(self, robot: RobotTracker) -> None:
        with self._state_lock:
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
            robot.current_residual_key = None
            robot.state = "IDLE"

    def _cancel_robot_goal(self, robot: RobotTracker) -> None:
        with self._state_lock:
            if robot.current_goal_handle is not None:
                try:
                    robot.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass
            self._reset_robot_nav(robot)

    def _check_action_timeouts(self, now: float) -> None:
        with self._state_lock:
            for robot in self.robots.values():
                if robot.state == "DISPATCHING" and now - robot.goal_start_time > 10.0:
                    target, key = robot.current_goal_world, robot.current_residual_key
                    self._cancel_robot_goal(robot)
                    self._record_failure(target, key)
                elif robot.state == "NAVIGATING":
                    n = max(1, len(robot.active_waypoints_world))
                    limit = self.goal_timeout + (self.tour_timeout_per_waypoint * (n - 1) if robot.tour_is_active else 0.0)
                    if now - robot.goal_start_time > limit:
                        target, key = robot.current_goal_world, robot.current_residual_key
                        self._increment_through_failure(robot)
                        self._cancel_robot_goal(robot)
                        self._record_failure(target, key)

    def _check_stalls(self, now: float) -> None:
        with self._state_lock:
            for robot in self.robots.values():
                if robot.state != "NAVIGATING" or robot.current_goal_world is None:
                    continue
                if now - robot.goal_start_time < 5.0:
                    continue
                moved = math.hypot(robot.world_x - robot.stall_ref_x, robot.world_y - robot.stall_ref_y)
                if moved >= self.tour_stall_min_translation:
                    robot.stall_ref_x, robot.stall_ref_y = robot.world_x, robot.world_y
                    robot.last_progress_time = now
                    continue
                if now - robot.last_progress_time < self.tour_stall_window:
                    continue
                target, key = robot.current_goal_world, robot.current_residual_key
                self._increment_through_failure(robot)
                self._cancel_robot_goal(robot)
                self._record_failure(target, key)
                if target is not None:
                    self.bookkeeper.blacklist_frontier_point(target[0], target[1])
                    self._start_backup_recovery(robot)

    def _check_goal_invalidation(self, occ: OccupancyView, now: float) -> None:
        with self._state_lock:
            if self._skip_invalidation_once:
                self._skip_invalidation_once = False
                return
            for robot in self.robots.values():
                if robot.state not in ("NAVIGATING", "DISPATCHING") or not robot.active_waypoints_world:
                    continue
                if now - robot.goal_start_time < 8.0:
                    continue
                start = max(0, robot.current_waypoint_index)
                for idx in range(start, len(robot.active_waypoints_world)):
                    gx, gy = robot.active_waypoints_world[idx]
                    if math.hypot(gx - robot.world_x, gy - robot.world_y) < 1.0:
                        continue
                    cand = robot.tour_candidates[idx] if idx < len(robot.tour_candidates) else None
                    rkey = self._residual_key_for(cand) if cand is not None else None
                    invalid, _reason = check_waypoint_invalid(occ, robot, gx, gy, self.bookkeeper, rkey)
                    if not invalid:
                        continue
                    if idx == robot.current_waypoint_index:
                        self._cancel_robot_goal(robot)
                        break
                    if self.tour_rewrite_on_invalidation and robot.rewrite_count < self.tour_max_rewrites:
                        old_rewrites = robot.rewrite_count
                        suffix = [
                            c
                            for j, c in enumerate(robot.tour_candidates)
                            if j >= robot.current_waypoint_index
                            and j != idx
                            and math.hypot(c.world_x - robot.world_x, c.world_y - robot.world_y) >= 1.0
                            and occ.state_at_world(c.world_x, c.world_y) == 0
                        ]
                        if len(suffix) >= 2:
                            self.dispatch_tour(robot, suffix)
                            robot.rewrite_count = old_rewrites + 1
                        else:
                            self._cancel_robot_goal(robot)
                        break
                    self._cancel_robot_goal(robot)
                    break

    def _start_backup_recovery(self, robot: RobotTracker) -> None:
        if self._exploration_halted or not self.backup_recovery_enable or robot.backup_active:
            return
        now = self.get_sim_time_sec()
        if now - robot.backup_last_time < self.backup_cooldown:
            return
        if not self._action_ready(robot.backup_client):
            return
        goal = BackUp.Goal()
        goal.target = Point(x=-abs(self.backup_distance), y=0.0, z=0.0)
        goal.speed = float(self.backup_speed)
        goal.time_allowance = Duration(seconds=15.0).to_msg()
        robot.backup_active = True
        robot.backup_last_time = now
        robot.backup_count += 1
        robot.state = "RECOVERING"
        try:
            robot.backup_client.send_goal_async(goal).add_done_callback(
                lambda fut, r=robot: self._on_backup_response(fut, r)
            )
        except Exception:
            self._finish_backup(robot)

    def _on_backup_response(self, future, robot: RobotTracker) -> None:
        try:
            gh = future.result()
        except Exception:
            self._finish_backup(robot)
            return
        if gh is None or not gh.accepted:
            self._finish_backup(robot)
            return
        robot.backup_handle = gh
        gh.get_result_async().add_done_callback(lambda _f, r=robot: self._finish_backup(r))

    def _finish_backup(self, robot: RobotTracker) -> None:
        with self._state_lock:
            robot.backup_handle = None
            robot.backup_active = False
            if robot.state == "RECOVERING":
                robot.state = "IDLE"

    def _check_backup_timeout(self, now: float) -> None:
        for robot in self.robots.values():
            if robot.backup_active and now - robot.backup_last_time > 25.0:
                self._finish_backup(robot)

    def _check_nav_stack(self, now: float) -> None:
        all_down = True
        for robot in self.robots.values():
            ready = self._action_ready(robot.action_client) or self._action_ready(robot.through_client)
            if ready:
                all_down = False
                if robot.nav_down_since >= 0.0:
                    robot.nav_down_since = -1.0
                    robot.nav_warn_time = -1e9
                continue
            if robot.nav_down_since < 0.0:
                robot.nav_down_since = now
            outage = now - robot.nav_down_since
            if outage >= self.nav_outage_warn_period and now - robot.nav_warn_time >= self.nav_outage_warn_period:
                robot.nav_warn_time = now
                self.get_logger().error(f"[{robot.namespace}] Nav2 action server unreachable for {outage:.0f}s.")
        if all_down and not self._nav_outage_all_logged:
            outages = [now - r.nav_down_since for r in self.robots.values() if r.nav_down_since >= 0.0]
            if outages and min(outages) >= self.nav_outage_warn_period:
                self._nav_outage_all_logged = True
                self.get_logger().error("CRITICAL: no robot has a Nav2 action server.")
        elif not all_down:
            self._nav_outage_all_logged = False

    def _warn_dispatch_starved(self, detail: str) -> None:
        now = self.get_sim_time_sec()
        if now - self._last_starved_log_time >= 10.0:
            self._last_starved_log_time = now
            self.get_logger().warning(f"[Dispatch] {detail}")

    def _on_exploration_halt(self, msg: Bool) -> None:
        self._exploration_halted = bool(msg.data)
        if self._exploration_halted:
            self._stop_all_robots()

    def _stop_all_robots(self) -> None:
        for robot in self.robots.values():
            if robot.backup_handle is not None:
                try:
                    robot.backup_handle.cancel_goal_async()
                except Exception:
                    pass
                self._finish_backup(robot)
            if robot.state != "IDLE" or robot.current_goal_handle is not None:
                self._cancel_robot_goal(robot)


def main(args=None) -> None:
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


if __name__ == "__main__":
    main()
