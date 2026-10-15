#!/usr/bin/env python3
"""
Multi-Robot Auction Exploration Coordinator for Nav2 + slam_toolbox + multirobot_map_merge.

Subscribes to merged map (/map in 'world' frame), extracts global frontiers,
conducts utility-based auction between robots (robot1, robot2), converts world
coordinates to robot-local coordinates, and drives /<robot>/navigate_to_pose action servers.
"""

import math
from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


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
    def __init__(self, world_x: float, world_y: float, size: int, grid_r: int, grid_c: int):
        self.world_x = world_x
        self.world_y = world_y
        self.size = size
        self.grid_r = grid_r
        self.grid_c = grid_c


class RobotTracker:
    def __init__(self, namespace: str, init_pose: List[float]):
        self.namespace = namespace.strip('/')
        self.init_x = float(init_pose[0])
        self.init_y = float(init_pose[1])
        self.init_yaw = float(init_pose[2])

        self.state = 'IDLE'  # 'IDLE', 'NAVIGATING', 'TIMEOUT'
        self.current_goal_world: Optional[Tuple[float, float]] = None
        self.current_goal_handle = None
        self.goal_start_time: float = 0.0

        # Current estimated pose in 'world' frame
        self.world_x = self.init_x
        self.world_y = self.init_y
        self.world_yaw = self.init_yaw

        self.action_client: Optional[ActionClient] = None
        self.failed_goals_count: Dict[Tuple[int, int], int] = {}
        self.goal_pub = None
        self.plan_pub = None
        self.plan_sub = None


class MultiRobotCoordinator(Node):
    def __init__(self):
        super().__init__('multi_robot_coordinator')

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

        namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        if not namespaces:
            namespaces = ['robot1', 'robot2']

        self.rate_hz = self.get_parameter('rate_hz').value
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.gain_scale = float(self.get_parameter('gain_scale').value)
        self.potential_scale = float(self.get_parameter('potential_scale').value)
        self.orientation_scale = float(self.get_parameter('orientation_scale').value)
        self.exclusion_radius = float(self.get_parameter('exclusion_radius').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.blacklist_retry_limit = int(self.get_parameter('blacklist_retry_limit').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.min_clearance = float(self.get_parameter('min_clearance').value)

        # Track blacklisted goal locations: list of (x, y)
        self.blacklisted_goals: List[Tuple[float, float]] = []
        # Global failure counter shared across all robots: (x, y) -> count
        self.global_failed_goals: Dict[Tuple[float, float], int] = {}

        # Setup robot trackers
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
            self.robots[clean_ns] = RobotTracker(clean_ns, init_pose)

        self.cb_group = ReentrantCallbackGroup()

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action clients, goal publishers, and plan relays for each robot
        for ns, robot in self.robots.items():
            action_topic = f'/{ns}/navigate_to_pose'
            self.get_logger().info(f'Connecting ActionClient to {action_topic} ...')
            robot.action_client = ActionClient(self, NavigateToPose, action_topic, callback_group=self.cb_group)

            # Global goal pose publisher in 'world' frame for RViz display
            robot.goal_pub = self.create_publisher(
                PoseStamped,
                f'/{ns}/goal_pose',
                10
            )

            # Transformed plan publisher in 'world' frame for RViz display
            robot.plan_pub = self.create_publisher(
                Path,
                f'/{ns}/plan_world',
                10
            )

            # Local plan subscriber from Nav2 planner_server (header.frame_id='map')
            robot.plan_sub = self.create_subscription(
                Path,
                f'/{ns}/plan',
                lambda msg, r=robot: self.plan_callback(msg, r),
                10,
                callback_group=self.cb_group
            )

        # Merged map subscription
        self.latest_map: Optional[OccupancyGrid] = None
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1,
            callback_group=self.cb_group
        )

        # Marker publisher for visualization in RViz
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/exploration/markers',
            10
        )

        # Coordination loop timer
        timer_period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(timer_period, self.coordination_cycle, callback_group=self.cb_group)

        self.get_logger().info(
            f'MultiRobotCoordinator initialized for robots: {list(self.robots.keys())} '
            f'at {self.rate_hz} Hz. Listening on /map...'
        )

    def get_sim_time_sec(self) -> float:
        """Get simulation / node clock time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, msg: OccupancyGrid):
        """Receive global merged map and check for active goal invalidation."""
        self.latest_map = msg
        self.check_goal_invalidation(msg)

    def check_goal_invalidation(self, grid_msg: OccupancyGrid):
        """If an active goal's area has been cleared or blocked, cancel it to avoid futile navigation."""
        now = self.get_sim_time_sec()
        if now == 0.0 and self.get_parameter('use_sim_time').value:
            return

        width = grid_msg.info.width
        height = grid_msg.info.height
        res = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))

        for ns, robot in self.robots.items():
            if robot.state != 'NAVIGATING' or robot.current_goal_world is None:
                continue

            # Grace period: let the robot drive for at least 8.0s (in sim time) before checking invalidation
            # to avoid false cancellations right after goal dispatch.
            if now - robot.goal_start_time < 8.0:
                continue

            gx, gy = robot.current_goal_world
            gc = int((gx - origin_x) / res)
            gr = int((gy - origin_y) / res)

            if not (0 <= gr < height and 0 <= gc < width):
                continue

            # Condition 1: Goal cell has become an obstacle (> 50)
            is_obstacle = data[gr, gc] > 50

            # Condition 2: Surrounding area (radius 1.5m) has been completely cleared
            check_radius_cells = max(3, int(1.5 / res))
            r_min = max(0, gr - check_radius_cells)
            r_max = min(height, gr + check_radius_cells + 1)
            c_min = max(0, gc - check_radius_cells)
            c_max = min(width, gc + check_radius_cells + 1)
            sub_window = data[r_min:r_max, c_min:c_max]

            unknown_count = np.sum(sub_window == -1)
            cleared = (unknown_count < 5)

            if is_obstacle or cleared:
                reason = "blocked by obstacle" if is_obstacle else "cleared by exploration"
                self.get_logger().info(
                    f'[{ns}] Active goal ({gx:.2f}, {gy:.2f}) has been {reason}! Cancelling goal...'
                )
                self.cancel_robot_goal(robot)

    def update_robot_poses(self):
        """Update each robot's position in the global 'world' frame via TF or analytical transform."""
        for ns, robot in self.robots.items():
            # Attempt 1: Lookup 'world' -> '<ns>/base_link' or '<ns>/base_footprint'
            found_tf = False
            for base_frame in [f'{ns}/base_footprint', f'{ns}/base_link', 'base_footprint', 'base_link']:
                try:
                    t = self.tf_buffer.lookup_transform('world', base_frame, Time())
                    robot.world_x = t.transform.translation.x
                    robot.world_y = t.transform.translation.y
                    robot.world_yaw = quaternion_to_yaw(t.transform.rotation)
                    found_tf = True
                    break
                except Exception:
                    continue

            if found_tf:
                continue

            # Attempt 2: Lookup '<ns>/map' -> '<ns>/base_link' and apply analytical init_pose offset
            for base_frame in [f'{ns}/base_footprint', f'{ns}/base_link', 'base_footprint', 'base_link']:
                for map_frame in [f'{ns}/map', 'map']:
                    try:
                        t = self.tf_buffer.lookup_transform(map_frame, base_frame, Time())
                        local_x = t.transform.translation.x
                        local_y = t.transform.translation.y
                        local_yaw = quaternion_to_yaw(t.transform.rotation)

                        # Project local to world using robot.init_x, robot.init_y, robot.init_yaw
                        c = math.cos(robot.init_yaw)
                        s = math.sin(robot.init_yaw)
                        robot.world_x = robot.init_x + local_x * c - local_y * s
                        robot.world_y = robot.init_y + local_x * s + local_y * c
                        robot.world_yaw = robot.init_yaw + local_yaw
                        found_tf = True
                        break
                    except Exception:
                        continue
                if found_tf:
                    break

    def split_cluster_if_large(
        self,
        cluster_cells: List[Tuple[int, int]],
        res: float,
        min_size: int = 8
    ) -> List[List[Tuple[int, int]]]:
        """Split very spread-out/long frontier clusters so multi-robot auction can dispatch to different flanks."""
        if len(cluster_cells) < min_size * 2:
            return [cluster_cells]

        coords = np.array(cluster_cells)
        span = np.ptp(coords, axis=0)  # [span_r, span_c]
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
                results.extend(self.split_cluster_if_large(part, res, min_size))
            else:
                results.append(part)
        return results

    def find_safe_goal(
        self,
        cluster_cells: List[Tuple[int, int]],
        is_free: np.ndarray,
        obstacle_dt: np.ndarray,
        res: float
    ) -> Optional[Tuple[int, int]]:
        """
        Check clearance from known obstacles. If centroid is too close (< min_clearance),
        retract via BFS in known free space towards open area.
        If entire region is trapped in a narrow slit (< min_clearance), return None to drop it.
        """
        height, width = is_free.shape
        min_clearance_cells = self.min_clearance / res
        max_retract_cells = int(1.5 / res)

        mean_r = sum(c[0] for c in cluster_cells) / len(cluster_cells)
        mean_c = sum(c[1] for c in cluster_cells) / len(cluster_cells)

        # 1. Direct check: any cell in the cluster that already satisfies min_clearance?
        safe_cells = [
            (r, c) for (r, c) in cluster_cells
            if obstacle_dt[r, c] >= min_clearance_cells
        ]
        if safe_cells:
            return min(safe_cells, key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2)

        # 2. Retract towards open free space via BFS
        sorted_cluster = sorted(cluster_cells, key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2)
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
                if 0 <= nr < height and 0 <= nc < width and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    if is_free[nr, nc]:
                        q.append((nr, nc, steps + 1))

        # 3. Slit / dead-end check: cluster is squeezed in a tight corridor/nook (< robot width). Discard.
        return None

    def extract_frontiers(self, grid_msg: OccupancyGrid) -> List[FrontierCluster]:
        """Vectorized extraction of frontier cells from OccupancyGrid + BFS clustering + Safe Clearance Retraction."""
        width = grid_msg.info.width
        height = grid_msg.info.height
        res = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))
        is_free = (data >= 0) & (data < 20)
        is_unknown = (data == -1)
        is_lethal = (data > 50)

        # Obstacle distance field (in cells) to nearest lethal obstacle
        obstacle_binary = np.ones((height, width), dtype=np.uint8) * 255
        obstacle_binary[is_lethal] = 0
        obstacle_dt = cv2.distanceTransform(obstacle_binary, cv2.DIST_L2, 5)

        # Fast 8-neighbor detection via shifted padding
        unknown_padded = np.pad(is_unknown, 1, mode='constant', constant_values=False)
        lethal_padded = np.pad(is_lethal, 1, mode='constant', constant_values=False)

        has_unknown_neighbor = np.zeros((height, width), dtype=bool)
        has_lethal_neighbor = np.zeros((height, width), dtype=bool)

        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                has_unknown_neighbor |= unknown_padded[1 + dr:height + 1 + dr, 1 + dc:width + 1 + dc]
                has_lethal_neighbor |= lethal_padded[1 + dr:height + 1 + dr, 1 + dc:width + 1 + dc]

        frontier_mask = is_free & has_unknown_neighbor & (~has_lethal_neighbor)

        # BFS clustering
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
                sub_clusters = self.split_cluster_if_large(cluster_cells, res, self.min_frontier_size)
                raw_clusters.extend(sub_clusters)

        clusters: List[FrontierCluster] = []
        for sub_cells in raw_clusters:
            safe_cell = self.find_safe_goal(sub_cells, is_free, obstacle_dt, res)
            if safe_cell is None:
                continue

            safe_r, safe_c = safe_cell
            wx = origin_x + (safe_c + 0.5) * res
            wy = origin_y + (safe_r + 0.5) * res

            # Check if close to blacklisted locations
            is_blacklisted = any(
                math.hypot(wx - bx, wy - by) < self.blacklist_radius
                for bx, by in self.blacklisted_goals
            )
            if is_blacklisted:
                continue

            # Deduplicate targets that are very close to an already added candidate (< 0.4m)
            if any(math.hypot(wx - c.world_x, wy - c.world_y) < 0.4 for c in clusters):
                continue

            clusters.append(FrontierCluster(wx, wy, len(sub_cells), safe_r, safe_c))

        return clusters

    def coordination_cycle(self):
        """Main periodic auction and dispatch loop."""
        if self.latest_map is None:
            return

        now = self.get_sim_time_sec()
        if now == 0.0 and self.get_parameter('use_sim_time').value:
            return

        self.update_robot_poses()

        # Check timeouts for navigating robots using simulation time
        for ns, robot in self.robots.items():
            if robot.state == 'NAVIGATING':
                if now - robot.goal_start_time > self.goal_timeout:
                    self.get_logger().warning(
                        f'[{ns}] Goal timed out after {self.goal_timeout}s sim time! Cancelling...'
                    )
                    self.cancel_robot_goal(robot)

        # Check which robots are available to bid
        idle_robots = [robot for robot in self.robots.values() if robot.state == 'IDLE']
        if not idle_robots:
            return

        # Extract candidate frontiers from merged map
        clusters = self.extract_frontiers(self.latest_map)
        res = self.latest_map.info.resolution

        # Filter out clusters that are within exclusion_radius of active goals
        active_goals = [
            robot.current_goal_world for robot in self.robots.values()
            if robot.state == 'NAVIGATING' and robot.current_goal_world is not None
        ]

        available_clusters = []
        for c in clusters:
            conflict = any(
                math.hypot(c.world_x - ag[0], c.world_y - ag[1]) < self.exclusion_radius
                for ag in active_goals
            )
            if not conflict:
                available_clusters.append(c)

        # Anti-starvation fallback: if idle robots have no available clusters due to exclusion_radius,
        # adaptively relax exclusion radius (down to 0.8m) so idle robots can explore nearby available flanks
        if not available_clusters and clusters and active_goals:
            reduced_radius = 0.8
            for c in clusters:
                conflict = any(
                    math.hypot(c.world_x - ag[0], c.world_y - ag[1]) < reduced_radius
                    for ag in active_goals
                )
                if not conflict:
                    available_clusters.append(c)
            if available_clusters:
                self.get_logger().info(
                    f'Anti-starvation: relaxed exclusion_radius to {reduced_radius}m, found {len(available_clusters)} clusters.'
                )

        # Publish visualization markers
        self.publish_markers(available_clusters)

        if not available_clusters:
            if all(r.state == 'IDLE' for r in self.robots.values()):
                self.get_logger().info('No remaining reachable frontiers. Exploration complete or paused.')
            return

        # Run Auction
        self.run_auction(idle_robots, available_clusters, res)

    def calculate_utility(self, robot: RobotTracker, cluster: FrontierCluster, res: float) -> float:
        """Utility = alpha * Gain - beta * Distance - gamma * AngleDiff."""
        dx = cluster.world_x - robot.world_x
        dy = cluster.world_y - robot.world_y
        dist = math.hypot(dx, dy)

        # Only ignore frontiers that the robot is literally standing on (< 0.2m, within goal tolerance)
        if dist < 0.2:
            return -float('inf')

        heading_to_goal = math.atan2(dy, dx)
        angle_diff = abs(math.atan2(
            math.sin(heading_to_goal - robot.world_yaw),
            math.cos(heading_to_goal - robot.world_yaw)
        ))

        # Cap cluster size so giant perimeter frontiers don't overpower reachable local frontiers
        capped_size = min(cluster.size, 1500)
        gain = capped_size * res

        # Distance penalty: heavily penalize goals > 5.0m in large worlds like depot
        # to prevent planning across unreachable unmapped warehouse walls
        dist_cost = dist
        if dist > 5.0:
            dist_cost += (dist - 5.0) * 2.5

        utility = (
            self.gain_scale * gain
            - self.potential_scale * dist_cost
            - self.orientation_scale * angle_diff
        )
        return utility

    def run_auction(self, idle_robots: List[RobotTracker], clusters: List[FrontierCluster], res: float):
        """Sequential single-item greedy auction."""
        remaining_robots = list(idle_robots)
        remaining_clusters = list(clusters)

        while remaining_robots and remaining_clusters:
            best_bid = -float('inf')
            best_match: Optional[Tuple[RobotTracker, FrontierCluster]] = None

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

            # Assign goal to winning robot
            self.dispatch_goal(winner_robot, won_cluster)

            remaining_robots.remove(winner_robot)

            # Exclude clusters near the assigned goal to enforce spatial dispersion
            next_clusters = [
                c for c in remaining_clusters
                if math.hypot(c.world_x - won_cluster.world_x, c.world_y - won_cluster.world_y) >= self.exclusion_radius
            ]
            # Anti-starvation in auction: if remaining idle robots have no clusters at full exclusion radius,
            # relax to 0.8m so second robot can also take a goal in the same cycle
            if not next_clusters and remaining_robots:
                next_clusters = [
                    c for c in remaining_clusters
                    if math.hypot(c.world_x - won_cluster.world_x, c.world_y - won_cluster.world_y) >= 0.8
                ]
            remaining_clusters = next_clusters

    def world_to_robot_local(self, robot: RobotTracker, xw: float, yw: float) -> Tuple[float, float]:
        """Convert world coordinates (xw, yw) to robot local map frame coordinates (xl, yl)."""
        # Try TF first
        for map_frame in [f'{robot.namespace}/map', 'map']:
            try:
                t = self.tf_buffer.lookup_transform(map_frame, 'world', Time())
                # Apply 2D rigid transform from transform message
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tyaw = quaternion_to_yaw(t.transform.rotation)
                c = math.cos(tyaw)
                s = math.sin(tyaw)
                xl = tx + xw * c - yw * s
                yl = ty + xw * s + yw * c
                return xl, yl
            except Exception:
                continue

        # Analytical inverse transform: P_local = R(-yaw0) * (P_world - T0)
        dx = xw - robot.init_x
        dy = yw - robot.init_y
        c = math.cos(robot.init_yaw)
        s = math.sin(robot.init_yaw)
        xl = dx * c + dy * s
        yl = -dx * s + dy * c
        return xl, yl

    def robot_local_to_world(self, robot: RobotTracker, xl: float, yl: float, yawl: float = 0.0) -> Tuple[float, float, float]:
        """Convert robot local coordinates (xl, yl, yawl) to global 'world' frame."""
        for map_frame in [f'{robot.namespace}/map', 'map']:
            try:
                t = self.tf_buffer.lookup_transform('world', map_frame, Time())
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tyaw = quaternion_to_yaw(t.transform.rotation)
                c = math.cos(tyaw)
                s = math.sin(tyaw)
                xw = tx + xl * c - yl * s
                yw = ty + xl * s + yl * c
                yaww = tyaw + yawl
                return xw, yw, yaww
            except Exception:
                continue

        # Analytical forward transform: P_world = R(init_yaw) * P_local + T_init
        c = math.cos(robot.init_yaw)
        s = math.sin(robot.init_yaw)
        xw = robot.init_x + xl * c - yl * s
        yw = robot.init_y + xl * s + yl * c
        yaww = robot.init_yaw + yawl
        return xw, yw, yaww

    def plan_callback(self, msg: Path, robot: RobotTracker):
        """Transform robot local path (header.frame_id='map') into global 'world' frame for RViz."""
        if not msg.poses or robot.plan_pub is None:
            return

        world_path = Path()
        world_path.header.stamp = self.get_clock().now().to_msg()
        world_path.header.frame_id = 'world'

        for p in msg.poses:
            xl = p.pose.position.x
            yl = p.pose.position.y
            yawl = quaternion_to_yaw(p.pose.orientation)
            xw, yw, yaww = self.robot_local_to_world(robot, xl, yl, yawl)

            pw = PoseStamped()
            pw.header.stamp = world_path.header.stamp
            pw.header.frame_id = 'world'
            pw.pose.position.x = xw
            pw.pose.position.y = yw
            pw.pose.position.z = p.pose.position.z
            pw.pose.orientation = yaw_to_quaternion(yaww)
            world_path.poses.append(pw)

        robot.plan_pub.publish(world_path)

    def dispatch_goal(self, robot: RobotTracker, cluster: FrontierCluster):
        """Convert goal to local coordinates and send NavigateToPose action."""
        if robot.action_client is None or not robot.action_client.server_is_ready():
            self.get_logger().warning(f'[{robot.namespace}] Action server not ready!')
            return

        xl, yl = self.world_to_robot_local(robot, cluster.world_x, cluster.world_y)

        # Orient towards frontier from robot's current local position
        # Local robot position:
        rlx, rly = self.world_to_robot_local(robot, robot.world_x, robot.world_y)
        heading_local = math.atan2(yl - rly, xl - rlx)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = xl
        goal_msg.pose.pose.position.y = yl
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(heading_local)

        robot.state = 'NAVIGATING'
        robot.current_goal_world = (cluster.world_x, cluster.world_y)
        robot.goal_start_time = self.get_sim_time_sec()

        # Publish global goal pose in 'world' frame for RViz visualization
        if robot.goal_pub is not None:
            gp = PoseStamped()
            gp.header.frame_id = 'world'
            gp.header.stamp = self.get_clock().now().to_msg()
            gp.pose.position.x = cluster.world_x
            gp.pose.position.y = cluster.world_y
            gp.pose.position.z = 0.05
            heading_world = math.atan2(cluster.world_y - robot.world_y, cluster.world_x - robot.world_x)
            gp.pose.orientation = yaw_to_quaternion(heading_world)
            robot.goal_pub.publish(gp)

        self.get_logger().info(
            f'[{robot.namespace}] Sending Nav2 goal: world=({cluster.world_x:.2f}, {cluster.world_y:.2f}) -> '
            f'local=({xl:.2f}, {yl:.2f})'
        )

        send_future = robot.action_client.send_goal_async(
            goal_msg,
            feedback_callback=None
        )
        send_future.add_done_callback(
            lambda future, r=robot, target=robot.current_goal_world: self.goal_response_callback(future, r, target)
        )

    def goal_response_callback(self, future, robot: RobotTracker, target: Tuple[float, float]):
        """Callback when action server accepts or rejects goal."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(f'[{robot.namespace}] Goal rejected by server!')
                self.handle_goal_failure(robot, target)
                return

            if robot.current_goal_world != target:
                self.get_logger().info(
                    f'[{robot.namespace}] Goal accepted by server, but target was cancelled/superseded. Cancelling handle.'
                )
                goal_handle.cancel_goal_async()
                return

            self.get_logger().info(f'[{robot.namespace}] Goal accepted.')
            robot.current_goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda res_future, r=robot, t=target: self.goal_result_callback(res_future, r, t)
            )
        except Exception as e:
            self.get_logger().error(f'[{robot.namespace}] Exception in goal response: {e}')
            self.handle_goal_failure(robot, target)

    def goal_result_callback(self, future, robot: RobotTracker, target: Tuple[float, float]):
        """Callback when action server finishes navigation."""
        try:
            result = future.result()
            status = result.status
            is_current = (robot.current_goal_world == target)

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'[{robot.namespace}] Goal reached successfully! Target: {target}')
                if is_current:
                    robot.state = 'IDLE'
                    robot.current_goal_world = None
                    robot.current_goal_handle = None
                    if robot.plan_pub is not None:
                        empty_path = Path()
                        empty_path.header.stamp = self.get_clock().now().to_msg()
                        empty_path.header.frame_id = 'world'
                        robot.plan_pub.publish(empty_path)
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(f'[{robot.namespace}] Goal canceled cleanly. Target: {target}')
                if is_current:
                    robot.state = 'IDLE'
                    robot.current_goal_world = None
                    robot.current_goal_handle = None
                    if robot.plan_pub is not None:
                        empty_path = Path()
                        empty_path.header.stamp = self.get_clock().now().to_msg()
                        empty_path.header.frame_id = 'world'
                        robot.plan_pub.publish(empty_path)
            else:
                self.get_logger().warning(
                    f'[{robot.namespace}] Navigation failed with status code {status}.'
                )
                self.handle_goal_failure(robot, target)
        except Exception as e:
            self.get_logger().error(f'[{robot.namespace}] Exception in goal result: {e}')
            self.handle_goal_failure(robot, target)

    def handle_goal_failure(self, robot: RobotTracker, target: Tuple[float, float]):
        """Track failure count globally across all robots and blacklist if threshold exceeded."""
        if robot.current_goal_world == target or robot.current_goal_world is None:
            robot.state = 'IDLE'
            robot.current_goal_world = None
            robot.current_goal_handle = None

        if target is None:
            return

        tx, ty = target
        # Find existing nearby failure record within 0.6m to group close failure points
        found_key = None
        for (fx, fy) in self.global_failed_goals.keys():
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
                f'Goal area near {found_key} failed {current_count} times! Adding to blacklisted goals (radius {self.blacklist_radius}m).'
            )
            if found_key not in self.blacklisted_goals:
                self.blacklisted_goals.append(found_key)

    def cancel_robot_goal(self, robot: RobotTracker):
        """Cancel an in-flight navigation goal."""
        if robot.current_goal_handle is not None:
            try:
                robot.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f'[{robot.namespace}] Error cancelling goal: {e}')
        robot.state = 'IDLE'
        robot.current_goal_world = None
        robot.current_goal_handle = None
        if robot.plan_pub is not None:
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'world'
            robot.plan_pub.publish(empty_path)

    def publish_markers(self, clusters: List[FrontierCluster]):
        """Publish RViz visualization markers for candidate frontiers and assigned goals."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. Candidate frontier points
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

        for c in clusters:
            p = Point()
            p.x = c.world_x
            p.y = c.world_y
            p.z = 0.05
            frontier_marker.points.append(p)
        ma.markers.append(frontier_marker)

        # 2. Active assigned goals
        colors = {
            'robot1': (1.0, 0.2, 0.2),  # Red
            'robot2': (0.2, 0.5, 1.0),  # Blue
        }
        marker_id = 1
        for ns, robot in self.robots.items():
            if robot.state == 'NAVIGATING' and robot.current_goal_world is not None:
                gx, gy = robot.current_goal_world
                cr, cg, cb = colors.get(ns, (1.0, 1.0, 0.0))

                goal_sphere = Marker()
                goal_sphere.header.frame_id = 'world'
                goal_sphere.header.stamp = stamp
                goal_sphere.ns = 'assigned_goals'
                goal_sphere.id = marker_id
                marker_id += 1
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
                ma.markers.append(goal_sphere)

                text_marker = Marker()
                text_marker.header.frame_id = 'world'
                text_marker.header.stamp = stamp
                text_marker.ns = 'assigned_labels'
                text_marker.id = marker_id
                marker_id += 1
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
                ma.markers.append(text_marker)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotCoordinator()
    executor = rclpy.executors.MultiThreadedExecutor()
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

