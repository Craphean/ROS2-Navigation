"""Per-robot state container supporting arbitrary N robots."""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple


class RobotTracker:
    """Tracks pose, active tour, recovery, and health state for one robot."""

    def __init__(self, namespace: str, init_pose: Sequence[float]):
        self.namespace = namespace.strip("/")
        self.init_x = float(init_pose[0])
        self.init_y = float(init_pose[1])
        self.init_yaw = float(init_pose[2])

        # 'IDLE', 'DISPATCHING', 'NAVIGATING', 'RECOVERING'
        self.state = "IDLE"
        self.current_goal_world: Optional[Tuple[float, float]] = None
        self.current_goal_handle = None
        self.goal_start_time: float = 0.0
        self.goal_token: int = 0

        # Current estimated pose in 'world' frame
        self.world_x = self.init_x
        self.world_y = self.init_y
        self.world_yaw = self.init_yaw

        # ROS handles populated by the coordinator node
        self.action_client = None
        self.through_client = None
        self.backup_client = None
        self.goal_pub = None
        self.plan_pub = None
        self.plan_sub = None
        self.teammates_pub = None
        self.latest_plan_world: List[Tuple[float, float]] = []
        self.latest_plan_time: float = -1e9
        self.yielding_corridor_id: Optional[int] = None
        self.yield_until_time: float = -1e9
        self.yield_start_time: float = -1e9
        self.yield_cooldown_until: float = -1e9

        # Out-of-bounds self recovery
        self.backup_active: bool = False
        self.backup_handle = None
        self.backup_last_time: float = -1e9
        self.backup_count: int = 0
        self.failed_goals_count: Dict[Tuple[int, int], int] = {}

        # Active route / tour state
        self.active_waypoints_world: List[Tuple[float, float]] = []
        self.tour_candidates: List[object] = []
        self.tour_poses_world: List[Tuple[float, float, float]] = []
        self.current_waypoint_index: int = 0
        self.tour_start_time: float = 0.0
        self.last_progress_time: float = 0.0
        self.last_distance_remaining: float = float("nan")
        self.stall_distance_ref: float = float("inf")
        self.stall_ref_x: float = self.init_x
        self.stall_ref_y: float = self.init_y
        self.stall_ref_yaw: float = self.init_yaw
        self.consecutive_stalls: int = 0
        self.goal_initial_dist: float = 0.0
        self.through_poses_failures: int = 0
        self.use_through_poses: bool = True
        self.tour_is_active: bool = False
        self.tour_uses_through_poses: bool = False
        self.rewrite_count: int = 0
        self.current_residual_key: Optional[Tuple[float, float]] = None

        # No-progress livelock breaker
        self.no_progress_key: Optional[Tuple[float, float]] = None
        self.no_progress_streak: int = 0
        self.visit_recorded_token: int = -1

        # Relocalization jump detection
        self.prev_world_x: Optional[float] = None
        self.prev_world_y: Optional[float] = None

        # Nav2 stack availability watchdog
        self.nav_down_since: float = -1.0
        self.nav_warn_time: float = -1e9
