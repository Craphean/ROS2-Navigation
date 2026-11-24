"""Execution supervision: timeouts, stall recovery, invalidation, blacklists, and watchdogs."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .occupancy_view import OccupancyView
from .robot_tracker import RobotTracker


class FailureBookkeeper:
    """Tracks frontier/residual blacklists and retry counts (pure Python, unit-testable)."""

    def __init__(
        self,
        blacklist_retry_limit: int = 2,
        blacklist_radius: float = 0.9,
        residual_blacklist_radius: float = 1.0,
        residual_max_attempts: int = 2,
        visit_blacklist_limit: int = 2,
        visit_cooldown_sec: float = 35.0,
        visit_radius: float = 1.2,
    ):
        self.blacklist_retry_limit = int(blacklist_retry_limit)
        self.blacklist_radius = float(blacklist_radius)
        self.residual_blacklist_radius = float(residual_blacklist_radius)
        self.residual_max_attempts = int(residual_max_attempts)
        self.visit_blacklist_limit = int(visit_blacklist_limit)
        self.visit_cooldown_sec = float(visit_cooldown_sec)
        self.visit_radius = float(visit_radius)

        self.blacklisted_goals: List[Tuple[float, float]] = []
        self.global_failed_goals: Dict[Tuple[float, float], int] = {}
        self.residual_blacklist: List[Tuple[float, float]] = []
        self.residual_attempts: Dict[Tuple[float, float], int] = {}
        self.visited_goals: Dict[Tuple[float, float], int] = {}
        self.visit_cooldowns: List[Tuple[float, float, float]] = []

    def is_frontier_blacklisted(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - bx, y - by) < self.blacklist_radius
            for bx, by in self.blacklisted_goals
        )

    def is_visit_cooled_down(self, x: float, y: float, now: float) -> bool:
        self.visit_cooldowns = [
            (vx, vy, exp) for vx, vy, exp in self.visit_cooldowns if now < exp
        ]
        return any(
            math.hypot(x - vx, y - vy) < self.visit_radius
            for vx, vy, _exp in self.visit_cooldowns
        )

    def has_active_visit_cooldowns(self, now: float) -> bool:
        self.visit_cooldowns = [
            (vx, vy, exp) for vx, vy, exp in self.visit_cooldowns if now < exp
        ]
        return bool(self.visit_cooldowns)

    def record_visit(
        self,
        target: Optional[Tuple[float, float]],
        now: float,
    ) -> Optional[str]:
        """Record arrival or lookahead handoff at a frontier pocket to break A<->B ping-pong."""
        if target is None:
            return None
        tx, ty = float(target[0]), float(target[1])
        self.visit_cooldowns = [
            (vx, vy, exp) for vx, vy, exp in self.visit_cooldowns if now < exp
        ]
        found_key = None
        for vx, vy in self.visited_goals:
            if math.hypot(tx - vx, ty - vy) < self.visit_radius:
                found_key = (vx, vy)
                break
        if found_key is None:
            found_key = (round(tx, 2), round(ty, 2))
            self.visited_goals[found_key] = 0

        self.visited_goals[found_key] += 1
        count = self.visited_goals[found_key]
        self.visit_cooldowns.append((found_key[0], found_key[1], float(now) + self.visit_cooldown_sec))

        if count >= self.visit_blacklist_limit:
            added = self.blacklist_frontier_point(found_key[0], found_key[1])
            if added:
                return (
                    f"Frontier pocket near {found_key} visited {count} times without clearing; "
                    f"permanently blacklisted (radius {self.blacklist_radius}m)."
                )
        return None

    def is_residual_blacklisted(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - bx, y - by) < self.residual_blacklist_radius
            for bx, by in self.residual_blacklist
        )

    def blacklist_frontier_point(self, x: float, y: float) -> bool:
        if self.is_frontier_blacklisted(x, y):
            return False
        self.blacklisted_goals.append((round(float(x), 2), round(float(y), 2)))
        return True

    def nearest_residual_key(
        self, key: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        best = None
        best_dist = self.residual_blacklist_radius
        for known in list(self.residual_attempts) + list(self.residual_blacklist):
            dist = math.hypot(key[0] - known[0], key[1] - known[1])
            if dist <= best_dist:
                best = known
                best_dist = dist
        return best

    def resolve_residual(self, key: Tuple[float, float]) -> bool:
        radius = self.residual_blacklist_radius
        if any(
            math.hypot(key[0] - bx, key[1] - by) <= radius
            for bx, by in self.residual_blacklist
        ):
            return False
        self.residual_blacklist.append(key)
        for known in list(self.residual_attempts):
            if math.hypot(key[0] - known[0], key[1] - known[1]) <= radius:
                self.residual_attempts.pop(known, None)
        return True

    def record_failure(
        self,
        target: Optional[Tuple[float, float]],
        residual_key: Optional[Tuple[float, float]] = None,
    ) -> Optional[str]:
        """Record a goal failure; returns a log message when a spot is blacklisted."""
        if target is None:
            return None

        if residual_key is not None:
            radius = self.residual_blacklist_radius
            store_key = self.nearest_residual_key(residual_key) or residual_key
            count = self.residual_attempts.get(store_key, 0) + 1
            self.residual_attempts[store_key] = count
            if count >= self.residual_max_attempts:
                if not any(
                    math.hypot(store_key[0] - bx, store_key[1] - by) <= radius
                    for bx, by in self.residual_blacklist
                ):
                    self.residual_blacklist.append(store_key)
                return (
                    f"[Residual] Blind spot near {store_key} failed "
                    f"{count} times; adding to residual blacklist."
                )
            return None

        tx, ty = target
        found_key = None
        for fx, fy in self.global_failed_goals:
            if math.hypot(tx - fx, ty - fy) < 0.6:
                found_key = (fx, fy)
                break
        if found_key is None:
            found_key = (round(tx, 2), round(ty, 2))
            self.global_failed_goals[found_key] = 0

        self.global_failed_goals[found_key] += 1
        current_count = self.global_failed_goals[found_key]
        if current_count >= self.blacklist_retry_limit:
            if found_key not in self.blacklisted_goals:
                self.blacklisted_goals.append(found_key)
            return (
                f"Goal area near {found_key} failed {current_count} times! "
                f"Adding to blacklisted goals (radius {self.blacklist_radius}m)."
            )
        return None


def is_out_of_bounds_failure(error_msg: str) -> bool:
    """True when NavfnPlanner rejected the start pose as outside the global costmap."""
    msg = (error_msg or "").lower()
    return "outside bounds" in msg or (
        "outside" in msg and ("bounds" in msg or "the map" in msg)
    )


def check_waypoint_invalid(
    occ: OccupancyView,
    robot: RobotTracker,
    gx: float,
    gy: float,
    bookkeeper: FailureBookkeeper,
    residual_key: Optional[Tuple[float, float]] = None,
) -> Tuple[bool, str]:
    """Check whether an active waypoint has been cleared, blocked, or disconnected."""
    gr, gc = occ.world_to_cell(gx, gy)
    if not occ.contains_cell(gr, gc):
        return True, "cleared by exploration"
    if int(occ.data[gr, gc]) > 50:
        return True, "blocked by obstacle"
    if bookkeeper.is_frontier_blacklisted(gx, gy):
        return True, "blocked by obstacle"

    if residual_key is not None:
        if bookkeeper.is_residual_blacklisted(residual_key[0], residual_key[1]):
            return True, "cleared by exploration"
    elif bookkeeper.is_residual_blacklisted(gx, gy):
        return True, "cleared by exploration"

    check_radius_cells = max(3, int(1.5 / occ.resolution))
    r_min = max(0, gr - check_radius_cells)
    r_max = min(occ.height, gr + check_radius_cells + 1)
    c_min = max(0, gc - check_radius_cells)
    c_max = min(occ.width, gc + check_radius_cells + 1)
    unknown_count = int(np.sum(occ.data[r_min:r_max, c_min:c_max] == -1))
    if unknown_count < 5:
        return True, "cleared by exploration"

    if occ.is_free[gr, gc]:
        goal_comp = int(occ.free_labels[gr, gc])
        robot_comp = occ.component_at_world(robot.world_x, robot.world_y)
        if goal_comp > 0 and robot_comp > 0 and goal_comp != robot_comp:
            return True, "blocked by obstacle"
    return False, ""


def check_no_progress_success(
    robot: RobotTracker,
    target: Optional[Tuple[float, float]],
    now: float,
    bookkeeper: FailureBookkeeper,
    no_progress_time: float = 2.0,
    no_progress_distance: float = 0.3,
    no_progress_limit: int = 2,
) -> Optional[str]:
    """Break instant-success livelocks on persistent frontier pockets."""
    if target is None:
        return None
    elapsed = now - robot.goal_start_time
    moved = math.hypot(
        robot.world_x - robot.stall_ref_x,
        robot.world_y - robot.stall_ref_y,
    )
    if elapsed >= no_progress_time or moved >= no_progress_distance:
        robot.no_progress_streak = 0
        robot.no_progress_key = None
        return None

    tx, ty = float(target[0]), float(target[1])
    if (
        robot.no_progress_key is not None
        and math.hypot(
            tx - robot.no_progress_key[0],
            ty - robot.no_progress_key[1],
        )
        <= bookkeeper.blacklist_radius
    ):
        robot.no_progress_streak += 1
    else:
        robot.no_progress_streak = 1
    robot.no_progress_key = (tx, ty)

    if robot.no_progress_streak >= no_progress_limit:
        key = robot.no_progress_key
        streak = robot.no_progress_streak
        if key not in bookkeeper.blacklisted_goals:
            bookkeeper.blacklisted_goals.append(key)
        robot.no_progress_key = None
        robot.no_progress_streak = 0
        return (
            f"[{robot.namespace}] Goal at ({tx:.2f}, {ty:.2f}) succeeded "
            f"instantly {streak} times without the frontier clearing. "
            f"Adding to blacklisted goals (radius {bookkeeper.blacklist_radius}m)."
        )

    return (
        f"[{robot.namespace}] Instant success at ({tx:.2f}, {ty:.2f}): "
        f"goal was already reached, frontier pocket may not clear "
        f"(no-progress {robot.no_progress_streak}/{no_progress_limit})."
    )
