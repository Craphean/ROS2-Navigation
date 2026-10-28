"""In-memory fleet state shared by ROS callbacks and the web API.

ROS callbacks only write plain data under ``StateStore.lock``; PNG/JPEG
encoding and WebSocket serialisation always happen outside the lock.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any


GOAL_STATUS_TEXT = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def nav_state_from_status(code: int) -> str:
    if code == 1:
        return "PLANNING"
    if code in (2, 3):
        return "NAVIGATING"
    if code in (5, 6):
        return "RECOVERY"
    return "IDLE"


@dataclass
class RobotState:
    ns: str
    world_pose: tuple[float, float, float] | None = None
    local_pose: tuple[float, float, float] | None = None
    vel: tuple[float, float] = (0.0, 0.0)
    scan: list[list[float]] = field(default_factory=list)
    scan_stamp: float = 0.0
    nav_state: str = "IDLE"
    nav_status: int = 0
    action_uuid: str = ""
    goal_world: tuple[float, float] | None = None
    collision: bool = False
    collision_polygon: str = ""
    recoveries: int = 0
    distance_remaining: float | None = None
    last_odom_wall: float = 0.0
    last_scan_wall: float = 0.0
    last_image_wall: float = 0.0

    def pose_json(self) -> list[float] | None:
        return list(self.world_pose) if self.world_pose else None


class GoalStatusMerger:
    """Merge the two Nav2 action status streams of one robot.

    ``navigate_to_pose`` and ``navigate_through_poses`` each republish their own
    ``GoalStatusArray``.  An empty list only means "this source has no goal",
    not "the robot is idle", so we keep per-source freshness and pick the
    newest goal by its ``goal_info.stamp``.
    """

    def __init__(self) -> None:
        self._sources: dict[str, tuple[str, int, float, float]] = {}

    def update(self, source: str, msg: Any) -> None:
        now = time.monotonic()
        if not msg.status_list:
            previous = self._sources.get(source)
            stamp = previous[2] if previous else -1.0
            self._sources[source] = ("", 0, stamp, now)
            return
        status = msg.status_list[-1]
        stamp = (
            float(status.goal_info.stamp.sec)
            + float(status.goal_info.stamp.nanosec) * 1e-9
        )
        previous = self._sources.get(source)
        if previous and stamp > 0.0 and stamp < previous[2] - 1e-6:
            return
        self._sources[source] = (
            status.goal_info.goal_id.uuid,
            int(status.status),
            stamp,
            now,
        )

    def merged(self) -> tuple[str, int]:
        best: tuple[str, int, float, float] | None = None
        for entry in self._sources.values():
            if entry[1] == 0:
                continue
            if best is None:
                best = entry
                continue
            # Prefer a newer goal; on identical stamps prefer an active state.
            active_new = entry[1] in (1, 2, 3)
            active_old = best[1] in (1, 2, 3)
            if entry[2] > best[2] + 1e-6 or (
                abs(entry[2] - best[2]) <= 1e-6 and active_new and not active_old
            ):
                best = entry
        if best is None:
            return "", 0
        # A goal older than 4 s that still says EXECUTING is stale.
        if best[1] in (1, 2, 3) and time.monotonic() - best[3] > 4.0:
            return best[0], 0
        return best[0], best[1]


class StateStore:
    def __init__(self, robots: tuple[str, ...]):
        self.lock = threading.RLock()
        self.robots: dict[str, RobotState] = {ns: RobotState(ns) for ns in robots}
        self.mergers: dict[str, GoalStatusMerger] = {
            ns: GoalStatusMerger() for ns in robots
        }
        self.map_msg: Any | None = None
        self.map_seq = 0
        self.map_stamp = 0.0
        self.map_wall = 0.0
        self.clock: float | None = None
        self.clock_wall = 0.0
        self.markers: dict[str, Any] = {}
        self.sim_meta: dict[str, Any] = {}
        self.video_ok: dict[str, bool] = {ns: False for ns in robots}

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def on_map(self, msg: Any) -> None:
        with self.lock:
            self.map_msg = msg
            self.map_seq += 1
            self.map_stamp = (
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            )
            self.map_wall = time.monotonic()

    def on_clock(self, msg: Any) -> None:
        with self.lock:
            self.clock = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
            self.clock_wall = time.monotonic()

    def on_odom(self, ns: str, msg: Any) -> None:
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        with self.lock:
            robot = self.robots[ns]
            robot.local_pose = (pose.position.x, pose.position.y, yaw)
            robot.vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
            robot.last_odom_wall = time.monotonic()

    def on_scan(self, ns: str, msg: Any) -> None:
        with self.lock:
            robot = self.robots[ns]
            robot.scan_stamp = time.monotonic()
            robot.last_scan_wall = robot.scan_stamp
            # Keep the raw scan compact; conversion to world happens in the pump.
            robot.scan = [
                [float(r), float(msg.angle_min + i * msg.angle_increment)]
                for i, r in enumerate(msg.ranges)
                if math.isfinite(r)
            ]

    def on_goal_status(self, ns: str, source: str, msg: Any) -> None:
        merger = self.mergers[ns]
        merger.update(source, msg)
        uuid, code = merger.merged()
        with self.lock:
            robot = self.robots[ns]
            robot.action_uuid = uuid
            robot.nav_status = code
            robot.nav_state = nav_state_from_status(code)

    def on_collision(self, ns: str, msg: Any) -> None:
        with self.lock:
            robot = self.robots[ns]
            robot.collision = int(msg.action_type) != 0
            robot.collision_polygon = msg.polygon_name

    def on_markers(self, msg: Any) -> None:
        counts: dict[str, int] = {}
        for marker in msg.markers:
            counts[marker.namespace] = counts.get(marker.namespace, 0) + 1
        with self.lock:
            self.markers = counts

    def on_image(self, ns: str) -> None:
        with self.lock:
            self.robots[ns].last_image_wall = time.monotonic()
            self.video_ok[ns] = True

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------
    def map_meta(self) -> dict[str, Any] | None:
        with self.lock:
            msg = self.map_msg
            seq = self.map_seq
            if msg is None:
                return None
            return {
                "seq": seq,
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": float(msg.info.resolution),
                "origin": [
                    float(msg.info.origin.position.x),
                    float(msg.info.origin.position.y),
                    float(msg.info.origin.orientation.z),
                ],
                "frame": msg.header.frame_id,
            }

    def map_snapshot(self) -> tuple[Any | None, int]:
        with self.lock:
            return self.map_msg, self.map_seq

    def set_world_pose(self, ns: str, pose: tuple[float, float, float]) -> None:
        with self.lock:
            self.robots[ns].world_pose = pose

    def snapshot(self, mission: dict[str, Any] | None = None, server_time: float | None = None) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            robots = {}
            for ns, robot in self.robots.items():
                scan_points = _scan_to_world(robot)
                sim_stale = bool(
                    self.clock is None or now - self.clock_wall > 3.0
                )
                robots[ns] = {
                    "state": robot.nav_state,
                    "status": GOAL_STATUS_TEXT.get(robot.nav_status, "UNKNOWN"),
                    "pose": robot.pose_json(),
                    "vel": [robot.vel[0], robot.vel[1]],
                    "goal": list(robot.goal_world) if robot.goal_world else None,
                    "collision": robot.collision,
                    "collision_polygon": robot.collision_polygon,
                    "scan": scan_points,
                    "odom_age": round(max(0.0, now - robot.last_odom_wall), 2)
                    if robot.last_odom_wall
                    else None,
                    "distance_remaining": robot.distance_remaining,
                    "video": self.video_ok.get(ns, False),
                }
            payload = {
                "type": "snapshot",
                "t": server_time or time.time(),
                "sim": {
                    "clock": self.clock,
                    "stale": bool(self.clock is None or now - self.clock_wall > 3.0),
                },
                "robots": robots,
                "map": self.map_meta(),
                "markers": dict(self.markers),
                "mission": mission,
            }
        return payload

    def load_sim_meta(self) -> dict[str, Any]:
        path = self.sim_meta.get("_path")
        if not path:
            return self.sim_meta
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, ValueError):
            return self.sim_meta
        with self.lock:
            self.sim_meta.update(data)
        return data


def _scan_to_world(robot: RobotState, max_points: int = 140) -> list[list[float]]:
    if not robot.scan or robot.world_pose is None:
        return []
    x0, y0, yaw0 = robot.world_pose
    step = max(1, len(robot.scan) // max_points)
    points: list[list[float]] = []
    for rng, angle in robot.scan[::step]:
        if rng <= 0.05 or rng > 12.0:
            continue
        a = yaw0 + angle
        points.append(
            [
                round(x0 + rng * math.cos(a), 3),
                round(y0 + rng * math.sin(a), 3),
            ]
        )
    return points
