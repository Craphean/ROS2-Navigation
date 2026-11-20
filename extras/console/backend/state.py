"""In-memory fleet state shared by ROS callbacks and the web API.

ROS callbacks only write plain data under ``StateStore.lock``; PNG/JPEG
encoding and WebSocket serialisation always happen outside the lock.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np


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
    if code == 6:
        # Aborted goals used to show up as RECOVERY ("脱困中") in the console for
        # the rest of the session: a terminal goal status is not a recovery.
        return "FAILED"
    return "IDLE"


@dataclass
class RobotState:
    ns: str
    world_pose: tuple[float, float, float] | None = None
    local_pose: tuple[float, float, float] | None = None
    vel: tuple[float, float] = (0.0, 0.0)
    # Ranges and their bearing in the laser frame, both float64, already
    # filtered to finite returns.  A list of [r, angle] pairs cost ~56 B per
    # beam and was rebuilt from Python objects on every scan (10-15 Hz).
    scan: np.ndarray | None = None
    scan_angles: np.ndarray | None = None
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

    def reset(self) -> None:
        self._sources.clear()

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
        # Occupancy summary, recomputed once per map message (1 Hz) rather than
        # per snapshot (10 Hz): counting a million cells is not free.
        self.map_stats: dict[str, float] = {}
        self.clock: float | None = None
        self.clock_wall = 0.0
        self.markers: dict[str, Any] = {}
        self.loop_closures: dict[str, int] = {ns: 0 for ns in robots}
        self.sim_meta: dict[str, Any] = {}
        self.video_ok: dict[str, bool] = {ns: False for ns in robots}

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def on_map(self, msg: Any) -> None:
        stats = _occupancy_stats(msg)
        with self.lock:
            self.map_msg = msg
            self.map_stats = stats
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
        # np.array (not asarray) so the store never pins the ROS message buffer.
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float64) * msg.angle_increment
        # Keep only finite returns, exactly as before, but as two flat arrays:
        # no per-beam Python objects, and the world conversion is vectorised.
        keep = np.isfinite(ranges)
        if not keep.all():
            ranges = ranges[keep]
            angles = angles[keep]
        with self.lock:
            robot = self.robots[ns]
            robot.scan_stamp = time.monotonic()
            robot.last_scan_wall = robot.scan_stamp
            robot.scan = ranges
            robot.scan_angles = angles

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
            # visualization_msgs/Marker names this field `ns`.
            key = getattr(marker, "ns", None) or getattr(marker, "namespace", "")
            counts[key] = counts.get(key, 0) + 1
        with self.lock:
            self.markers = counts

    def on_loop_closure(self, ns: str) -> None:
        with self.lock:
            self.loop_closures[ns] = self.loop_closures.get(ns, 0) + 1

    def on_image(self, ns: str) -> None:
        with self.lock:
            self.robots[ns].last_image_wall = time.monotonic()
            self.video_ok[ns] = True

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------
    def map_stats_snapshot(self) -> dict[str, float]:
        with self.lock:
            return dict(self.map_stats)

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
                "loop_closures": dict(self.loop_closures),
                "mission": mission,
            }
        return payload

    def reset_world(self) -> None:
        with self.lock:
            self.map_msg = None
            self.map_seq += 1
            self.map_stamp = 0.0
            self.clock = None
            self.clock_wall = 0.0
            self.markers.clear()
            for robot in self.robots.values():
                robot.world_pose = None
                robot.local_pose = None
                robot.scan = None
                robot.scan_angles = None
                robot.goal_world = None
                robot.distance_remaining = None
                # Goal statuses are stamped with sim time, which restarts at 0
                # with the simulation.  Keeping the previous run's entry made
                # the merger reject every new status as "older" and pinned the
                # UI to the last run's ABORTED/RECOVERY state.
                robot.nav_status = 0
                robot.nav_state = "IDLE"
                robot.action_uuid = ""
            for merger in self.mergers.values():
                merger.reset()
            self.loop_closures = {ns: 0 for ns in self.robots}

    def refresh_sim_meta(self) -> None:
        path = self.sim_meta.get("_path")
        if not path:
            return
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        if mtime == self.sim_meta.get("_mtime"):
            return
        self.load_sim_meta()
        self.sim_meta["_mtime"] = mtime

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
    ranges = robot.scan
    angles = robot.scan_angles
    if ranges is None or angles is None or ranges.size == 0 or robot.world_pose is None:
        return []
    x0, y0, yaw0 = robot.world_pose
    step = max(1, ranges.size // max_points)
    ranges = ranges[::step]
    angles = angles[::step]
    keep = (ranges > 0.05) & (ranges <= 12.0)
    if not keep.any():
        return []
    ranges = ranges[keep]
    heading = yaw0 + angles[keep]
    xs = np.round(x0 + ranges * np.cos(heading), 3)
    ys = np.round(y0 + ranges * np.sin(heading), 3)
    return np.stack((xs, ys), axis=1).tolist()

def _occupancy_stats(msg: Any) -> dict[str, float]:
    """Known / unknown split of an OccupancyGrid.

    "Known" is anything the SLAM front end has committed (free 0 or occupied
    100); -1 is unexplored.  Coverage over the *current* map extent is what the
    console shows - the merged map grows as the robots explore, so the ratio
    dips when a new area appears and climbs again as it gets filled.
    """
    try:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        data = np.asarray(msg.data, dtype=np.int8)
    except (AttributeError, TypeError, ValueError):
        return {}
    total = int(data.size) or (width * height)
    if total <= 0:
        return {}
    known = int(np.count_nonzero(data >= 0))
    return {
        "coverage": known / total,
        "known": float(known),
        "total": float(total),
        "area": known * resolution * resolution,
        "width": float(width),
        "height": float(height),
        "resolution": resolution,
    }
