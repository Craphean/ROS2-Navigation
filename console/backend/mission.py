"""Inspection mission state machine.

Two levels of state:

* mission: IDLE / RUNNING / PAUSED / ESTOPPED / COMPLETED / ABORTED / INTERRUPTED
* waypoint: PENDING / DISPATCHING / NAVIGATING / ARRIVED / DWELLING / DONE / SKIPPED / FAILED

The engine is the single source of truth.  Nav2 action status is only an input;
the UI never derives mission state from it.  Every transition is written to
``console/run/missions/<id>.json`` so a backend restart can offer "resume from
waypoint N".
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from .config import Settings
from .ros_node import ConsoleNode


PENDING = "PENDING"
DISPATCHING = "DISPATCHING"
NAVIGATING = "NAVIGATING"
ARRIVED = "ARRIVED"
DWELLING = "DWELLING"
DONE = "DONE"
SKIPPED = "SKIPPED"
FAILED = "FAILED"

IDLE = "IDLE"
RUNNING = "RUNNING"
PAUSED = "PAUSED"
ESTOPPED = "ESTOPPED"
COMPLETED = "COMPLETED"
ABORTED = "ABORTED"
INTERRUPTED = "INTERRUPTED"


@dataclass
class Waypoint:
    index: int
    x: float
    y: float
    yaw: float | None = None
    label: str = ""
    dwell: float = 5.0
    retries: int = 1
    status: str = PENDING
    message: str = ""
    t_dispatched: float | None = None
    t_done: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "label": self.label,
            "dwell": self.dwell,
            "retries": self.retries,
            "status": self.status,
            "message": self.message,
            "t_dispatched": self.t_dispatched,
            "t_done": self.t_done,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any], index: int = 0) -> "Waypoint":
        return cls(
            index=int(data.get("index", index)),
            x=float(data["x"]),
            y=float(data["y"]),
            yaw=None if data.get("yaw") is None else float(data["yaw"]),
            label=str(data.get("label", "")),
            dwell=float(data.get("dwell", 5.0)),
            retries=int(data.get("retries", 1)),
            status=str(data.get("status", PENDING)),
            message=str(data.get("message", "")),
            t_dispatched=data.get("t_dispatched"),
            t_done=data.get("t_done"),
        )


@dataclass
class Mission:
    id: str
    name: str
    robot: str
    waypoints: list[Waypoint] = field(default_factory=list)
    loop: bool = False
    on_fail: str = "retry_then_skip"
    state: str = IDLE
    cursor: int = 0
    created_at: float = field(default_factory=time.time)
    started_at: float | None = None
    ended_at: float | None = None
    error: str | None = None

    def to_dict(self, with_waypoints: bool = True) -> dict[str, Any]:
        payload = {
            "id": self.id,
            "name": self.name,
            "robot": self.robot,
            "loop": self.loop,
            "on_fail": self.on_fail,
            "state": self.state,
            "cursor": self.cursor,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "error": self.error,
            "total": len(self.waypoints),
            "done": sum(1 for w in self.waypoints if w.status in (DONE, SKIPPED)),
        }
        if with_waypoints:
            payload["waypoints"] = [w.to_dict() for w in self.waypoints]
        return payload

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Mission":
        waypoints = [
            Waypoint.from_dict(item, index)
            for index, item in enumerate(data.get("waypoints", []))
        ]
        return cls(
            id=str(data["id"]),
            name=str(data.get("name", data["id"])),
            robot=str(data.get("robot", "robot1")),
            waypoints=waypoints,
            loop=bool(data.get("loop", False)),
            on_fail=str(data.get("on_fail", "retry_then_skip")),
            state=str(data.get("state", IDLE)),
            cursor=int(data.get("cursor", 0)),
            started_at=data.get("started_at"),
            ended_at=data.get("ended_at"),
            error=data.get("error"),
        )


class MissionStore:
    def __init__(self, settings: Settings):
        self.settings = settings
        self.defs_dir = settings.mission_dir
        self.run_dir = settings.run_dir / "missions"
        self.defs_dir.mkdir(parents=True, exist_ok=True)
        self.run_dir.mkdir(parents=True, exist_ok=True)

    def load(self) -> dict[str, Mission]:
        missions: dict[str, Mission] = {}
        for path in sorted(self.defs_dir.glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                mission = Mission.from_dict(data)
                missions[mission.id] = mission
            except (OSError, ValueError, KeyError):
                continue
        for path in sorted(self.run_dir.glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            mission = missions.get(str(data.get("id", "")))
            if mission is None:
                continue
            mission.cursor = int(data.get("cursor", mission.cursor))
            mission.state = str(data.get("state", mission.state))
            mission.error = data.get("error")
            mission.started_at = data.get("started_at")
            mission.ended_at = data.get("ended_at")
            by_index = {w.index: w for w in mission.waypoints}
            for item in data.get("waypoints", []):
                wp = by_index.get(int(item.get("index", -1)))
                if wp is None:
                    continue
                wp.status = str(item.get("status", wp.status))
                wp.message = str(item.get("message", ""))
                wp.t_dispatched = item.get("t_dispatched")
                wp.t_done = item.get("t_done")
            if mission.state in (RUNNING, PAUSED, ESTOPPED):
                mission.state = INTERRUPTED
                mission.error = "backend restarted during mission"
        return missions

    def save_definition(self, mission: Mission) -> None:
        path = self.defs_dir / f"{mission.id}.json"
        data = {
            "id": mission.id,
            "name": mission.name,
            "robot": mission.robot,
            "loop": mission.loop,
            "on_fail": mission.on_fail,
            "waypoints": [
                {
                    "label": w.label,
                    "x": w.x,
                    "y": w.y,
                    "yaw": w.yaw,
                    "dwell": w.dwell,
                }
                for w in mission.waypoints
            ],
        }
        _atomic_write(path, data)

    def save_runtime(self, mission: Mission) -> None:
        _atomic_write(self.run_dir / f"{mission.id}.json", mission.to_dict())


def _atomic_write(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


class MissionEngine:
    def __init__(
        self,
        node: ConsoleNode,
        settings: Settings,
        emit: Callable[[dict[str, Any]], None] | None = None,
    ):
        self.node = node
        self.settings = settings
        self.store = MissionStore(settings)
        self.missions = self.store.load()
        self.active_id: str | None = None
        self._task: asyncio.Task | None = None
        self._resume = asyncio.Event()
        self._emit = emit or (lambda event: None)

    # ------------------------------------------------------------------
    def snapshot(self) -> dict[str, Any] | None:
        mission = self.active
        return mission.to_dict() if mission else None

    @property
    def active(self) -> Mission | None:
        return self.missions.get(self.active_id or "")

    def list_missions(self) -> list[dict[str, Any]]:
        return [m.to_dict(with_waypoints=False) for m in self.missions.values()]

    def create(self, payload: dict[str, Any]) -> Mission:
        mission_id = str(payload.get("id") or f"mission_{int(time.time())}")
        if mission_id in self.missions:
            raise ValueError(f"任务 {mission_id} 已存在")
        robot = str(payload.get("robot", self.settings.robots[0]))
        if robot not in self.settings.robots:
            raise ValueError(f"未知机器人 {robot}")
        points = payload.get("waypoints") or []
        if not points:
            raise ValueError("任务至少需要一个巡检点")
        waypoints = [
            Waypoint.from_dict(item, index) for index, item in enumerate(points)
        ]
        mission = Mission(
            id=mission_id,
            name=str(payload.get("name", mission_id)),
            robot=robot,
            waypoints=waypoints,
            loop=bool(payload.get("loop", False)),
            on_fail=str(payload.get("on_fail", "retry_then_skip")),
        )
        self.missions[mission.id] = mission
        self.store.save_definition(mission)
        return mission

    # ------------------------------------------------------------------
    def append_waypoint(self, mission_id: str, payload: dict[str, Any]) -> Mission:
        mission = self.missions.get(mission_id)
        if mission is None:
            raise ValueError("任务不存在")
        if mission.state in (RUNNING, PAUSED, ESTOPPED):
            raise ValueError("任务运行中，不能修改航点")
        waypoint = Waypoint.from_dict(payload, len(mission.waypoints))
        mission.waypoints.append(waypoint)
        self.store.save_definition(mission)
        return mission

    def update_options(
        self,
        mission_id: str,
        robot: str | None = None,
        loop: bool | None = None,
    ) -> Mission:
        """Change the executing robot / looping without touching waypoints."""
        mission = self.missions.get(mission_id)
        if mission is None:
            raise ValueError("任务不存在")
        if mission.state in (RUNNING, PAUSED, ESTOPPED):
            raise ValueError("任务运行中，不能修改执行设置")
        if robot:
            if robot not in self.settings.robots:
                raise ValueError(f"未知机器人 {robot}")
            mission.robot = robot
        if loop is not None:
            mission.loop = bool(loop)
        self.store.save_definition(mission)
        return mission

    def start(self, mission_id: str) -> Mission:
        mission = self.missions.get(mission_id)
        if mission is None:
            raise ValueError("任务不存在")
        if self._task is not None and not self._task.done():
            raise ValueError("已有任务在运行")
        if not mission.waypoints:
            raise ValueError("任务没有巡检点")
        for waypoint in mission.waypoints:
            waypoint.status = PENDING
            waypoint.message = ""
        mission.cursor = 0
        mission.state = RUNNING
        mission.error = None
        mission.started_at = time.time()
        mission.ended_at = None
        self.active_id = mission_id
        self.store.save_runtime(mission)
        self._task = asyncio.create_task(self._run(mission), name=f"mission-{mission_id}")
        self._event("info", "MISSION_START", mission.robot, f"开始任务「{mission.name}」")
        return mission

    def pause(self) -> Mission:
        mission = self._require_active()
        if mission.state == RUNNING:
            mission.state = PAUSED
            self.store.save_runtime(mission)
            asyncio.create_task(self.node.cancel_goal(mission.robot))
            self._event("warn", "MISSION_PAUSE", mission.robot, "任务已暂停")
        return mission

    def resume(self, mission_id: str | None = None) -> Mission:
        if self.active_id is None and mission_id:
            self.active_id = mission_id
        mission = self._require_active()
        if mission.state in (PAUSED, INTERRUPTED):
            mission.state = RUNNING
            mission.error = None
            if self._task is None or self._task.done():
                self._task = asyncio.create_task(
                    self._run(mission), name=f"mission-{mission.id}"
                )
            self._resume.set()
            self.store.save_runtime(mission)
            self._event("info", "MISSION_RESUME", mission.robot, "任务继续")
        return mission

    def skip(self) -> Mission:
        mission = self._require_active()
        if mission.cursor < len(mission.waypoints):
            current = mission.waypoints[mission.cursor]
            if current.status in (DISPATCHING, NAVIGATING, ARRIVED, DWELLING):
                current.status = SKIPPED
                current.message = "操作员跳过"
                asyncio.create_task(self.node.cancel_goal(mission.robot))
                mission.cursor += 1
                mission.state = RUNNING
                self._resume.set()
                self.store.save_runtime(mission)
                self._event("warn", "MISSION_SKIP", mission.robot, f"跳过 {current.label}")
        return mission

    def stop(self) -> Mission:
        mission = self._require_active()
        mission.state = ABORTED
        mission.error = "操作员终止"
        mission.ended_at = time.time()
        asyncio.create_task(self.node.cancel_goal(mission.robot))
        self._resume.set()
        self.store.save_runtime(mission)
        self._event("warn", "MISSION_STOP", mission.robot, "任务已终止")
        return mission

    def estop(self) -> None:
        mission = self.active
        if mission and mission.state in (RUNNING, PAUSED):
            mission.state = ESTOPPED
            mission.error = "急停"
            self.store.save_runtime(mission)
        for ns in self.settings.robots:
            asyncio.create_task(self.node.cancel_goal(ns))
        self._event("error", "ESTOP", "", "急停触发")

    def clear_estop(self) -> None:
        mission = self.active
        if mission and mission.state == ESTOPPED:
            mission.state = PAUSED
            mission.error = None
            self.store.save_runtime(mission)

    # ------------------------------------------------------------------
    async def _run(self, mission: Mission) -> None:
        fail_streak = 0
        while True:
            if mission.state in (ABORTED, COMPLETED):
                return
            if mission.state == ESTOPPED:
                await self._wait_resume()
                continue
            if mission.state == PAUSED:
                await self._wait_resume()
                continue
            if mission.cursor >= len(mission.waypoints):
                if mission.loop and mission.state == RUNNING:
                    for waypoint in mission.waypoints:
                        waypoint.status = PENDING
                    mission.cursor = 0
                    continue
                mission.state = COMPLETED
                mission.ended_at = time.time()
                self.store.save_runtime(mission)
                self._event("info", "MISSION_DONE", mission.robot, "任务完成")
                return

            waypoint = mission.waypoints[mission.cursor]
            if waypoint.status in (DONE, SKIPPED):
                mission.cursor += 1
                continue

            outcome = await self._run_waypoint(mission, waypoint)
            if mission.state == ABORTED:
                return
            if outcome == "PAUSED":
                continue
            if outcome == "OK":
                fail_streak = 0
                mission.cursor += 1
            else:
                fail_streak += 1
                waypoint.status = FAILED
                if mission.on_fail == "abort" or fail_streak >= 2:
                    mission.state = ABORTED
                    mission.error = f"{waypoint.label or waypoint.index} 连续失败"
                    mission.ended_at = time.time()
                    self.store.save_runtime(mission)
                    self._event(
                        "error", "MISSION_ABORT", mission.robot, mission.error
                    )
                    return
                waypoint.status = SKIPPED
                waypoint.message = "失败后跳过"
                mission.cursor += 1
            self.store.save_runtime(mission)

    async def _run_waypoint(self, mission: Mission, waypoint: Waypoint) -> str:
        attempts = max(0, waypoint.retries) + 1
        for attempt in range(attempts):
            if mission.state in (PAUSED, ESTOPPED):
                return "PAUSED"
            waypoint.status = DISPATCHING
            waypoint.t_dispatched = time.time()
            self.store.save_runtime(mission)
            self._event(
                "info",
                "WAYPOINT_GO",
                mission.robot,
                f"前往 {waypoint.label or waypoint.index} "
                f"({waypoint.x:.1f}, {waypoint.y:.1f})",
            )
            distance = self._distance(mission.robot, waypoint)
            timeout = max(60.0, min(300.0, 3.0 * distance / 0.15))
            result = await self.node.navigate_world(
                mission.robot, waypoint.x, waypoint.y, waypoint.yaw or 0.0, timeout
            )
            if result == "CANCELED" and mission.state in (PAUSED, ESTOPPED):
                return "PAUSED"
            if result == "OK":
                waypoint.status = ARRIVED
                self.store.save_runtime(mission)
                waypoint.status = DWELLING
                self.store.save_runtime(mission)
                await self._dwell(mission, waypoint.dwell)
                if mission.state in (PAUSED, ESTOPPED):
                    return "PAUSED"
                waypoint.status = DONE
                waypoint.t_done = time.time()
                waypoint.message = ""
                self.store.save_runtime(mission)
                self._event(
                    "info", "WAYPOINT_DONE", mission.robot, f"到达 {waypoint.label}"
                )
                return "OK"
            waypoint.message = result
            self._event(
                "warn",
                "WAYPOINT_FAIL",
                mission.robot,
                f"{waypoint.label or waypoint.index} 失败（{result}）",
            )
            if attempt < attempts - 1:
                await self._dwell(mission, 3.0)
                if mission.state in (PAUSED, ESTOPPED):
                    return "PAUSED"
        return "FAILED"

    async def _dwell(self, mission: Mission, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            if mission.state in (PAUSED, ESTOPPED):
                return
            await asyncio.sleep(0.2)

    async def _wait_resume(self) -> None:
        while self.active and self.active.state in (PAUSED, ESTOPPED):
            self._resume.clear()
            try:
                await asyncio.wait_for(self._resume.wait(), timeout=0.5)
            except asyncio.TimeoutError:
                pass

    def _distance(self, ns: str, waypoint: Waypoint) -> float:
        with self.node.store.lock:
            pose = self.node.store.robots[ns].world_pose
        if pose is None:
            return 10.0
        return math.hypot(waypoint.x - pose[0], waypoint.y - pose[1])

    def _require_active(self) -> Mission:
        mission = self.active
        if mission is None:
            raise ValueError("没有活动任务")
        return mission

    def _event(self, level: str, code: str, ns: str, text: str) -> None:
        self._emit(
            {
                "type": "event",
                "level": level,
                "code": code,
                "ns": ns,
                "text": text,
                "ts": time.time(),
            }
        )
