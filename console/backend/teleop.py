"""Manual teleop with a lease, a deadman switch and collision-monitor handover.

Why this is not just ``teleop_twist_keyboard``: the Nav2 chain is

    controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
               -> collision_monitor -> cmd_vel -> ros_gz_bridge

The smoother publishes zeros at 20 Hz and the collision monitor forwards them
to ``cmd_vel``.  Publishing teleop directly would race with both.  The gate
therefore disables the collision monitor for the duration of the lease, then
publishes ``TwistStamped`` at a fixed 20 Hz with a 300 ms deadman.
"""

from __future__ import annotations

import asyncio
import time
from typing import Any

from .config import Settings
from .ros_node import ConsoleNode


class TeleopError(RuntimeError):
    pass


class TeleopGate:
    def __init__(self, node: ConsoleNode, settings: Settings):
        self.node = node
        self.settings = settings
        self.owner: str | None = None
        self.ns: str | None = None
        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd = 0.0
        self.estopped = False
        self._lock = asyncio.Lock()
        self._task: asyncio.Task | None = None
        self._releasing = False

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._pump(), name="teleop-pump")

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        if self.owner:
            await self.release()

    # ------------------------------------------------------------------
    async def acquire(self, ns: str, owner: str) -> dict[str, Any]:
        if ns not in self.settings.robots:
            raise TeleopError(f"未知机器人 {ns}")
        if self.estopped:
            raise TeleopError("急停未解除，无法接管")
        async with self._lock:
            if self.owner and self.owner != owner:
                raise TeleopError("另一名操作员正在控制")
            with self.node.store.lock:
                status = self.node.store.robots[ns].nav_status
            if status in (1, 2, 3):
                raise TeleopError("机器人正在执行导航任务，请先取消")
            ok, message = await self.node.toggle_collision(ns, False)
            if not ok:
                raise TeleopError(f"无法关闭碰撞监控：{message}")
            self.owner = owner
            self.ns = ns
            self.vx = 0.0
            self.wz = 0.0
            self.last_cmd = 0.0
            for _ in range(5):
                self.node.publish_cmd(ns, 0.0, 0.0)
                await asyncio.sleep(0.02)
        return {"ok": True, "ns": ns, "owner": owner}

    async def release(self) -> None:
        async with self._lock:
            if self._releasing:
                return
            self._releasing = True
            ns = self.ns
            try:
                if ns:
                    for _ in range(10):
                        self.node.publish_cmd(ns, 0.0, 0.0)
                        await asyncio.sleep(0.05)
                    await self.node.toggle_collision(ns, True)
            finally:
                self.owner = None
                self.ns = None
                self.vx = self.wz = 0.0
                self.last_cmd = 0.0
                self._releasing = False

    def set_cmd(self, ns: str, vx: float, wz: float) -> None:
        if not self.owner or self.ns != ns:
            raise TeleopError("尚未取得该机器人的控制权")
        self.vx = max(-0.15, min(0.35, float(vx)))
        self.wz = max(-0.8, min(0.8, float(wz)))
        self.last_cmd = time.monotonic()

    async def estop(self) -> None:
        self.estopped = True
        self.vx = self.wz = 0.0
        ns_list = list(self.settings.robots)
        for _ in range(20):
            for ns in ns_list:
                self.node.publish_cmd(ns, 0.0, 0.0)
            await asyncio.sleep(0.05)
        for ns in ns_list:
            await self.node.cancel_goal(ns)
        if self.owner:
            await self.release()

    def clear_estop(self) -> None:
        self.estopped = False

    # ------------------------------------------------------------------
    async def _pump(self) -> None:
        while True:
            try:
                await asyncio.sleep(0.05)
                if not self.owner or not self.ns:
                    continue
                age = time.monotonic() - self.last_cmd if self.last_cmd else 1e9
                vx, wz = self.vx, self.wz
                if self.estopped or age > 0.3:
                    vx = wz = 0.0
                if vx > 0.0:
                    clearance = self.node.front_clearance(self.ns)
                    if clearance is not None and clearance < 0.22:
                        vx = 0.0
                self.node.publish_cmd(self.ns, vx, wz)
                if age > 10.0 and not self._releasing:
                    # Long idle: hand control back and re-arm the monitor.  The
                    # 300 ms deadman above already stopped the robot.
                    asyncio.create_task(self.release())
            except asyncio.CancelledError:
                raise
            except Exception:  # noqa: BLE001 - the pump must never die
                await asyncio.sleep(0.2)

    def status(self) -> dict[str, Any]:
        return {
            "owner": self.owner,
            "ns": self.ns,
            "estopped": self.estopped,
            "vx": self.vx,
            "wz": self.wz,
        }
