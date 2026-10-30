"""WebSocket hub: 10 Hz state snapshots plus an event stream.

Each connection has a one-slot queue: when the browser (or a slow cpolar
tunnel) falls behind, the oldest snapshot is dropped instead of accumulating a
backlog that would arrive in a burst when the tab becomes visible again.
"""

from __future__ import annotations

import asyncio
import time
from collections import deque
from typing import Any

from fastapi import APIRouter, Query, WebSocket, WebSocketDisconnect


class WsClient:
    def __init__(self, websocket: WebSocket, role: str):
        self.websocket = websocket
        self.role = role
        self.queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=1)
        self._sender: asyncio.Task | None = None

    def offer(self, message: dict[str, Any]) -> None:
        if self.queue.full():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        try:
            self.queue.put_nowait(message)
        except asyncio.QueueFull:
            pass

    async def send_now(self, message: dict[str, Any]) -> None:
        await self.websocket.send_json(message)

    def start(self) -> None:
        self._sender = asyncio.create_task(self._send_loop(), name="ws-sender")

    async def _send_loop(self) -> None:
        try:
            while True:
                message = await self.queue.get()
                await self.websocket.send_json(message)
        except (WebSocketDisconnect, RuntimeError, asyncio.CancelledError):
            return

    async def close(self) -> None:
        if self._sender is not None:
            self._sender.cancel()
            try:
                await self._sender
            except asyncio.CancelledError:
                pass
            self._sender = None


class WsHub:
    def __init__(self) -> None:
        self.clients: set[WsClient] = set()
        self.history: deque[dict[str, Any]] = deque(maxlen=60)

    async def register(self, websocket: WebSocket, role: str) -> WsClient:
        client = WsClient(websocket, role)
        client.start()
        self.clients.add(client)
        return client

    async def unregister(self, client: WsClient) -> None:
        self.clients.discard(client)
        await client.close()

    def publish_nowait(self, message: dict[str, Any]) -> None:
        if message.get("type") == "event":
            if message not in self.history:
                self.history.append(message)
        for client in list(self.clients):
            client.offer(message)

    def count(self) -> int:
        return len(self.clients)

    async def close_all(self) -> None:
        for client in list(self.clients):
            self.clients.discard(client)
            await client.close()


async def state_pump(
    node: Any,
    engine: Any,
    hub: WsHub,
    period: float = 0.1,
    telemetry: Any = None,
) -> None:
    store = node.store
    while True:
        try:
            store.refresh_sim_meta()
            clock = store.clock if store.clock is not None else time.time()
            stats = store.map_stats_snapshot()
            if telemetry is not None and stats:
                telemetry.add_coverage(clock, float(stats.get("area", 0.0)))
            for ns in store.robots:
                pose = node.lookup_world_pose(ns)
                if pose is not None:
                    store.set_world_pose(ns, pose)
                    if telemetry is not None:
                        telemetry.add_pose(ns, pose[0], pose[1], clock)
            if telemetry is not None and telemetry.due(time.time()):
                # 1 Hz: the trail is a context drawing, not the state itself.
                hub.publish_nowait(telemetry.snapshot())
            hub.publish_nowait(store.snapshot(engine.snapshot(), time.time()))
        except Exception:  # noqa: BLE001 - never kill the pump
            pass
        await asyncio.sleep(period)


def build_ws_router() -> APIRouter:
    router = APIRouter()

    @router.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket, token: str = Query(default="")):
        settings = websocket.app.state.settings
        hub: WsHub = websocket.app.state.hub
        store = websocket.app.state.node.store
        engine = websocket.app.state.engine
        teleop = websocket.app.state.teleop

        await websocket.accept()
        role = "operator" if token and token == settings.token else "viewer"
        client = await hub.register(websocket, role)
        try:
            await client.send_now(store.snapshot(engine.snapshot(), time.time()))
            telemetry = getattr(websocket.app.state, "telemetry", None)
            if telemetry is not None:
                await client.send_now(telemetry.snapshot())
            await client.send_now(
                {"type": "role", "role": role, "server_time": time.time()}
            )
            for event in list(hub.history)[-20:]:
                await client.send_now(event)

            while True:
                message = await websocket.receive_json()
                kind = message.get("type")
                if kind == "ping":
                    await client.send_now({"type": "pong", "t": message.get("t")})
                    continue
                if kind in {"teleop", "estop", "clear_estop"} and role != "operator":
                    await client.send_now(
                        {"type": "error", "text": "只读模式，请先登录操作员"}
                    )
                    continue
                if kind == "teleop":
                    ns = str(message.get("ns", ""))
                    try:
                        teleop.set_cmd(
                            ns,
                            float(message.get("vx", 0.0)),
                            float(message.get("wz", 0.0)),
                        )
                    except Exception as exc:  # noqa: BLE001
                        await client.send_now({"type": "error", "text": str(exc)})
                elif kind == "estop":
                    await teleop.estop()
                    engine.estop()
                elif kind == "clear_estop":
                    await teleop.clear_estop()
                    engine.clear_estop()
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001
            pass
        finally:
            await hub.unregister(client)
            if role == "operator" and not any(
                other.role == "operator" for other in hub.clients
            ):
                # Nobody can drive any more: stop and re-arm collision monitor.
                try:
                    await teleop.release()
                except Exception:  # noqa: BLE001
                    pass

    return router
