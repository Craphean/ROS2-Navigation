"""HTTP API for the console."""

from __future__ import annotations

import asyncio
import importlib.util
import json
import re
import time
from typing import Any

from fastapi import APIRouter, Body, Depends, HTTPException, Request, Response
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, Field

from .auth import COOKIE_NAME, is_operator, require_operator
from .map_render import grid_to_png
from .video import mjpeg_stream


router = APIRouter(prefix="/api")

DETECTION_TTL = 3.0
# inspection_20261029_142537.pcd - the only names /api/cloud/file will serve.
_PCD_NAME = re.compile(r"inspection_[0-9]{8}_[0-9]{6}\.pcd\Z")


class GoalBody(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class LeaseBody(BaseModel):
    ns: str


class CmdBody(BaseModel):
    ns: str
    vx: float = 0.0
    wz: float = 0.0


class LoginBody(BaseModel):
    token: str


class SimStartBody(BaseModel):
    world: str | None = None
    mode: str | None = None
    headless: bool = True


class CloudSaveBody(BaseModel):
    binary: bool = True


class WorldViewBody(BaseModel):
    ns: str | None = None
    yaw: float | None = None
    pitch: float | None = None
    distance: float | None = None


class SimRestartBody(SimStartBody):
    clean: bool = True


# Worlds tb3_multi_explore_slam.sh can load; the UI only offers these.
WORLDS: tuple[dict[str, str], ...] = (
    {"id": "mine", "label": "矿井 mine"},
    {"id": "house", "label": "住宅 house"},
    {"id": "hospital", "label": "医院 hospital"},
    {"id": "sandbox", "label": "沙盒 sandbox"},
    {"id": "empty", "label": "空地 empty"},
    {"id": "warehouse", "label": "仓储 warehouse"},
    {"id": "depot", "label": "补给站 depot"},
)


class DetectionBox(BaseModel):
    cls: str
    conf: float = Field(ge=0.0, le=1.0)
    x: float
    y: float
    w: float
    h: float


class DetectionBody(BaseModel):
    ns: str
    ts: float | None = None
    frame_id: str = ""
    boxes: list[DetectionBox] = Field(default_factory=list)


# ----------------------------------------------------------------------
# Session / health
# ----------------------------------------------------------------------
@router.get("/health")
async def health(request: Request) -> dict[str, Any]:
    node = request.app.state.node
    proc = request.app.state.proc
    meta = node.store.map_meta()
    return {
        "ok": True,
        "map": bool(meta),
        "sim": proc.sim_status(),
        "exploration": proc.exploration_status(),
        "ws_clients": request.app.state.hub.count(),
    }


@router.get("/state")
async def state(request: Request) -> dict[str, Any]:
    node = request.app.state.node
    engine = request.app.state.engine
    return node.store.snapshot(engine.snapshot(), time.time())


@router.post("/login")
async def login(request: Request, response: Response, body: LoginBody) -> dict[str, Any]:
    settings = request.app.state.settings
    if not body.token or body.token != settings.token:
        raise HTTPException(status_code=401, detail="口令错误")
    response.set_cookie(
        COOKIE_NAME,
        settings.token,
        httponly=True,
        samesite="lax",
        max_age=12 * 3600,
    )
    return {"ok": True, "role": "operator"}


@router.post("/logout")
async def logout(response: Response) -> dict[str, Any]:
    response.delete_cookie(COOKIE_NAME)
    return {"ok": True}


@router.get("/session")
async def session(request: Request) -> dict[str, Any]:
    return {"role": "operator" if is_operator(request) else "viewer"}


# ----------------------------------------------------------------------
# Map
# ----------------------------------------------------------------------
@router.get("/map/meta")
async def map_meta(request: Request) -> dict[str, Any]:
    meta = request.app.state.node.store.map_meta()
    if meta is None:
        raise HTTPException(status_code=404, detail="地图尚未生成")
    return meta


@router.get("/map.png")
async def map_png(request: Request) -> Response:
    node = request.app.state.node
    cache: dict[str, Any] = request.app.state.map_cache
    message, seq = node.store.map_snapshot()
    if message is None:
        raise HTTPException(status_code=404, detail="地图尚未生成")
    if cache.get("seq") != seq or time.monotonic() - cache.get("wall", 0.0) > 2.0:
        cache["png"] = grid_to_png(message)
        cache["seq"] = seq
        cache["wall"] = time.monotonic()
    return Response(
        cache["png"],
        media_type="image/png",
        headers={
            "Cache-Control": "public, max-age=1",
            "ETag": f'"{seq}"',
            "X-Map-Seq": str(seq),
        },
    )


@router.post("/map/save", dependencies=[Depends(require_operator)])
async def save_map(request: Request) -> dict[str, Any]:
    settings = request.app.state.settings
    stamp = time.strftime("%Y%m%d_%H%M%S")
    target = settings.root / "maps" / f"inspection_{stamp}"
    target.parent.mkdir(parents=True, exist_ok=True)
    argv = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        str(target),
        "--ros-args",
        "-r",
        "map:=/map",
    ]
    try:
        process = await asyncio.create_subprocess_exec(
            *argv,
            cwd=str(settings.root),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        stdout, _ = await asyncio.wait_for(process.communicate(), timeout=30.0)
    except asyncio.TimeoutError:
        raise HTTPException(status_code=504, detail="保存地图超时")
    if process.returncode != 0:
        raise HTTPException(
            status_code=500, detail=stdout.decode(errors="replace")[-400:]
        )
    return {"ok": True, "path": str(target.with_suffix(".yaml"))}


@router.post("/cloud/save", dependencies=[Depends(require_operator)])
async def save_cloud(request: Request, body: CloudSaveBody | None = None) -> dict[str, Any]:
    """Export the accumulated 3D cloud as PCD.

    Written from the console's own voxel cache, which uses the same hash and
    z-gate as scripts/export_merged_cloud.py, so a click gives the whole run
    instantly instead of subscribing for another N seconds.  The PCD writer
    itself is imported from that script - one implementation, one format.
    """
    settings = request.app.state.settings
    points, voxel = request.app.state.node.cloud.points()
    if not points.size:
        raise HTTPException(status_code=404, detail="暂无三维点云")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    target = settings.root / "maps" / f"inspection_{stamp}.pcd"
    target.parent.mkdir(parents=True, exist_ok=True)
    binary = True if body is None else body.binary
    writer = _pcd_writer(settings)
    try:
        await asyncio.to_thread(writer, target, points, binary)
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"写入失败: {exc}") from exc
    return {
        "ok": True,
        "path": str(target),
        "points": int(points.shape[0]),
        "voxel": voxel,
    }


@router.get("/cloud/file/{name}", dependencies=[Depends(require_operator)])
async def download_cloud(request: Request, name: str) -> FileResponse:
    """Serve an exported PCD so the operator can pull it to their own machine.

    The export itself can only write where the console runs; this is the only
    way to get the file onto the laptop that is watching the dashboard.
    """
    if not _PCD_NAME.match(name):
        raise HTTPException(status_code=404, detail="文件名不合法")
    path = request.app.state.settings.root / "maps" / name
    if not path.is_file():
        raise HTTPException(status_code=404, detail="文件不存在")
    return FileResponse(
        path,
        media_type="application/octet-stream",
        filename=name,
    )


def _pcd_writer(settings: Any):
    """Load write_pcd() from scripts/export_merged_cloud.py."""
    path = settings.root / "scripts" / "export_merged_cloud.py"
    spec = importlib.util.spec_from_file_location("mine_pcd_writer", path)
    if spec is None or spec.loader is None:
        raise HTTPException(status_code=500, detail=f"找不到 {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.write_pcd


# ----------------------------------------------------------------------
# Robot control
# ----------------------------------------------------------------------
def _check_ns(request: Request, ns: str) -> None:
    if ns not in request.app.state.settings.robots:
        raise HTTPException(status_code=404, detail=f"未知机器人 {ns}")


async def _take_manual_control(request: Request) -> None:
    """Stop autonomous exploration before a manual goal / teleop lease.

    The coordinator re-dispatches to any IDLE robot every 0.5 s, so a manual
    goal would otherwise be preempted within one cycle.  If the coordinator was
    started outside the console we cannot stop it safely, so reject with a
    clear message instead of silently losing the goal.
    """
    proc = request.app.state.proc
    node = request.app.state.node
    if proc.exploration_status().get("running"):
        await asyncio.to_thread(proc.stop_exploration)
        request.app.state.hub.publish_nowait(
            {
                "type": "event",
                "level": "warn",
                "code": "MANUAL_TAKEOVER",
                "ns": "",
                "text": "人工接管，已停止自主勘探",
                "ts": time.time(),
            }
        )
        await asyncio.sleep(0.5)
        return
    if "multi_robot_coordinator" in node.get_node_names():
        raise HTTPException(
            status_code=409,
            detail="自主勘探正在外部运行（不是控制台启动的），请先停止协调器再手动控制",
        )


@router.post("/robot/{ns}/goal", dependencies=[Depends(require_operator)])
async def robot_goal(request: Request, ns: str, body: GoalBody) -> dict[str, Any]:
    _check_ns(request, ns)
    node = request.app.state.node
    if not node.nav_clients[ns].server_is_ready():
        raise HTTPException(status_code=503, detail="Nav2 尚未就绪")
    await _take_manual_control(request)
    await node.cancel_goal(ns)
    asyncio.create_task(
        node.navigate_world(ns, body.x, body.y, body.yaw, 300.0),
        name=f"goal-{ns}",
    )
    return {"ok": True, "ns": ns, "goal": [body.x, body.y, body.yaw]}


@router.post("/robot/{ns}/cancel", dependencies=[Depends(require_operator)])
async def robot_cancel(request: Request, ns: str) -> dict[str, Any]:
    _check_ns(request, ns)
    await request.app.state.node.cancel_goal(ns)
    return {"ok": True}


@router.post("/robot/{ns}/recall", dependencies=[Depends(require_operator)])
async def robot_recall(request: Request, ns: str) -> dict[str, Any]:
    _check_ns(request, ns)
    node = request.app.state.node
    meta = request.app.state.settings.sim_meta_path
    try:
        data = json.loads(meta.read_text(encoding="utf-8"))
        pose = data.get(ns, [0.0, 0.0, 0.0])
    except (OSError, ValueError):
        raise HTTPException(status_code=409, detail="缺少 sim_meta.json，无法召回")
    await _take_manual_control(request)
    await node.cancel_goal(ns)
    asyncio.create_task(
        node.navigate_world(ns, float(pose[0]), float(pose[1]), float(pose[2]), 300.0),
        name=f"recall-{ns}",
    )
    return {"ok": True, "home": pose}


# ----------------------------------------------------------------------
# Missions
# ----------------------------------------------------------------------
@router.get("/missions")
async def list_missions(request: Request) -> dict[str, Any]:
    engine = request.app.state.engine
    return {"missions": engine.list_missions(), "active": engine.snapshot()}


@router.post("/missions", dependencies=[Depends(require_operator)])
async def create_mission(request: Request, payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
    engine = request.app.state.engine
    try:
        mission = engine.create(payload)
    except (ValueError, KeyError, TypeError) as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    return {"ok": True, "mission": mission.to_dict(with_waypoints=False)}


@router.get("/missions/{mission_id}")
async def get_mission(request: Request, mission_id: str) -> dict[str, Any]:
    engine = request.app.state.engine
    mission = engine.missions.get(mission_id)
    if mission is None:
        raise HTTPException(status_code=404, detail="任务不存在")
    return mission.to_dict()


@router.post("/missions/{mission_id}/waypoints", dependencies=[Depends(require_operator)])
async def append_waypoint(
    request: Request, mission_id: str, payload: dict[str, Any] = Body(...)
) -> dict[str, Any]:
    engine = request.app.state.engine
    try:
        mission = engine.append_waypoint(mission_id, payload)
    except (ValueError, KeyError, TypeError) as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    return {"ok": True, "mission": mission.to_dict()}


@router.post("/missions/{mission_id}/start", dependencies=[Depends(require_operator)])
async def start_mission(request: Request, mission_id: str) -> dict[str, Any]:
    engine = request.app.state.engine
    try:
        mission = engine.start(mission_id)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    return {"ok": True, "mission": mission.to_dict(with_waypoints=False)}


@router.post("/missions/{mission_id}/pause", dependencies=[Depends(require_operator)])
async def pause_mission(request: Request, mission_id: str) -> dict[str, Any]:
    engine = request.app.state.engine
    try:
        mission = engine.pause()
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    return {"ok": True, "mission": mission.to_dict(with_waypoints=False)}


@router.post("/missions/{mission_id}/resume", dependencies=[Depends(require_operator)])
async def resume_mission(request: Request, mission_id: str) -> dict[str, Any]:
    engine = request.app.state.engine
    try:
        mission = engine.resume(mission_id)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    return {"ok": True, "mission": mission.to_dict(with_waypoints=False)}


@router.post("/missions/{mission_id}/skip", dependencies=[Depends(require_operator)])
async def skip_waypoint(request: Request, mission_id: str) -> dict[str, Any]:
    engine = request.app.state.engine
    try:
        mission = engine.skip()
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    return {"ok": True, "mission": mission.to_dict(with_waypoints=False)}


@router.post("/missions/{mission_id}/stop", dependencies=[Depends(require_operator)])
async def stop_mission(request: Request, mission_id: str) -> dict[str, Any]:
    engine = request.app.state.engine
    try:
        mission = engine.stop()
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    return {"ok": True, "mission": mission.to_dict(with_waypoints=False)}


# ----------------------------------------------------------------------
# Teleop / estop
# ----------------------------------------------------------------------
@router.post("/teleop/lease", dependencies=[Depends(require_operator)])
async def teleop_lease(request: Request, body: LeaseBody) -> dict[str, Any]:
    _check_ns(request, body.ns)
    owner = request.headers.get("x-client-id") or (
        request.client.host if request.client else "web"
    )
    await _take_manual_control(request)
    await request.app.state.node.cancel_goal(body.ns)
    try:
        return await request.app.state.teleop.acquire(body.ns, owner)
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=409, detail=str(exc))


@router.post("/teleop/release", dependencies=[Depends(require_operator)])
async def teleop_release(request: Request) -> dict[str, Any]:
    await request.app.state.teleop.release()
    return {"ok": True}


@router.post("/teleop/cmd", dependencies=[Depends(require_operator)])
async def teleop_cmd(request: Request, body: CmdBody) -> dict[str, Any]:
    try:
        request.app.state.teleop.set_cmd(body.ns, body.vx, body.wz)
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=409, detail=str(exc))
    return {"ok": True}


@router.post("/estop", dependencies=[Depends(require_operator)])
async def estop(request: Request) -> dict[str, Any]:
    await request.app.state.teleop.estop()
    request.app.state.engine.estop()
    return {"ok": True, "exploration_halted": True}


@router.post("/estop/clear", dependencies=[Depends(require_operator)])
async def clear_estop(request: Request) -> dict[str, Any]:
    await request.app.state.teleop.clear_estop()
    request.app.state.engine.clear_estop()
    return {"ok": True, "exploration_halted": request.app.state.node.exploration_halted}


# ----------------------------------------------------------------------
# Simulation / exploration
# ----------------------------------------------------------------------
@router.get("/sim/status")
async def sim_status(request: Request) -> dict[str, Any]:
    proc = request.app.state.proc
    return {
        "sim": proc.sim_status(),
        "exploration": proc.exploration_status(),
    }


@router.post("/sim/start", dependencies=[Depends(require_operator)])
async def sim_start(request: Request, body: SimStartBody) -> dict[str, Any]:
    proc = request.app.state.proc
    if not request.app.state.settings.manage_sim:
        raise HTTPException(status_code=409, detail="控制台未启用仿真管理")
    result = proc.start_sim(body.world, body.mode, body.headless)
    if result.get("ok") and "已在运行" not in str(result.get("message", "")):
        request.app.state.node.reset_session()
    return result


@router.post("/sim/stop", dependencies=[Depends(require_operator)])
async def sim_stop(request: Request) -> dict[str, Any]:
    if not request.app.state.settings.manage_sim:
        raise HTTPException(status_code=409, detail="控制台未启用仿真管理")
    result = request.app.state.proc.stop_sim()
    request.app.state.world_preview.stop()
    request.app.state.node.reset_session()
    return result


@router.get("/sim/worlds")
async def sim_worlds(request: Request) -> dict[str, Any]:
    meta = request.app.state.proc.sim_meta() or {}
    return {
        "worlds": [dict(world) for world in WORLDS],
        "active": meta.get("world_name"),
        "default": request.app.state.settings.default_world,
    }


@router.post("/sim/restart", dependencies=[Depends(require_operator)])
async def sim_restart(request: Request, body: SimRestartBody) -> dict[str, Any]:
    if not request.app.state.settings.manage_sim:
        raise HTTPException(status_code=409, detail="控制台未启用仿真管理")
    proc = request.app.state.proc
    # Stopping the launch script can take up to 25 s; keep the event loop free
    # so the map / video streams to other viewers do not stall meanwhile.
    result = await asyncio.to_thread(
        proc.restart_sim, body.world, body.mode, body.headless, body.clean
    )
    # The preview camera model lives inside the old world; it is gone now.
    request.app.state.world_preview.stop()
    request.app.state.node.reset_session()
    return result


@router.post("/exploration/start", dependencies=[Depends(require_operator)])
async def exploration_start(request: Request) -> dict[str, Any]:
    node = request.app.state.node
    if request.app.state.teleop.estopped:
        raise HTTPException(status_code=409, detail="急停未解除，无法启动自主勘探")
    proc = request.app.state.proc
    was_halted = bool(node.exploration_halted)
    was_running = bool(proc.exploration_status().get("running"))
    # Publish before spawn so a new coordinator latches "run", not the old halt.
    node.set_exploration_halt(False)
    if was_running:
        status = proc.exploration_status()
        return {
            "ok": True,
            "message": "自主勘探继续" if was_halted else "自主勘探已在运行",
            **status,
        }
    if "multi_robot_coordinator" in node.get_node_names():
        return {
            "ok": True,
            "message": "自主勘探继续" if was_halted else "自主勘探已在外部运行",
            "running": True,
        }
    return proc.start_exploration()


@router.post("/exploration/stop", dependencies=[Depends(require_operator)])
async def exploration_stop(request: Request) -> dict[str, Any]:
    return request.app.state.proc.stop_exploration()


# ----------------------------------------------------------------------
# Video
# ----------------------------------------------------------------------
@router.get("/cloud.bin")
async def cloud_bin(request: Request) -> Response:
    payload, seq, count = request.app.state.node.cloud.snapshot()
    if not payload:
        raise HTTPException(status_code=404, detail="暂无三维点云")
    return Response(
        payload,
        media_type="application/octet-stream",
        headers={
            "Cache-Control": "no-store",
            "X-Cloud-Seq": str(seq),
            "X-Cloud-Count": str(count),
        },
    )


# ----------------------------------------------------------------------
# Gazebo world preview
# ----------------------------------------------------------------------
@router.get("/world/preview/status")
async def world_preview_status(request: Request) -> dict[str, Any]:
    return request.app.state.world_preview.status()


@router.post("/world/preview/start", dependencies=[Depends(require_operator)])
async def world_preview_start(request: Request, body: WorldViewBody) -> dict[str, Any]:
    try:
        return await asyncio.to_thread(
            request.app.state.world_preview.start,
            yaw=body.yaw,
            pitch=body.pitch,
            distance=body.distance,
            ns=body.ns,
        )
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@router.post("/world/preview/view", dependencies=[Depends(require_operator)])
async def world_preview_view(request: Request, body: WorldViewBody) -> dict[str, Any]:
    # One gz service round-trip per call (~80 ms), so the UI throttles drags.
    return await asyncio.to_thread(
        request.app.state.world_preview.set_view,
        yaw=body.yaw,
        pitch=body.pitch,
        distance=body.distance,
        ns=body.ns,
    )


@router.post("/world/preview/stop", dependencies=[Depends(require_operator)])
async def world_preview_stop(request: Request) -> dict[str, Any]:
    return await asyncio.to_thread(request.app.state.world_preview.stop)


@router.get("/world/preview.mjpeg")
async def world_preview_stream(request: Request, fps: float = 0.0):
    hub = request.app.state.world_hub
    if request.app.state.settings.max_streams and hub.clients >= request.app.state.settings.max_streams:
        raise HTTPException(status_code=503, detail="视频连接数已达上限")
    hub.clients += 1

    async def generator():
        try:
            async for chunk in mjpeg_stream(hub, fps):
                if await request.is_disconnected():
                    break
                yield chunk
        finally:
            hub.clients = max(0, hub.clients - 1)

    return StreamingResponse(
        generator(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/video/{ns}.mjpeg")
async def video_stream(request: Request, ns: str, fps: float = 0.0):
    _check_ns(request, ns)
    hubs = request.app.state.video_hubs
    hub = hubs.get(ns)
    if hub is None:
        raise HTTPException(status_code=404, detail="no camera")
    if hub.clients >= request.app.state.settings.max_streams:
        raise HTTPException(status_code=503, detail="视频连接数已达上限")
    hub.clients += 1

    async def generator():
        try:
            async for chunk in mjpeg_stream(hub, fps):
                if await request.is_disconnected():
                    break
                yield chunk
        finally:
            hub.clients = max(0, hub.clients - 1)

    return StreamingResponse(
        generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store", "Connection": "close"},
    )


@router.get("/video/{ns}/snapshot.jpg")
async def video_snapshot(request: Request, ns: str) -> Response:
    _check_ns(request, ns)
    hub = request.app.state.video_hubs.get(ns)
    jpeg = hub.snapshot() if hub else None
    if not jpeg:
        raise HTTPException(status_code=404, detail="暂无画面")
    return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


# ----------------------------------------------------------------------
# YOLO placeholder
# ----------------------------------------------------------------------
@router.post("/detections", dependencies=[Depends(require_operator)])
async def push_detections(request: Request, body: DetectionBody):
    _check_ns(request, body.ns)
    payload = {
        "type": "detections",
        "ns": body.ns,
        "ts": body.ts or time.time(),
        "frame_id": body.frame_id,
        "boxes": [box.model_dump() for box in body.boxes],
    }
    request.app.state.detections[body.ns] = payload
    request.app.state.hub.publish_nowait(payload)
    return {"ok": True, "count": len(body.boxes)}


@router.get("/detections")
async def get_detections(request: Request) -> dict[str, Any]:
    now = time.time()
    return {
        ns: payload
        for ns, payload in request.app.state.detections.items()
        if now - payload.get("ts", 0) <= DETECTION_TTL
    }
