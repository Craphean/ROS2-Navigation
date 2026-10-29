"""FastAPI application assembly and lifecycle."""

from __future__ import annotations

import asyncio
import contextlib
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .api import router as api_router
from .config import Settings
from .mission import MissionEngine
from .procman import ProcSupervisor
from .ros_runtime import RosRuntime
from .teleop import TeleopGate
from .video import FrameHub
from .worldview import WorldPreview
from .ws import WsHub, build_ws_router, state_pump


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or Settings()
    settings.ensure_dirs()

    video_hubs = {ns: FrameHub(settings.jpeg_quality) for ns in settings.robots}
    # Gazebo preview: 15 Hz source. Encoding is the only per-frame CPU cost, and
    # the camera moves with the operator's drag, so keep most of the source rate.
    # No encode throttle: the preview frames are small (640x360) and the hub
    # receives them in near-duplicate pairs, which a rate limit silently halves.
    world_hub = FrameHub(settings.jpeg_quality, max_fps=0.0)
    hub = WsHub()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        loop = asyncio.get_running_loop()
        for frame_hub in video_hubs.values():
            frame_hub.bind_loop(loop)
        world_hub.bind_loop(loop)

        runtime = RosRuntime(settings, video_hubs, world_hub)
        app.state.runtime = runtime
        app.state.node = runtime.node
        app.state.world_preview = WorldPreview(settings, runtime.node)

        engine = MissionEngine(runtime.node, settings, emit=hub.publish_nowait)
        app.state.engine = engine

        teleop = TeleopGate(runtime.node, settings)
        teleop.start()
        app.state.teleop = teleop

        pump = asyncio.create_task(
            state_pump(runtime.node, engine, hub, period=0.1), name="state-pump"
        )
        app.state.started_at = time.time()
        try:
            yield
        finally:
            pump.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await pump
            await teleop.stop()
            app.state.world_preview.stop()
            app.state.proc.shutdown()
            await hub.close_all()
            runtime.shutdown()

    app = FastAPI(
        title="矿井智能巡检机器人控制台",
        version="1.0.0",
        lifespan=lifespan,
        docs_url="/api/docs",
        openapi_url="/api/openapi.json",
    )
    app.state.settings = settings
    app.state.hub = hub
    app.state.video_hubs = video_hubs
    app.state.world_hub = world_hub
    app.state.proc = ProcSupervisor(settings)
    app.state.map_cache = {}
    app.state.detections = {}

    app.include_router(api_router)
    app.include_router(build_ws_router())

    if settings.web_dir.exists():
        app.mount("/", StaticFiles(directory=str(settings.web_dir), html=True), name="web")

    return app


app = create_app()
