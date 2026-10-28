"""Supervise the Gazebo simulation and the exploration coordinator.

The console must be able to start/stop the simulation without killing itself:

* every child is started in its own session (``start_new_session``) and stopped
  with ``killpg``, because the launch script itself spawns ``setsid`` children;
* ``kill_nav2_sim.sh`` is never invoked directly - it deletes ``/dev/shm``
  segments that the console's own DDS node is using.  Children get
  ``SKIP_DDS_SHM_CLEAN=1`` so their cleanup trap leaves shm alone.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

from .config import Settings


class ProcSupervisor:
    def __init__(self, settings: Settings):
        self.settings = settings
        self.run_dir = settings.run_dir
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._sim: subprocess.Popen | None = None
        self._coord: subprocess.Popen | None = None
        self._sim_started_at = 0.0
        self._coord_started_at = 0.0
        # Serialise start/stop: two browser tabs (or a curl retry) must never
        # spawn two launch scripts, because each one calls kill_nav2_sim.sh and
        # would kill the other's wrapper.
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    def _base_env(self) -> dict[str, str]:
        env = dict(os.environ)
        env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        env.setdefault(
            "CYCLONEDDS_URI",
            f"file://{self.settings.root / 'config' / 'cyclonedds.xml'}",
        )
        env["SKIP_DDS_SHM_CLEAN"] = "1"
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def _spawn(
        self, name: str, argv: list[str], extra_env: dict[str, str]
    ) -> subprocess.Popen:
        env = self._base_env()
        env.update(extra_env)
        log_path = self.run_dir / f"{name}.log"
        log = open(log_path, "ab", buffering=0)
        process = subprocess.Popen(
            argv,
            cwd=str(self.settings.root),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        (self.run_dir / f"{name}.pid").write_text(
            json.dumps(
                {
                    "pid": process.pid,
                    "pgid": os.getpgid(process.pid),
                    "started": time.time(),
                }
            ),
            encoding="utf-8",
        )
        return process

    @staticmethod
    def _stop_process(process: subprocess.Popen | None, timeout: float = 25.0) -> None:
        if process is None:
            return
        if process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    # ------------------------------------------------------------------
    def sim_status(self) -> dict[str, Any]:
        running = self._sim is not None and self._sim.poll() is None
        return {
            "running": running,
            "pid": self._sim.pid if self._sim else None,
            "uptime": round(time.time() - self._sim_started_at, 1)
            if running
            else 0,
            "log": str(self.run_dir / "sim.log"),
        }

    def start_sim(
        self, world: str | None = None, mode: str | None = None, headless: bool = True
    ) -> dict[str, Any]:
        with self._lock:
            if self._sim is not None and self._sim.poll() is None:
                return {"ok": True, "message": "仿真已在运行", **self.sim_status()}
            script = self.settings.root / "scripts" / "tb3_multi_explore_slam.sh"
            if not script.exists():
                return {"ok": False, "message": f"找不到 {script}"}
            env = {
                "WORLD": world or self.settings.default_world,
                "EXPLORATION_MODE": mode or self.settings.default_mode,
                "HEADLESS": "True" if headless else "False",
                "USE_RVIZ": "False",
                "DASHBOARD": "False",
            }
            self._sim = self._spawn("sim", ["bash", str(script)], env)
            self._sim_started_at = time.time()
            return {"ok": True, **self.sim_status()}

    def stop_sim(self) -> dict[str, Any]:
        with self._lock:
            self._stop_process(self._sim)
            self._sim = None
            self._sim_started_at = 0.0
            return {"ok": True, **self.sim_status()}

    # ------------------------------------------------------------------
    def exploration_status(self) -> dict[str, Any]:
        running = self._coord is not None and self._coord.poll() is None
        return {
            "running": running,
            "pid": self._coord.pid if self._coord else None,
            "uptime": round(time.time() - self._coord_started_at, 1)
            if running
            else 0,
            "log": str(self.run_dir / "coordinator.log"),
        }

    def start_exploration(self) -> dict[str, Any]:
        with self._lock:
            if self._coord is not None and self._coord.poll() is None:
                return {"ok": True, "message": "自主勘探已在运行", **self.exploration_status()}
            meta = self._read_sim_meta()
            if meta is None:
                return {"ok": False, "message": "缺少 sim_meta.json，请先启动仿真"}
            r1 = meta.get("robot1", [0, 0, 0])
            r2 = meta.get("robot2", [0, 0, 0])
            argv = [
                sys.executable,
                str(self.settings.root / "scripts" / "multi_robot_coordinator.py"),
                "--ros-args",
                "--params-file",
                str(self.settings.root / "config" / "coordinator_params.yaml"),
                "-p",
                "use_sim_time:=true",
                "-p",
                "robot_namespaces:=['robot1', 'robot2']",
                "-p",
                f"robot1_init_pose:=[{r1[0]}, {r1[1]}, {r1[2]}]",
                "-p",
                f"robot2_init_pose:=[{r2[0]}, {r2[1]}, {r2[2]}]",
            ]
            self._coord = self._spawn("coordinator", argv, {})
            self._coord_started_at = time.time()
            return {"ok": True, **self.exploration_status()}

    def stop_exploration(self) -> dict[str, Any]:
        with self._lock:
            self._stop_process(self._coord, timeout=10.0)
            self._coord = None
            self._coord_started_at = 0.0
            return {"ok": True, **self.exploration_status()}

    # ------------------------------------------------------------------
    def _read_sim_meta(self) -> dict[str, Any] | None:
        path = self.settings.sim_meta_path
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def shutdown(self) -> None:
        self._stop_process(self._coord, timeout=5.0)
        self._coord = None
        if self.settings.kill_sim_on_exit:
            self._stop_process(self._sim)
            self._sim = None
