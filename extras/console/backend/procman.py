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
        self._sim_pid: int | None = None
        self._sim_pgid: int | None = None
        self._sim_started_at = 0.0
        self._coord_started_at = 0.0
        # Serialise start/stop: two browser tabs (or a curl retry) must never
        # spawn two launch scripts, because each one calls kill_nav2_sim.sh and
        # would kill the other's wrapper.
        self._lock = threading.Lock()
        # The console restarts far more often than the sim does (a code change
        # is enough).  Re-attach to whatever the last console left running,
        # otherwise "restart sim" would stack a second gz-sim on top of it.
        self._adopt_running_sim()

    def _adopt_running_sim(self) -> None:
        try:
            data = json.loads((self.run_dir / "sim.pid").read_text(encoding="utf-8"))
            pid = int(data["pid"])
            pgid = int(data.get("pgid", pid))
            started = float(data.get("started", 0.0))
        except (OSError, ValueError, KeyError, TypeError):
            return
        try:
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes()
        except OSError:
            return
        if b"tb3_multi_explore_slam.sh" not in cmd:
            return
        self._sim_pid = pid
        self._sim_pgid = pgid
        self._sim_started_at = started

    def _sim_alive(self) -> bool:
        if self._sim is not None:
            return self._sim.poll() is None
        if not self._sim_pgid:
            return False
        try:
            os.killpg(self._sim_pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _stop_sim(self, timeout: float = 25.0) -> None:
        if self._sim is not None:
            self._stop_process(self._sim, timeout)
            self._sim = None
        elif self._sim_pgid and self._sim_alive():
            # Not our child: signal the process group and poll for its exit.
            try:
                os.killpg(self._sim_pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.time() + timeout
            while time.time() < deadline and self._sim_alive():
                time.sleep(0.2)
            if self._sim_alive():
                try:
                    os.killpg(self._sim_pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                time.sleep(1.0)
        self._sim_pid = None
        self._sim_pgid = None
        self._sim_started_at = 0.0

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
    def sim_meta(self) -> dict[str, Any] | None:
        return self._read_sim_meta()

    def sim_status(self) -> dict[str, Any]:
        running = self._sim_alive()
        meta = self._read_sim_meta() or {}
        return {
            "running": running,
            "pid": self._sim_pid,
            "uptime": round(time.time() - self._sim_started_at, 1)
            if running
            else 0,
            "world": meta.get("world_name"),
            "log": str(self.run_dir / "sim.log"),
        }

    def start_sim(
        self, world: str | None = None, mode: str | None = None, headless: bool = True
    ) -> dict[str, Any]:
        with self._lock:
            if self._sim_alive():
                return {"ok": True, "message": "仿真已在运行", **self.sim_status()}
            return self._start_locked(world, mode, headless)

    def restart_sim(
        self,
        world: str | None = None,
        mode: str | None = None,
        headless: bool = True,
        clean: bool = True,
    ) -> dict[str, Any]:
        """Stop, drop stale runtime junk, then start a fresh run.

        The previous run's map / cloud / TF are dropped by the API layer
        (node.reset_session); this only owns processes and /dev/shm.
        """
        with self._lock:
            self._stop_sim()
            cleaned = self._clean_runtime() if clean else []
            return {**self._start_locked(world, mode, headless), "cleaned": cleaned}

    def _start_locked(
        self, world: str | None, mode: str | None, headless: bool
    ) -> dict[str, Any]:
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
        self._sim_pid = self._sim.pid
        self._sim_pgid = os.getpgid(self._sim.pid)
        self._sim_started_at = time.time()
        return {"ok": True, **self.sim_status()}

    # ------------------------------------------------------------------
    _SHM_PATTERNS = (
        "cyclonedds*",
        "fastrtps_*",
        "fastdds*",
        "sem.fastdds*",
        "sem.fastrtps*",
    )

    def _clean_runtime(self) -> list[str]:
        """Delete DDS segments and node logs left behind by the stopped run.

        A sim that exits cleanly removes its own segments, but anything that
        had to be SIGKILLed leaves /dev/shm files behind and the next run may
        attach to them and then block.  Segments a live process still maps -
        the console's own rclpy node above all - are kept: unlinking those
        would give the sim a private shm namespace and the console would stop
        receiving anything.
        """
        removed: list[str] = []
        for path in sorted(Path("/dev/shm").glob("*")):
            if not any(path.match(pattern) for pattern in self._SHM_PATTERNS):
                continue
            if self._shm_in_use(path):
                continue
            try:
                path.unlink()
                removed.append(str(path))
            except OSError:
                continue
        for path in Path("/tmp").glob("tb3_multi_*.log"):
            try:
                path.unlink()
                removed.append(str(path))
            except OSError:
                continue
        return removed

    @staticmethod
    def _shm_in_use(path: Path) -> bool:
        try:
            completed = subprocess.run(
                ["fuser", str(path)], capture_output=True, timeout=5
            )
        except (OSError, subprocess.SubprocessError):
            return True
        return completed.returncode == 0

    def stop_sim(self) -> dict[str, Any]:
        with self._lock:
            self._stop_sim()
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
            self._stop_sim()
