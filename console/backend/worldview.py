"""Live 3D preview of the running Gazebo world.

The console is headless, so "look at the world" means asking gz-sim for a
camera: spawn a static model carrying a camera sensor, bridge its image topic
into ROS, and orbit it through the world's set_pose service. The rendering
itself is the same EGL path the robots' cameras already use - no Gazebo GUI,
no X server.

Everything here is world-agnostic: the model is created by name into whatever
world the sim metadata points at, so switching world only invalidates the
model, never this code.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import signal
import subprocess
import threading
from pathlib import Path
from typing import Any

from .config import Settings

MODEL = "preview_cam"
TOPIC = "/preview_cam/image"

_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model}">
    <static>true</static>
    <pose>0 0 2.0 0 0 0</pose>
    <link name="link">
      <sensor name="cam" type="camera">
        <topic>{topic}</topic>
        <update_rate>{fps:.0f}</update_rate>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>{fov:.3f}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>400</far>
          </clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


class WorldPreview:
    def __init__(self, settings: Settings, node: Any = None) -> None:
        self.settings = settings
        self.node = node
        self.bridge: subprocess.Popen | None = None
        # Shoulder camera: the eye sits just behind the tracked robot and looks
        # along its heading.  Orbiting a fixed world point put the eye inside
        # walls in every indoor world; the space right behind a robot that got
        # there by driving is the one place guaranteed to be free.
        self.ns = settings.robots[0]
        # Orbit camera, like dragging in the Gazebo GUI: the eye circles a
        # pivot in the world, right-drag slides that pivot and the wheel decides
        # the radius.  A fixed "camera on the robot" view cannot be navigated -
        # this one lets the operator fly out of a room and look back in.
        self.target = [0.0, 0.0, 0.8]
        self.yaw = 0.0        # degrees, camera azimuth around the pivot
        self.pitch = 12.0     # degrees, elevation above the pivot
        # Roofed worlds hide everything above the ceiling slab, so the default
        # stays at eye level; orbit up only in open worlds.
        self.distance = 3.0
        self.active = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # gz helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _gz() -> str | None:
        return shutil.which("gz")

    def _env(self) -> dict[str, str]:
        env = dict(os.environ)
        env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        env.setdefault(
            "CYCLONEDDS_URI",
            f"file://{self.settings.root / 'config' / 'cyclonedds.xml'}",
        )
        env["SKIP_DDS_SHM_CLEAN"] = "1"
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def world_name(self) -> str | None:
        try:
            meta = json.loads(self.settings.sim_meta_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        name = meta.get("world_name")
        return str(name) if name else None

    def _service(self, service: str, reqtype: str, request: str) -> tuple[bool, str]:
        gz = self._gz()
        if gz is None:
            return False, "找不到 gz 命令"
        argv = [
            gz, "service", "-s", service, "--reqtype", reqtype,
            "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request,
        ]
        try:
            done = subprocess.run(
                argv, capture_output=True, text=True, timeout=10.0, env=self._env()
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return False, str(exc)
        output = ((done.stdout or "") + (done.stderr or "")).strip()
        return done.returncode == 0, output[-300:]

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def status(self) -> dict[str, Any]:
        return {
            "active": self.active,
            "world": self.world_name(),
            "bridge": bool(self.bridge and self.bridge.poll() is None),
            "ns": self.ns,
            "yaw": round(self.yaw, 1),
            "pitch": round(self.pitch, 1),
            "distance": round(self.distance, 2),
            "target": [round(v, 2) for v in self.target],
        }

    def start(self, **view: Any) -> dict[str, Any]:
        with self._lock:
            if self._gz() is None:
                raise RuntimeError("控制台环境里没有 gz 命令")
            world = self.world_name()
            if not world:
                raise RuntimeError("没有仿真世界信息，先启动仿真")
            self._write_sdf()
            self._create_model(world)
            self._start_bridge()
            self.active = True
            self._recenter(str(view.get("ns") or self.ns))
        # An explicit ns in the request wins over the recentered default.
        return self.set_view(**view)

    def set_view(self, **view: Any) -> dict[str, Any]:
        if not self.active:
            return {"ok": False, "message": "预览未开启", **self.status()}
        with self._lock:
            ns = view.get("ns")
            if ns and ns in self.settings.robots:
                self.ns = ns
                self._recenter(ns)
            target_xyz = view.get("target")
            if isinstance(target_xyz, (list, tuple)) and len(target_xyz) == 3:
                self.target = [float(v) for v in target_xyz]
            if view.get("yaw") is not None:
                self.yaw = float(view["yaw"]) % 360.0
            if view.get("pitch") is not None:
                self.pitch = min(88.0, max(2.0, float(view["pitch"])))
            if view.get("distance") is not None:
                self.distance = min(40.0, max(1.5, float(view["distance"])))
            eye, quat, target = self._eye()
            world = self.world_name()
            request = (
                f'name: "{MODEL}" '
                f"position: {{x: {eye[0]:.4f}, y: {eye[1]:.4f}, z: {eye[2]:.4f}}} "
                f"orientation: {{x: {quat[0]:.6f}, y: {quat[1]:.6f}, "
                f"z: {quat[2]:.6f}, w: {quat[3]:.6f}}}"
            )
            ok, output = self._service(f"/world/{world}/set_pose", "gz.msgs.Pose", request)
        if not ok:
            # A restarted world lost the model; the next start() recreates it.
            self.active = False
            return {"ok": False, "message": output or "设置相机位姿失败", **self.status()}
        return {
            "ok": True,
            **self.status(),
            "eye": [round(v, 2) for v in eye],
            "target": [round(v, 2) for v in target],
        }

    def stop(self) -> dict[str, Any]:
        with self._lock:
            self._kill_bridge()
            self.active = False
        return {"ok": True, **self.status()}

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------
    def _write_sdf(self) -> Path:
        path = self.settings.run_dir / f"{MODEL}.sdf"
        path.write_text(
            _SDF.format(
                model=MODEL,
                topic=TOPIC,
                fps=15.0,
                fov=1.15,
                width=960,
                height=540,
            ),
            encoding="utf-8",
        )
        return path

    def _create_model(self, world: str) -> None:
        path = self.settings.run_dir / f"{MODEL}.sdf"
        request = f'sdf_filename: "{path}"'
        ok, output = self._service(f"/world/{world}/create", "gz.msgs.EntityFactory", request)
        if not ok:
            # "already exists" is the normal case for a second start; anything
            # else (bad SDF) is worth surfacing in the log.
            if "exist" not in output.lower():
                raise RuntimeError(output or "创建预览相机失败")

    def _kill_stray_bridges(self) -> None:
        """Kill bridges left behind by a previous console process.

        Two bridges on the same ROS topic interleave frames and the view stops
        matching the camera pose; the console is SIGTERM'd far more often than
        it exits cleanly, so this cannot rely on stop() having run.
        """
        marker = TOPIC.encode()
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                cmd = (entry / "cmdline").read_bytes()
            except OSError:
                continue
            if b"parameter_bridge" in cmd and marker in cmd:
                try:
                    os.kill(int(entry.name), signal.SIGTERM)
                except OSError:
                    continue

    def _start_bridge(self) -> None:
        self._kill_stray_bridges()
        if self.bridge is not None and self.bridge.poll() is None:
            return
        argv = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image",
        ]
        log = open(self.settings.run_dir / "preview_bridge.log", "ab", buffering=0)
        self.bridge = subprocess.Popen(
            argv,
            cwd=str(self.settings.root),
            env=self._env(),
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def _kill_bridge(self) -> None:
        process = self.bridge
        self.bridge = None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            return
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    def _robot_pose(self, ns: str | None = None) -> tuple[float, float, float]:
        """(x, y, yaw) of a robot in world, TF first."""
        name = ns or self.ns
        if self.node is not None:
            pose = self.node.lookup_world_pose(name)
            if pose is not None:
                return (float(pose[0]), float(pose[1]), float(pose[2]))
        try:
            meta = json.loads(self.settings.sim_meta_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return (0.0, 0.0, 0.0)
        pose = meta.get(name) or meta.get("robot1") or [0.0, 0.0, 0.0]
        return (float(pose[0]), float(pose[1]), float(pose[2]))

    def _recenter(self, ns: str) -> None:
        """Put the pivot on a robot and look along its heading."""
        x, y, heading = self._robot_pose(ns)
        self.target = [x, y, 0.8]
        self.yaw = (math.degrees(heading) + 180.0) % 360.0

    def _eye(
        self,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float],
    ]:
        azimuth = math.radians(self.yaw)
        elevation = math.radians(self.pitch)
        ce = math.cos(elevation)
        eye = (
            self.target[0] + self.distance * ce * math.cos(azimuth),
            self.target[1] + self.distance * ce * math.sin(azimuth),
            self.target[2] + self.distance * math.sin(elevation),
        )
        dx = self.target[0] - eye[0]
        dy = self.target[1] - eye[1]
        dz = self.target[2] - eye[2]
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(dz, math.hypot(dx, dy))
        # Camera +X looks at the pivot: q = qz(yaw) * qy(pitch).
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        quat = (-sy * sp, cy * sp, sy * cp, cy * cp)
        return eye, quat, (self.target[0], self.target[1], self.target[2])
