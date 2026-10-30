"""Runtime settings for the mine inspection console."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class Settings:
    root: Path = ROOT
    robots: tuple[str, ...] = ("robot1", "robot2")
    # Bind all interfaces by default so the host machine / phone on the LAN can
    # reach the console.  Use CONSOLE_HOST=127.0.0.1 to restrict to loopback
    # (cpolar still works in that case).
    host: str = os.environ.get("CONSOLE_HOST", "0.0.0.0")
    port: int = int(os.environ.get("CONSOLE_PORT", "8080"))
    # Shared operator password.  Judges may read the dashboard anonymously,
    # but every control action requires this token.
    token: str = os.environ.get("CONSOLE_TOKEN", "mine2026")
    map_hz: float = float(os.environ.get("CONSOLE_MAP_HZ", "1.0"))
    video_fps: float = float(os.environ.get("CONSOLE_VIDEO_FPS", "5.0"))
    jpeg_quality: int = int(os.environ.get("CONSOLE_JPEG_QUALITY", "72"))
    max_streams: int = int(os.environ.get("CONSOLE_MAX_STREAMS", "6"))
    manage_sim: bool = _env_bool("CONSOLE_MANAGE_SIM", True)
    kill_sim_on_exit: bool = _env_bool("KILL_SIM_ON_EXIT", False)
    default_world: str = os.environ.get("CONSOLE_WORLD", "mine")
    # Floor area that counts as "the world explored" for the progress bar.  It
    # is a demo target, not something the console can measure.
    target_area: float = float(os.environ.get("CONSOLE_TARGET_AREA", "300"))
    default_mode: str = os.environ.get("CONSOLE_EXPLORATION_MODE", "off")

    @property
    def web_dir(self) -> Path:
        return self.root / "console" / "web"

    @property
    def run_dir(self) -> Path:
        return self.root / "console" / "run"

    @property
    def mission_dir(self) -> Path:
        return self.root / "console" / "missions"

    @property
    def sim_meta_path(self) -> Path:
        return self.run_dir / "sim_meta.json"

    def ensure_dirs(self) -> None:
        for path in (self.run_dir, self.run_dir / "missions", self.mission_dir):
            path.mkdir(parents=True, exist_ok=True)
