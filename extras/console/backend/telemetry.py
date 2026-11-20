"""Trail and coverage history for the progress panel.

Kept server-side rather than in the browser: the numbers have to survive a
page reload (a judge refreshing the tab must not reset the demo) and every
viewer should see the same trail.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any

# Comet trail: only the last few seconds of motion stay on the map, so the
# picture keeps following the robot instead of turning into a permanent
# scribble of every route ever driven this session.
_TRAIL_MAX = 1200
_TRAIL_MIN_DIST = 0.05
# Coverage moves slowly; a sample every two seconds is plenty.
_COV_MAX = 900
_COV_MIN_DT = 2.0


class Telemetry:
    def __init__(
        self,
        robots: tuple[str, ...],
        target_area: float = 300.0,
        trail_seconds: float = 5.0,
    ) -> None:
        self.target_area = max(1.0, float(target_area))
        self.trail_seconds = max(0.5, float(trail_seconds))
        self._lock = threading.Lock()
        self._trail: dict[str, deque] = {ns: deque(maxlen=_TRAIL_MAX) for ns in robots}
        self._last_pose: dict[str, tuple[float, float, float]] = {}
        self._coverage: deque = deque(maxlen=_COV_MAX)
        # One clock for both throttling and the x axis.  Mixing the sim clock
        # (seconds since Gazebo started) with wall time made every "now - last"
        # negative, so the series never advanced after the first sample.
        self._t0 = time.monotonic()
        self._last_cov = -1e9
        self._last_pub = 0.0
        self._last_trail = 0.0

    def add_pose(self, ns: str, x: float, y: float, _clock: float = 0.0) -> None:
        now = time.monotonic()
        with self._lock:
            prev = self._last_pose.get(ns)
            if prev is not None:
                # A parked robot must not grow the trail: distance alone decides,
                # otherwise it logs the same spot twice a second.
                if math.hypot(x - prev[0], y - prev[1]) < _TRAIL_MIN_DIST:
                    return
            self._last_pose[ns] = (x, y, now)
            points = self._trail[ns]
            points.append((round(x, 2), round(y, 2), now))
            self._prune_locked(points, now)

    def _prune_locked(self, points: deque, now: float) -> None:
        limit = now - self.trail_seconds
        while points and points[0][2] < limit:
            points.popleft()

    def add_coverage(self, _clock: float, area: float, target: float | None = None) -> None:
        """Record explored area; the bar is area against a configured target.

        A ratio against the current map bbox is useless (the merged map is
        already ~97% known from the first publish), so the progress the demo
        shows is "how much floor has been mapped" against CONSOLE_TARGET_AREA.
        """
        if area <= 0.0:
            return
        if target and target > 0:
            self.target_area = float(target)
        with self._lock:
            now = time.monotonic()
            if now - self._last_cov < _COV_MIN_DT:
                return
            self._last_cov = now
            progress = min(1.0, area / self.target_area)
            self._coverage.append(
                [round(now - self._t0, 1), round(progress, 4), round(area, 1)]
            )

    def due(self, now: float, period: float = 1.0) -> bool:
        with self._lock:
            if now - self._last_pub < period:
                return False
            self._last_pub = now
            return True

    def due_trail(self, now: float, period: float = 0.33) -> bool:
        """Trail alone, 3 Hz: a one-second comet looks like it teleports."""
        with self._lock:
            if now - self._last_trail < period:
                return False
            self._last_trail = now
            return True

    def trail_snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            trail = {}
            for ns, points in self._trail.items():
                self._prune_locked(points, now)
                trail[ns] = [[p[0], p[1]] for p in points]
            return {"type": "trail", "trail": trail, "server_time": time.time()}

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            trail = {}
            for ns, points in self._trail.items():
                self._prune_locked(points, now)
                trail[ns] = [[p[0], p[1]] for p in points]
            return {
                "type": "telemetry",
                "trail": trail,
                "coverage": [list(row) for row in self._coverage],
                "target_area": self.target_area,
                "server_time": time.time(),
            }

    def clear(self) -> None:
        with self._lock:
            for points in self._trail.values():
                points.clear()
            self._last_pose.clear()
            self._coverage.clear()
            self._last_cov = -1e9
