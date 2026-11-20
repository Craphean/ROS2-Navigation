"""Three-criterion re-openable convergence tracker (2D frontier + 3D residual + voxel growth)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ConvergenceParams:
    enabled: bool = True
    cycles_required: int = 6
    residual_gain_threshold: float = 0.50
    voxel_growth_epsilon: int = 25
    voxel_growth_rate: float = 0.005
    require_3d: bool = True
    fallback_timeout: float = 120.0


@dataclass
class ConvergenceState:
    converged: bool = False
    streak: int = 0
    two_d_done: bool = False
    residual_done: bool = False
    growth_done: bool = False
    reason: str = ""


class ConvergenceTracker:
    def __init__(self, params: ConvergenceParams):
        self.params = params
        self.converged = False
        self.streak = 0
        self._last_activity = 0.0
        self._initialized = False
        self.last_reset_reason = ""

    def reset(self, reason: str = "") -> None:
        self.converged = False
        self.streak = 0
        self._last_activity = 0.0
        self._initialized = False
        self.last_reset_reason = reason

    def update(
        self,
        now: float,
        *,
        map_valid: bool,
        frontier_count: int,
        all_idle: bool,
        residual_count: int,
        residual_raw_gain: float,
        index_available: bool,
        new_columns: int,
        total_columns: int,
    ) -> ConvergenceState:
        p = self.params
        if not p.enabled:
            self.reset("disabled")
            return ConvergenceState(reason="disabled")

        if not self._initialized:
            self._last_activity = now
            self._initialized = True

        busy = (
            not map_valid
            or frontier_count > 0
            or residual_count > 0
            or not all_idle
        )
        if busy:
            self._last_activity = now

        two_d_done = bool(map_valid and frontier_count == 0)

        if index_available:
            residual_done = (
                residual_count == 0
                and residual_raw_gain <= p.residual_gain_threshold
            )
            growth_done = (
                new_columns <= p.voxel_growth_epsilon
                or new_columns <= p.voxel_growth_rate * max(total_columns, 1)
            )
        elif p.require_3d:
            residual_done = False
            growth_done = False
        else:
            quiet_for = now - self._last_activity
            residual_done = quiet_for >= p.fallback_timeout
            growth_done = True

        ready = all_idle and two_d_done and residual_done and growth_done
        if ready:
            self.streak += 1
        else:
            self.streak = 0
            reopened = (
                not map_valid
                or frontier_count > 0
                or residual_count > 0
                or not all_idle
                or (index_available and not growth_done)
            )
            if self.converged and reopened:
                self.converged = False
                self.last_reset_reason = "convergence conditions no longer hold"

        if not self.converged and self.streak >= max(1, p.cycles_required):
            self.converged = True

        return ConvergenceState(
            converged=self.converged,
            streak=self.streak,
            two_d_done=two_d_done,
            residual_done=residual_done,
            growth_done=growth_done,
            reason=(
                f"2d={two_d_done} residual={residual_done} "
                f"growth={growth_done} idle={all_idle} streak={self.streak}"
            ),
        )
