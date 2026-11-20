"""Pure-Python 2D geometry, SE(2) transforms, and pose continuity helpers."""

from __future__ import annotations

import math
from typing import Tuple

# Default thresholds for rejecting discontinuous TF/SLAM pose jumps.
MAX_ROBOT_SPEED_MPS = 0.65
POSE_STEP_MARGIN_M = 0.35


def wrap_angle(angle: float) -> float:
    """Wrap angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def se2_apply(
    transform: Tuple[float, float, float],
    x: float,
    y: float,
    yaw: float = 0.0,
) -> Tuple[float, float, float]:
    """Apply SE(2) transform (tx, ty, tyaw) as p_out = R(tyaw) * p + t."""
    tx, ty, tyaw = transform
    c = math.cos(tyaw)
    s = math.sin(tyaw)
    return (
        tx + x * c - y * s,
        ty + x * s + y * c,
        wrap_angle(tyaw + yaw),
    )


def se2_inverse(transform: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Invert an SE(2) transform (tx, ty, tyaw)."""
    tx, ty, tyaw = transform
    c = math.cos(tyaw)
    s = math.sin(tyaw)
    return (
        -(c * tx + s * ty),
        s * tx - c * ty,
        wrap_angle(-tyaw),
    )


def se2_compose(
    t_ab: Tuple[float, float, float],
    t_bc: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Compose two SE(2) transforms T_ac = T_ab * T_bc."""
    return se2_apply(t_ab, t_bc[0], t_bc[1], t_bc[2])


def pose_is_discontinuous(
    dt: float,
    dist: float,
    max_speed: float = MAX_ROBOT_SPEED_MPS,
    margin: float = POSE_STEP_MARGIN_M,
) -> bool:
    """True when a pose moved farther than the robot could physically travel.

    Backward or zero time deltas clamp ``dt`` to 0 so jitter stays below
    ``margin`` while real pose teleports are dropped.
    """
    return dist > margin + max_speed * max(0.0, dt)
