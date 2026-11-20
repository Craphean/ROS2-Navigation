#!/usr/bin/env python3
"""Backwards-compatible shim re-exporting hier_explore_core."""

from __future__ import annotations

import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1] / "src" / "hier_explore_core"
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from hier_explore_core import (  # noqa: E402, F401
    FREE,
    LETHAL,
    UNKNOWN,
    AgentState,
    ConvergenceParams,
    ConvergenceState,
    ConvergenceTracker,
    InfoGainParams,
    OccupancyView,
    Ownership,
    ResidualCandidate,
    ResidualParams,
    RoiParams,
    TourParams,
    VoxelColumnIndex,
    VoxelParams,
    WindowStats,
    assign_ownership,
    build_local_tour,
    detect_residuals,
    exclude_near_waypoints,
    global_bid,
    nearest_neighbor_order,
    open_path_2opt,
    path_length,
    wrap_angle,
)
from hier_explore_core.occupancy_view import _classify  # noqa: E402, F401
