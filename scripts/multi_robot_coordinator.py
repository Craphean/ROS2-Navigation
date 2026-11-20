#!/usr/bin/env python3
"""Backwards-compatible launcher shim delegating to hier_explore_core.coordinator_node."""

from __future__ import annotations

import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1] / "src" / "hier_explore_core"
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from hier_explore_core.coordinator_node import (  # noqa: E402, F401
    MultiRobotCoordinator,
    main,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from hier_explore_core.frontier import FrontierCluster  # noqa: E402, F401
from hier_explore_core.robot_tracker import RobotTracker  # noqa: E402, F401

if __name__ == "__main__":
    main()
