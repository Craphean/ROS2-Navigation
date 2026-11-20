"""Backwards-compatible shim re-exporting pose continuity helper from hier_explore_core."""

from __future__ import annotations

import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1] / "src" / "hier_explore_core"
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from hier_explore_core.geometry import (  # noqa: E402, F401
    MAX_ROBOT_SPEED_MPS,
    POSE_STEP_MARGIN_M,
    pose_is_discontinuous,
)
