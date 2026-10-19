#!/usr/bin/env bash
# ==============================================================================
# Live Exploration Telemetry & Stall Diagnostics Monitor Dashboard
# ==============================================================================
# Real-time interactive TUI dashboard displaying:
#   - Robot status (IDLE, NAVIGATING, STALLED), linear/angular velocity
#   - Exploration Markers (Candidate green spheres, assigned goals, blacklisted dead-ends)
#   - Stall root-cause diagnostics (why did the robot stop?)
#   - Clean structured events stream from log/exploration_debug.log
#
# Usage:
#   ./scripts/monitor_exploration.sh
#   ./scripts/monitor_exploration.sh --stream       # Append stream mode
#   ./scripts/monitor_exploration.sh --interval 1.0 # 1-second refresh rate
# ==============================================================================

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source ROS 2 if available
if [[ -f "/opt/ros/lyrical/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/lyrical/setup.bash" 2>/dev/null || true
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash" 2>/dev/null || true
fi

exec python3 "${ROOT_DIR}/scripts/monitor_dashboard.py" "$@"
