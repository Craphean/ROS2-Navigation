#!/usr/bin/env bash
# ==============================================================================
# Foxglove WebSocket Bridge Launcher for ROS 2 Navigation & Gazebo Simulation
# ==============================================================================
# Bridges ROS 2 topics (CycloneDDS) to Foxglove Studio over WebSockets.
#
# Defaults:
#   Port: 8765
#   Address: 0.0.0.0 (allows external connections from host OS browser/app)
#   Buffer: 100MB (for high-res merged OccupancyGrid /map and point clouds)
#   use_sim_time: true (synchronized with Gazebo clock)
#
# Usage:
#   ./scripts/foxglove_bridge.sh
#   ./scripts/foxglove_bridge.sh -p port:=8766
# ==============================================================================

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source ROS 2
if [[ -f "/opt/ros/lyrical/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/lyrical/setup.bash" 2>/dev/null || true
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash" 2>/dev/null || true
fi

# Ensure identical DDS environment as simulation
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${ROOT_DIR}/config/cyclonedds.xml}"

PORT="${PORT:-8765}"
ADDRESS="${ADDRESS:-0.0.0.0}"
BUFFER="${BUFFER:-100000000}"

# Detect primary IP address for external connections from host OS
VM_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")

echo "=============================================================================="
echo " 🌐 Launching Foxglove WebSocket Bridge"
echo "=============================================================================="
echo " - RMW Implementation:  ${RMW_IMPLEMENTATION}"
echo " - CycloneDDS Config:   ${CYCLONEDDS_URI}"
echo " - Listening Port:      ${PORT}"
echo " - Listen Address:      ${ADDRESS}"
echo " - Send Buffer Limit:   $((BUFFER / 1024 / 1024)) MB"
echo "------------------------------------------------------------------------------"
echo " 👉 Connect Foxglove Studio to:"
echo "    • Local (in VM):    ws://localhost:${PORT}"
echo "    • Host (from Host): ws://${VM_IP}:${PORT}"
echo "=============================================================================="

exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p port:="${PORT}" \
  -p address:="${ADDRESS}" \
  -p send_buffer_limit:="${BUFFER}" \
  -p use_sim_time:=true \
  "$@"

