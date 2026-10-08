#!/usr/bin/env bash
# Launch Nav2 official TurtleBot3 + Gazebo + SLAM demo (ROS 2 Lyrical).
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS setup scripts reference optional unbound vars; keep nounset off until sourced.
source /opt/ros/lyrical/setup.bash
set -u

# Attach to the local GNOME Wayland session (SSH / Cursor terminals often lack DISPLAY).
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export DISPLAY="${DISPLAY:-:0}"

if [[ -z "${XAUTHORITY:-}" ]]; then
  for auth in "${XDG_RUNTIME_DIR}"/.mutter-Xwaylandauth.*; do
    if [[ -f "${auth}" ]]; then
      export XAUTHORITY="${auth}"
      break
    fi
  done
fi

# Prefer XWayland for Qt (Gazebo GUI / RViz) on VMware guests.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# VMware / no real GPU: software GL so gpu_lidar can raycast via Mesa.
# See: https://github.com/gazebosim/gz-sensors/issues/26
# Use grep (not rg) so this works in a normal GNOME/SSH terminal without Cursor PATH.
is_vmware=0
if lspci 2>/dev/null | grep -qi 'VMware'; then
  is_vmware=1
fi

if [[ "${is_vmware}" -eq 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  # Skip Gazebo 3D GUI by default (RViz still starts). Override: HEADLESS=False
  HEADLESS="${HEADLESS:-True}"
  # Low-RAM VMs often hang loading composable Nav2 into one container;
  # run servers as separate processes so RViz can find controller/planner.
  USE_COMPOSITION="${USE_COMPOSITION:-False}"
  echo "[tb3_slam_gazebo] Detected VMware guest"
else
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-True}"
fi

if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == "1" ]]; then
  echo "[tb3_slam_gazebo] Using software OpenGL (LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}, MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE:-unset})"
fi

# Official gpu_lidar SDF by default. Optional override, e.g. ROBOT_SDF=/path/to/custom.xacro
ROBOT_SDF_ARGS=()
if [[ -n "${ROBOT_SDF:-}" ]]; then
  ROBOT_SDF_ARGS+=(robot_sdf:="${ROBOT_SDF}")
fi

# World file. Examples:
#   WORLD=empty ./scripts/tb3_slam_gazebo.sh
#   WORLD=/path/to/my_world.sdf.xacro ./scripts/tb3_slam_gazebo.sh
SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb3_sim)/share/nav2_minimal_tb3_sim"
case "${WORLD:-sandbox}" in
  sandbox|tb3_sandbox)
    WORLD_FILE="${SIM_SHARE}/worlds/tb3_sandbox.sdf.xacro"
    ;;
  empty|tb3_empty|tb3_empty_world)
    WORLD_FILE="${SIM_SHARE}/worlds/tb3_empty_world.sdf.xacro"
    ;;
  *)
    WORLD_FILE="${WORLD}"
    ;;
esac
if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[tb3_slam_gazebo] World not found: ${WORLD_FILE}" >&2
  echo "Built-in: sandbox | empty" >&2
  exit 1
fi

echo "[tb3_slam_gazebo] DISPLAY=${DISPLAY} WAYLAND_DISPLAY=${WAYLAND_DISPLAY} XAUTHORITY=${XAUTHORITY:-unset}"
echo "[tb3_slam_gazebo] world=${WORLD_FILE} headless=${HEADLESS} use_composition=${USE_COMPOSITION}"
echo "[tb3_slam_gazebo] Launching: ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True ..."

exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=True \
  headless:="${HEADLESS}" \
  use_composition:="${USE_COMPOSITION}" \
  use_sim_time:=True \
  world:="${WORLD_FILE}" \
  "${ROBOT_SDF_ARGS[@]}" \
  "$@"
