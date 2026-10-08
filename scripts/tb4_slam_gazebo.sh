#!/usr/bin/env bash
# Launch Nav2 TurtleBot4 + Gazebo + SLAM on a large official world (depot / warehouse).
# No custom world authoring needed — models download from Fuel on first run.
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

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

is_vmware=0
if lspci 2>/dev/null | grep -qi 'VMware'; then
  is_vmware=1
fi

if [[ "${is_vmware}" -eq 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  HEADLESS="${HEADLESS:-True}"
  USE_COMPOSITION="${USE_COMPOSITION:-False}"
  echo "[tb4_slam_gazebo] Detected VMware guest"
else
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-True}"
fi

if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == "1" ]]; then
  echo "[tb4_slam_gazebo] Using software OpenGL (LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}, MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE:-unset})"
fi

SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim"
# Default: warehouse (larger / more complex). Alternative: WORLD=depot
WORLD_NAME="${WORLD:-warehouse}"
case "${WORLD_NAME}" in
  warehouse)
    WORLD_FILE="${SIM_SHARE}/worlds/warehouse.sdf"
    X_POSE="2.00"
    Y_POSE="-19.65"
    YAW="1.57"
    ;;
  depot)
    WORLD_FILE="${SIM_SHARE}/worlds/depot.sdf"
    X_POSE="0.00"
    Y_POSE="0.00"
    YAW="0.00"
    ;;
  *)
    # Treat WORLD as a path to an SDF/xacro
    WORLD_FILE="${WORLD_NAME}"
    X_POSE="${X_POSE:-0.00}"
    Y_POSE="${Y_POSE:-0.00}"
    YAW="${YAW:-0.00}"
    ;;
esac

if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[tb4_slam_gazebo] World not found: ${WORLD_FILE}" >&2
  echo "Built-in: warehouse | depot" >&2
  exit 1
fi

echo "[tb4_slam_gazebo] DISPLAY=${DISPLAY} WAYLAND_DISPLAY=${WAYLAND_DISPLAY} XAUTHORITY=${XAUTHORITY:-unset}"
echo "[tb4_slam_gazebo] world=${WORLD_FILE} spawn=(${X_POSE},${Y_POSE},yaw=${YAW}) headless=${HEADLESS} use_composition=${USE_COMPOSITION}"
echo "[tb4_slam_gazebo] First run downloads Fuel models (Warehouse/Depot); needs working network, may take several minutes."
echo "[tb4_slam_gazebo] Launching: ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True ..."

exec ros2 launch nav2_bringup tb4_simulation_launch.py \
  slam:=True \
  headless:="${HEADLESS}" \
  use_composition:="${USE_COMPOSITION}" \
  use_sim_time:=True \
  world:="${WORLD_FILE}" \
  x_pose:="${X_POSE}" \
  y_pose:="${Y_POSE}" \
  yaw:="${YAW}" \
  "$@"
