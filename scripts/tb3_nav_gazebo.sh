#!/usr/bin/env bash
# Launch Nav2 TurtleBot3 + Gazebo with a known map (AMCL localization, no SLAM).
# Prerequisite: save a map while running tb3_slam_gazebo.sh, e.g.:
#   ros2 run nav2_map_server map_saver_cli -f ~/tb3_map
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
is_vmware=0
if lspci 2>/dev/null | grep -qi 'VMware'; then
  is_vmware=1
fi

if [[ "${is_vmware}" -eq 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-False}"
  echo "[tb3_nav_gazebo] Detected VMware guest"
else
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-True}"
fi

if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == "1" ]]; then
  echo "[tb3_nav_gazebo] Using software OpenGL (LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}, MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE:-unset})"
fi

# Map YAML: MAP=~/tb3_map.yaml ./scripts/tb3_nav_gazebo.sh
# or: ./scripts/tb3_nav_gazebo.sh /path/to/map.yaml
if [[ $# -ge 1 && ( "${1}" == *.yaml || "${1}" == *.yml ) ]]; then
  MAP="${1}"
  shift
fi
MAP="${MAP:-${HOME}/tb3_map.yaml}"

if [[ ! -f "${MAP}" ]]; then
  echo "[tb3_nav_gazebo] Map not found: ${MAP}" >&2
  echo "Save one during SLAM first, e.g.:" >&2
  echo "  ros2 run nav2_map_server map_saver_cli -f ~/tb3_map" >&2
  exit 1
fi

ROBOT_SDF_ARGS=()
if [[ -n "${ROBOT_SDF:-}" ]]; then
  ROBOT_SDF_ARGS+=(robot_sdf:="${ROBOT_SDF}")
fi

# World must match the map you saved. Examples:
#   WORLD=empty MAP=~/empty_map.yaml ./scripts/tb3_nav_gazebo.sh
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
  echo "[tb3_nav_gazebo] World not found: ${WORLD_FILE}" >&2
  echo "Built-in: sandbox | empty" >&2
  exit 1
fi

# Map frame origin is the SLAM start pose (default Gazebo spawn -2, -0.5).
SLAM_WORLD_X="${SLAM_WORLD_X:--2.0}"
SLAM_WORLD_Y="${SLAM_WORLD_Y:--0.5}"
X_POSE="${X_POSE:-0.00}"
Y_POSE="${Y_POSE:-0.50}"
YAW_POSE="${YAW_POSE:-0.00}"
MAP_X="$(python3 -c "print(f'{float(\"${X_POSE}\") - float(\"${SLAM_WORLD_X}\"):.2f}')")"
MAP_Y="$(python3 -c "print(f'{float(\"${Y_POSE}\") - float(\"${SLAM_WORLD_Y}\"):.2f}')")"

echo "[tb3_nav_gazebo] DISPLAY=${DISPLAY} WAYLAND_DISPLAY=${WAYLAND_DISPLAY} XAUTHORITY=${XAUTHORITY:-unset}"
echo "[tb3_nav_gazebo] world=${WORLD_FILE} map=${MAP} headless=${HEADLESS} use_composition=${USE_COMPOSITION}"
echo "[tb3_nav_gazebo] Gazebo spawn (${X_POSE}, ${Y_POSE}), map initial pose (${MAP_X}, ${MAP_Y}, yaw ${YAW_POSE})."
echo "[tb3_nav_gazebo] Launching: ros2 launch nav2_bringup tb3_simulation_launch.py slam:=False map:=${MAP} ..."

# Publish spawn pose early so AMCL creates map->odom before Nav2 planner activation times out.
(
  sleep 8
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ros2 topic list 2>/dev/null | grep -qx '/initialpose'; then
      break
    fi
    sleep 1
  done
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: ${MAP_X}, y: ${MAP_Y}, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
    }
  }" >/dev/null 2>&1 || true
  sleep 5
  # If navigation lifecycle aborted during bringup, try startup again after pose exists.
  ros2 service call /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1 || true
) &

exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=False \
  map:="${MAP}" \
  world:="${WORLD_FILE}" \
  headless:="${HEADLESS}" \
  use_composition:="${USE_COMPOSITION}" \
  use_sim_time:=True \
  x_pose:="${X_POSE}" \
  y_pose:="${Y_POSE}" \
  yaw:="${YAW_POSE}" \
  "${ROBOT_SDF_ARGS[@]}" \
  "$@"
