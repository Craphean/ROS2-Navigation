#!/usr/bin/env bash
# Launch Nav2 TurtleBot3 + Gazebo with RL local controller (TD3) and Navfn global planner.
# Prerequisite: save a map while running tb3_slam_gazebo.sh, e.g.:
#   ros2 run nav2_map_server map_saver_cli -f ~/tb3_map
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS setup scripts reference optional unbound vars; keep nounset off until sourced.
source /opt/ros/lyrical/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
else
  echo "[tb3_nav_rl_gazebo] Missing ${ROOT_DIR}/install — build first:" >&2
  echo "  cd ${ROOT_DIR} && colcon build --symlink-install --packages-select rl_nav2_controller" >&2
  exit 1
fi

# Prefer CycloneDDS (same as explore / tb4 nav scripts) to avoid FastDDS SHM locks.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${ROOT_DIR}/config/cyclonedds.xml}"

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

# VMware / no real GPU: software GL so lidar can raycast via Mesa.
is_vmware=0
if lspci 2>/dev/null | grep -qi 'VMware'; then
  is_vmware=1
fi

if [[ "${is_vmware}" -eq 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  HEADLESS="${HEADLESS:-True}"
  USE_COMPOSITION="${USE_COMPOSITION:-False}"
  echo "[tb3_nav_rl_gazebo] Detected VMware guest"
else
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-True}"
fi

if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == "1" ]]; then
  echo "[tb3_nav_rl_gazebo] Using software OpenGL (LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}, MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE:-unset})"
fi

# Map YAML: defaults to built-in ${ROOT_DIR}/maps/tb3_map.yaml.
# Override via: MAP=~/tb3_map.yaml ./scripts/tb3_nav_rl_gazebo.sh
#           or: ./scripts/tb3_nav_rl_gazebo.sh /path/to/map.yaml
if [[ $# -ge 1 && ( "${1}" == *.yaml || "${1}" == *.yml ) ]]; then
  MAP="${1}"
  shift
fi
MAP="${MAP:-${ROOT_DIR}/maps/tb3_map.yaml}"
if [[ ! -f "${MAP}" && -f "${HOME}/tb3_map.yaml" ]]; then
  MAP="${HOME}/tb3_map.yaml"
fi
if [[ ! -f "${MAP}" && -f "${HOME}/桌面/tb3_map.yaml" ]]; then
  MAP="${HOME}/桌面/tb3_map.yaml"
fi

if [[ ! -f "${MAP}" ]]; then
  echo "[tb3_nav_rl_gazebo] Map not found: ${MAP}" >&2
  echo "Save one during SLAM first, e.g.:" >&2
  echo "  ros2 run nav2_map_server map_saver_cli -f ~/tb3_map" >&2
  exit 1
fi

PARAMS_FILE="${PARAMS_FILE:-${ROOT_DIR}/config/nav2_params_rl.yaml}"
# Default to the official waffle SDF so Gazebo lidar frames match robot_state_publisher URDF
# (base_scan). Optional override: ROBOT_SDF=/path/to/custom.xacro
SIM_SHARE_FOR_ROBOT="$(ros2 pkg prefix nav2_minimal_tb3_sim)/share/nav2_minimal_tb3_sim"
ROBOT_SDF="${ROBOT_SDF:-${SIM_SHARE_FOR_ROBOT}/urdf/gz_waffle.sdf.xacro}"
if [[ ! -f "${ROBOT_SDF}" ]]; then
  echo "[tb3_nav_rl_gazebo] ROBOT_SDF not found: ${ROBOT_SDF}" >&2
  exit 1
fi

# World must match the map you saved. Examples:
#   WORLD=empty MAP=~/empty_map.yaml ./scripts/tb3_nav_rl_gazebo.sh
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
  echo "[tb3_nav_rl_gazebo] World not found: ${WORLD_FILE}" >&2
  echo "Built-in: sandbox | empty" >&2
  exit 1
fi

# Gazebo world pose. The saved map is in the SLAM odom frame, which is zero at
# the pose used while mapping (default spawn -2, -0.5), not at the world origin.
# AMCL must be given that map pose. World (0, 0.5) is map (2, 1) on ~/tb3_map.
SLAM_WORLD_X="${SLAM_WORLD_X:--2.0}"
SLAM_WORLD_Y="${SLAM_WORLD_Y:--0.5}"
X_POSE="${X_POSE:-0.00}"
Y_POSE="${Y_POSE:-0.50}"
YAW_POSE="${YAW_POSE:-0.00}"
MAP_X="$(python3 -c "print(f'{float(\"${X_POSE}\") - float(\"${SLAM_WORLD_X}\"):.2f}')")"
MAP_Y="$(python3 -c "print(f'{float(\"${Y_POSE}\") - float(\"${SLAM_WORLD_Y}\"):.2f}')")"

echo "[tb3_nav_rl_gazebo] DISPLAY=${DISPLAY} WAYLAND_DISPLAY=${WAYLAND_DISPLAY} XAUTHORITY=${XAUTHORITY:-unset}"
echo "[tb3_nav_rl_gazebo] world=${WORLD_FILE} map=${MAP}"
echo "[tb3_nav_rl_gazebo] params=${PARAMS_FILE} robot_sdf=${ROBOT_SDF}"
echo "[tb3_nav_rl_gazebo] headless=${HEADLESS} use_composition=${USE_COMPOSITION}"
echo "[tb3_nav_rl_gazebo] Gazebo spawn (${X_POSE}, ${Y_POSE}), map initial pose (${MAP_X}, ${MAP_Y}, yaw ${YAW_POSE})."
echo "[tb3_nav_rl_gazebo] Launching Nav2 with rl_nav2_controller::RlController ..."

# Publish the spawn pose once so AMCL can create map->odom. Do not publish again:
# a later /initialpose would overwrite the user's 2D Pose Estimate.
(
  set +u
  source /opt/ros/lyrical/setup.bash
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
  fi
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI
  fi
  sleep 8
  for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
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
  # Bringup usually finishes on its own. Only retry startup if the controller
  # never became active (a second startup on an active stack logs a false failure).
  state="$(ros2 lifecycle get /controller_server 2>/dev/null || true)"
  if [[ "${state}" != *active* ]]; then
    ros2 service call /lifecycle_manager_navigation/manage_nodes \
      nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" >/dev/null 2>&1 || true
  fi
) &

exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=False \
  map:="${MAP}" \
  world:="${WORLD_FILE}" \
  params_file:="${PARAMS_FILE}" \
  robot_sdf:="${ROBOT_SDF}" \
  headless:="${HEADLESS}" \
  use_composition:="${USE_COMPOSITION}" \
  use_sim_time:=True \
  x_pose:="${X_POSE}" \
  y_pose:="${Y_POSE}" \
  yaw:="${YAW_POSE}" \
  "$@"
