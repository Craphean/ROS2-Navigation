#!/usr/bin/env bash
# TurtleBot3 Gazebo + Nav2 SLAM + explore_lite autonomous frontier exploration.
# Based on https://github.com/robo-friends/m-explore-ros2
#
# Usage:
#   ./scripts/tb3_explore_slam.sh
#   WORLD=empty ./scripts/tb3_explore_slam.sh
#
# Pause / resume exploration:
#   ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
#   ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
#
# Save map when done:
#   ros2 run nav2_map_server map_saver_cli -f ~/tb3_explore_map
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
else
  echo "[tb3_explore_slam] Missing ${ROOT_DIR}/install — build first:" >&2
  echo "  cd ${ROOT_DIR} && colcon build --symlink-install --packages-select explore_lite_msgs explore_lite" >&2
  exit 1
fi
set -u

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
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${ROOT_DIR}/config/cyclonedds.xml}"

"${ROOT_DIR}/scripts/kill_nav2_sim.sh" || true
sleep 2

is_vmware=0
if lspci 2>/dev/null | grep -qi 'VMware'; then
  is_vmware=1
fi

if [[ "${is_vmware}" -eq 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  HEADLESS="${HEADLESS:-True}"
  USE_COMPOSITION="${USE_COMPOSITION:-False}"
  echo "[tb3_explore_slam] Detected VMware guest"
else
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-True}"
fi

PARAMS_FILE="${PARAMS_FILE:-${ROOT_DIR}/config/nav2_params_vmware.yaml}"
if [[ ! -f "${PARAMS_FILE}" ]]; then
  PARAMS_FILE="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml"
fi
EXPLORE_PARAMS="${EXPLORE_PARAMS:-${ROOT_DIR}/config/explore_params.yaml}"

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
  echo "[tb3_explore_slam] World not found: ${WORLD_FILE}" >&2
  echo "Built-in: sandbox | empty" >&2
  exit 1
fi

echo "[tb3_explore_slam] RMW=${RMW_IMPLEMENTATION} world=${WORLD_FILE}"
echo "[tb3_explore_slam] nav2 params=${PARAMS_FILE}"
echo "[tb3_explore_slam] explore params=${EXPLORE_PARAMS}"
echo "[tb3_explore_slam] RViz: add MarkerArray topic /explore/frontiers to see frontiers"

SLAM_LOG="${ROOT_DIR}/tb3_explore_slam_bringup.log"
setsid bash -c "
source /opt/ros/lyrical/setup.bash
source '${ROOT_DIR}/install/setup.bash'
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
export CYCLONEDDS_URI='${CYCLONEDDS_URI}'
export LIBGL_ALWAYS_SOFTWARE='${LIBGL_ALWAYS_SOFTWARE:-}'
export MESA_GL_VERSION_OVERRIDE='${MESA_GL_VERSION_OVERRIDE:-}'
export DISPLAY='${DISPLAY}'
export WAYLAND_DISPLAY='${WAYLAND_DISPLAY}'
export XDG_RUNTIME_DIR='${XDG_RUNTIME_DIR}'
export XAUTHORITY='${XAUTHORITY:-}'
export QT_QPA_PLATFORM='${QT_QPA_PLATFORM}'
exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=True \
  headless:='${HEADLESS}' \
  use_composition:='${USE_COMPOSITION}' \
  use_sim_time:=True \
  world:='${WORLD_FILE}' \
  params_file:='${PARAMS_FILE}'
" </dev/null >"${SLAM_LOG}" 2>&1 &
SLAM_PID=$!
echo "[tb3_explore_slam] Nav2+SLAM PID=${SLAM_PID} log=${SLAM_LOG}"

cleanup() {
  echo "[tb3_explore_slam] Shutting down..."
  kill "${SLAM_PID}" 2>/dev/null || true
  sleep 1
  pkill -f 'tb3_simulation_launch' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[tb3_explore_slam] Waiting for /map, slam_toolbox, and navigation ..."
for i in $(seq 1 180); do
  if ! kill -0 "${SLAM_PID}" 2>/dev/null; then
    echo "[tb3_explore_slam] Nav2 bringup exited early — see ${SLAM_LOG}" >&2
    exit 1
  fi
  has_map=0
  has_nav=0
  has_slam=0
  if ros2 topic list 2>/dev/null | grep -qx '/map'; then has_map=1; fi
  if ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose'; then has_nav=1; fi
  if ros2 node list 2>/dev/null | grep -qx '/slam_toolbox'; then has_slam=1; fi
  if [[ "${has_map}" -eq 1 && "${has_nav}" -eq 1 && "${has_slam}" -eq 1 ]]; then
    if ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null \
        | grep -q 'success=True'; then
      # Extra settle so costmaps/TF are warm before first explore goal
      if [[ "${i}" -ge 15 ]]; then
        echo "[tb3_explore_slam] Ready after ${i}s"
        break
      fi
    fi
    st=$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)
    if [[ "${st}" == *inactive* ]]; then
      echo "[tb3_explore_slam] activating /bt_navigator ..."
      ros2 lifecycle set /bt_navigator activate >/dev/null 2>&1 || true
    fi
  fi
  if [[ "${i}" -eq 180 ]]; then
    echo "[tb3_explore_slam] Timed out waiting for Nav2/SLAM — see ${SLAM_LOG}" >&2
    exit 1
  fi
  sleep 1
done

echo "[tb3_explore_slam] Starting explore_lite (autonomous frontiers) ..."
# ros2 run so our params file is used (upstream launch hardcodes package params.yaml)
ros2 run explore_lite explore --ros-args \
  --params-file "${EXPLORE_PARAMS}" \
  -p use_sim_time:=true \
  -r /tf:=tf \
  -r /tf_static:=tf_static \
  "$@"
