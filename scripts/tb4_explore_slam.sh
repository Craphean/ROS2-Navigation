#!/usr/bin/env bash
# TurtleBot4 Gazebo + Nav2 SLAM + explore_lite autonomous frontier exploration.
# Based on https://github.com/robo-friends/m-explore-ros2
#
# Usage:
#   ./scripts/tb4_explore_slam.sh              # warehouse (default, large)
#   WORLD=depot ./scripts/tb4_explore_slam.sh
#
# Pause / resume exploration:
#   ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
#   ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
#
# Save map when done:
#   ros2 run nav2_map_server map_saver_cli -f ~/tb4_explore_map
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
else
  echo "[tb4_explore_slam] Missing ${ROOT_DIR}/install — build first:" >&2
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

# Always clear previous sim — leftover TB3/TB4 + DDS shm causes:
# "Failed to find a free participant index" and RViz showing the old map.
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
  echo "[tb4_explore_slam] Detected VMware guest"
else
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-True}"
fi
# Start RViz as a separate process (default on). Never via the Nav2 launch —
# an RViz GL abort would otherwise tear down Gazebo/Nav2.
USE_RVIZ="${USE_RVIZ:-True}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz}"
RVIZ_PID=""

PARAMS_FILE="${PARAMS_FILE:-${ROOT_DIR}/config/nav2_params_vmware.yaml}"
if [[ ! -f "${PARAMS_FILE}" ]]; then
  PARAMS_FILE="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml"
fi
EXPLORE_PARAMS="${EXPLORE_PARAMS:-${ROOT_DIR}/config/explore_params.yaml}"

SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim"
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
    echo "[tb4_explore_slam] Unknown WORLD=${WORLD_NAME} (use warehouse|depot)" >&2
    exit 1
    ;;
esac

if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[tb4_explore_slam] World not found: ${WORLD_FILE}" >&2
  exit 1
fi

# Warehouse is large; allow longer settle so SLAM has a usable map before explore.
MIN_READY_SEC="${MIN_READY_SEC:-40}"
echo "[tb4_explore_slam] RMW=${RMW_IMPLEMENTATION} world=${WORLD_FILE}"
echo "[tb4_explore_slam] spawn=(${X_POSE},${Y_POSE},yaw=${YAW}) headless=${HEADLESS} rviz=${USE_RVIZ}"
echo "[tb4_explore_slam] nav2 params=${PARAMS_FILE}"
echo "[tb4_explore_slam] explore params=${EXPLORE_PARAMS}"
echo "[tb4_explore_slam] First run may download Fuel models; needs network."
if [[ "${USE_RVIZ}" == "True" || "${USE_RVIZ}" == "true" ]]; then
  echo "[tb4_explore_slam] RViz: separate process (MarkerArray /explore/frontiers for frontiers)"
else
  echo "[tb4_explore_slam] RViz disabled (set USE_RVIZ=True to enable)"
fi

SLAM_LOG="${ROOT_DIR}/tb4_explore_slam_bringup.log"
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
exec ros2 launch nav2_bringup tb4_simulation_launch.py \
  slam:=True \
  headless:='${HEADLESS}' \
  use_rviz:=False \
  use_composition:='${USE_COMPOSITION}' \
  use_sim_time:=True \
  use_keepout_zones:=False \
  use_speed_zones:=False \
  world:='${WORLD_FILE}' \
  params_file:='${PARAMS_FILE}' \
  x_pose:='${X_POSE}' \
  y_pose:='${Y_POSE}' \
  yaw:='${YAW}'
" </dev/null >"${SLAM_LOG}" 2>&1 &
SLAM_PID=$!
echo "[tb4_explore_slam] Nav2+SLAM PID=${SLAM_PID} log=${SLAM_LOG}"

cleanup() {
  echo "[tb4_explore_slam] Shutting down..."
  if [[ -n "${RVIZ_PID}" ]]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
  kill "${SLAM_PID}" 2>/dev/null || true
  sleep 1
  # Avoid pkill -f patterns that match this shell's cmdline.
  while read -r pid; do
    [[ -z "${pid}" || "${pid}" == "$$" ]] && continue
    kill "${pid}" 2>/dev/null || true
  done < <(ps -eo pid=,args= | awk '/tb4_simulation_launch/ && !/awk/ {print $1}')
}
trap cleanup EXIT INT TERM

echo "[tb4_explore_slam] Waiting for /map, slam_toolbox, and navigation ..."
# Wall-clock ready gate — avoid 40x slow ros2 CLI iterations.
start_ts=$(date +%s)
has_map=0; has_nav=0; has_slam=0; nav_active=0; map_w=""
while true; do
  now_ts=$(date +%s)
  elapsed=$((now_ts - start_ts))
  if ! kill -0 "${SLAM_PID}" 2>/dev/null; then
    echo "[tb4_explore_slam] Nav2 bringup exited early — see ${SLAM_LOG}" >&2
    exit 1
  fi
  if [[ "${has_map}" -eq 0 ]]; then
    if timeout 4 ros2 topic list 2>/dev/null | grep -qx '/map'; then has_map=1; fi
  fi
  if [[ "${has_nav}" -eq 0 ]]; then
    if timeout 4 ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose'; then has_nav=1; fi
  fi
  if [[ "${has_slam}" -eq 0 ]]; then
    if timeout 4 ros2 node list 2>/dev/null | grep -qx '/slam_toolbox'; then has_slam=1; fi
  fi
  if [[ "${has_map}" -eq 1 && "${has_nav}" -eq 1 && "${has_slam}" -eq 1 && "${nav_active}" -eq 0 ]]; then
    if timeout 6 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null \
        | grep -q 'success=True'; then
      nav_active=1
    fi
  fi
  if [[ "${nav_active}" -eq 1 && -z "${map_w}" ]]; then
    # echo may print YAML '---' separators; keep digits only.
    map_w=$(timeout 6 ros2 topic echo /map --field info.width --once 2>/dev/null \
      | grep -oE '[0-9]+' | head -1 || true)
  fi
  if [[ "${nav_active}" -eq 1 && "${elapsed}" -ge "${MIN_READY_SEC}" ]]; then
    if [[ -n "${map_w}" && "${map_w}" -ge 40 ]]; then
      echo "[tb4_explore_slam] Ready after ${elapsed}s (map width=${map_w})"
      break
    fi
    if [[ "${elapsed}" -ge $((MIN_READY_SEC + 30)) ]]; then
      echo "[tb4_explore_slam] Ready after ${elapsed}s (map width=${map_w:-unknown}, continuing)"
      break
    fi
  fi
  if [[ "${elapsed}" -ge 300 ]]; then
    echo "[tb4_explore_slam] Timed out waiting for Nav2/SLAM — see ${SLAM_LOG}" >&2
    exit 1
  fi
  sleep 2
done

# RViz outside the launch so a GL crash cannot kill Nav2/Gazebo.
if [[ "${USE_RVIZ}" == "True" || "${USE_RVIZ}" == "true" ]]; then
  echo "[tb4_explore_slam] Starting RViz (soft-GL OK if it flickers) ..."
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
exec ros2 run rviz2 rviz2 -d '${RVIZ_CONFIG}' --ros-args -p use_sim_time:=true
" </dev/null >"${ROOT_DIR}/tb4_explore_rviz.log" 2>&1 &
  RVIZ_PID=$!
  echo "[tb4_explore_slam] RViz PID=${RVIZ_PID} log=${ROOT_DIR}/tb4_explore_rviz.log"
fi

echo "[tb4_explore_slam] Spinning in place once to grow the SLAM map ..."
# Lyrical CLI: ros2 action send_goal [-t N] <name> <type> <goal>
# (no --spin-time / --ros-args). Prefer cmd_vel rotate: Spin often stalls
# under VMware soft-GL while the bridge still accepts TwistStamped.
spin_ok=0
if timeout 20 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: 'base_link'}, twist: {angular: {z: 0.55}}}" \
    >/dev/null 2>&1; then
  spin_ok=1
elif timeout 20 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{angular: {z: 0.55}}" >/dev/null 2>&1; then
  spin_ok=1
elif ros2 action send_goal -t 120 /spin nav2_msgs/action/Spin \
    "{target_yaw: 6.2831853, time_allowance: {sec: 90, nanosec: 0}}"; then
  spin_ok=1
fi
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {}}" >/dev/null 2>&1 || true
if [[ "${spin_ok}" -eq 1 ]]; then
  echo "[tb4_explore_slam] spin done"
  sleep 3
else
  echo "[tb4_explore_slam] spin skipped/failed — continuing"
fi

echo "[tb4_explore_slam] Starting explore_lite (autonomous frontiers) ..."
ros2 run explore_lite explore --ros-args \
  --params-file "${EXPLORE_PARAMS}" \
  -p use_sim_time:=true \
  -r /tf:=tf \
  -r /tf_static:=tf_static \
  "$@"
