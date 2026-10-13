#!/usr/bin/env bash
# Dual-TurtleBot3 Gazebo + Nav2 + slam_toolbox + explore_lite + multirobot_map_merge.
# Launches two namespaced TB3 robots (/robot1 and /robot2), runs real-time SLAM and
# autonomous frontier exploration on each robot, and merges both maps into /map.
#
# Defaults:
#   HEADLESS=True          (set HEADLESS=False to open Gazebo GUI)
#   USE_RVIZ=False         (set USE_RVIZ=True to open RViz with map_merge.rviz)
#   KNOWN_INIT_POSES=true  (set KNOWN_INIT_POSES=false for feature-matching mode)
#
# Usage:
#   ./scripts/tb3_multi_explore_slam.sh
#   USE_RVIZ=True ./scripts/tb3_multi_explore_slam.sh
#   HEADLESS=False USE_RVIZ=True ./scripts/tb3_multi_explore_slam.sh
#
# Save merged map when done:
#   ros2 run nav2_map_server map_saver_cli -f ~/tb3_merged_map --ros-args -r map:=/map
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
else
  echo "[tb3_multi_explore_slam] Missing ${ROOT_DIR}/install — build first:" >&2
  echo "  cd ${ROOT_DIR} && colcon build --symlink-install --packages-select explore_lite_msgs explore_lite multirobot_map_merge" >&2
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
  echo "[tb3_multi_explore_slam] Detected VMware guest (software OpenGL enabled)"
fi

# Default headless=True as requested
HEADLESS="${HEADLESS:-True}"
USE_COMPOSITION="${USE_COMPOSITION:-False}"
USE_RVIZ="${USE_RVIZ:-False}"
KNOWN_INIT_POSES="${KNOWN_INIT_POSES:-true}"

SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb3_sim)/share/nav2_minimal_tb3_sim"
TB4_SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb4_sim 2>/dev/null)/share/nav2_minimal_tb4_sim" || true

case "${WORLD:-sandbox}" in
  sandbox|tb3_sandbox)
    WORLD_FILE="${SIM_SHARE}/worlds/tb3_sandbox.sdf.xacro"
    DEFAULT_R1_X="-2.0"; DEFAULT_R1_Y="-0.5"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.0";  DEFAULT_R2_Y="0.5";  DEFAULT_R2_YAW="0.0"
    ;;
  empty|tb3_empty|tb3_empty_world)
    WORLD_FILE="${SIM_SHARE}/worlds/tb3_empty_world.sdf.xacro"
    DEFAULT_R1_X="-2.0"; DEFAULT_R1_Y="-0.5"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.0";  DEFAULT_R2_Y="0.5";  DEFAULT_R2_YAW="0.0"
    ;;
  warehouse|tb4_warehouse)
    WORLD_FILE="${TB4_SIM_SHARE}/worlds/warehouse.sdf"
    DEFAULT_R1_X="2.00"; DEFAULT_R1_Y="-19.65"; DEFAULT_R1_YAW="1.57"
    DEFAULT_R2_X="0.50"; DEFAULT_R2_Y="-19.65"; DEFAULT_R2_YAW="1.57"
    ;;
  depot|tb4_depot)
    WORLD_FILE="${TB4_SIM_SHARE}/worlds/depot.sdf"
    DEFAULT_R1_X="0.00"; DEFAULT_R1_Y="0.00"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.55"; DEFAULT_R2_Y="2.55"; DEFAULT_R2_YAW="0.0"
    ;;
  *)
    WORLD_FILE="${WORLD}"
    DEFAULT_R1_X="-2.0"; DEFAULT_R1_Y="-0.5"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.0";  DEFAULT_R2_Y="0.5";  DEFAULT_R2_YAW="0.0"
    ;;
esac

if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[tb3_multi_explore_slam] World not found: ${WORLD_FILE}" >&2
  echo "Supported presets: sandbox | empty | warehouse | depot (or full path to .sdf/.sdf.xacro)" >&2
  exit 1
fi

# Spawn poses in Gazebo world (meters / radians) — also passed to map_merge init_pose_*
ROBOT1_X="${ROBOT1_X:-${DEFAULT_R1_X}}"
ROBOT1_Y="${ROBOT1_Y:-${DEFAULT_R1_Y}}"
ROBOT1_YAW="${ROBOT1_YAW:-${DEFAULT_R1_YAW}}"

ROBOT2_X="${ROBOT2_X:-${DEFAULT_R2_X}}"
ROBOT2_Y="${ROBOT2_Y:-${DEFAULT_R2_Y}}"
ROBOT2_YAW="${ROBOT2_YAW:-${DEFAULT_R2_YAW}}"

PARAMS_FILE="${PARAMS_FILE:-${ROOT_DIR}/config/nav2_multirobot_slam_params.yaml}"
EXPLORE_PARAMS="${EXPLORE_PARAMS:-${ROOT_DIR}/config/explore_params.yaml}"
MAP_MERGE_PARAMS="${MAP_MERGE_PARAMS:-${ROOT_DIR}/src/m-explore-ros2/map_merge/config/params.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/m-explore-ros2/map_merge/launch/map_merge.rviz}"

echo "[tb3_multi_explore_slam] world=${WORLD_FILE} headless=${HEADLESS} use_rviz=${USE_RVIZ} known_init_poses=${KNOWN_INIT_POSES}"
echo "[tb3_multi_explore_slam] robot1 spawn=(${ROBOT1_X}, ${ROBOT1_Y}, ${ROBOT1_YAW}) | robot2 spawn=(${ROBOT2_X}, ${ROBOT2_Y}, ${ROBOT2_YAW})"

ROBOT1_LOG="/tmp/tb3_multi_robot1_bringup.log"
ROBOT2_LOG="/tmp/tb3_multi_robot2_bringup.log"
MERGE_LOG="/tmp/tb3_multi_map_merge.log"
EXPLORE1_LOG="/tmp/tb3_multi_explore1.log"
EXPLORE2_LOG="/tmp/tb3_multi_explore2.log"

# 1. Launch Gazebo server + robot1 (Nav2 + slam_toolbox in /robot1)
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
  namespace:=robot1 \
  robot_name:=robot1 \
  slam:=True \
  use_keepout_zones:=False \
  use_speed_zones:=False \
  use_simulator:=True \
  use_rviz:=False \
  headless:='${HEADLESS}' \
  use_composition:='${USE_COMPOSITION}' \
  use_respawn:=True \
  use_sim_time:=True \
  world:='${WORLD_FILE}' \
  params_file:='${PARAMS_FILE}' \
  x_pose:='${ROBOT1_X}' \
  y_pose:='${ROBOT1_Y}' \
  yaw:='${ROBOT1_YAW}'
" </dev/null >"${ROBOT1_LOG}" 2>&1 &
R1_PID=$!
echo "[tb3_multi_explore_slam] Launched Gazebo + robot1 (PID=${R1_PID}, log=${ROBOT1_LOG})"

sleep 4

# 2. Spawn robot2 into same Gazebo server + launch robot2 (Nav2 + slam_toolbox in /robot2)
# Uses tb3_bridge_no_clock.yaml so /clock is only bridged once (by robot1),
# preventing duplicate /clock jumps that clear tf2 buffers.
BRIDGE_NO_CLOCK="${ROOT_DIR}/config/tb3_bridge_no_clock.yaml"
URDF_FILE="${SIM_SHARE}/urdf/turtlebot3_waffle.urdf"
SDF_XACRO="${SIM_SHARE}/urdf/gz_waffle.sdf.xacro"

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

export GZ_SIM_RESOURCE_PATH=\"\${GZ_SIM_RESOURCE_PATH:+\${GZ_SIM_RESOURCE_PATH}:}${SIM_SHARE}/models:\$(dirname '${SIM_SHARE}')\"

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -r __ns:=/robot2 \
  -p use_sim_time:=true \
  -p \"robot_description:=\$(cat '${URDF_FILE}')\" \
  -r /tf:=tf -r /tf_static:=tf_static &

ros2 run ros_gz_sim create \
  -name robot2 \
  -string \"\$(xacro namespace:=robot2 '${SDF_XACRO}')\" \
  -x '${ROBOT2_X}' -y '${ROBOT2_Y}' -z 0.05 \
  -R 0.0 -P 0.0 -Y '${ROBOT2_YAW}' \
  --ros-args -r __ns:=/robot2 &

ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -r __ns:=/robot2 \
  -p config_file:='${BRIDGE_NO_CLOCK}' \
  -p use_sim_time:=true \
  -p expand_gz_topic_names:=true &

sleep 2

exec ros2 launch nav2_bringup bringup_launch.py \
  namespace:=robot2 \
  use_namespace:=True \
  slam:=True \
  use_keepout_zones:=False \
  use_speed_zones:=False \
  use_sim_time:=True \
  params_file:='${PARAMS_FILE}' \
  autostart:=True \
  use_composition:='${USE_COMPOSITION}' \
  use_respawn:=True
" </dev/null >"${ROBOT2_LOG}" 2>&1 &
R2_PID=$!
echo "[tb3_multi_explore_slam] Launched robot2 (PID=${R2_PID}, log=${ROBOT2_LOG})"

MERGE_PID=""
EXP1_PID=""
EXP2_PID=""
RVIZ_PID=""

cleanup() {
  echo "[tb3_multi_explore_slam] Shutting down dual-robot exploration & map merge..."
  for pid in "${EXP1_PID}" "${EXP2_PID}" "${MERGE_PID}" "${RVIZ_PID}" "${R2_PID}" "${R1_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
  "${ROOT_DIR}/scripts/kill_nav2_sim.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[tb3_multi_explore_slam] Waiting for /robot1 and /robot2 SLAM & Nav2 stacks to become active..."
for i in $(seq 1 240); do
  if ! kill -0 "${R1_PID}" 2>/dev/null; then
    echo "[tb3_multi_explore_slam] robot1 bringup exited early — see ${ROBOT1_LOG}" >&2
    exit 1
  fi
  if ! kill -0 "${R2_PID}" 2>/dev/null; then
    echo "[tb3_multi_explore_slam] robot2 bringup exited early — see ${ROBOT2_LOG}" >&2
    exit 1
  fi

  r1_ok=0
  r2_ok=0
  if grep -q "lifecycle_manager_navigation.*Managed nodes are active" "${ROBOT1_LOG}" 2>/dev/null && \
     grep -q "StaticLayer: Resizing costmap" "${ROBOT1_LOG}" 2>/dev/null; then
    r1_ok=1
  fi

  if grep -q "lifecycle_manager_navigation.*Managed nodes are active" "${ROBOT2_LOG}" 2>/dev/null && \
     grep -q "StaticLayer: Resizing costmap" "${ROBOT2_LOG}" 2>/dev/null; then
    r2_ok=1
  fi

  if [[ "${r1_ok}" -eq 1 && "${r2_ok}" -eq 1 ]]; then
    echo "[tb3_multi_explore_slam] Both robot1 and robot2 are active after ${i}s!"
    break
  fi

  if [[ "${i}" -eq 240 ]]; then
    echo "[tb3_multi_explore_slam] Timed out waiting for dual-robot Nav2/SLAM — check ${ROBOT1_LOG} and ${ROBOT2_LOG}" >&2
    exit 1
  fi
  sleep 1
done

# 3. Start multirobot_map_merge node
echo "[tb3_multi_explore_slam] Starting multirobot_map_merge -> publishing merged map on /map ..."
ros2 run multirobot_map_merge map_merge --ros-args \
  --params-file "${MAP_MERGE_PARAMS}" \
  -p use_sim_time:=true \
  -p known_init_poses:="${KNOWN_INIT_POSES}" \
  -p merging_rate:=4.0 \
  -p discovery_rate:=1.0 \
  -p world_frame:=world \
  -p merged_map_topic:=map \
  -p robot_map_topic:=map \
  -p robot_map_updates_topic:=map_updates \
  -p robot_namespace:=robot \
  -p /robot1/map_merge/init_pose_x:="${ROBOT1_X}" \
  -p /robot1/map_merge/init_pose_y:="${ROBOT1_Y}" \
  -p /robot1/map_merge/init_pose_z:=0.0 \
  -p /robot1/map_merge/init_pose_yaw:="${ROBOT1_YAW}" \
  -p /robot2/map_merge/init_pose_x:="${ROBOT2_X}" \
  -p /robot2/map_merge/init_pose_y:="${ROBOT2_Y}" \
  -p /robot2/map_merge/init_pose_z:=0.0 \
  -p /robot2/map_merge/init_pose_yaw:="${ROBOT2_YAW}" \
  >"${MERGE_LOG}" 2>&1 &
MERGE_PID=$!
echo "[tb3_multi_explore_slam] map_merge PID=${MERGE_PID} log=${MERGE_LOG}"

if [[ "${USE_RVIZ,,}" == "true" ]]; then
  echo "[tb3_multi_explore_slam] Launching RViz with ${RVIZ_CONFIG} ..."
  rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
  RVIZ_PID=$!
fi

# 4. Start explore_lite for robot1 and robot2
echo "[tb3_multi_explore_slam] Starting explore_lite for /robot1 and /robot2 ..."
ros2 run explore_lite explore --ros-args \
  -r __ns:=/robot1 \
  --params-file "${EXPLORE_PARAMS}" \
  -p use_sim_time:=true \
  -r /tf:=tf \
  -r /tf_static:=tf_static \
  >"${EXPLORE1_LOG}" 2>&1 &
EXP1_PID=$!

ros2 run explore_lite explore --ros-args \
  -r __ns:=/robot2 \
  --params-file "${EXPLORE_PARAMS}" \
  -p use_sim_time:=true \
  -r /tf:=tf \
  -r /tf_static:=tf_static \
  >"${EXPLORE2_LOG}" 2>&1 &
EXP2_PID=$!

echo "[tb3_multi_explore_slam] Dual exploration running!"
echo "  - robot1 explore log: ${EXPLORE1_LOG}"
echo "  - robot2 explore log: ${EXPLORE2_LOG}"
echo "  - map_merge log:      ${MERGE_LOG}"
echo "  - Merged map topic:   /map (frame_id: world)"
echo "Press Ctrl-C to stop."

wait "${MERGE_PID}"

