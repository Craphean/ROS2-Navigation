#!/usr/bin/env bash
# Stop leftover Gazebo / Nav2 / RViz / explore processes and clear DDS shm.
# Usage: ./scripts/kill_nav2_sim.sh
#
# Safe to call from tb*_explore_slam.sh: does NOT kill the calling wrapper
# (avoids matching kill_nav2_sim / tb4_explore_slam via broad patterns).
set +e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SELF_PID=$$
PARENT_PID="${PPID:-0}"

echo "[kill_nav2_sim] Stopping Gazebo / Nav2 nodes / RViz ..."

# Kill by matching process args via ps/awk — never embed broad pkill -f
# patterns in this shell's cmdline (that suicides the killer).
kill_matching() {
  local sig="$1"
  shift
  local re
  for re in "$@"; do
    while read -r pid; do
      [[ -z "${pid}" ]] && continue
      [[ "${pid}" == "${SELF_PID}" || "${pid}" == "${PARENT_PID}" ]] && continue
      kill "-${sig}" "${pid}" 2>/dev/null
    done < <(ps -eo pid=,args= | awk -v re="${re}" 'index($0, re) && $1 != '"${SELF_PID}"' {print $1}')
  done
}

kill_matching TERM \
  'tb3_simulation_launch' \
  'tb4_simulation_launch' \
  'gz-sim-main' \
  'gz-sim-gui-client' \
  '/lib/rviz2/rviz2' \
  'sync_slam_toolbox_node' \
  'async_slam_toolbox_node' \
  '/lib/nav2_' \
  '/lib/ros_gz' \
  '/lib/opennav_' \
  'parameter_bridge' \
  'image_bridge' \
  '/lib/robot_state_publisher/robot_state_publisher' \
  '/lib/explore_lite/explore' \
  'rl_cmd_vel_relay.py' \
  'lifecycle_manager_navigation/manage_nodes'
sleep 1
kill_matching KILL \
  'gz-sim-main' \
  'gz-sim-gui-client' \
  'tb3_simulation_launch' \
  'tb4_simulation_launch' \
  '/lib/rviz2/rviz2' \
  'rl_cmd_vel_relay.py' \
  'lifecycle_manager_navigation/manage_nodes'

# Optionally stop *other* wrapper scripts (not ourselves / our parent).
kill_other_wrappers() {
  local pat pid
  for pat in \
      'scripts/tb3_explore_slam.sh' \
      'scripts/tb4_explore_slam.sh' \
      'scripts/tb3_slam_gazebo.sh' \
      'scripts/tb4_slam_gazebo.sh' \
      'scripts/tb3_nav_gazebo.sh' \
      'scripts/tb4_nav_gazebo.sh' \
      'scripts/tb3_nav_rl_gazebo.sh'
  do
    while read -r pid; do
      [[ -z "${pid}" ]] && continue
      [[ "${pid}" == "${SELF_PID}" || "${pid}" == "${PARENT_PID}" ]] && continue
      kill "${pid}" 2>/dev/null
    done < <(pgrep -f "${pat}" 2>/dev/null)
  done
}
kill_other_wrappers
sleep 1
kill_other_wrappers  # SIGTERM first pass; second pass for stragglers
while read -r pid; do
  [[ -z "${pid}" ]] && continue
  [[ "${pid}" == "${SELF_PID}" || "${pid}" == "${PARENT_PID}" ]] && continue
  kill -9 "${pid}" 2>/dev/null
done < <(pgrep -f 'scripts/tb[34]_(explore_slam|slam_gazebo|nav_gazebo|nav_rl_gazebo)\.sh' 2>/dev/null)

echo "[kill_nav2_sim] Clearing DDS shared memory ..."
rm -f /dev/shm/cyclonedds* /dev/shm/fastrtps_* /dev/shm/fastdds* /dev/shm/sem.fastdds* /dev/shm/sem.fastrtps* 2>/dev/null

echo "[kill_nav2_sim] Done. Safe to start a new sim."
exit 0
