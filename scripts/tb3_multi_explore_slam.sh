#!/usr/bin/env bash
# Dual-TurtleBot3 Gazebo + Nav2 + slam_toolbox + explore_lite + multirobot_map_merge.
# Launches two namespaced TB3 robots (/robot1 and /robot2), runs real-time SLAM and
# autonomous frontier exploration on each robot, and merges both maps into /map.
#
# Defaults:
#   HEADLESS=True          (set HEADLESS=False to open Gazebo GUI)
#   USE_RVIZ=False         (set USE_RVIZ=True to open RViz with map_merge.rviz)
#   KNOWN_INIT_POSES=true  (set KNOWN_INIT_POSES=false for feature-matching mode)
#   RANDOM_SPAWN=false     (set RANDOM_SPAWN=true to randomly pick 2 non-overlapping inspection stations)
#
# 3D point-cloud mapping (headless by default):
#   - Each robot gets a 16-line / 3 Hz 3D LiDAR from config/gz_waffle_3d_lidar.sdf.xacro.
#   - /robotX/points feeds Nav2 costmaps and scripts/merge_pointclouds.py.
#   - The merged cloud is published as /points_merged in the world frame and fed to
#     octomap_server (3D map + projected_map + octomap_point_cloud_centers).
#   - Archive after exploration:
#       ros2 run octomap_server octomap_saver_node --ros-args \
#         -p octomap_path:="$PWD/maps/tb3_3d_map.bt" -p full:=true
#       python3 scripts/save_cloud_pcd.py \
#         --topic /octomap_point_cloud_centers --output maps/tb3_3d_map.pcd
#       ros2 run nav2_map_server map_saver_cli -f maps/tb3_3d_projected \
#         --ros-args -r map:=/projected_map -p map_subscribe_transient_local:=false
#
# Usage:
#   ./scripts/tb3_multi_explore_slam.sh
#   USE_RVIZ=True ./scripts/tb3_multi_explore_slam.sh
#   HEADLESS=False USE_RVIZ=True ./scripts/tb3_multi_explore_slam.sh
#   RANDOM_SPAWN=true ./scripts/tb3_multi_explore_slam.sh
#   WORLD=warehouse RANDOM_SPAWN=true USE_RVIZ=True ./scripts/tb3_multi_explore_slam.sh
#   ./scripts/tb3_multi_explore_slam.sh --random-spawn
#
# Save merged 2D map when done:
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
EXPLORATION_MODE="${EXPLORATION_MODE:-auction}"
RANDOM_SPAWN="${RANDOM_SPAWN:-false}"

# Parse optional command-line arguments
for arg in "$@"; do
  case "${arg}" in
    --random-spawn|--random|random_spawn:=true|random_spawn:=True)
      RANDOM_SPAWN="true"
      ;;
    --fixed-spawn|--fixed|random_spawn:=false|random_spawn:=False)
      RANDOM_SPAWN="false"
      ;;
    WORLD=*|world:=*)
      WORLD="${arg#*=}"
      ;;
  esac
done

# If random spawn is requested and WORLD is not explicitly specified, target warehouse
if [[ "${RANDOM_SPAWN,,}" =~ ^(true|1|yes)$ && -z "${WORLD:-}" ]]; then
  WORLD="warehouse"
fi

SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb3_sim)/share/nav2_minimal_tb3_sim"
TB4_SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb4_sim 2>/dev/null)/share/nav2_minimal_tb4_sim" || true

# =============================================================================
# Warehouse 10 Curated Inspection Base Stations (合法的巡检基站候选池)
# All 10 stations are vetted in open corridors with >= 1.0m obstacle clearance,
# guaranteed 100% free of walls, shelves, barriers, and dynamic models.
# =============================================================================
WAREHOUSE_STATION_NAMES=(
  "STATION_SOUTH_DOCK_1"        # [0] 南区装卸站 1 (默认起点 R1)
  "STATION_SOUTH_DOCK_2"        # [1] 南区装卸站 2 (默认起点 R2)
  "STATION_SOUTHWEST_AISLE"     # [2] 西南主通道路口巡检站
  "STATION_SOUTHEAST_AISLE"     # [3] 东南主通道路口巡检站
  "STATION_SOUTHWEST_RACK"      # [4] 西南仓储货架中继站
  "STATION_SOUTHEAST_RACK"      # [5] 东南立体货架巡检站
  "STATION_CENTRAL_CROSS_WEST"  # [6] 中区横向主干道西枢纽站
  "STATION_CENTRAL_CROSS_MID"   # [7] 中区十字交叉核心枢纽站
  "STATION_CENTRAL_CROSS_EAST"  # [8] 中区横向主干道东枢纽站
  "STATION_NORTHWEST_BAY"       # [9] 西北高位货架巡检基站
)

WAREHOUSE_STATION_POSES=(
  "2.00 -19.65 1.57 南区装卸站1(默认起点R1)"
  "0.50 -19.65 1.57 南区装卸站2(默认起点R2)"
  "-11.50 -19.65 1.57 西南主通道路口巡检站"
  "8.75 -19.65 1.57 东南主通道路口巡检站"
  "-6.25 -10.75 1.57 西南仓储货架中继站"
  "8.75 -10.75 1.57 东南立体货架巡检站"
  "-6.25 6.00 0.00 中区横向主干道西枢纽站"
  "1.60 6.00 0.00 中区十字交叉核心枢纽站"
  "8.75 6.00 0.00 中区横向主干道东枢纽站"
  "-9.00 13.50 1.57 西北高位货架巡检基站"
)

# Hospital 8 Curated Inspection Base Stations (医院病房与走廊巡检基站池)
HOSPITAL_STATION_NAMES=(
  "STATION_WEST_CORRIDOR_SOUTH" # [0] 西主走廊南侧站 (默认起点 R1)
  "STATION_WEST_CORRIDOR_NORTH" # [1] 西主走廊北侧站 (默认起点 R2)
  "STATION_CENTRAL_CORRIDOR"    # [2] 中区中央走廊十字枢纽站
  "STATION_NORTHWEST_WARD_A"    # [3] 西北双人病房A巡检基站
  "STATION_NORTHEAST_ICU"       # [4] 东北重症监护ICU巡检基站
  "STATION_SOUTHWEST_TRIAGE"    # [5] 西南门诊分诊大厅基站
  "STATION_SOUTHEAST_DOCTOR"    # [6] 东南医生诊断与药房基站
  "STATION_EAST_CORRIDOR_END"   # [7] 东主走廊巡检折返站
)

HOSPITAL_STATION_POSES=(
  "-6.00 -0.60 0.00 西主走廊南侧站(默认起点R1)"
  "-6.00 0.60 0.00 西主走廊北侧站(默认起点R2)"
  "0.00 0.00 0.00 中区中央走廊十字枢纽站"
  "-3.50 2.50 1.57 西北双人病房A巡检基站"
  "3.50 2.50 1.57 东北重症监护ICU巡检基站"
  "-4.50 -4.20 -1.57 西南门诊分诊大厅基站"
  "3.50 -4.50 -1.57 东南医生诊断与药房基站"
  "6.00 0.00 3.14 东主走廊巡检折返站"
)

# House 6 Curated Inspection Base Stations (家庭室内生活环境巡检基站池)
HOUSE_STATION_NAMES=(
  "STATION_HALLWAY_SOUTH"       # [0] 玄关走廊南基站 (默认起点 R1)
  "STATION_HALLWAY_NORTH"       # [1] 玄关走廊北基站 (默认起点 R2)
  "STATION_LIVING_ROOM"         # [2] 客厅开阔活动区
  "STATION_MASTER_BEDROOM"      # [3] 主卧室入口巡检区
  "STATION_STUDY_ROOM"          # [4] 独立书房工作区
  "STATION_DINING_ROOM"         # [5] 餐厅就餐过渡区
)

HOUSE_STATION_POSES=(
  "0.00 -0.50 1.57 玄关走廊南基站(默认起点R1)"
  "0.00 0.50 0.00 玄关走廊北基站(默认起点R2)"
  "-2.00 1.20 1.57 客厅开阔活动区"
  "2.00 1.20 0.00 主卧室入口巡检区"
  "2.00 -1.50 -1.57 独立书房工作区"
  "-2.00 -1.50 -1.57 餐厅就餐过渡区"
)

case "${WORLD:-sandbox}" in
  sandbox|tb3_sandbox)
    WORLD_FILE="${SIM_SHARE}/worlds/tb3_sandbox.sdf.xacro"
    DEFAULT_R1_X="-2.0"; DEFAULT_R1_Y="-0.5"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.0";  DEFAULT_R2_Y="0.5";  DEFAULT_R2_YAW="0.0"
    if [[ "${RANDOM_SPAWN,,}" =~ ^(true|1|yes)$ ]]; then
      echo "[tb3_multi_explore_slam] Note: Random inspection stations are calibrated for warehouse/hospital/house. Using default poses for sandbox."
    fi
    ;;
  empty|tb3_empty|tb3_empty_world)
    WORLD_FILE="${SIM_SHARE}/worlds/tb3_empty_world.sdf.xacro"
    DEFAULT_R1_X="-2.0"; DEFAULT_R1_Y="-0.5"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.0";  DEFAULT_R2_Y="0.5";  DEFAULT_R2_YAW="0.0"
    ;;
  hospital|tb3_hospital|clinic)
    WORLD_FILE="${ROOT_DIR}/worlds/hospital.sdf.xacro"
    DEFAULT_R1_X="-6.00"; DEFAULT_R1_Y="-0.60"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="-6.00"; DEFAULT_R2_Y="0.60";  DEFAULT_R2_YAW="0.0"

    if [[ "${RANDOM_SPAWN,,}" =~ ^(true|1|yes)$ ]]; then
      if command -v shuf >/dev/null 2>&1; then
        mapfile -t indices < <(shuf -i 0-7 -n 2)
        idx1="${indices[0]}"; idx2="${indices[1]}"
      else
        idx1=$(( RANDOM % 8 ))
        idx2=$(( (idx1 + 1 + (RANDOM % 7)) % 8 ))
      fi
      read -r r1_x r1_y r1_yaw r1_desc <<< "${HOSPITAL_STATION_POSES[idx1]}"
      read -r r2_x r2_y r2_yaw r2_desc <<< "${HOSPITAL_STATION_POSES[idx2]}"
      DEFAULT_R1_X="${r1_x}"; DEFAULT_R1_Y="${r1_y}"; DEFAULT_R1_YAW="${r1_yaw}"
      DEFAULT_R2_X="${r2_x}"; DEFAULT_R2_Y="${r2_y}"; DEFAULT_R2_YAW="${r2_yaw}"
      sep_dist=$(python3 -c "import math; print(f'{math.hypot(float(\"${r1_x}\") - float(\"${r2_x}\"), float(\"${r1_y}\") - float(\"${r2_y}\")):.2f}')" 2>/dev/null || echo "N/A")
      echo "[tb3_multi_explore_slam] [Random Spawn] Hospital Base Stations Activated:"
      echo "[tb3_multi_explore_slam]   Robot1 -> Station #${idx1} [${HOSPITAL_STATION_NAMES[idx1]}]: (${DEFAULT_R1_X}, ${DEFAULT_R1_Y}) [${r1_desc}]"
      echo "[tb3_multi_explore_slam]   Robot2 -> Station #${idx2} [${HOSPITAL_STATION_NAMES[idx2]}]: (${DEFAULT_R2_X}, ${DEFAULT_R2_Y}) [${r2_desc}]"
      echo "[tb3_multi_explore_slam]   Spawn Distance: ${sep_dist}m"
    fi
    ;;
  house|tb3_house|home)
    WORLD_FILE="${ROOT_DIR}/worlds/house.sdf.xacro"
    DEFAULT_R1_X="0.00"; DEFAULT_R1_Y="-0.50"; DEFAULT_R1_YAW="1.57"
    DEFAULT_R2_X="0.00"; DEFAULT_R2_Y="0.50";  DEFAULT_R2_YAW="0.0"

    if [[ "${RANDOM_SPAWN,,}" =~ ^(true|1|yes)$ ]]; then
      if command -v shuf >/dev/null 2>&1; then
        mapfile -t indices < <(shuf -i 0-5 -n 2)
        idx1="${indices[0]}"; idx2="${indices[1]}"
      else
        idx1=$(( RANDOM % 6 ))
        idx2=$(( (idx1 + 1 + (RANDOM % 5)) % 6 ))
      fi
      read -r r1_x r1_y r1_yaw r1_desc <<< "${HOUSE_STATION_POSES[idx1]}"
      read -r r2_x r2_y r2_yaw r2_desc <<< "${HOUSE_STATION_POSES[idx2]}"
      DEFAULT_R1_X="${r1_x}"; DEFAULT_R1_Y="${r1_y}"; DEFAULT_R1_YAW="${r1_yaw}"
      DEFAULT_R2_X="${r2_x}"; DEFAULT_R2_Y="${r2_y}"; DEFAULT_R2_YAW="${r2_yaw}"
      sep_dist=$(python3 -c "import math; print(f'{math.hypot(float(\"${r1_x}\") - float(\"${r2_x}\"), float(\"${r1_y}\") - float(\"${r2_y}\")):.2f}')" 2>/dev/null || echo "N/A")
      echo "[tb3_multi_explore_slam] [Random Spawn] House Base Stations Activated:"
      echo "[tb3_multi_explore_slam]   Robot1 -> Station #${idx1} [${HOUSE_STATION_NAMES[idx1]}]: (${DEFAULT_R1_X}, ${DEFAULT_R1_Y}) [${r1_desc}]"
      echo "[tb3_multi_explore_slam]   Robot2 -> Station #${idx2} [${HOUSE_STATION_NAMES[idx2]}]: (${DEFAULT_R2_X}, ${DEFAULT_R2_Y}) [${r2_desc}]"
      echo "[tb3_multi_explore_slam]   Spawn Distance: ${sep_dist}m"
    fi
    ;;
  warehouse|tb4_warehouse)
    WORLD_FILE="${TB4_SIM_SHARE}/worlds/warehouse.sdf"
    DEFAULT_R1_X="2.00"; DEFAULT_R1_Y="-19.65"; DEFAULT_R1_YAW="1.57"
    DEFAULT_R2_X="0.50"; DEFAULT_R2_Y="-19.65"; DEFAULT_R2_YAW="1.57"

    if [[ "${RANDOM_SPAWN,,}" =~ ^(true|1|yes)$ ]]; then
      # Sample two distinct station indices (0..9) without replacement
      if command -v shuf >/dev/null 2>&1; then
        mapfile -t indices < <(shuf -i 0-9 -n 2)
        idx1="${indices[0]}"
        idx2="${indices[1]}"
      else
        idx1=$(( RANDOM % 10 ))
        idx2=$(( (idx1 + 1 + (RANDOM % 9)) % 10 ))
      fi

      read -r r1_x r1_y r1_yaw r1_desc <<< "${WAREHOUSE_STATION_POSES[idx1]}"
      read -r r2_x r2_y r2_yaw r2_desc <<< "${WAREHOUSE_STATION_POSES[idx2]}"

      DEFAULT_R1_X="${r1_x}"
      DEFAULT_R1_Y="${r1_y}"
      DEFAULT_R1_YAW="${r1_yaw}"

      DEFAULT_R2_X="${r2_x}"
      DEFAULT_R2_Y="${r2_y}"
      DEFAULT_R2_YAW="${r2_yaw}"

      sep_dist=$(python3 -c "import math; print(f'{math.hypot(float(\"${r1_x}\") - float(\"${r2_x}\"), float(\"${r1_y}\") - float(\"${r2_y}\")):.2f}')" 2>/dev/null || echo "N/A")

      echo "[tb3_multi_explore_slam] ======================================================================"
      echo "[tb3_multi_explore_slam] [Random Spawn] Random base station mode ACTIVATED for warehouse:"
      echo "[tb3_multi_explore_slam]   Robot1 -> Station #${idx1} [${WAREHOUSE_STATION_NAMES[idx1]}]: (${DEFAULT_R1_X}, ${DEFAULT_R1_Y}, yaw=${DEFAULT_R1_YAW}) [${r1_desc}]"
      echo "[tb3_multi_explore_slam]   Robot2 -> Station #${idx2} [${WAREHOUSE_STATION_NAMES[idx2]}]: (${DEFAULT_R2_X}, ${DEFAULT_R2_Y}, yaw=${DEFAULT_R2_YAW}) [${r2_desc}]"
      echo "[tb3_multi_explore_slam]   Spawn Distance: ${sep_dist}m (guaranteed non-overlapping & 100% in free space)"
      echo "[tb3_multi_explore_slam] ======================================================================"
    else
      echo "[tb3_multi_explore_slam] [Fixed Spawn] Using default warehouse inspection base stations (Robot1: South Dock 1, Robot2: South Dock 2)."
    fi
    ;;
  depot|tb4_depot)
    WORLD_FILE="${TB4_SIM_SHARE}/worlds/depot.sdf"
    DEFAULT_R1_X="0.00"; DEFAULT_R1_Y="0.00"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.55"; DEFAULT_R2_Y="2.55"; DEFAULT_R2_YAW="0.0"
    if [[ "${RANDOM_SPAWN,,}" =~ ^(true|1|yes)$ ]]; then
      echo "[tb3_multi_explore_slam] Note: Random inspection stations are calibrated for warehouse. Using default poses for depot."
    fi
    ;;
  *)
    WORLD_FILE="${WORLD}"
    DEFAULT_R1_X="-2.0"; DEFAULT_R1_Y="-0.5"; DEFAULT_R1_YAW="0.0"
    DEFAULT_R2_X="0.0";  DEFAULT_R2_Y="0.5";  DEFAULT_R2_YAW="0.0"
    ;;
esac

if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[tb3_multi_explore_slam] World not found: ${WORLD_FILE}" >&2
  echo "Supported presets: sandbox | empty | warehouse | depot | hospital | house (or full path to .sdf/.sdf.xacro)" >&2
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
COORDINATOR_PARAMS="${COORDINATOR_PARAMS:-${ROOT_DIR}/config/coordinator_params.yaml}"
MAP_MERGE_PARAMS="${MAP_MERGE_PARAMS:-${ROOT_DIR}/src/m-explore-ros2/map_merge/config/params.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${ROOT_DIR}/src/m-explore-ros2/map_merge/launch/map_merge.rviz}"

echo "[tb3_multi_explore_slam] world=${WORLD_FILE} headless=${HEADLESS} use_rviz=${USE_RVIZ} mode=${EXPLORATION_MODE} known_init_poses=${KNOWN_INIT_POSES}"
echo "[tb3_multi_explore_slam] robot1 spawn=(${ROBOT1_X}, ${ROBOT1_Y}, ${ROBOT1_YAW}) | robot2 spawn=(${ROBOT2_X}, ${ROBOT2_Y}, ${ROBOT2_YAW})"
echo "[tb3_multi_explore_slam] 3D LiDAR: 16 beams / 360 horizontal / 3 Hz (headless default)"
if command -v free >/dev/null 2>&1; then
  LC_ALL=C free -h | awk '/^Mem:/ {printf "[tb3_multi_explore_slam] memory: %s total, %s available\n", $2, $7}'
fi

ROBOT1_LOG="/tmp/tb3_multi_robot1_bringup.log"
ROBOT2_LOG="/tmp/tb3_multi_robot2_bringup.log"
MERGE_LOG="/tmp/tb3_multi_map_merge.log"
CLOUD_LOG="/tmp/tb3_multi_merge_clouds.log"
OCTOMAP_LOG="/tmp/tb3_multi_octomap.log"
EXPLORE1_LOG="/tmp/tb3_multi_explore1.log"
EXPLORE2_LOG="/tmp/tb3_multi_explore2.log"
COORDINATOR_LOG="/tmp/tb3_multi_coordinator.log"

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
  robot_sdf:='${ROOT_DIR}/config/gz_waffle_3d_lidar.sdf.xacro' \
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

# Extra bridge for robot1's 3D point cloud. The official tb3_simulation_launch.py
# bridge config does not include PointCloud2, so bridge it separately.
POINTS1_LOG="/tmp/tb3_multi_robot1_points_bridge.log"
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -r __ns:=/robot1 \
  -p config_file:="${ROOT_DIR}/config/tb3_points_bridge.yaml" \
  -p use_sim_time:=true \
  -p expand_gz_topic_names:=true \
  >"${POINTS1_LOG}" 2>&1 &
POINTS1_PID=$!
echo "[tb3_multi_explore_slam] robot1 points bridge PID=${POINTS1_PID} log=${POINTS1_LOG}"

sleep 4

# 2. Spawn robot2 into same Gazebo server + launch robot2 (Nav2 + slam_toolbox in /robot2)
# Uses tb3_bridge_no_clock.yaml so /clock is only bridged once (by robot1),
# preventing duplicate /clock jumps that clear tf2 buffers.
BRIDGE_NO_CLOCK="${ROOT_DIR}/config/tb3_bridge_no_clock.yaml"
URDF_FILE="${SIM_SHARE}/urdf/turtlebot3_waffle.urdf"
SDF_XACRO="${ROOT_DIR}/config/gz_waffle_3d_lidar.sdf.xacro"

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
CLOUD_PID=""
OCTOMAP_PID=""
EXP1_PID=""
EXP2_PID=""
COORD_PID=""
RVIZ_PID=""
RELAY_PID=""

cleanup() {
  echo "[tb3_multi_explore_slam] Shutting down dual-robot exploration & 3D mapping..."
  for pid in "${RELAY_PID}" "${COORD_PID}" "${EXP1_PID}" "${EXP2_PID}" "${OCTOMAP_PID}" "${CLOUD_PID}" "${MERGE_PID}" "${RVIZ_PID}" "${POINTS1_PID:-}" "${R2_PID}" "${R1_PID}"; do
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
  if grep -qE "lifecycle_manager.*Managed nodes are active" "${ROBOT1_LOG}" 2>/dev/null && \
     grep -q "StaticLayer: Resizing costmap" "${ROBOT1_LOG}" 2>/dev/null; then
    r1_ok=1
  fi

  if grep -qE "lifecycle_manager.*Managed nodes are active" "${ROBOT2_LOG}" 2>/dev/null && \
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

# 3b. Merge both robots' 3D point clouds into the world frame and build OctoMap.
sleep 2

# The merger needs world -> robotX/map, which map_merge publishes once it
# receives both SLAM maps. It retries TF lookups until then, so it is safe to
# start right after map_merge.
python3 "${ROOT_DIR}/scripts/merge_pointclouds.py" --ros-args \
  -p use_sim_time:=true \
  -p robots:="['robot1', 'robot2']" \
  -p world_frame:=world \
  -p voxel_size:=0.05 \
  -p publish_rate:=3.0 \
  >"${CLOUD_LOG}" 2>&1 &
CLOUD_PID=$!
echo "[tb3_multi_explore_slam] point-cloud merger PID=${CLOUD_PID} log=${CLOUD_LOG}"

if ros2 pkg prefix octomap_server >/dev/null 2>&1; then
  ros2 run octomap_server octomap_server_node --ros-args \
    --params-file "${ROOT_DIR}/config/octomap_params.yaml" \
    -p use_sim_time:=true \
    -r cloud_in:=/points_merged \
    >"${OCTOMAP_LOG}" 2>&1 &
  OCTOMAP_PID=$!
  echo "[tb3_multi_explore_slam] octomap_server PID=${OCTOMAP_PID} log=${OCTOMAP_LOG}"
else
  echo "[tb3_multi_explore_slam] WARNING: ros-lyrical-octomap-server is not installed; 3D OctoMap disabled." >&2
  echo "[tb3_multi_explore_slam]   sudo apt install ros-lyrical-octomap-server ros-lyrical-octomap-msgs" >&2
fi

if [[ "${USE_RVIZ,,}" == "true" ]]; then
  echo "[tb3_multi_explore_slam] Launching RViz with ${RVIZ_CONFIG} ..."
  rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
  RVIZ_PID=$!
fi

# 4. Start Exploration Decision Layer (Auction Coordinator or Legacy explore_lite)
if [[ "${EXPLORATION_MODE,,}" == "auction" ]]; then
  echo "[tb3_multi_explore_slam] Starting multi_robot_coordinator (Auction Coordination on /map) ..."
  python3 "${ROOT_DIR}/scripts/multi_robot_coordinator.py" --ros-args \
    --params-file "${COORDINATOR_PARAMS}" \
    -p use_sim_time:=true \
    -p robot_namespaces:="['robot1', 'robot2']" \
    -p robot1_init_pose:="[${ROBOT1_X}, ${ROBOT1_Y}, ${ROBOT1_YAW}]" \
    -p robot2_init_pose:="[${ROBOT2_X}, ${ROBOT2_Y}, ${ROBOT2_YAW}]" \
    >"${COORDINATOR_LOG}" 2>&1 &
  COORD_PID=$!

  echo "[tb3_multi_explore_slam] Dual-robot auction exploration running!"
  echo "  - Coordinator log:    ${COORDINATOR_LOG}"
  echo "  - Merged map topic:   /map (frame_id: world)"
  echo "  - Markers topic:      /exploration/markers"
  echo "  - map_merge log:      ${MERGE_LOG}"
else
  echo "[tb3_multi_explore_slam] Starting legacy explore_lite for /robot1 and /robot2 ..."
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

  if [[ "${USE_RVIZ,,}" == "true" ]]; then
    python3 "${ROOT_DIR}/scripts/multi_robot_plan_relay.py" --ros-args \
      -p use_sim_time:=true \
      -p robot_namespaces:="['robot1', 'robot2']" \
      -p robot1_init_pose:="[${ROBOT1_X}, ${ROBOT1_Y}, ${ROBOT1_YAW}]" \
      -p robot2_init_pose:="[${ROBOT2_X}, ${ROBOT2_Y}, ${ROBOT2_YAW}]" \
      >/dev/null 2>&1 &
    RELAY_PID=$!
  fi

  echo "[tb3_multi_explore_slam] Legacy dual exploration running!"
  echo "  - robot1 explore log: ${EXPLORE1_LOG}"
  echo "  - robot2 explore log: ${EXPLORE2_LOG}"
  echo "  - map_merge log:      ${MERGE_LOG}"
  echo "  - Merged map topic:   /map (frame_id: world)"
fi
echo "Press Ctrl-C to stop."

wait "${MERGE_PID}"

