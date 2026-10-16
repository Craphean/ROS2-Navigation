#!/usr/bin/env bash
# Launch Nav2 TurtleBot4 + Gazebo with official known map (AMCL, no SLAM).
# Worlds: warehouse (default, larger) | depot — matching maps ship with nav2_bringup.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
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

# Fast DDS on VMware/multi-process Nav2 often fails bt_navigator action discovery
# ("compute_path_to_pose not available"). CycloneDDS is more reliable here.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

is_vmware=0
if lspci 2>/dev/null | grep -qi 'VMware'; then
  is_vmware=1
fi

if [[ "${is_vmware}" -eq 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  HEADLESS="${HEADLESS:-True}"
  USE_COMPOSITION="${USE_COMPOSITION:-False}"
  echo "[tb4_nav_gazebo] Detected VMware guest"
else
  HEADLESS="${HEADLESS:-False}"
  USE_COMPOSITION="${USE_COMPOSITION:-True}"
fi

if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == "1" ]]; then
  echo "[tb4_nav_gazebo] Using software OpenGL (LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}, MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE:-unset})"
fi

SIM_SHARE="$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim"
BRINGUP_SHARE="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup"
WORLD_NAME="${WORLD:-warehouse}"

case "${WORLD_NAME}" in
  warehouse)
    WORLD_FILE="${SIM_SHARE}/worlds/warehouse.sdf"
    MAP_DEFAULT="${BRINGUP_SHARE}/maps/warehouse.yaml"
    KEEPOUT="${BRINGUP_SHARE}/maps/warehouse_keepout.yaml"
    SPEED="${BRINGUP_SHARE}/maps/warehouse_speed.yaml"
    GRAPH="${BRINGUP_SHARE}/graphs/warehouse_graph.geojson"
    X_POSE="2.00"
    Y_POSE="-19.65"
    YAW="1.57"
    QZ="0.70710678"
    QW="0.70710678"
    ;;
  depot)
    WORLD_FILE="${SIM_SHARE}/worlds/depot.sdf"
    MAP_DEFAULT="${BRINGUP_SHARE}/maps/depot.yaml"
    KEEPOUT="${BRINGUP_SHARE}/maps/depot_keepout.yaml"
    SPEED="${BRINGUP_SHARE}/maps/depot_speed.yaml"
    GRAPH="${BRINGUP_SHARE}/graphs/depot_graph.geojson"
    X_POSE="0.00"
    Y_POSE="0.00"
    YAW="0.00"
    QZ="0.0"
    QW="1.0"
    ;;
  hospital|tb3_hospital|clinic)
    WORLD_FILE="${ROOT_DIR}/worlds/hospital.sdf.xacro"
    MAP_DEFAULT="${ROOT_DIR}/maps/hospital_map.yaml"
    KEEPOUT=""
    SPEED=""
    GRAPH=""
    X_POSE="-6.00"
    Y_POSE="0.00"
    YAW="0.00"
    QZ="0.0"
    QW="1.0"
    ;;
  house|tb3_house|home)
    WORLD_FILE="${ROOT_DIR}/worlds/house.sdf.xacro"
    MAP_DEFAULT="${ROOT_DIR}/maps/house_map.yaml"
    KEEPOUT=""
    SPEED=""
    GRAPH=""
    X_POSE="0.00"
    Y_POSE="-0.40"
    YAW="1.57"
    QZ="0.70710678"
    QW="0.70710678"
    ;;
  *)
    echo "[tb4_nav_gazebo] Unknown WORLD=${WORLD_NAME} (use warehouse|depot|hospital|house)" >&2
    exit 1
    ;;
esac

if [[ $# -ge 1 && ( "${1}" == *.yaml || "${1}" == *.yml ) ]]; then
  MAP="${1}"
  shift
fi
MAP="${MAP:-${MAP_DEFAULT}}"

if [[ ! -f "${MAP}" ]]; then
  echo "[tb4_nav_gazebo] Map not found: ${MAP}" >&2
  exit 1
fi
if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[tb4_nav_gazebo] World not found: ${WORLD_FILE}" >&2
  exit 1
fi

PARAMS_FILE="${PARAMS_FILE:-${ROOT_DIR}/config/nav2_params_vmware.yaml}"
if [[ ! -f "${PARAMS_FILE}" ]]; then
  PARAMS_FILE="${BRINGUP_SHARE}/params/nav2_params.yaml"
fi

echo "[tb4_nav_gazebo] DISPLAY=${DISPLAY} WAYLAND_DISPLAY=${WAYLAND_DISPLAY} XAUTHORITY=${XAUTHORITY:-unset}"
echo "[tb4_nav_gazebo] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[tb4_nav_gazebo] world=${WORLD_FILE} map=${MAP} spawn=(${X_POSE},${Y_POSE},yaw=${YAW})"
echo "[tb4_nav_gazebo] params=${PARAMS_FILE} headless=${HEADLESS} use_composition=${USE_COMPOSITION}"
echo "[tb4_nav_gazebo] First run may download Fuel models; needs network."

# Pose + bt_navigator recovery in a separate session so SIGINT/timeout in helpers
# cannot take down ros2 launch (same process-group issue on VMware terminals).
HELPER_LOG="${ROOT_DIR}/tb4_nav_helper.log"
setsid bash -c "
source /opt/ros/lyrical/setup.bash
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
X_POSE='${X_POSE}'; Y_POSE='${Y_POSE}'; QZ='${QZ}'; QW='${QW}'
for _ in \$(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -qx '/initialpose'; then break; fi
  sleep 1
done
for _ in 1 2 3 4 5 6 7 8; do
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"{
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: \${X_POSE}, y: \${Y_POSE}, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: \${QZ}, w: \${QW}}
      },
      covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
    }
  }\" >/dev/null 2>&1 || true
  sleep 2
done
for _ in \$(seq 1 60); do
  if ros2 topic echo /amcl_pose --once 2>/dev/null | grep -q 'position:'; then
    echo '[tb4_nav_gazebo] amcl_pose OK'
    break
  fi
  sleep 1
done
for _ in \$(seq 1 40); do
  if ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null \\
      | grep -q 'success=True'; then
    echo '[tb4_nav_gazebo] navigation lifecycle active'
    exit 0
  fi
  for n in bt_navigator waypoint_follower docking_server following_server; do
    st=\$(ros2 lifecycle get \"/\${n}\" 2>/dev/null || true)
    if [[ \"\${st}\" == *inactive* ]]; then
      echo \"[tb4_nav_gazebo] activating /\${n} ...\"
      ros2 lifecycle set \"/\${n}\" activate >/dev/null 2>&1 || true
    fi
  done
  sleep 3
done
echo '[tb4_nav_gazebo] warning: navigation still not fully active' >&2
" </dev/null >"${HELPER_LOG}" 2>&1 &

exec ros2 launch nav2_bringup tb4_simulation_launch.py \
  slam:=False \
  map:="${MAP}" \
  keepout_mask:="${KEEPOUT}" \
  speed_mask:="${SPEED}" \
  graph:="${GRAPH}" \
  world:="${WORLD_FILE}" \
  params_file:="${PARAMS_FILE}" \
  headless:="${HEADLESS}" \
  use_composition:="${USE_COMPOSITION}" \
  use_sim_time:=True \
  x_pose:="${X_POSE}" \
  y_pose:="${Y_POSE}" \
  yaw:="${YAW}" \
  "$@"
