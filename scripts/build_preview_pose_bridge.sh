#!/usr/bin/env bash
# Build tools/preview_pose_bridge.cc into console/bin/preview_pose_bridge.
#
# The console drives the Gazebo preview camera through this helper instead of
# spawning 'gz service' for every drag event: spawning costs ~400 ms here, a
# request on a warm transport node costs ~1 ms.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${ROOT_DIR}/tools/preview_pose_bridge.cc"
OUT_DIR="${ROOT_DIR}/console/bin"
BIN="${OUT_DIR}/preview_pose_bridge"

set +u
source /opt/ros/lyrical/setup.bash
set -u

export PKG_CONFIG_PATH="$(ls -d /opt/ros/lyrical/opt/*/lib/pkgconfig 2>/dev/null | tr '\n' ':')"
LIB_PATH="$(ls -d /opt/ros/lyrical/opt/*/lib 2>/dev/null | tr '\n' ':')"

mkdir -p "${OUT_DIR}"
g++ -std=c++17 -O2 "${SRC}" \
  $(pkg-config --cflags --libs gz-transport gz-msgs) \
  -Wl,-rpath,${LIB_PATH%:} \
  -o "${BIN}"

echo "[build] ${BIN}"
