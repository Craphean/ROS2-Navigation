#!/usr/bin/env bash
# Start the mine inspection console.
#
#   ./console/run.sh
#   CONSOLE_PORT=8080 CONSOLE_TOKEN=mysecret ./console/run.sh
#
# The ROS 2 Python bindings are built for /usr/bin/python3 (3.14).  The
# python3 on PATH may be a different interpreter and cannot import rclpy, so
# the venv is always created from /usr/bin/python3.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONSOLE_DIR="${ROOT_DIR}/extras/console"
cd "${ROOT_DIR}"

# ROS setup scripts reference unset variables, which `set -u` rejects.
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${ROOT_DIR}/config/cyclonedds.xml}"
export PYTHONPATH="${ROOT_DIR}/extras:${PYTHONPATH:-}"

VENV="${CONSOLE_DIR}/.venv"
if [[ ! -x "${VENV}/bin/python" ]]; then
  echo "[console] creating venv with /usr/bin/python3 ..."
  /usr/bin/python3 -m venv --system-site-packages "${VENV}"
  "${VENV}/bin/pip" install -q --upgrade pip \
    -i https://pypi.tuna.tsinghua.edu.cn/simple
  "${VENV}/bin/pip" install -q fastapi 'uvicorn[standard]' \
    -i https://pypi.tuna.tsinghua.edu.cn/simple
fi

if ! "${VENV}/bin/python" -c "import fastapi, uvicorn, websockets" >/dev/null 2>&1; then
  echo "[console] installing fastapi/uvicorn[standard] ..."
  "${VENV}/bin/pip" install -q fastapi 'uvicorn[standard]' \
    -i https://pypi.tuna.tsinghua.edu.cn/simple
fi

mkdir -p "${CONSOLE_DIR}/run/missions" "${CONSOLE_DIR}/missions"

PORT="${CONSOLE_PORT:-8080}"
HOST="${CONSOLE_HOST:-0.0.0.0}"
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

echo "======================================================================"
echo " Optional Visualization Console (excluded from paper benchmarks)"
echo "   Local : http://127.0.0.1:${PORT}"
echo "   LAN   : http://${IP:-<vm-ip>}:${PORT}"
echo "   Token : ${CONSOLE_TOKEN:-mine2026}"
echo "======================================================================"

exec "${VENV}/bin/python" -m console.backend
