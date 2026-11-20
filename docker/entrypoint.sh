#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
if [ -f "/ws/install/setup.bash" ]; then
  source "/ws/install/setup.bash"
fi

if [ -f "/ws/config/cyclonedds.xml" ] && [ -z "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI="file:///ws/config/cyclonedds.xml"
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

exec "$@"
