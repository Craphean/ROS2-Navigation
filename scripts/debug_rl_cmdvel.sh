#!/usr/bin/env bash
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/lyrical/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -e

echo "=== lifecycle ==="
timeout 3 ros2 lifecycle get /bt_navigator || true
timeout 3 ros2 lifecycle get /controller_server || true
timeout 3 ros2 lifecycle get /collision_monitor || true

echo "=== restart relay ==="
pkill -f rl_cmd_vel_relay.py 2>/dev/null || true
sleep 1
setsid python3 "${ROOT_DIR}/scripts/rl_cmd_vel_relay.py" --ros-args -p use_sim_time:=true \
  </dev/null >/tmp/rl_cmd_vel_relay.log 2>&1 &
sleep 2
tail -5 /tmp/rl_cmd_vel_relay.log || true

echo "=== initialpose ==="
# Map pose for the default Gazebo spawn (0, 0.5). SLAM origin is world (-2, -0.5).
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]
  }
}" || true

sleep 2
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" || true
sleep 8

echo "=== send goal ==="
timeout 40 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {position: {x: -0.5, y: -0.5, z: 0.0}, orientation: {w: 1.0}}
  }
}" >/tmp/goal_fix3.log 2>&1 &
GP=$!
sleep 5

python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
rclpy.init()
n = Node('v3')
d = {'nav': [], 'sm': [], 'cmd': [], 'odom': []}
n.create_subscription(TwistStamped, '/cmd_vel_nav', lambda m: d['nav'].append(m.twist.linear.x), 10)
n.create_subscription(TwistStamped, '/cmd_vel_smoothed', lambda m: d['sm'].append(m.twist.linear.x), 10)
n.create_subscription(TwistStamped, '/cmd_vel', lambda m: d['cmd'].append(m.twist.linear.x), 10)
n.create_subscription(Odometry, '/odom', lambda m: d['odom'].append(m.pose.pose.position.x), 10)
end = time.time() + 12
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
for k in ['nav', 'sm', 'cmd']:
    v = d[k]
    print(k, 'n', len(v), 'avg', (sum(v)/len(v) if v else None), 'last', (v[-1] if v else None))
o = d['odom']
print('odom n', len(o), 'delta', (o[-1]-o[0] if len(o) > 1 else None), 'end', (o[-1] if o else None))
n.destroy_node()
rclpy.shutdown()
PY

wait "$GP" || true
echo "=== goal result ==="
tail -25 /tmp/goal_fix3.log
