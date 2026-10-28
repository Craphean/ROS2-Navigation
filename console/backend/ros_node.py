"""Single rclpy node that backs the web console.

It owns every ROS interaction: subscriptions for the dashboard, Nav2 action
clients for goals, the collision-monitor toggle for teleop, and cmd_vel
publishers.  All methods that await a ROS future are ``async`` and are meant to
be called from the FastAPI event loop; the node itself is spun by
``RosRuntime`` on a separate thread.
"""

from __future__ import annotations

import asyncio
import math
from typing import Any

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import Toggle
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from .config import Settings
from .state import RobotState, StateStore, yaw_from_quaternion


MAP_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
SENSOR_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


async def wait_rclpy_future(future: Any, timeout: float) -> Any:
    """Bridge an rclpy Future into asyncio without blocking the loop."""
    loop = asyncio.get_running_loop()
    bridge: asyncio.Future = loop.create_future()

    def _done(done_future: Any) -> None:
        if bridge.done():
            return
        try:
            bridge.set_result(done_future.result())
        except Exception as exc:  # noqa: BLE001 - forwarded to the caller
            if not bridge.done():
                bridge.set_exception(exc)

    future.add_done_callback(_done)
    return await asyncio.wait_for(bridge, timeout)


class ConsoleNode(Node):
    def __init__(self, settings: Settings, video_hubs: dict[str, Any]):
        super().__init__("mine_inspection_console")
        self.settings = settings
        self.store = StateStore(settings.robots)
        self.store.sim_meta = {"_path": str(settings.sim_meta_path)}
        self.store.load_sim_meta()
        self.video_hubs = video_hubs
        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.cmd_pubs: dict[str, Any] = {}
        self.nav_clients: dict[str, ActionClient] = {}
        self.toggle_clients: dict[str, Any] = {}
        self._handles: dict[str, Any] = {}

        for ns in settings.robots:
            self.cmd_pubs[ns] = self.create_publisher(
                TwistStamped, f"/{ns}/cmd_vel", 10
            )
            self.nav_clients[ns] = ActionClient(
                self,
                NavigateToPose,
                f"/{ns}/navigate_to_pose",
                callback_group=self.cb_group,
            )
            self.toggle_clients[ns] = self.create_client(
                Toggle, f"/{ns}/collision_monitor/toggle", callback_group=self.cb_group
            )
            self._subscribe_robot(ns)

        self.create_subscription(
            OccupancyGrid, "/map", self._guard(self.store.on_map), MAP_QOS
        )
        self.create_subscription(Clock, "/clock", self._guard(self.store.on_clock), 10)
        self.create_subscription(
            MarkerArray,
            "/exploration/markers",
            self._guard(self.store.on_markers),
            5,
        )

        self.get_logger().info(
            f"console node ready for {list(settings.robots)}; "
            f"sim_meta={settings.sim_meta_path}"
        )

    # ------------------------------------------------------------------
    # Wiring
    # ------------------------------------------------------------------
    def _subscribe_robot(self, ns: str) -> None:
        self.create_subscription(
            Odometry,
            f"/{ns}/odom",
            self._guard(lambda msg, n=ns: self.store.on_odom(n, msg)),
            10,
        )
        self.create_subscription(
            LaserScan,
            f"/{ns}/scan",
            self._guard(lambda msg, n=ns: self.store.on_scan(n, msg)),
            SENSOR_QOS,
        )
        self.create_subscription(
            GoalStatusArray,
            f"/{ns}/navigate_to_pose/_action/status",
            self._guard(lambda msg, n=ns: self.store.on_goal_status(n, "to_pose", msg)),
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            f"/{ns}/navigate_through_poses/_action/status",
            self._guard(
                lambda msg, n=ns: self.store.on_goal_status(n, "through_poses", msg)
            ),
            10,
        )
        self.create_subscription(
            CollisionMonitorState,
            f"/{ns}/collision_monitor_state",
            self._guard(lambda msg, n=ns: self.store.on_collision(n, msg)),
            5,
        )
        hub = self.video_hubs.get(ns)
        if hub is not None:
            self.create_subscription(
                Image,
                f"/{ns}/camera/image",
                self._guard(
                    lambda msg, n=ns, h=hub: (h.on_image(msg), self.store.on_image(n))
                ),
                SENSOR_QOS,
            )

    def _guard(self, callback: Any) -> Any:
        def wrapper(msg: Any) -> None:
            try:
                callback(msg)
            except Exception as exc:  # noqa: BLE001 - a dead callback kills the spin thread
                self.get_logger().error(f"callback failed: {exc!r}")

        return wrapper

    # ------------------------------------------------------------------
    # Poses / transforms
    # ------------------------------------------------------------------
    def _spawn_pose(self, ns: str) -> tuple[float, float, float]:
        entry = self.store.sim_meta.get(ns)
        if isinstance(entry, (list, tuple)) and len(entry) >= 3:
            return float(entry[0]), float(entry[1]), float(entry[2])
        return 0.0, 0.0, 0.0

    def lookup_world_pose(self, ns: str) -> tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", f"{ns}/base_footprint", Time()
            )
            translation = tf.transform.translation
            rotation = tf.transform.rotation
            return (
                float(translation.x),
                float(translation.y),
                yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
            )
        except Exception:  # noqa: BLE001 - TF is expected to be missing at boot
            pass
        with self.store.lock:
            local = self.store.robots[ns].local_pose
        if local is None:
            return None
        x0, y0, yaw0 = self._spawn_pose(ns)
        lx, ly, lyaw = local
        cos0, sin0 = math.cos(yaw0), math.sin(yaw0)
        return (
            x0 + cos0 * lx - sin0 * ly,
            y0 + sin0 * lx + cos0 * ly,
            yaw0 + lyaw,
        )

    def world_to_map(
        self, ns: str, x: float, y: float, yaw: float
    ) -> tuple[float, float, float]:
        """SE(2) transform of a world pose into ``<ns>/map``.

        Nav2 only understands its own local map frame; sending world
        coordinates directly makes the robot drive off to infinity.
        """
        try:
            tf = self.tf_buffer.lookup_transform(f"{ns}/map", "world", Time())
            rotation = tf.transform.rotation
            yaw_t = yaw_from_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w
            )
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
        except Exception:  # noqa: BLE001
            x0, y0, yaw0 = self._spawn_pose(ns)
            yaw_t = -yaw0
            tx = -(math.cos(yaw_t) * x0 - math.sin(yaw_t) * y0)
            ty = -(math.sin(yaw_t) * x0 + math.cos(yaw_t) * y0)
        cos_t, sin_t = math.cos(yaw_t), math.sin(yaw_t)
        local_x = tx + cos_t * x - sin_t * y
        local_y = ty + sin_t * x + cos_t * y
        return local_x, local_y, yaw + yaw_t

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    async def navigate_world(
        self, ns: str, x: float, y: float, yaw: float, timeout: float
    ) -> str:
        """Send a NavigateToPose goal.  Returns OK / FAILED / CANCELED / TIMEOUT."""
        client = self.nav_clients.get(ns)
        if client is None or not client.server_is_ready():
            return "FAILED"

        local_x, local_y, local_yaw = self.world_to_map(ns, x, y, yaw)
        goal = NavigateToPose.Goal()
        # Nav2 runs inside the robot namespace and listens to the *local* TF
        # tree, whose global frame is the unprefixed `map`.  The coordinates
        # above are already expressed in the physical frame that map_merge
        # calls `<ns>/map` (identical pose, different label), so the goal must
        # be labelled `map`; sending `<ns>/map` makes bt_navigator fail with
        # error 9002 "source_frame does not exist".
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = local_x
        goal.pose.pose.position.y = local_y
        goal.pose.pose.orientation.z = math.sin(local_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(local_yaw / 2.0)

        self.set_goal(ns, x, y, yaw)
        try:
            send_future = client.send_goal_async(
                goal,
                feedback_callback=lambda feedback, n=ns: self._on_feedback(n, feedback),
            )
            handle = await wait_rclpy_future(send_future, 10.0)
        except asyncio.TimeoutError:
            return "TIMEOUT"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[{ns}] send_goal failed: {exc!r}")
            return "FAILED"

        if handle is None or not handle.accepted:
            return "FAILED"

        self._handles[ns] = handle
        try:
            result = await wait_rclpy_future(handle.get_result_async(), timeout)
        except asyncio.TimeoutError:
            await self.cancel_goal(ns)
            return "TIMEOUT"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[{ns}] goal result failed: {exc!r}")
            return "FAILED"
        finally:
            self._handles.pop(ns, None)

        status = int(getattr(result, "status", 6))
        if status == 4:  # STATUS_SUCCEEDED
            return "OK"
        if status == 5:  # STATUS_CANCELED
            return "CANCELED"
        return "FAILED"

    async def cancel_goal(self, ns: str) -> None:
        handle = self._handles.pop(ns, None)
        if handle is not None:
            try:
                await wait_rclpy_future(handle.cancel_goal_async(), 5.0)
            except Exception:  # noqa: BLE001
                pass

    def set_goal(self, ns: str, x: float, y: float, yaw: float) -> None:
        with self.store.lock:
            self.store.robots[ns].goal_world = (x, y)

    def _on_feedback(self, ns: str, feedback: Any) -> None:
        try:
            remaining = float(feedback.feedback.distance_remaining)
        except Exception:  # noqa: BLE001
            return
        with self.store.lock:
            robot: RobotState = self.store.robots[ns]
            robot.distance_remaining = remaining

    # ------------------------------------------------------------------
    # Actuation
    # ------------------------------------------------------------------
    def publish_cmd(self, ns: str, vx: float, wz: float) -> None:
        publisher = self.cmd_pubs.get(ns)
        if publisher is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        publisher.publish(msg)

    def front_clearance(self, ns: str, half_angle: float = 0.44) -> float | None:
        """Minimum range inside a forward cone, in metres."""
        with self.store.lock:
            samples = list(self.store.robots[ns].scan)
        best: float | None = None
        for rng, angle in samples:
            if abs(angle) > half_angle or rng <= 0.02:
                continue
            if best is None or rng < best:
                best = rng
        return best

    async def toggle_collision(self, ns: str, enable: bool) -> tuple[bool, str]:
        client = self.toggle_clients.get(ns)
        if client is None:
            return False, "no client"
        if not client.service_is_ready():
            return False, "collision_monitor/toggle 不可用"
        try:
            response = await wait_rclpy_future(
                client.call_async(Toggle.Request(enable=enable)), 2.0
            )
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        return bool(response.success), str(response.message)

    def safe_stop(self) -> None:
        for ns in self.settings.robots:
            for _ in range(5):
                self.publish_cmd(ns, 0.0, 0.0)
