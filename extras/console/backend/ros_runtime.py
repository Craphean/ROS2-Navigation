"""Owns the rclpy context and the spin thread.

``rclpy.init`` must run on the main thread, so it is called from the FastAPI
lifespan (which runs on the event-loop thread).  Spinning happens on a daemon
thread; shutdown always runs from the main thread, never from the spin thread.
"""

from __future__ import annotations

import threading
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .config import Settings
from .ros_node import ConsoleNode


class RosRuntime:
    def __init__(
        self,
        settings: Settings,
        video_hubs: dict[str, Any],
        world_hub: Any = None,
    ):
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        self.node = ConsoleNode(settings, video_hubs, world_hub)
        # Callbacks must not starve while an action result is awaited, and the
        # 15 Hz preview image is a 1 MB message: two threads left the image
        # subscription dropping frames under load.
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name="rclpy", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:  # noqa: BLE001 - keep the web server alive
            self.node.get_logger().error(f"spin thread exited: {exc!r}")

    def shutdown(self) -> None:
        try:
            self.node.safe_stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:  # noqa: BLE001
            pass
        self._thread.join(timeout=5.0)
        try:
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()
