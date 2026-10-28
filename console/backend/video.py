"""Camera frame fan-out and MJPEG streaming.

One JPEG is encoded per incoming ROS image and shared by every browser.  A
naive implementation that calls ``imencode`` per client melts a 4-core VM as
soon as two operators open the page.
"""

from __future__ import annotations

import asyncio
import time
from typing import Any, AsyncIterator

import cv2
import numpy as np


class FrameHub:
    def __init__(self, jpeg_quality: int = 72):
        self.jpeg_quality = jpeg_quality
        self._loop: asyncio.AbstractEventLoop | None = None
        self._event: asyncio.Event | None = None
        self._jpeg: bytes | None = None
        self._seq = 0
        self._last_image_wall = 0.0
        self.clients = 0

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._event = asyncio.Event()

    # ------------------------------------------------------------------
    # Called from the rclpy executor thread
    # ------------------------------------------------------------------
    def on_image(self, msg: Any) -> None:
        frame = _decode_image(msg)
        if frame is None:
            return
        ok, buffer = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            return
        self._jpeg = buffer.tobytes()
        self._seq += 1
        self._last_image_wall = time.monotonic()
        if self._loop is not None and self._event is not None:
            self._loop.call_soon_threadsafe(self._event.set)

    # ------------------------------------------------------------------
    # Async side
    # ------------------------------------------------------------------
    def fresh(self, max_age: float = 5.0) -> bool:
        return (
            self._jpeg is not None
            and time.monotonic() - self._last_image_wall <= max_age
        )

    async def wait_next(self, last_seq: int, timeout: float = 2.0) -> tuple[int, bytes | None]:
        if self._event is None:
            return self._seq, self._jpeg
        if self._seq == last_seq:
            self._event.clear()
            try:
                await asyncio.wait_for(self._event.wait(), timeout)
            except asyncio.TimeoutError:
                pass
        return self._seq, self._jpeg

    def snapshot(self) -> bytes | None:
        return self._jpeg


def _decode_image(msg: Any) -> np.ndarray | None:
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = (msg.encoding or "").lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None or height <= 0 or width <= 0:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < height * step:
        return None
    rows = raw[: height * step].reshape(height, step)
    pixels = rows[:, : width * channels].reshape(height, width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    if encoding == "mono8":
        return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
    return pixels


async def mjpeg_stream(hub: FrameHub, fps: float = 0.0) -> AsyncIterator[bytes]:
    """Yield multipart MJPEG chunks.

    ``fps`` caps the output rate (useful over a slow cpolar tunnel); 0 streams
    every new frame.
    """
    min_dt = 1.0 / fps if fps and fps > 0 else 0.0
    last_seq = -1
    last_sent = 0.0
    while True:
        seq, jpeg = await hub.wait_next(last_seq, timeout=2.0)
        if jpeg is None or seq == last_seq:
            # Keep the connection alive even when the camera is missing.
            await asyncio.sleep(0.2)
            continue
        now = time.monotonic()
        if min_dt and now - last_sent < min_dt:
            continue
        last_seq = seq
        last_sent = now
        yield (
            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(jpeg)).encode()
            + b"\r\n\r\n"
            + jpeg
            + b"\r\n"
        )
