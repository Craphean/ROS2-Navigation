"""Camera frame fan-out and MJPEG streaming.

One JPEG is encoded per *redraw*, not per incoming frame, and shared by every
browser:

* with no viewer the hub keeps only the last decoded frame — it used to decode
  and JPEG-encode every camera frame for nobody;
* with viewers the encode rate is capped at the rate the console actually
  displays (10 Hz), because a naive implementation that encodes at the camera
  rate and again per client melts a 4-core VM as soon as two operators open
  the page.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any, AsyncIterator

import cv2
import numpy as np

# The web UI polls snapshot.jpg at 10 Hz, so encoding faster than this only
# burns CPU.  Bounds the JPEG work whatever the camera frame rate is.
_MAX_ENCODE_FPS = 10.0


class FrameHub:
    def __init__(self, jpeg_quality: int = 72, max_fps: float = _MAX_ENCODE_FPS):
        self.jpeg_quality = jpeg_quality
        self._min_dt = 1.0 / max_fps if max_fps and max_fps > 0 else 0.0
        self._loop: asyncio.AbstractEventLoop | None = None
        self._event: asyncio.Event | None = None
        # Encodes can be triggered from the rclpy thread (streaming clients)
        # and from the event loop (snapshot.jpg polls), so guard the frame and
        # the encoded result.
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._frame_seq = 0
        self._jpeg: bytes | None = None
        self._jpeg_frame = -1
        self._encode_wall = 0.0
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
        if not frame.flags.owndata:
            # bgr8 is a view on msg.data; copy so the DDS buffer is not pinned
            # for as long as this hub holds the frame.
            frame = frame.copy()
        with self._lock:
            self._frame = frame
            self._frame_seq += 1
            self._last_image_wall = time.monotonic()
            streaming = self.clients > 0
        if streaming:
            self.encode()

    # ------------------------------------------------------------------
    # Encoding
    # ------------------------------------------------------------------
    def encode(self, force: bool = False) -> bytes | None:
        """Return the newest JPEG, encoding at most max_fps times a second."""
        wake = False
        with self._lock:
            frame = self._frame
            if frame is None:
                return None
            if self._jpeg_frame == self._frame_seq:
                return self._jpeg
            now = time.monotonic()
            if not force and now - self._encode_wall < self._min_dt:
                return self._jpeg
            ok, buffer = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if not ok:
                return self._jpeg
            self._jpeg = buffer.tobytes()
            self._jpeg_frame = self._frame_seq
            self._encode_wall = now
            self._seq += 1
            jpeg = self._jpeg
            wake = True
        if wake:
            self._wake()
        return jpeg

    def _wake(self) -> None:
        if self._loop is not None and self._event is not None:
            self._loop.call_soon_threadsafe(self._event.set)

    # ------------------------------------------------------------------
    # Async side
    # ------------------------------------------------------------------
    def fresh(self, max_age: float = 5.0) -> bool:
        return (
            self._frame is not None
            and time.monotonic() - self._last_image_wall <= max_age
        )

    async def wait_next(self, last_seq: int, timeout: float = 2.0) -> tuple[int, bytes | None]:
        if self._event is None:
            jpeg = self.snapshot()
            return self._seq, jpeg
        if self._seq == last_seq:
            self._event.clear()
            try:
                await asyncio.wait_for(self._event.wait(), timeout)
            except asyncio.TimeoutError:
                pass
        jpeg = self.snapshot()
        return self._seq, jpeg

    def snapshot(self) -> bytes | None:
        """Latest frame as JPEG, encoded on demand for snapshot.jpg polls."""
        return self.encode()

    def stats(self) -> dict[str, int]:
        """Counters for the diagnostics endpoint (no lock needed: ints only)."""
        return {
            "received": self._frame_seq,
            "encoded": self._seq,
            "clients": self.clients,
        }


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

    fps caps the output rate (useful over a slow cpolar tunnel); 0 streams
    every encoded frame.
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
            await asyncio.sleep(min_dt - (now - last_sent))
            jpeg = hub.snapshot() or jpeg
        last_seq = seq
        last_sent = now
        yield (
            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(jpeg)).encode()
            + b"\r\n\r\n"
            + jpeg
            + b"\r\n"
        )
