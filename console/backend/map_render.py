"""OccupancyGrid -> PNG rendering for the browser map view.

The merged map is 900x560 cells at 0.05 m in the warehouse and about
640x440 in the mine.  Sending the raw int8 grid as JSON would be ~1.5 MB per
frame, so the console ships a PNG and keeps the geometry in ``/api/map/meta``.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


# BGR, matching the console's dark "mine control room" palette.
UNKNOWN_BGR = (18, 13, 10)      # RGB 10,13,18
FREE_BGR = (74, 56, 38)         # RGB 38,56,74
OCCUPIED_BGR = (246, 238, 226)  # RGB 226,238,246


def grid_to_png(msg: Any) -> bytes:
    width = int(msg.info.width)
    height = int(msg.info.height)
    if width <= 0 or height <= 0:
        raise ValueError("empty occupancy grid")

    data = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(height, width)
    image = np.empty((height, width, 3), dtype=np.uint8)
    image[:] = UNKNOWN_BGR

    free = data == 0
    image[free] = FREE_BGR

    occupied = data >= 65
    image[occupied] = OCCUPIED_BGR

    # Partially known cells form a soft grey rim around obstacles.
    mid = (data > 0) & (data < 65)
    if mid.any():
        grey = (60 + data[mid].astype(np.int16) * 2).clip(0, 255).astype(np.uint8)
        image[mid] = np.stack((grey, grey, grey), axis=-1)

    # OccupancyGrid row 0 is the bottom of the map; images are top-down.
    image = np.ascontiguousarray(image[::-1])
    ok, buffer = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise RuntimeError("PNG encode failed")
    return buffer.tobytes()


def world_to_px(x: float, y: float, info: dict[str, Any]) -> tuple[float, float]:
    resolution = float(info["resolution"])
    origin_x, origin_y, _ = info["origin"]
    return (x - origin_x) / resolution, float(info["height"]) - (y - origin_y) / resolution


def px_to_world(px: float, py: float, info: dict[str, Any]) -> tuple[float, float]:
    resolution = float(info["resolution"])
    origin_x, origin_y, _ = info["origin"]
    return origin_x + px * resolution, origin_y + (float(info["height"]) - py) * resolution
