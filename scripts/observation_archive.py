"""Append each display point once, as XYZ float32 + acquisition time float64.

observed_points.bin is little endian, 20 bytes per record, with no header.
The format identifier is stored in each lightweight frame and events.json.
No cumulative point arrays or raw scan arrays are stored in frame files.
"""
from types import SimpleNamespace
from pathlib import Path
import os

import numpy as np


POINT_FORMAT = "xyz_first_seen_v1"
POINT_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("first_seen", "<f8")])


class PointTimelineWriter:
    def __init__(self, path):
        self.path = Path(path)
        self.path.write_bytes(b"")
        self.count = 0
        self.closed = False

    def append(self, cloud):
        if self.closed:
            raise ValueError("Point timeline is closed")
        count = len(cloud.positions)
        if count <= self.count:
            return
        records = np.empty(count-self.count, dtype=POINT_DTYPE)
        records["xyz"] = cloud.positions[self.count:]
        records["first_seen"] = cloud.first_stamps[self.count:]
        # Open each append against the current path. A replaced file must not
        # leave subsequent measurements writing to an unlinked old descriptor.
        # Only new records are written; closing publishes each tiny batch.
        with self.path.open("ab") as stream:
            if os.fstat(stream.fileno()).st_size != self.count*POINT_DTYPE.itemsize:
                raise ValueError("Point timeline length changed during recording")
            data = records.tobytes()
            if stream.write(data) != len(data):
                raise OSError("Incomplete point timeline write")
        self.count = count

    def flush(self):
        if self.path.stat().st_size != self.count*POINT_DTYPE.itemsize:
            raise ValueError("Point timeline length does not match the recorded count")

    def close(self):
        self.flush()
        self.closed = True


class PointTimeline:
    def __init__(self, path):
        if path.stat().st_size % POINT_DTYPE.itemsize:
            raise ValueError("Incomplete point timeline record")
        self.records = (np.memmap(path, dtype=POINT_DTYPE, mode="r")
                        if path.stat().st_size else np.empty(0, dtype=POINT_DTYPE))

    def at(self, stamp, since, scan_stamps):
        observed = self.records["first_seen"] <= stamp
        new = observed & (self.records["first_seen"] > since)
        return SimpleNamespace(points=self.records["xyz"][observed],
                               current=self.records["xyz"][new],
                               stamps=dict(enumerate(scan_stamps)))
