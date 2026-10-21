"""Shared rendering of measured maps/clouds, current poses and planned routes.

This module has no scene geometry or planner dependency. All visible surfaces
must come from the supplied measurements, never from the simulator world.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class MapImage:
    data: np.ndarray                 # OccupancyGrid convention: -1 / 0..100
    resolution: float
    origin: tuple                   # x, y, yaw in world
    stamp: float


class ObservedPoints:
    """Keep the first real return and its acquisition time per display voxel."""

    def __init__(self, resolution=.05):
        self.resolution = float(resolution)
        self.cells = {}
        self.positions, self.first_stamps = [], []
        self.latest = {}
        self.stamps = {}
        self._array = np.empty((0, 3), dtype=np.float32)
        self._times = np.empty(0, dtype=np.float64)
        self._dirty = False

    def add(self, robot, points, stamp):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        self.latest[robot], self.stamps[robot] = points.copy(), float(stamp)
        keys = np.floor(points / self.resolution).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        for i in indices:
            key = tuple(keys[i])
            if key not in self.cells:
                self.cells[key] = len(self.positions)
                self.positions.append(points[i].copy())
                self.first_stamps.append(float(stamp))
                self._dirty = True

    @property
    def points(self):
        if self._dirty:
            self._array = np.asarray(self.positions, dtype=np.float32).reshape(-1, 3)
            self._times = np.asarray(self.first_stamps, dtype=np.float64)
            self._dirty = False
        return self._array

    @property
    def first_seen(self):
        self.points  # Refresh the paired caches together.
        return self._times

    @property
    def current(self):
        return np.concatenate(list(self.latest.values())) if self.latest else np.empty((0, 3))


def remaining_path(path, xy):
    """Clip the first travelled portion at the closest segment (display only)."""
    p = np.asarray(path, dtype=float).reshape(-1, 2)
    if len(p) < 2 or xy is None:
        return p
    d = p[1:] - p[:-1]
    lengths = np.sum(d*d, axis=1)
    t = np.clip(np.sum((np.asarray(xy[:2])-p[:-1])*d, axis=1) / np.maximum(lengths, 1e-20), 0, 1)
    projections = p[:-1] + t[:, None]*d
    i = int(np.argmin(np.sum((projections-np.asarray(xy[:2]))**2, axis=1)))
    # A localization jump must not create a fictitious straight path to the route.
    return np.vstack((projections[i], p[i+1:]))


class ObservationRenderer:
    width, height = 1600, 960
    robot_colors = ((85, 119, 255), (255, 204, 69))  # BGR
    map_palette = ((87, 80, 72), (222, 228, 232), (30, 32, 35))

    def __init__(self, bounds=(-8, 8, -6, 6, .05, 2.85), voxel_resolution=.05, scene_name="INDOOR"):
        self.bounds = np.asarray(bounds, dtype=float)
        self.voxel_resolution = float(voxel_resolution)
        self.background = (26, 22, 18)
        self.scene_name = str(scene_name).upper()

    @staticmethod
    def text(image, text, xy, size=.52, color=(222, 223, 227), weight=1):
        cv2.putText(image, str(text), xy, cv2.FONT_HERSHEY_SIMPLEX,
                    size, color, weight, cv2.LINE_AA)

    def topdown(self, xy):
        p = np.asarray(xy, dtype=float).reshape(-1, 2)
        b = self.bounds
        return np.column_stack((32+(p[:, 0]-b[0])/(b[1]-b[0])*736,
                                714-(p[:, 1]-b[2])/(b[3]-b[2])*552)).round().astype(np.int32)

    def project(self, xyz):
        p = np.asarray(xyz, dtype=float).reshape(-1, 3).copy()
        b = self.bounds
        p[:, 0] -= (b[0]+b[1])/2
        p[:, 1] -= (b[2]+b[3])/2
        scale = min(690/((b[1]-b[0])+.65*(b[3]-b[2])), 420/(
            .42*(b[1]-b[0])+.65*(b[3]-b[2])))
        return np.column_stack((1190+(p[:, 0]+.65*p[:, 1])*scale,
                                484+(.42*p[:, 0]-.65*p[:, 1])*scale-p[:, 2]*scale*1.65)
                               ).round().astype(np.int32)

    def measured_map(self, grid):
        """Resample a possibly rotated ROS grid to the common world viewport."""
        if grid is None:
            return np.full((552, 736), -1, dtype=np.int8)
        b = self.bounds
        x = b[0]+(np.arange(736)+.5)*(b[1]-b[0])/736
        y = b[3]-(np.arange(552)+.5)*(b[3]-b[2])/552
        xx, yy = np.meshgrid(x-grid.origin[0], y-grid.origin[1])
        c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
        cols = np.floor((c*xx+s*yy)/grid.resolution).astype(int)
        rows = np.floor((-s*xx+c*yy)/grid.resolution).astype(int)
        inside = (cols >= 0) & (cols < grid.data.shape[1]) & (rows >= 0) & (rows < grid.data.shape[0])
        data = np.full(rows.shape, -1, dtype=np.int8)
        data[inside] = grid.data[rows[inside], cols[inside]]
        return data

    def draw_cloud(self, image, points, current=False):
        points = np.asarray(points).reshape(-1, 3)
        if not len(points):
            return
        order = np.argsort(-points[:, 0]+points[:, 1])
        points = points[order]
        pixels = self.project(points)
        mask = (pixels[:, 0] > 810) & (pixels[:, 0] < 1570) & (pixels[:, 1] > 165) & (pixels[:, 1] < 720)
        pixels, points = pixels[mask], points[mask]
        t = np.clip(points[:, 2]/self.bounds[5], 0, 1)[:, None]
        colors = ((1-t)*np.array([205, 151, 75])+t*np.array([103, 205, 240])).astype(np.uint8)
        if current:
            colors = (colors*.45+np.array([245, 250, 255])*.55).astype(np.uint8)
        image[pixels[:, 1], pixels[:, 0]] = colors
        image[pixels[:, 1]+1, pixels[:, 0]] = colors

    def render(self, grid, cloud, poses, routes, elapsed, *, event=0, stats=None,
               route_stamp=None, pose_stamps=None, active_paths=None, status="",
               source="OFFLINE SENSOR REPLAY", map_source="measured planar LiDAR scans",
               highlight_label="latest scans"):
        stats = stats or {}
        image = np.full((self.height, self.width, 3), self.background, dtype=np.uint8)
        self.text(image, f"{self.scene_name} / CURRENT OBSERVATIONS", (32, 43), .88, (244, 244, 245), 2)
        profile = stats.get("lidar_profile")
        self.text(image, "OFFLINE 360 LIDAR REPLAY" if profile and source == "OFFLINE SENSOR REPLAY" else source,
                  (1040, 42), .56, (147, 203, 236))
        self.text(image, f"t = {elapsed:.2f} s    observation event {event}    world frame", (33, 79), .54)
        label = (f"360 deg / {profile['vertical_samples']} x {profile['horizontal_samples']} / {profile['scan_hz']:g} Hz"
                 if profile else f"{self.voxel_resolution*100:g} cm voxels / red: R1 / blue: R2")
        self.text(image, label, (1040, 79), .54)
        self.text(image, "2D / OBSERVED OCCUPANCY + ROBOT POSES + ROUTES", (32, 120), .52)
        self.text(image, "3D / OBSERVED POINT CLOUD + ROBOT POSES + ROUTES", (816, 120), .50)
        map_time = f"{grid.stamp:.2f} s" if grid is not None else "waiting"
        self.text(image, f"{map_source} | map stamp {map_time}", (32, 146), .43, (172, 178, 185))
        self.text(image, f"Accumulated returns; {highlight_label} highlighted / height color",
                  (816, 146), .43, (172, 178, 185))

        data = self.measured_map(grid)
        labels = np.where(data < 0, 0, np.where(data >= 50, 2, 1))
        image[162:714, 32:768] = np.asarray(self.map_palette, dtype=np.uint8)[labels]
        cv2.rectangle(image, (31, 161), (768, 714), (130, 133, 141), 1)
        if grid is None:
            self.text(image, "Waiting for observed /map", (180, 430), .65)
        # A neutral coordinate grid, not unmeasured classroom surfaces.
        b = self.bounds
        for x in np.arange(math.ceil(b[0]), b[1]+.01, 2):
            p = self.project([[x, b[2], 0], [x, b[3], 0]])
            cv2.line(image, tuple(p[0]), tuple(p[1]), (52, 48, 43), 1, cv2.LINE_AA)
        for y in np.arange(math.ceil(b[2]), b[3]+.01, 2):
            p = self.project([[b[0], y, 0], [b[1], y, 0]])
            cv2.line(image, tuple(p[0]), tuple(p[1]), (52, 48, 43), 1, cv2.LINE_AA)
        self.draw_cloud(image, cloud.points)
        self.draw_cloud(image, cloud.current, current=True)
        sensor_range = stats.get("sensor_range")
        if sensor_range:
            angles = np.linspace(0., 2*math.pi, 161)
            for r, pose in enumerate(poses):
                if pose is None:
                    continue
                yaw = pose[2] if len(pose) > 2 else 0.
                ox, oy = stats.get("sensor_offset", [-.064, 0.])
                origin = np.array([pose[0]+ox*math.cos(yaw)-oy*math.sin(yaw),
                                   pose[1]+ox*math.sin(yaw)+oy*math.cos(yaw)])
                ring = origin + sensor_range*np.column_stack((np.cos(angles), np.sin(angles)))
                color = tuple(int(c*.55) for c in self.robot_colors[r])
                panels = [(image[162:714, 32:768], self.topdown(ring)-[32, 162]),
                          (image[162:714, 810:1570], self.project(
                              np.column_stack((ring, np.full(len(ring), stats.get("sensor_height", .22)))))-[810, 162])]
                for panel, pixels in panels:
                    for i in range(0, len(pixels)-2, 4):
                        cv2.polylines(panel, [pixels[i:i+3]], False, color, 1, cv2.LINE_AA)
        for r in range(2):
            pose = poses[r]
            color = self.robot_colors[r]
            route = remaining_path(routes[r], pose)
            for points, is3d in ((route, False), (route, True)):
                if len(points) < 2:
                    continue
                pixels = self.project(np.column_stack((points, np.full(len(points), .09)))) if is3d else self.topdown(points)
                cv2.polylines(image, [pixels], False, tuple(int(c*.7) for c in color), 1, cv2.LINE_AA)
            if active_paths is not None:
                active = remaining_path(active_paths[r], pose)
                if len(active):
                    for pixels in (self.topdown(active), self.project(np.column_stack((active, np.full(len(active), .1))))):
                        cv2.polylines(image, [pixels], False, color, 3, cv2.LINE_AA)
                        cv2.drawMarker(image, tuple(pixels[-1]), color, cv2.MARKER_DIAMOND, 13, 2)
            if pose is not None:
                yaw = pose[2] if len(pose) > 2 else 0.
                heading = (pose[0]+.48*math.cos(yaw), pose[1]+.48*math.sin(yaw))
                for pixels in (self.topdown([pose[:2], heading]),
                               self.project([[*pose[:2], .20], [*heading, .20]])):
                    xy, tip = tuple(pixels[0]), tuple(pixels[1])
                    cv2.circle(image, xy, 10, (18, 20, 24), -1, cv2.LINE_AA)
                    cv2.circle(image, xy, 7, color, -1, cv2.LINE_AA)
                    cv2.arrowedLine(image, xy, tip, (250, 250, 250), 2, cv2.LINE_AA, tipLength=.35)
                    self.text(image, f"R{r+1}", (xy[0]+12, xy[1]-9), .51, (10, 10, 10), 3)
                    self.text(image, f"R{r+1}", (xy[0]+12, xy[1]-9), .51, color, 1)

        for i, label in enumerate(("Unknown", "Observed free", "Observed occupied")):
            x = 32+i*243
            cv2.rectangle(image, (x, 731), (x+12, 744), self.map_palette[i], -1)
            self.text(image, label, (x+20, 744), .46)
        self.text(image, f"{len(cloud.points):,} accumulated points / {len(cloud.current):,} {highlight_label}",
                  (816, 744), .49)
        self.text(image, "Thin line: remaining plan    Bold line + diamond: next observation", (32, 776), .47)
        scan_times = "  /  ".join(f"R{r+1}: {cloud.stamps.get(r, float('nan')):.2f}s" for r in range(2))
        self.text(image, f"Cloud acquisition: {scan_times}", (816, 776), .46)
        for r, pose in enumerate(poses):
            x = 32+r*784
            if pose is None:
                value = f"R{r+1}  position unavailable (waiting for TF)"
            else:
                yaw = math.degrees(pose[2]) if len(pose) > 2 else 0.
                value = f"R{r+1}   x {pose[0]:+.3f} m   y {pose[1]:+.3f} m   yaw {yaw:+.1f} deg"
                if pose_stamps and pose_stamps[r] is not None:
                    value += f"   @{pose_stamps[r]:.2f}s"
            self.text(image, value, (x, 818), .51, self.robot_colors[r])
        known_percent = 100*np.mean(grid.data >= 0) if grid is not None else 0.
        unknown = stats.get("unknown", 0)
        classification = "" if stats.get("classification_fresh", True) else " (updating)"
        self.text(image, f"2D observed {known_percent:.1f}%    3D unknown {unknown:,}", (32, 861), .50)
        self.text(image, f"Plan{classification}: actionable {stats.get('actionable', 0):,} / "
                  f"deferred {stats.get('deferred', 0):,} / excluded {stats.get('excluded', 0):,}",
                  (816, 861), .43)
        route_time = f"{route_stamp:.2f}s" if route_stamp is not None else "waiting"
        self.text(image, f"Plan stamp {route_time}    Estimate {stats.get('makespan_seconds', 0):.1f}s",
                  (32, 882), .43)
        if stats.get("target_policy") == "frontier":
            self.text(image, f"Phase: {stats.get('selection_phase', 'frontier')}  /  "
                      f"views {stats.get('selected_views', stats.get('candidate_views', 0))}  /  "
                      f"ignored small unknowns {stats.get('ignored_unknown', 0):,}", (816, 882), .43)
        self.text(image, status, (32, 903), .49, (161, 218, 176))
        note = ("Current pose and future plan are shown separately; blank 3D space has no surface evidence."
                if not sensor_range else
                f"Dashed rings: {sensor_range:g} m maximum range (occlusion applies) | "
                f"{stats.get('scan_hz', 5):g} Hz scans | Historical returns remain outside the current range.")
        self.text(image, note, (32, 936), .44, (155, 164, 177))
        return image
