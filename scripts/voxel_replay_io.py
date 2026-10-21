#!/usr/bin/env python3
"""Fine-voxel replay of worlds/classroom.sdf.xacro (not a Gazebo recording)."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image

from classroom_voxel_scene import DEFAULT_CONFIG, DEFAULT_WORLD, ROOT
from voxel_exploration_core import OCCUPIED, UNKNOWN, VoxelMap
from observation_visualization import MapImage, ObservationRenderer
from observation_archive import POINT_FORMAT, PointTimeline
from voxel_native import library


def lidar_vectors(max_range=None):
    model = ET.parse(ROOT / "config/gz_waffle_3d_lidar.sdf.xacro")
    ray = model.find(".//sensor[@name='tb3_3d_lidar']/ray")
    angles = []
    for axis in ("vertical", "horizontal"):
        item = ray.find(f"scan/{axis}")
        angles.append(np.linspace(float(item.findtext("min_angle")), float(item.findtext("max_angle")),
                                  int(item.findtext("samples"))))
    elevation, azimuth = np.meshgrid(*angles, indexing="ij")
    hardware_range = float(ray.findtext("range/max"))
    distance = hardware_range if max_range is None else float(max_range)
    if not 0 < distance <= hardware_range:
        raise ValueError(f"Effective 3D range must be in (0, {hardware_range}] m")
    return distance * np.column_stack((
        (np.cos(elevation) * np.cos(azimuth)).ravel(),
        (np.cos(elevation) * np.sin(azimuth)).ravel(), np.sin(elevation).ravel()))


def ray_returns(truth, origin, vectors):
    origin = np.asarray(origin, dtype=np.float64)
    endpoints = np.ascontiguousarray(origin + vectors)
    native = library()
    hit_ids = np.full(len(endpoints), -1, dtype=np.int64)
    if native is not None:
        native.voxel_raycast(truth.state, truth.shape_xyz, truth.origin, truth.resolution,
                             origin, endpoints, len(endpoints), hit_ids)
    else:
        active = np.ones(len(endpoints), dtype=bool)
        for rays, ids, inside, _ in truth._walk(origin, endpoints, active):
            hit = inside & (truth.state.ravel()[ids] == OCCUPIED) & active[rays]
            hit_ids[rays[hit]] = ids[hit]
            active[rays[hit]] = False
    hit = hit_ids >= 0
    centers = truth.centers[hit_ids[hit]]
    low, high = centers - truth.resolution / 2, centers + truth.resolution / 2
    with np.errstate(divide="ignore", invalid="ignore"):
        a, b = (low - origin) / vectors[hit], (high - origin) / vectors[hit]
    # Parallel components impose no entry distance; avoid 0/0 at cell faces.
    entry = np.maximum(0., np.where(np.abs(vectors[hit]) > 1e-15,
                                    np.minimum(a, b), -np.inf).max(axis=1))
    indices = np.flatnonzero(hit)
    # A voxel touched only beyond the finite ray end is not a return.
    valid = entry < 1.-1e-8
    hit[indices[~valid]] = False
    endpoints[indices[valid]] = origin + vectors[indices[valid]] * (entry[valid, None] + 1e-8)
    return endpoints, hit


def scan_pose(xy, offset, height, vectors, yaw=0.):
    if yaw == 0.:
        return np.array([xy[0]+offset[0], xy[1]+offset[1], height], dtype=float), np.ascontiguousarray(vectors)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    origin = np.array([*xy[:2], height], dtype=float) + rotation @ np.array([*offset, 0.])
    return origin, np.ascontiguousarray(vectors @ rotation.T)


def observe(truth, known, xy, sensor, vectors, cloud=None, robot=0, stamp=0., yaw=0.):
    origin, directions = scan_pose(xy, (sensor.offset_x, sensor.offset_y), sensor.height, vectors, yaw)
    endpoints, hit = ray_returns(truth, origin, directions)
    valid = np.linalg.norm(endpoints-origin, axis=1) >= sensor.min_range
    if cloud is not None:
        cloud.add(robot, endpoints[hit & valid], stamp)
    before = int(np.count_nonzero(known.state == UNKNOWN))
    known.integrate(origin, endpoints[valid], hit[valid])
    return before - int(np.count_nonzero(known.state == UNKNOWN))



class PlanarObservations:
    """Independent observed 2D map; never copies the planner's ground prior."""

    def __init__(self, truth, sensor, max_range=None, profile=None):
        if profile is not None:
            self.height, self.offset = sensor.height, (sensor.offset_x, sensor.offset_y)
            self.max_range = sensor.max_range
            self.vectors = profile.vectors(sensor.max_range, planar=True)
            self.map = VoxelMap([*truth.bounds[:4], self.height-truth.resolution/2,
                                 self.height+truth.resolution/2], truth.resolution)
            self.stamp = 0.
            return
        model = ET.parse(ROOT / "config/gz_waffle_3d_lidar.sdf.xacro")
        planar = model.find(".//sensor[@name='hls_lfcd_lds']")
        spatial = model.find(".//sensor[@name='tb3_3d_lidar']")
        pose2 = np.fromstring(planar.findtext("pose"), sep=" ")
        pose3 = np.fromstring(spatial.findtext("pose"), sep=" ")
        self.height = sensor.height+pose2[2]-pose3[2]
        self.offset = pose2[:2]
        scan = planar.find("ray/scan/horizontal")
        angle = np.linspace(float(scan.findtext("min_angle")), float(scan.findtext("max_angle")),
                            int(scan.findtext("samples")))
        distance = float(planar.findtext("ray/range/max"))
        if max_range is not None:
            if not 0 < max_range <= distance:
                raise ValueError(f"Effective 2D range must be in (0, {distance}] m")
            distance = float(max_range)
        self.max_range = distance
        self.vectors = distance*np.column_stack((np.cos(angle), np.sin(angle), np.zeros(len(angle))))
        self.map = VoxelMap([*truth.bounds[:4], self.height-truth.resolution/2,
                             self.height+truth.resolution/2], truth.resolution)
        self.stamp = 0.

    def observe(self, truth, xy, stamp, yaw=0.):
        origin, vectors = scan_pose(xy, self.offset, self.height, self.vectors, yaw)
        endpoints, hit = ray_returns(truth, origin, vectors)
        self.map.integrate(origin, endpoints, hit)
        self.stamp = float(stamp)

    def image(self):
        data = self.map.state[0].copy()
        data[data == OCCUPIED] = 100
        return MapImage(data, self.map.resolution, (*self.map.origin[:2], 0.), self.stamp)


def save_frame(output, renderer, grid, cloud, poses, plan, active, coverage, event, elapsed,
               status, plan_stamp, number, unknown, sensor_range=None, scan_hz=5.,
               classification_fresh=True, new_points_since=-1., sensor=None, lidar_metadata=None,
               cycle_phase=None, execution_metadata=None):
    """Save small per-frame state; the separate point timeline supplies geometry."""
    stats = {"unknown": unknown, "excluded": len(coverage.excluded), "makespan_seconds": plan.makespan,
             "actionable": len(coverage.actionable), "deferred": len(coverage.deferred),
             "classification_fresh": classification_fresh}
    stats.update(target_policy=getattr(coverage, "target_policy", "exhaustive"),
                 planning_stats=getattr(coverage, "planning_stats", {}),
                 selection_phase=getattr(coverage, "selection_phase", "all_targets"),
                 ignored_unknown=len(getattr(coverage, "ignored", ())),
                 selected_views=len(getattr(coverage, "candidates", ())),
                 frontier_candidates=getattr(coverage, "frontier_candidates", 0),
                 available_candidates=getattr(coverage, "available_candidates", 0))
    if sensor_range is not None:
        stats.update(sensor_range=sensor_range, scan_hz=scan_hz)
    if sensor is not None:
        stats.update(sensor_offset=[sensor.offset_x, sensor.offset_y], sensor_height=sensor.height)
    if lidar_metadata is not None:
        stats["lidar_profile"] = lidar_metadata
    if cycle_phase is not None:
        stats["cycle_phase"] = cycle_phase
    if execution_metadata is not None:
        stats["execution"] = execution_metadata
    params = dict(event=event, stats=stats, route_stamp=plan_stamp, pose_stamps=[elapsed, elapsed],
                  active_paths=active, status=status, highlight_label="new points")
    display = SimpleNamespace(points=cloud.points,
                              current=cloud.points[cloud.first_seen > new_points_since],
                              stamps=cloud.stamps)
    image = renderer.render(grid, display, poses, plan.paths, elapsed, **params)
    folder = output / "frames"
    folder.mkdir(exist_ok=True)
    np.savez_compressed(
        folder / f"frame_{number:04d}.npz", bounds=renderer.bounds, grid=grid.data,
        resolution=grid.resolution, origin=grid.origin, map_stamp=grid.stamp,
        point_format=POINT_FORMAT, new_points_since=new_points_since,
        scan_stamps=[cloud.stamps.get(r, 0.) for r in range(2)],
        poses=np.asarray(poses), route0=np.asarray(plan.paths[0]).reshape(-1, 2),
        route1=np.asarray(plan.paths[1]).reshape(-1, 2), active0=np.asarray(active[0]).reshape(-1, 2),
        active1=np.asarray(active[1]).reshape(-1, 2), elapsed=elapsed,
        metadata=json.dumps({k: v for k, v in params.items() if k != "active_paths"}))
    return Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))


def export_frames(frames, output):
    frames[-1].save(output / "final.png")
    frames[-1].save(output / "latest.png")
    frames[0].save(output / "initial.png")
    frames[min(6, len(frames)-1)].save(output / "replanning.png")
    frames[0].save(output / "exploration.gif", save_all=True, append_images=frames[1:]+[frames[-1]]*3,
                   duration=700, loop=0, optimize=False)
    writer = cv2.VideoWriter(str(output / "exploration.mp4"), cv2.VideoWriter_fourcc(*"avc1"),
                             10, (ObservationRenderer.width, ObservationRenderer.height))
    if not writer.isOpened():
        raise RuntimeError("H.264 video writer unavailable; GIF and snapshots were saved")
    try:
        for frame in frames+[frames[-1]]*3:
            bgr = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)
            for _ in range(7):
                writer.write(bgr)
    finally:
        writer.release()


def render_saved(output):
    report_path = output/"events.json"
    report = json.loads(report_path.read_text()) if report_path.exists() else {}
    if "record_period" in report:
        from continuous_voxel_replay import VideoOutput
        video = VideoOutput(output, report["record_period"], report.get("playback_speed", 1.))
    else:
        video = None
    frames = []
    frame = None
    timeline = None
    replan_saved = False
    try:
        for path in sorted((output / "frames").glob("frame_*.npz")):
            with np.load(path, allow_pickle=False) as data:
                elapsed = float(data["elapsed"])
                if "point_format" in data:
                    if str(data["point_format"]) != POINT_FORMAT:
                        raise ValueError("Unsupported point timeline format")
                    if timeline is None:
                        timeline = PointTimeline(output/"observed_points.bin")
                    cloud = timeline.at(elapsed, float(data["new_points_since"]), data["scan_stamps"])
                else:  # Older recordings remain readable.
                    cloud = SimpleNamespace(points=data["points"], current=data["current"],
                                            stamps=dict(enumerate(data["scan_stamps"])))
                grid = MapImage(data["grid"], float(data["resolution"]), data["origin"], float(data["map_stamp"]))
                scene_name = Path(report.get("world", "indoor")).name.split('.')[0].replace('_', ' ')
                renderer = ObservationRenderer(data["bounds"], float(data["resolution"]), scene_name)
                params = json.loads(str(data["metadata"]))
                if report.get("observation_model") == "lidar360":
                    profile = report["lidar_profile"]
                    params["stats"].setdefault("lidar_profile", profile)
                    params["stats"].setdefault("sensor_offset", profile.get("sensor_offset", [0., 0.]))
                    params["stats"].setdefault("sensor_height", profile.get("sensor_height", .22))
                image = renderer.render(grid, cloud, data["poses"], [data["route0"], data["route1"]],
                                        elapsed, active_paths=[data["active0"], data["active1"]],
                                        **params)
            frame = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            if not replan_saved and elapsed > 0 and params.get("stats", {}).get("execution", {}).get("planning"):
                frame.save(output/"replanning.png")
                replan_saved = True
            if video:
                video.append(frame, elapsed)
            else:
                frames.append(frame)
        if frame is None:
            raise ValueError("No recorded frames; first run the demo without --render-only")
        if video:
            video.close(frame, elapsed)
        else:
            export_frames(frames, output)
    finally:
        if video:
            video.writer.release()
    return 0


def run(args):
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    if args.render_only:
        return render_saved(output)
    from continuous_voxel_replay import run_continuous
    return run_continuous(args)


def parse_args(argv=None, defaults=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--resolution", type=float, default=None)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "maps/classroom_voxel_5cm_long")
    parser.add_argument("--duration", type=float, default=0., help="Optional simulation time cap; 0 (default) runs until exploration completes")
    parser.add_argument("--max-events", type=int, default=None, help="Optional extra stopping limit; default has no event cap")
    parser.add_argument("--sensor-range", type=float, default=None, help="Effective 2D/3D range, no greater than configured hardware")
    parser.add_argument("--scan-hz", type=float, default=5.)
    parser.add_argument("--record-period", type=float, default=1., help="Seconds of simulated time between saved visual frames")
    parser.add_argument("--simulation-rate", type=float, default=1., help="Sim seconds per wall second; 0 runs unpaced")
    parser.add_argument("--playback-speed", type=float, default=1., help="Video playback factor; 1 preserves actual simulation duration")
    parser.add_argument("--render-only", action="store_true", help="Restyle saved frame inputs without replanning")
    parser.set_defaults(**(defaults or {}))
    args = parser.parse_args(argv)
    if args.max_events is not None and args.max_events < 0:
        parser.error("--max-events must be nonnegative")
    if args.duration < 0 or args.scan_hz <= 0 or args.record_period <= 0 or args.simulation_rate < 0 or args.playback_speed <= 0:
        parser.error("Invalid duration, scan frequency, record period or playback speed")
    return args
