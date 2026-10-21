"""Recover the requested 160 s prefix without running the planner again."""
from datetime import datetime, timezone
from pathlib import Path
import csv
import hashlib
import json
import math
import shutil
import statistics

import numpy as np

from classroom_voxel_scene import ROOT, scene
from continuous_voxel_replay import Motion
from voxel_replay_io import PlanarObservations, render_saved
from lidar360_observation import Lidar360
from observation_archive import POINT_DTYPE, POINT_FORMAT
from voxel_exploration_core import OCCUPIED


source = ROOT / "maps/classroom_voxel360_frontier_1500s"
output = ROOT / "maps/classroom_voxel360_frontier_160s"
cutoff = 160.0
manifest = json.loads((source / "run_manifest.json").read_text())
for relative, expected in manifest["source_sha256"].items():
    actual = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
    assert actual == expected, f"Source changed: {relative}"

entries = [
    json.loads(line)
    for line in (source / "run.log").read_text().splitlines()
    if line.startswith("{")
]
initial = next(e for e in entries if "initial_planning_seconds" in e)
progress = [e for e in entries if "distance_m" in e and e["time"] <= cutoff]
exact = next(e for e in progress if e["time"] == cutoff)
plans = [e for e in entries if "replan" in e and e["time"] <= cutoff]
assert len(plans) + 1 == exact["replans"]
frames = []
last_recorded = 0.0
for path in sorted((source / "frames").glob("frame_*.npz")):
    with np.load(path, allow_pickle=False) as data:
        stamp = float(data["elapsed"])
        last_recorded = max(last_recorded, stamp)
        if stamp <= cutoff:
            frames.append(path)
assert frames and not output.exists(), "Do not overwrite an existing recovery."
output.mkdir()
(output / "frames").mkdir()
for path in frames:
    shutil.copy2(path, output / "frames" / path.name)
for name in ("run_manifest.json", "source_snapshot.tar.gz"):
    shutil.copy2(source / name, output / ("source_" + name if name.endswith(".json") else name))
shutil.copy2(source / "run.log", output / "source_run.log")
shutil.copy2(__file__, output / "recover_160s.py")

records = np.fromfile(source / "observed_points.bin", dtype=POINT_DTYPE)
records = records[records["first_seen"] <= cutoff]
assert len(records) and np.all(np.diff(records["first_seen"]) >= 0)
records.tofile(output / "observed_points.bin")

with np.load(frames[-1], allow_pickle=False) as data:
    final = {name: data[name].copy() for name in data.files}
metadata = json.loads(str(final["metadata"]))
base_stamp = float(final["elapsed"])
assert base_stamp == plans[-1]["time"] == 159.9
assert metadata["stats"]["cycle_phase"] == "EXECUTING"
assert metadata["route_stamp"] == base_stamp

truth, known, ground, starts, configured_sensor, boxes, config = scene(
    ROOT / "config/voxel360_planner_params.yaml"
)
profile = Lidar360.load(config["lidar_profile"])
sensor = profile.sensor(float(metadata["stats"]["sensor_range"]))
planar = PlanarObservations(truth, sensor, sensor.max_range, profile)
planar.map.state[0] = np.where(final["grid"] == 100, OCCUPIED, final["grid"])
planar.stamp = base_stamp

# The previous frame was captured immediately after accepting this plan.
# Replay only its next 0.1 s step and the two planar scans, using the unchanged
# scene and motion code. All 3D returns are already in the timestamped archive.
poses = []
route_times = []
for r in range(2):
    old_pose = final["poses"][r]
    path = final[f"active{r}"].tolist()
    motion = Motion(tuple(old_pose[:2]), float(old_pose[2]), config["robot_speeds"][r])
    motion.path = path
    motion.route = [0]
    motion.length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    motion.observation_yaw = None
    arrived = motion.step(base_stamp, cutoff - base_stamp, 0.0)
    assert not arrived
    assert abs(motion.distance - 0.022) < 1e-10
    poses.append([*motion.xy, motion.yaw])
    planar.observe(truth, motion.xy, cutoff, 0.0)
    original_route = final[f"route{r}"]
    assert np.allclose(original_route[:len(path)], path)
    route = np.concatenate([
        np.asarray(ground.path(motion.xy, path[-1])),
        original_route[len(path):],
    ])
    final[f"route{r}"] = route
    length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    route_times.append(length / motion.speed
                       + plans[-1]["remaining_tasks"][r] * config["observation_time"])

image = planar.image()
final.update(
    poses=np.asarray(poses), grid=image.data, map_stamp=np.asarray(cutoff),
    elapsed=np.asarray(cutoff), scan_stamps=np.asarray([cutoff, cutoff]),
    new_points_since=np.asarray(base_stamp),
)
metadata.update(
    pose_stamps=[cutoff, cutoff],
    status="USER STOP AT 160s / exploration unfinished",
)
metadata["stats"].update(unknown=exact["unknown"], makespan_seconds=max(route_times),
                         classification_fresh=False)
final["metadata"] = np.asarray(json.dumps(metadata))
last_index = int(frames[-1].stem.split("_")[-1]) + 1
np.savez_compressed(output / "frames" / f"frame_{last_index:04d}.npz", **final)

planning_seconds = [initial["initial_planning_seconds"]] + [p["planning_seconds"] for p in plans]
voxel_count = int(truth.state.size)
summary = dict(
    simulated_seconds=cutoff,
    complete=False,
    stop_reason="user_requested_160s_prefix",
    last_original_recorded_seconds=last_recorded,
    source_directory=str(source),
    sensor_replay_not_gazebo=True,
    world=str(ROOT / "worlds/classroom.sdf.xacro"),
    voxel_resolution=float(truth.resolution),
    voxel_count=voxel_count,
    initial_unknown=initial["unknown"],
    final_unknown=exact["unknown"],
    newly_known_voxels=initial["unknown"] - exact["unknown"],
    initial_unknown_resolved_percent=100 * (1 - exact["unknown"] / initial["unknown"]),
    known_roi_percent=100 * (1 - exact["unknown"] / voxel_count),
    known_2d_cells=int(np.count_nonzero(image.data >= 0)),
    total_2d_cells=int(image.data.size),
    known_2d_percent=100 * float(np.mean(image.data >= 0)),
    robot_distance_m=exact["distance_m"],
    total_distance_m=sum(exact["distance_m"]),
    robot_poses=poses,
    arrival_events=exact["events"],
    total_plans_including_initial=exact["replans"],
    arrival_replans=len(plans),
    last_plan_stamp=base_stamp,
    last_plan_selection_phase=metadata["stats"]["selection_phase"],
    ignored_small_unknown_at_last_plan=metadata["stats"]["ignored_unknown"],
    ignored_small_unknown_stamp=base_stamp,
    classification_fresh=False,
    observable_unknown_remaining=None,
    observable_unknown_note="Not audited at cutoff; final_unknown includes all raw unknowns.",
    target_policy="frontier",
    replan_policy="arrival_cycle",
    planning_time_model="instantaneous",
    planning_wall_seconds=sum(planning_seconds),
    planning_mean_seconds=statistics.mean(planning_seconds),
    planning_median_seconds=statistics.median(planning_seconds),
    planning_max_seconds=max(planning_seconds),
    sensor_range=sensor.max_range,
    scan_hz=profile.scan_hz,
    scheduled_scans_per_robot_including_initial=int(round(cutoff * profile.scan_hz)) + 1,
    observation_model="lidar360",
    lidar_profile=profile.metadata(),
    point_format=POINT_FORMAT,
    point_record_bytes=POINT_DTYPE.itemsize,
    saved_points=len(records),
    point_storage_bytes=int(records.nbytes),
    record_period=manifest["record_period"],
    playback_speed=manifest["playback_speed"],
    gif_is_accelerated_preview=True,
    recovery=dict(
        sources_hash_verified=True,
        progress_source="Exact original 160.0 s log entry",
        point_source="Original archive filtered by first_seen <= 160.0",
        final_pose_and_2d_source="Replay 159.9 -> 160.0 from newly accepted saved path",
        final_3d_occupancy_snapshot_available=False,
        original_stop_time_is_not_160=True,
    ),
)
for filename in ("events.json", "summary_160s.json"):
    (output / filename).write_text(json.dumps(summary, indent=2) + "\n")
(source / "stop_status.json").write_text(json.dumps(dict(
    stopped=True,
    reason="User requested stopping at 160 s; background was already beyond cutoff",
    last_recorded_simulated_seconds=last_recorded,
    requested_cutoff_seconds=cutoff,
    recovered_output=str(output),
    status_recorded_at_utc=datetime.now(timezone.utc).isoformat(),
), indent=2) + "\n")

with (output / "progress_10s.csv").open("w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["simulated_seconds", "unknown_voxels", "known_roi_percent",
                     "arrival_events", "plans_including_initial",
                     "robot1_distance_m", "robot2_distance_m"])
    writer.writerow([0, initial["unknown"], 100 * (1 - initial["unknown"] / voxel_count),
                     0, 1, 0, 0])
    for entry in progress:
        writer.writerow([entry["time"], entry["unknown"],
                         100 * (1 - entry["unknown"] / voxel_count),
                         entry["events"], entry["replans"], *entry["distance_m"]])
(output / "planning_log.json").write_text(json.dumps(
    {"initial": initial, "accepted_replans": plans}, indent=2) + "\n")
print(json.dumps(summary, indent=2), flush=True)
render_saved(output)
print(f"Rendered {output}", flush=True)
