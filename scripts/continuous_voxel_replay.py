"""Completion-driven replay with instantaneous or measured-latency planning."""
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field, replace
import hashlib
import json
import math
import time

import cv2
import numpy as np
from PIL import Image

from classroom_voxel_scene import scene
from voxel_motion import advance
from voxel_replay_io import (
    PlanarObservations, lidar_vectors, observe, save_frame, scan_pose,
)
from observation_visualization import ObservationRenderer, ObservedPoints
from observation_archive import POINT_DTYPE, POINT_FORMAT, PointTimelineWriter
from lidar360_observation import Lidar360, evaluate_candidates_360
from voxel_frontier_policy import FrontierPolicy
from voxel_fast_policy import FastFrontierEvaluator, FastPolicy
from voxel_async_execution import (AsyncV2Policy, AsyncV2Window, latency_deadline, rebase_plan,
                                   path_length, path_tail, queued_path)
from voxel_exploration_core import (
    UNKNOWN, CompletionMonitor, Coverage, GroundDistanceCache, GroundGrid, RoutePlan, coverage_exhausted,
    continue_route, evaluate_candidates, pending_route, solve_coverage_routes,
)

DISTANCE_CACHE = GroundDistanceCache()
FAST_EVALUATOR = FastFrontierEvaluator()

def compute_plan(snapshot, ground_data, resolution, origin, clearance, starts, sensor, config, blocked):
    begin = time.monotonic()
    ground = GroundGrid(ground_data, resolution, origin, clearance, distance_cache=DISTANCE_CACHE)
    if config.get("candidate_evaluation", "exhaustive") not in ("exhaustive", "incremental_frontier"):
        raise ValueError("Unknown candidate_evaluation strategy")
    if config.get("candidate_evaluation") == "incremental_frontier":
        if config.get("observation_model") != "lidar360" or config.get("target_policy") != "frontier":
            raise ValueError("Incremental frontier requires lidar360 and frontier targets")
        coverage = FAST_EVALUATOR.evaluate(
            snapshot, ground, starts, sensor, config["candidate_position_stride"],
            Lidar360.load(config["lidar_profile"]), blocked=blocked,
            policy=FrontierPolicy.from_config(config), fast=FastPolicy.from_config(config),
            speeds=config["robot_speeds"], observation_time=config["observation_time"])
    elif config.get("observation_model") == "lidar360":
        coverage = evaluate_candidates_360(snapshot, ground, starts, sensor, config["candidate_position_stride"],
                                           Lidar360.load(config["lidar_profile"]),
                                           ownership=config.get("candidate_ownership", "nearest"),
                                           audit_when_idle=config.get("audit_when_idle", True), blocked=blocked,
                                           target_policy=config.get("target_policy", "exhaustive"),
                                           frontier_policy=FrontierPolicy.from_config(config),
                                           speeds=config["robot_speeds"], observation_time=config["observation_time"])
    else:
        coverage = evaluate_candidates(snapshot, ground, starts, sensor, config["candidate_position_stride"])
    available = [c for c in coverage.candidates if c.cell not in blocked]
    active = set().union(*(c.coverage for c in available))
    coverage = replace(coverage, candidates=available, actionable=active,
                       deferred=coverage.deferred | (coverage.actionable-active))
    solve_begin = time.monotonic()
    plan = solve_coverage_routes(coverage, ground, starts, config["robot_speeds"],
                                 config["observation_time"], config["exact_candidate_limit"])
    if coverage.planning_stats:
        coverage.planning_stats.update(route_solver_seconds=time.monotonic()-solve_begin)
    return coverage, plan, time.monotonic()-begin


@dataclass
class Motion:
    xy: tuple
    yaw: float
    speed: float
    route: list = field(default_factory=list)
    path: list = field(default_factory=list)
    progress: float = 0.
    length: float = 0.
    finish_at: float = math.inf
    distance: float = 0.
    observations: int = 0
    observation_yaw: float | None = 0.

    def next_path(self, ground, coverage):
        self.path = ground.path(self.xy, coverage.candidates[self.route[0]].xy) if self.route else [self.xy]
        self.progress = 0.
        self.length = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
        self.finish_at = math.inf

    def step(self, now, dt, dwell):
        if not self.route:
            return False
        if not math.isfinite(self.finish_at):
            travelled = min(self.length-self.progress, dt*self.speed)
            self.distance += travelled
            self.progress += travelled
            before = self.xy
            self.xy = advance(self.path, self.progress)
            if math.dist(before, self.xy) > 1e-10:
                self.yaw = math.atan2(self.xy[1]-before[1], self.xy[0]-before[0])
            if self.progress >= self.length-1e-9:
                if self.observation_yaw is not None:
                    self.yaw = self.observation_yaw
                self.finish_at = now+travelled/self.speed+dwell
        return now+dt >= self.finish_at-1e-9


def apply_plan(motions, coverage, plan, ground, known, blocked, active_cells=None, *, task_event=False):
    """Rebase a delayed plan at live positions; never teleport to snapshot starts."""
    for r, motion in enumerate(motions):
        active = active_cells[r] if active_cells else None
        route = pending_route(plan.routes[r], coverage.candidates, known, blocked)
        motion.route = (route if task_event else continue_route(
            route, coverage.candidates, active, ground if active_cells is not None else None, motion.xy))
        if not motion.route or coverage.candidates[motion.route[0]].cell != active:
            motion.next_path(ground, coverage)


def prune_observed_routes(motions, coverage, ground, known):
    """Continuous scans can finish a task before arrival or for the other car."""
    skipped = []
    for motion in motions:
        previous = motion.route
        motion.route = pending_route(previous, coverage.candidates, known)
        skipped.append(len(previous)-len(motion.route))
        if previous[:1] != motion.route[:1]:
            motion.next_path(ground, coverage)
    return skipped


def wait_for_task_replan(motions, completed, ground, coverage):
    """No car starts a second task from the obsolete plan after an event."""
    for r, motion in enumerate(motions):
        motion.route = [] if r in completed else motion.route[:1]
        if not motion.route:
            motion.next_path(ground, coverage)


def step_until_arrival(motions, now, dt):
    """Advance both cars by the same time, stopping at the first actual arrival."""
    remaining = [max(0., m.length-m.progress)/m.speed for m in motions if m.route]
    if remaining:
        dt = min(dt, min(remaining))
    arrived = [r for r, m in enumerate(motions) if m.step(now, dt, 0.)]
    return dt, arrived


def stop_for_arrival_replan(motions, ground, coverage):
    """Hold both physical positions and discard the previous joint solution."""
    for motion in motions:
        motion.route = []
        motion.next_path(ground, coverage)


def live_plan(motions, coverage, ground, dwell, method):
    routes, paths, times = [], [], []
    for motion in motions:
        path, previous = [], motion.xy
        for rank, index in enumerate(motion.route):
            xy = coverage.candidates[index].xy
            segment = (path_tail(motion.path, motion.progress) if rank == 0 and motion.path
                       else ground.path(previous, xy))
            path.extend(segment if not path else segment[1:])
            previous = xy
        distance = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        routes.append(list(motion.route))
        paths.append(path)
        times.append(distance/motion.speed+len(motion.route)*dwell)
    return RoutePlan(routes, tuple(times), max(times), method, paths)


class VideoOutput:
    """Stream full-length video; keep only occasional frames for a GIF preview."""

    def __init__(self, output, record_period, playback_speed):
        self.output = output
        self.period, self.playback = record_period, playback_speed
        self.writer = cv2.VideoWriter(str(output/"exploration.mp4"), cv2.VideoWriter_fourcc(*"avc1"),
                                     10., (ObservationRenderer.width, ObservationRenderer.height))
        if not self.writer.isOpened():
            raise RuntimeError("H.264 writer unavailable")
        self.previous = None
        self.previous_time = 0.
        self.frames_written = 0
        self.preview, self.preview_time = [], -math.inf
        self.preview_period = 5.
        self.count = 0

    def append(self, image, stamp):
        if self.previous is not None:
            # Quantize cumulative time, avoiding accumulated frame-rounding drift.
            target_count = round(stamp/self.playback*10)
            for _ in range(max(0, target_count-self.frames_written)):
                self.writer.write(self.previous)
            self.frames_written = target_count
        self.previous = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        self.previous_time = stamp
        if stamp-self.preview_time >= self.preview_period-1e-8:
            self.preview.append(image.resize((1000, 600), Image.Resampling.LANCZOS))
            self.preview_time = stamp
            if len(self.preview) >= 240:
                self.preview = self.preview[::2]
                self.preview_period *= 2
        if self.count == 0:
            image.save(self.output/"initial.png")
        image.save(self.output/"latest.png")
        self.count += 1

    def close(self, final, duration):
        if not self.frames_written:
            self.writer.write(cv2.cvtColor(np.asarray(final), cv2.COLOR_RGB2BGR))
            self.frames_written = 1
        self.writer.release()
        final.save(self.output/"final.png")
        final.save(self.output/"latest.png")
        if not self.preview or self.preview_time < duration-1e-6:
            self.preview.append(final.resize((1000, 600), Image.Resampling.LANCZOS))
        self.preview[0].save(self.output/"exploration.gif", save_all=True,
                             append_images=self.preview[1:], duration=350, loop=0)


def run_continuous(args):
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    for path in (output/"frames").glob("frame_*.npz"):
        path.unlink()
    truth, known, ground, starts, sensor, boxes, config = scene(args.config, args.world, args.resolution)
    ground.distance_cache = DISTANCE_CACHE
    limit = config["sensor_max_range"] if args.sensor_range is None else args.sensor_range
    profile = Lidar360.load(config["lidar_profile"]) if config.get("observation_model") == "lidar360" else None
    replan_policy = config.get("replan_policy", "arrival_cycle") if profile is not None else "continue_queue"
    if replan_policy not in ("arrival_cycle", "async_arrival", "async_v2", "task_event", "continue_queue"):
        raise ValueError("Unknown replan_policy")
    arrival_cycle_mode = replan_policy == "arrival_cycle"
    v2 = AsyncV2Window(AsyncV2Policy.from_config(config)) if replan_policy == "async_v2" else None
    async_arrival_mode = replan_policy in ("async_arrival", "async_v2")
    physical_arrival_mode = arrival_cycle_mode or async_arrival_mode
    task_event_mode = replan_policy in ("task_event", "arrival_cycle", "async_arrival", "async_v2")
    cycle_phase = "EXECUTING"
    last_arrival_time = -math.inf
    if profile is not None:
        if sensor != profile.sensor(sensor.max_range):
            raise ValueError("360 scene parameters must match lidar_profile range, FOV and centered mounting")
        sensor = profile.sensor(limit)
        if args.scan_hz != profile.scan_hz:
            raise ValueError("360 scan rate must match the configured LiDAR mode")
        vectors = profile.vectors(limit)
        planar = PlanarObservations(truth, sensor, limit, profile=profile)
    else:
        sensor = replace(sensor, max_range=limit)
        vectors = lidar_vectors(limit)  # Validates against the configured sensor limit.
        planar = PlanarObservations(truth, sensor, limit)
    cloud = ObservedPoints(known.resolution)
    motions = [Motion(tuple(p), float(config[f"robot{r+1}_init_pose"][2]), config["robot_speeds"][r])
               for r, p in enumerate(starts)]
    if profile is not None:
        for motion in motions:
            motion.observation_yaw = None
    renderer = ObservationRenderer(known.bounds, known.resolution, args.world.name.split('.')[0].replace('_', ' '))
    cooldowns, events, replans = {}, [], []
    skipped_observed = [0, 0]
    max_return = [0., 0.]
    scans = [0, 0]
    scan_stamps = [0., 0.]
    distance_while_planning = [0., 0.]
    idle_while_planning = [0., 0.]
    idle_total = [0., 0.]
    goal_switches = [0, 0]
    refill_events = []
    idle_search_events = []
    elapsed, event, next_record = 0., 0, 0.
    scan_period = 1/args.scan_hz
    next_scan_time = scan_period
    dwell = config["observation_time"]
    completion = CompletionMonitor(config["completion_stable_seconds"])
    idle_period = config["idle_replan_period"]

    def scan_all(stamp):
        progress = 0
        for r, motion in enumerate(motions):
            # A revolution is one position observation. The offline encoder's
            # phase is fixed to world axes; body yaw is used only for driving.
            scan_yaw = 0. if profile is not None else motion.yaw
            progress += observe(truth, known, motion.xy, sensor, vectors, cloud, r, stamp, scan_yaw)
            planar.observe(truth, motion.xy, stamp, scan_yaw)
            origin, _ = scan_pose(motion.xy, (sensor.offset_x, sensor.offset_y), sensor.height, vectors[:1], scan_yaw)
            if len(cloud.latest[r]):
                distance = float(np.linalg.norm(cloud.latest[r]-origin, axis=1).max())
                if distance > limit+1e-6:
                    raise RuntimeError("A measured return exceeded the effective LiDAR range")
                max_return[r] = max(max_return[r], distance)
            scans[r] += 1
            scan_stamps[r] = stamp
        return progress

    def blocked_views():
        return {cell for cell, until in cooldowns.items() if until > elapsed}

    def execution_paths():
        return [queued_path(ground, m.xy, [coverage.candidates[i].xy for i in m.route],
                            path_tail(m.path, m.progress)) for m in motions]

    def arguments(starts_override=None):
        return (known.snapshot(), ground.data.copy(), ground.resolution, ground.origin,
                config["min_clearance"], starts_override or [m.xy for m in motions], sensor, config, blocked_views())

    scan_all(0.)
    initial_unknown = len(known.unknown)
    cap = f"{args.duration:g}s safety cap" if args.duration else "no time cap"
    print(f"Initial plan: range={limit:g}m, stop=observable unknowns exhausted ({cap}), scans={args.scan_hz:g}Hz", flush=True)
    plan_args = arguments()
    coverage, plan, planning_seconds = compute_plan(*plan_args)
    exhausted = coverage_exhausted(plan_args[0], coverage)
    if not async_arrival_mode:
        apply_plan(motions, coverage, plan, ground, known, blocked_views(), task_event=task_event_mode)
    replans.append(dict(request_time=0., accepted_time=0., planning_seconds=planning_seconds))
    requests = [dict(request_id=0, event=0, request_time=0., accepted_time=0.,
                     status="accepted", planning_seconds=planning_seconds,
                     scan_times=[0., 0.], robot_positions=[m.xy for m in motions],
                     candidate_views=len(coverage.candidates), actionable=len(coverage.actionable),
                     selection_phase=coverage.selection_phase, ignored_unknown=len(coverage.ignored),
                     available_candidates=coverage.available_candidates,
                     frontier_candidates=coverage.frontier_candidates,
                     planning_stats=coverage.planning_stats,
                     claimed_voxels=sum(len(c.coverage) for c in coverage.candidates))]
    pending_request = None
    discarded_replans = 0
    print(json.dumps({"initial_planning_seconds": planning_seconds, "unknown": initial_unknown,
                      "tasks": [len(m.route) for m in motions]}), flush=True)
    plan_stamp = 0.
    accepted_revision = plan_args[0].revision
    pending_revision = None
    pending_snapshot = None
    last_request_time = 0.
    requested = False
    pending, request_time = None, 0.
    pending_result, ready_at, handoff_at = None, math.inf, math.inf
    worker = ThreadPoolExecutor(max_workers=1)
    if async_arrival_mode:
        # Offline discrete-event replay: measure a snapshot computation once,
        # then execute/scan the old queue for precisely that duration. This
        # stays correct even if rendering is slower than real time or unpaced.
        pending_result = coverage, plan, planning_seconds
        pending = Future()
        pending.set_result(pending_result)
        ready_at = latency_deadline(0., planning_seconds)
        handoff_at = ready_at
        if v2:
            v2.observe_latency(planning_seconds)
        pending_request = requests[0]
        pending_request.pop("accepted_time")
        pending_request.update(status="computing", ready_time=ready_at,
                               handoff_time=handoff_at, commit_until=0.,
                               distance_at_request=[0., 0.], distance_during_planning=[0., 0.])
        pending_revision, pending_snapshot = plan_args[0].revision, plan_args[0]
        accepted_revision, exhausted = -1, False
        replans.clear()
        cycle_phase = "PLANNING"
    video = VideoOutput(output, args.record_period, args.playback_speed)
    point_log = PointTimelineWriter(output/"observed_points.bin")
    point_log.append(cloud)
    start_wall = time.monotonic()
    next_progress = 10.
    frame_number = 0
    complete, stop_reason = False, None
    frame = None
    previous_frame_time = -1.
    captured_replan = 1
    captured_phase = None

    def record_frame(status):
        nonlocal frame, frame_number, next_record, previous_frame_time, captured_replan, captured_phase
        displayed_plan = live_plan(motions, coverage, ground, dwell, plan.method)
        point_log.append(cloud)
        point_log.flush()
        frame = save_frame(output, renderer, planar.image(), cloud,
                           [(*m.xy, m.yaw) for m in motions], displayed_plan,
                           [m.path for m in motions], coverage, event, elapsed, status,
                           plan_stamp, frame_number, len(known.unknown), limit, args.scan_hz,
                           classification_fresh=accepted_revision == known.revision and coverage.classification_complete,
                           new_points_since=previous_frame_time, sensor=sensor,
                           cycle_phase=cycle_phase if physical_arrival_mode else None,
                           execution_metadata=(dict(
                               mode=replan_policy, planning=pending is not None,
                               request_time=request_time if pending is not None else None,
                               ready_time=ready_at if pending is not None else None,
                               handoff_time=handoff_at if pending is not None else None,
                               source_revision=accepted_revision,
                               motion_during_planning_m=list(distance_while_planning))
                               if async_arrival_mode else None),
                           lidar_metadata=profile.metadata() if profile is not None else None)
        previous_frame_time = elapsed
        video.append(frame, elapsed)
        if frame_number == 1 or len(replans) > captured_replan:
            frame.save(output/"replanning.png")
            captured_replan = len(replans)
        frame_number += 1
        next_record = elapsed+args.record_period
        captured_phase = cycle_phase

    try:
        try:
            while True:
                if async_arrival_mode and pending is not None and pending_result is None:
                    # Host work does not define the motion clock. Schedule its
                    # measured latency, then replay moving scans up to ready_at.
                    wait_begin = time.monotonic()
                    pending_result = pending.result()
                    start_wall += time.monotonic()-wait_begin
                    ready_at = latency_deadline(request_time, pending_result[2])
                    handoff_at = max(ready_at, v2.commit_until) if v2 else ready_at
                    if v2:
                        v2.observe_latency(pending_result[2])
                    pending_request.update(ready_time=ready_at, handoff_time=handoff_at,
                                           planning_seconds=pending_result[2])
                if pending is not None and (arrival_cycle_mode
                        or (async_arrival_mode and elapsed >= handoff_at-1e-9)
                        or (not async_arrival_mode and pending.done())):
                    active_cells = [coverage.candidates[m.route[0]].cell if m.route else None for m in motions]
                    wait_begin = time.monotonic()
                    result = pending_result if async_arrival_mode else pending.result()
                    if arrival_cycle_mode:
                        # Solver latency belongs to wall time only. No motion,
                        # scan, completion timer or video frame advances here.
                        start_wall += time.monotonic()-wait_begin
                    pending = None
                    pending_result = None
                    pending_request["planning_seconds"] = result[2]
                    pending_request.update(candidate_views=len(result[0].candidates),
                                           actionable=len(result[0].actionable),
                                           selection_phase=result[0].selection_phase,
                                           ignored_unknown=len(result[0].ignored),
                                           available_candidates=result[0].available_candidates,
                                           frontier_candidates=result[0].frontier_candidates,
                                           planning_stats=result[0].planning_stats,
                                           claimed_voxels=sum(len(c.coverage) for c in result[0].candidates))
                    if task_event_mode and not async_arrival_mode and pending_request["event"] != event:
                        # A later completion invalidates this result, even when
                        # the old tour still has individually useful targets.
                        pending_request.update(status="superseded", discarded_time=elapsed)
                        discarded_replans += 1
                        requested = True
                    else:
                        coverage, plan, planning_seconds = result
                        if async_arrival_mode:
                            plan, handoff = rebase_plan(coverage, plan, ground, known,
                                                       [m.xy for m in motions], blocked_views(),
                                                       [m.speed for m in motions])
                            pending_request.update(handoff=handoff, accepted_event=event,
                                distance_until_handoff=[m.distance-d for m, d in
                                                          zip(motions, pending_request["distance_at_request"])])
                            handoff['previous_goals'] = [list(map(int, c)) if c is not None else None
                                                         for c in active_cells]
                            for r, route in enumerate(plan.routes):
                                new_cell = coverage.candidates[route[0]].cell if route else None
                                if active_cells[r] is not None and new_cell != active_cells[r]:
                                    goal_switches[r] += 1
                        accepted_revision = pending_revision
                        exhausted = coverage_exhausted(pending_snapshot, coverage)
                        apply_plan(motions, coverage, plan, ground, known, blocked_views(),
                                   None if async_arrival_mode and not v2 else active_cells,
                                   task_event=task_event_mode)
                        cycle_phase = "EXECUTING"
                        plan_stamp = elapsed
                        pending_request.update(status="accepted", accepted_time=elapsed)
                        replans.append(dict(request_time=request_time, accepted_time=elapsed,
                                            event=pending_request["event"], revision=accepted_revision,
                                            planning_seconds=planning_seconds, map_age_seconds=elapsed-request_time))
                        print(json.dumps({"replan": len(replans), "event": pending_request["event"], "time": elapsed,
                                          "planning_seconds": planning_seconds,
                                          "remaining_tasks": [len(m.route) for m in motions]}), flush=True)
                    pending_snapshot = None
                idle = not any(m.route for m in motions)
                fresh = accepted_revision == known.revision
                complete = completion.update(elapsed, exhausted=exhausted, version=known.revision,
                                             fresh=fresh, idle=idle,
                                             planning=pending is not None or requested)
                # Caps interrupt work. Only the map-based predicate can report success.
                if complete:
                    stop_reason = "complete"
                elif args.duration and elapsed >= args.duration-1e-8:
                    stop_reason = "duration_limit"
                elif args.max_events is not None and event >= args.max_events:
                    stop_reason = "event_limit"
                if (elapsed >= next_record-1e-8 or stop_reason
                        or (physical_arrival_mode and cycle_phase != captured_phase)):
                    status = (("COMPLETE / retained targets resolved; small unknown islands ignored"
                               if coverage.ignored else "COMPLETE / all observable unknown voxels resolved") if complete else
                              f"{stop_reason.upper()} / exploration unfinished" if stop_reason else
                              "Verifying completion / stable map and idle robots" if completion.since is not None else
                              "SCANNING / both robots stopped after arrival" if arrival_cycle_mode and cycle_phase == "SCANNING" else
                              "SCANNED / preparing new joint plan" if arrival_cycle_mode and cycle_phase == "READY_TO_PLAN" else
                              "PLANNING / both robots stopped" if arrival_cycle_mode and pending is not None else
                              "COMMITTED PREFIX / new plan ready; following previous routes"
                              if v2 and pending is not None and elapsed >= ready_at else
                              (f"PLANNING {elapsed-request_time:.2f}/{ready_at-request_time:.2f}s / "
                               "following previous routes") if async_arrival_mode and pending is not None and not idle else
                              (f"PLANNING {elapsed-request_time:.2f}/{ready_at-request_time:.2f}s / "
                               "no remaining previous route") if async_arrival_mode and pending is not None else
                              "Continuous scans / replan computing" if pending is not None else
                              "Waiting / retrying remaining unknowns" if idle else
                              "Continuous scans / current joint plan")
                    record_frame(status)
                if stop_reason:
                    break
                retry_period = (min(idle_period, scan_period) if coverage.planning_stats.get("search_pending")
                                else idle_period)
                if idle and pending is None and cycle_phase != "SCANNING" and (
                        not fresh or (not exhausted and elapsed-last_request_time >= retry_period-1e-9)):
                    requested = True
                if v2 and pending is None and not requested:
                    remaining = [path_length(p)/m.speed for p, m in zip(execution_paths(), motions)]
                    if v2.needs_refill(elapsed, remaining):
                        requested = True
                        refill_events.append(dict(time=elapsed, remaining_seconds=remaining,
                                                  threshold_seconds=v2.refill_threshold))
                    elif v2.needs_idle_search(elapsed, remaining, bool(coverage.deferred), last_request_time):
                        requested = True
                        idle_search_events.append(dict(time=elapsed, remaining_seconds=remaining,
                                                       deferred_unknown=len(coverage.deferred)))
                if requested and pending is None:
                    request_time = last_request_time = elapsed
                    predicted, window = (v2.begin(elapsed, execution_paths(), [m.speed for m in motions])
                                         if v2 else (None, {}))
                    plan_args = arguments(predicted)
                    pending_snapshot = plan_args[0]
                    pending_revision = pending_snapshot.revision
                    pending_request = dict(request_id=len(requests), event=event, request_time=elapsed,
                                           revision=pending_revision, status="computing",
                                           scan_times=list(scan_stamps),
                                           robot_positions=[m.xy for m in motions],
                                           distance_at_request=[m.distance for m in motions],
                                           distance_during_planning=[0., 0.], **window)
                    requests.append(pending_request)
                    pending = worker.submit(compute_plan, *plan_args)
                    requested = False
                    cycle_phase = "PLANNING"
                    if physical_arrival_mode:
                        # Legacy arrival mode accepts at this timestamp. Async
                        # mode first measures latency, then advances old routes.
                        continue
                # Arrival-cycle cars have empty execution queues throughout
                # scanning/planning; both resume only after plan acceptance.
                dt = max(0., next_scan_time-elapsed)
                if args.duration:
                    dt = min(dt, args.duration-elapsed)
                if async_arrival_mode and pending is not None:
                    deadline = ready_at if elapsed < ready_at-1e-9 else handoff_at
                    dt = min(dt, max(0., deadline-elapsed))
                active_before = [m.route[0] if m.route else None for m in motions]
                targets_before = [coverage.candidates[i].xy if i is not None else None for i in active_before]
                before_distance = [m.distance for m in motions]
                computing = async_arrival_mode and pending is not None and elapsed < ready_at-1e-9
                if physical_arrival_mode:
                    dt, finished = step_until_arrival(motions, elapsed, dt)
                else:
                    finished = [r for r, m in enumerate(motions) if m.step(elapsed, dt, dwell)]
                elapsed = round(elapsed+dt, 10)
                for r, motion in enumerate(motions):
                    if not motion.route:
                        idle_total[r] += dt
                    if computing:
                        distance_while_planning[r] += motion.distance-before_distance[r]
                        pending_request['distance_during_planning'][r] += motion.distance-before_distance[r]
                        if not motion.route:
                            idle_while_planning[r] += dt
                scanned = elapsed >= next_scan_time-1e-9
                gained = scan_all(elapsed) if scanned else 0
                if scanned:
                    next_scan_time = round(next_scan_time+scan_period, 10)
                point_log.append(cloud)
                if finished:
                    for r in finished:
                        motion = motions[r]
                        cell = coverage.candidates[motion.route.pop(0)].cell
                        # A completed visit is not proof that its remaining targets
                        # are impossible. Let this position become eligible again.
                        cooldowns[cell] = elapsed+config["failed_view_cooldown"]
                        motion.observations += 1
                        motion.next_path(ground, coverage)
                # En-route measurements update the map but cannot finish or
                # cancel an arrival-cycle visit before its physical destination.
                skipped = [0, 0] if physical_arrival_mode else prune_observed_routes(motions, coverage, ground, known)
                skipped_observed = [a+b for a, b in zip(skipped_observed, skipped)]
                completed = {r for r, index in enumerate(active_before)
                             if index is not None and index not in motions[r].route}
                trigger = bool(finished) if physical_arrival_mode else (
                    bool(completed) if task_event_mode else bool(finished or any(skipped)))
                if trigger:
                    if arrival_cycle_mode:
                        stop_for_arrival_replan(motions, ground, coverage)
                        cycle_phase, last_arrival_time = "SCANNING", elapsed
                    elif task_event_mode and not async_arrival_mode:
                        for r in completed-set(finished):
                            motions[r].observations += 1
                        wait_for_task_replan(motions, completed, ground, coverage)
                    event += 1
                    requested = not arrival_cycle_mode  # First acquire the stopped-pose scan.
                    events.append(dict(event=event, time=elapsed,
                                       replan_requested_time=elapsed,
                                       trigger="arrival" if physical_arrival_mode else "task_completion",
                                       task_positions=targets_before,
                                       finished_robots=[r+1 for r in sorted(completed)] if task_event_mode else [r+1 for r in finished],
                                       arrived_robots=[r+1 for r in finished],
                                       robot_positions=[m.xy for m in motions], unknown_remaining=len(known.unknown),
                                       newly_known_this_scan=gained, skipped_observed_views=skipped,
                                       replan_pending=pending is not None,
                                       next_goals=[coverage.candidates[m.route[0]].xy if m.route else None
                                                   for m in motions]))
                if (arrival_cycle_mode and cycle_phase == "SCANNING" and scanned
                        and elapsed > last_arrival_time+1e-9):
                    cycle_phase = "READY_TO_PLAN"
                    requested = True
                    events[-1]["post_stop_scan_time"] = elapsed
                    events[-1]["post_stop_revision"] = known.revision
                if elapsed >= next_progress:
                    print(json.dumps({"time": elapsed, "events": event, "replans": len(replans),
                                      "unknown": len(known.unknown), "distance_m": [m.distance for m in motions]}), flush=True)
                    next_progress += 10.
                if args.simulation_rate > 0:
                    delay = elapsed/args.simulation_rate-(time.monotonic()-start_wall)
                    if delay > 0:
                        time.sleep(delay)
        except KeyboardInterrupt:
            complete, stop_reason = False, "interrupted"
            record_frame("INTERRUPTED / exploration unfinished")
        video.close(frame, elapsed)
        known.save(output/"final_voxels.npz")
        report = dict(
            world=str(args.world), world_sha256=hashlib.sha256(args.world.read_bytes()).hexdigest(),
            collision_boxes=len(boxes), voxel_resolution=known.resolution, voxel_count=known.state.size,
            candidate_stride=config["candidate_position_stride"], sensor_range=limit,
            planar_range=planar.max_range, scan_hz=args.scan_hz,
            scan_counts=scans, maximum_return_distance=max_return,
            requested_duration=args.duration, simulated_seconds=elapsed,
            video_seconds=video.frames_written/10, playback_speed=args.playback_speed,
            record_period=args.record_period, simulation_rate=args.simulation_rate,
            initial_unknown=initial_unknown, final_unknown=len(known.unknown),
            robot_distance_m=[m.distance for m in motions], robot_tasks=[m.observations for m in motions],
            skipped_observed_views=skipped_observed,
            complete=complete, stop_reason=stop_reason, events=events, replans=replans,
            planning_requests=requests, discarded_replans=discarded_replans,
            replan_policy=replan_policy, cycle_phase=cycle_phase,
            planning_time_model=("measured_latency" if async_arrival_mode else
                                 "instantaneous" if arrival_cycle_mode else "asynchronous"),
            motion_during_planning_m=distance_while_planning,
            idle_during_planning_seconds=idle_while_planning,
            idle_total_seconds=idle_total, goal_switches=goal_switches,
            early_refill_events=refill_events,
            idle_search_events=idle_search_events,
            planning_wall_seconds=sum(r.get("planning_seconds", 0.) for r in requests),
            candidate_ownership=coverage.ownership,
            candidate_evaluation=config.get("candidate_evaluation", "exhaustive"),
            planning_stats=coverage.planning_stats,
            target_policy=coverage.target_policy, selection_phase=coverage.selection_phase,
            ignored_unknown=len(coverage.ignored),
            stopping_policy=("all_retained_observable_unknowns_resolved"
                             if coverage.target_policy == "frontier" else "all_observable_unknowns_resolved"),
            completion_stable_seconds=completion.stable_seconds,
            classification_revision=accepted_revision, current_revision=known.revision,
            classification_fresh=accepted_revision == known.revision and coverage.classification_complete,
            classification_complete=coverage.classification_complete,
            actionable=len(coverage.actionable), deferred=len(coverage.deferred), excluded=len(coverage.excluded),
            pending_replan_at_stop=pending is not None,
            known_2d_ground_prior=True, sensor_replay_not_gazebo=True,
            display_2d="measured_planar_lidar_only", display_3d="measured_returns_only",
            gif_is_accelerated_preview=True)
        report.update(point_format=POINT_FORMAT, point_record_bytes=POINT_DTYPE.itemsize,
                      saved_points=point_log.count, point_storage_bytes=point_log.count*POINT_DTYPE.itemsize)
        report["observation_model"] = config.get("observation_model", "ideal_visibility")
        if profile is not None:
            report["lidar_profile"] = profile.metadata()
        if coverage.target_policy == "frontier":
            report["frontier_policy"] = vars(FrontierPolicy.from_config(config))
        if config.get("candidate_evaluation") == "incremental_frontier":
            report["fast_policy"] = vars(FastPolicy.from_config(config))
        (output/"events.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({k: v for k, v in report.items() if k not in ("events", "replans", "planning_requests")}), flush=True)
    finally:
        video.writer.release()
        point_log.close()
        worker.shutdown(wait=True, cancel_futures=True)
        FAST_EVALUATOR.close()
    return 0
