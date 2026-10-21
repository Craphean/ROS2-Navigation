"""Live-pose validation shared by the asynchronous replay and ROS adapter."""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from voxel_exploration_core import RoutePlan, pending_route


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def path_tail(path, distance):
    """Trim an executed polyline without shortcutting bends."""
    for i, (a, b) in enumerate(zip(path, path[1:])):
        length = math.dist(a, b)
        if length > distance and length > 1e-12:
            point = tuple(np.asarray(a)+(np.asarray(b)-a)*distance/length)
            return [point, *path[i+1:]]
        distance -= length
    return list(path[-1:])


def queued_path(ground, start, goals, first_path=None):
    path = [tuple(start)]
    for i, goal in enumerate(goals):
        segment = first_path if i == 0 and first_path else ground.path(path[-1], goal)
        if not segment:
            break  # No prediction through an unreachable segment.
        path.extend(segment[1:])
    return path


@dataclass(frozen=True)
class AsyncV2Policy:
    async_commit_seconds: float = 1.5
    async_refill_margin: float = .4
    async_refill_interval: float = 2.
    async_initial_latency: float = 1.
    async_latency_window: int = 20
    async_idle_search_seconds: float = 5.

    def __post_init__(self):
        if (not all(math.isfinite(v) and v > 0 for v in vars(self).values())
                or int(self.async_latency_window) != self.async_latency_window):
            raise ValueError('Positive finite V2 timing parameters required')

    @classmethod
    def from_config(cls, config):
        return cls(**{k: config[k] for k in cls.__dataclass_fields__ if k in config})


class AsyncV2Window:
    """One bounded commitment per request, with edge-triggered early refill.

    The prediction changes cost/assignment only, never the measured voxel map.
    A completed worker waits at most the original commitment deadline. Arrivals
    coalesce in the caller and never extend that deadline or discard the result.
    """
    def __init__(self, policy=None):
        self.policy = policy or AsyncV2Policy()
        self.latencies = deque(maxlen=int(self.policy.async_latency_window))
        self.low = set()
        self.last_refill = -math.inf
        self.commit_until = 0.
        self.refill_count = 0
        self.idle_search_count = 0
        self.last_idle_search = -math.inf

    def needs_idle_search(self, now, remaining, deferred, last_request):
        """An empty per-robot queue is not proof that it has no useful views.

        Retry bounded candidate search while its peer drives, even without an
        arrival. Global all-idle completion/auditing stays in the caller.
        """
        if (deferred and any(t <= 0 for t in remaining) and any(t > 0 for t in remaining)
                and now-max(last_request, self.last_idle_search) >= self.policy.async_idle_search_seconds-1e-9):
            self.last_idle_search = now
            self.idle_search_count += 1
            return True
        return False

    def observe_latency(self, seconds):
        latency_deadline(0., seconds)
        self.latencies.append(seconds)

    @property
    def p95(self):
        return float(np.percentile(self.latencies, 95)) if self.latencies else self.policy.async_initial_latency

    @property
    def refill_threshold(self):
        return max(self.p95, self.policy.async_commit_seconds)+self.policy.async_refill_margin

    def needs_refill(self, now, remaining):
        threshold = self.refill_threshold
        self.low = {r for r in self.low if remaining[r] <= threshold+self.policy.async_refill_margin}
        low = {r for r, seconds in enumerate(remaining) if 0 < seconds <= threshold}
        if low-self.low and now-self.last_refill >= self.policy.async_refill_interval:
            self.low |= low
            self.last_refill = now
            self.refill_count += 1
            return True
        return False

    def begin(self, now, paths, speeds):
        remaining = [path_length(path)/speed for path, speed in zip(paths, speeds)]
        # Never force an empty-queue car to wait for a commitment on its peer.
        self.commit_until = now+min(self.policy.async_commit_seconds, min(remaining, default=0.))
        horizon = max(self.p95, self.commit_until-now)
        predicted = [path_tail(path, horizon*speed)[0] for path, speed in zip(paths, speeds)]
        return predicted, dict(commit_until=self.commit_until, prediction_seconds=horizon,
                               predicted_positions=predicted, remaining_route_seconds=remaining,
                               latency_p95_seconds=self.p95)


def latency_deadline(request_time, planning_seconds):
    """Use measured compute latency as a discrete-event simulation duration."""
    if not math.isfinite(planning_seconds) or planning_seconds < 0:
        raise ValueError("Planning latency must be finite and nonnegative")
    return request_time + planning_seconds


def rebase_plan(coverage, plan, ground, known, starts, blocked=(), speeds=(.22, .22)):
    """Validate BOTH new tours before replacing either old execution queue.

    Keep the new solver's assignment/order, remove obsolete/unreachable tasks,
    and connect from live positions on the current inflated ground graph.
    Filtering never classifies any voxel as observed or unobservable.
    """
    if len(starts) != len(plan.routes):
        raise ValueError("A live pose is required for every robot")
    if any(not ground.valid(ground.cell(p), ground.safe) for p in starts):
        raise RuntimeError("Live robot pose is outside known safe ground")
    routes, paths, times = [], [], []
    removed = 0
    for r, (start, original) in enumerate(zip(starts, plan.routes)):
        pending = pending_route(original, coverage.candidates, known, blocked)
        distance = ground.distances(start)
        route, path, previous = [], [], start
        for index in pending:
            candidate = coverage.candidates[index]
            cell = ground.cell(candidate.xy)
            if not ground.valid(cell, ground.safe) or not np.isfinite(distance[cell]):
                continue
            segment = ground.path(previous, candidate.xy)
            if not segment:
                continue
            route.append(index)
            path.extend(segment if not path else segment[1:])
            previous = candidate.xy
        removed += len(original)-len(route)
        routes.append(route)
        paths.append(path)
        times.append(sum(math.dist(a, b) for a, b in zip(path, path[1:]))/speeds[r])
    return (RoutePlan(routes, tuple(times), max(times, default=0.), plan.method, paths),
            dict(filtered_tasks=removed, live_positions=[tuple(p) for p in starts],
                 execution_goals=[[coverage.candidates[i].xy for i in route] for route in routes]))
