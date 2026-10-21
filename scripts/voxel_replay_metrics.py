#!/usr/bin/env python3
"""Shared metrics for A1 Algorithm replay validation."""
import json
from pathlib import Path

import numpy as np


def goal_switches(report):
    queues, switches = [[], []], [0, 0]
    timeline = [(e['time'], 0, e) for e in report['events']]
    timeline += [(r['accepted_time'], 1, r) for r in report['planning_requests'] if r['status'] == 'accepted']
    for _, kind, event in sorted(timeline, key=lambda e: (e[0], e[1])):
        if kind == 0:
            for robot in event['arrived_robots']:
                r = robot-1
                assert queues[r], 'Arrival without an assigned task'
                np.testing.assert_allclose(queues[r][0], event['robot_positions'][r], atol=1e-8)
                queues[r].pop(0)
        else:
            updated = event['handoff']['execution_goals']
            for r in range(2):
                if queues[r] and (not updated[r] or not np.allclose(queues[r][0], updated[r][0], atol=1e-8)):
                    switches[r] += 1
            queues = [[tuple(p) for p in route] for route in updated]
    if 'goal_switches' in report:
        assert switches == report['goal_switches'], (switches, report['goal_switches'])
    return switches


def metrics(folder):
    report = json.loads((folder/'events.json').read_text())
    values = [r['planning_seconds'] for r in report['planning_requests']]
    startup = report['planning_requests'][0]['planning_seconds']
    accepted = [r for r in report['planning_requests'] if r['status'] == 'accepted']
    return dict(
        seconds=report['simulated_seconds'], initial_unknown=report['initial_unknown'],
        final_unknown=report['final_unknown'], mean_planning_seconds=float(np.mean(values)),
        p95_planning_seconds=float(np.percentile(values, 95)),
        planning_requests=len(values), accepted_plans=len(accepted), arrivals=len(report['events']),
        startup_seconds=startup,
        post_startup_idle_during_compute_seconds=[max(0., t-startup) for t in report['idle_during_planning_seconds']],
        total_idle_seconds=report.get('idle_total_seconds',
            [report['simulated_seconds']-d/.22 for d in report['robot_distance_m']]),
        distance_m=report['robot_distance_m'], changed_unfinished_goals=goal_switches(report),
        early_refills=len(report.get('early_refill_events', [])),
        prefix_hold_seconds=sum(r['accepted_time']-r['ready_time'] for r in accepted),
        complete=report['complete'], stop_reason=report['stop_reason'])
