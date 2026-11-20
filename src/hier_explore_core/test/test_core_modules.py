"""Unit tests for newly extracted hier_explore_core modules (geometry, frontier, scheduler, supervisor)."""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from hier_explore_core import (
    AgentState,
    FrontierParams,
    OccupancyView,
    RobotTracker,
    TourParams,
    build_local_tour,
    extract_frontiers,
    pose_is_discontinuous,
    se2_apply,
    se2_compose,
    se2_inverse,
    wrap_angle,
)
from hier_explore_core.supervisor import (
    FailureBookkeeper,
    check_no_progress_success,
    check_waypoint_invalid,
    is_out_of_bounds_failure,
)


def _make_half_explored_room(width_m=12.0, height_m=8.0, res=0.1):
    w = int(round(width_m / res))
    h = int(round(height_m / res))
    data = np.full((h, w), -1, dtype=np.int8)
    # Left half free, bounded by lethal walls on top/bottom/left
    data[2 : h - 2, 2 : w // 2] = 0
    data[0:2, :] = 100
    data[h - 2 :, :] = 100
    data[:, 0:2] = 100
    return OccupancyView.from_numpy(data, 0.0, 0.0, res)


def test_se2_roundtrip_and_composition():
    t_ab = (1.5, -2.3, 0.72)
    t_ba = se2_inverse(t_ab)
    ident = se2_compose(t_ab, t_ba)
    assert math.isclose(ident[0], 0.0, abs_tol=1e-9)
    assert math.isclose(ident[1], 0.0, abs_tol=1e-9)
    assert math.isclose(ident[2], 0.0, abs_tol=1e-9)

    px, py, pyaw = se2_apply(t_ab, 3.0, -1.0, 0.5)
    rx, ry, ryaw = se2_apply(t_ba, px, py, pyaw)
    assert math.isclose(rx, 3.0, abs_tol=1e-9)
    assert math.isclose(ry, -1.0, abs_tol=1e-9)
    assert math.isclose(wrap_angle(ryaw - 0.5), 0.0, abs_tol=1e-9)


def test_pose_continuity_helper():
    assert not pose_is_discontinuous(0.5, 0.25)
    assert pose_is_discontinuous(0.5, 2.0)
    assert not pose_is_discontinuous(-0.1, 0.1)
    assert pose_is_discontinuous(-0.1, 1.0)


def test_frontier_extraction_splits_long_boundary_and_retracts():
    occ = _make_half_explored_room(width_m=12.0, height_m=8.0, res=0.1)
    params = FrontierParams(min_frontier_size=5, min_clearance=0.55)
    clusters = extract_frontiers(occ, params)
    # 7.6 m tall frontier should split into >= 2 sub-clusters, all safely retracted
    assert len(clusters) >= 2
    for c in clusters:
        assert occ.clearance_at_world(c.world_x, c.world_y) >= 0.55 - 1e-6
        assert c.component_label > 0
        # Unknown side is to the right (+x), so frontier yaw should point roughly +x
        assert abs(wrap_angle(c.yaw_world)) < math.pi / 2


def test_scheduler_accepts_custom_geodesic_distance_fn():
    c_near_wall = SimpleNamespace(x=2.0, y=0.0, total_gain=5.0)
    c_open = SimpleNamespace(x=4.0, y=0.0, total_gain=5.0)

    # Custom geodesic distance penalizes crossing the wall to c_near_wall
    def fake_geodesic(x1, y1, x2, y2):
        touches_walled = (
            math.isclose(x1, 2.0, abs_tol=0.1) != math.isclose(x2, 2.0, abs_tol=0.1)
        )
        if touches_walled:
            return 20.0
        return math.hypot(x2 - x1, y2 - y1)

    tour = build_local_tour(
        [c_near_wall, c_open],
        (0.0, 0.0),
        TourParams(max_waypoints=2, min_spacing=0.5, max_tour_length=12.0),
        distance_fn=fake_geodesic,
    )
    assert len(tour) == 1
    assert tour[0].x == pytest.approx(4.0)


def test_failure_bookkeeper_and_livelock_breaker():
    bk = FailureBookkeeper(blacklist_retry_limit=2, blacklist_radius=0.9)
    assert bk.record_failure((3.0, 4.0)) is None
    msg = bk.record_failure((3.1, 4.0))
    assert msg is not None
    assert bk.is_frontier_blacklisted(3.0, 4.0)

    robot = RobotTracker("robot1", [0.0, 0.0, 0.0])
    robot.goal_start_time = 10.0
    robot.stall_ref_x, robot.stall_ref_y = 0.0, 0.0
    m1 = check_no_progress_success(robot, (5.0, 5.0), 10.5, bk, no_progress_limit=2)
    assert m1 and "1/2" in m1
    assert not bk.is_frontier_blacklisted(5.0, 5.0)
    m2 = check_no_progress_success(robot, (5.0, 5.0), 10.6, bk, no_progress_limit=2)
    assert m2 and "blacklisted" in m2
    assert bk.is_frontier_blacklisted(5.0, 5.0)


def test_waypoint_invalidation_and_oob_detection():
    occ = _make_half_explored_room(width_m=10.0, height_m=10.0, res=0.1)
    robot = RobotTracker("robot1", [2.0, 5.0, 0.0])
    bk = FailureBookkeeper()

    # Deep inside known free region -> cleared (<5 unknown neighbors within 1.5m)
    invalid, reason = check_waypoint_invalid(occ, robot, 2.0, 5.0, bk)
    assert invalid and "cleared" in reason

    # Right near frontier boundary -> still valid
    invalid, _ = check_waypoint_invalid(occ, robot, 4.8, 5.0, bk)
    assert not invalid

    assert is_out_of_bounds_failure("Start Coordinates (-5.86, 0.82) was outside bounds")
    assert not is_out_of_bounds_failure("Goal rejected")
