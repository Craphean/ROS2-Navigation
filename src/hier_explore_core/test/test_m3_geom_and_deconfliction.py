"""M3 unit tests: C++17 _geom_kernel facade, LoS/FoV residual NBV, two-layer deconfliction."""

from __future__ import annotations

import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from hier_explore_core import OccupancyView
from hier_explore_core.deconfliction import (
    CorridorMutexManager,
    CorridorMutexParams,
    TeammateCloudParams,
    build_teammate_cloud_points,
    sample_plan_horizon,
)
from hier_explore_core.geom_kernel import (
    GeodesicCache,
    has_cpp_kernel,
    has_line_of_sight_cpp,
    optimize_residual_viewpoints_cpp,
)
from hier_explore_core.scheduler import filter_corridor_safe_candidates

needs_cpp = pytest.mark.skipif(not has_cpp_kernel(), reason="_geom_kernel not built")

RES = 0.05


def _walled_box(w_m: float, h_m: float, res: float = RES) -> np.ndarray:
    w = int(round(w_m / res))
    h = int(round(h_m / res))
    data = np.zeros((h, w), dtype=np.int8)
    data[0:2, :] = 100
    data[h - 2 :, :] = 100
    data[:, 0:2] = 100
    data[:, w - 2 :] = 100
    return data


def _two_rooms_with_door(door_w_m: float, w_m: float = 10.0, h_m: float = 6.0) -> np.ndarray:
    """Two rooms split by a vertical wall at x = w/2 with a door centred at y = h/2."""
    data = _walled_box(w_m, h_m)
    h, w = data.shape
    wc = w // 2
    data[:, wc - 2 : wc + 2] = 100
    half = int(round(door_w_m / RES / 2.0))
    hc = h // 2
    data[hc - half : hc + half, wc - 2 : wc + 2] = 0
    return data


def _cell(x: float, y: float, res: float = RES):
    return int(math.floor(y / res)), int(math.floor(x / res))


# ---------------------------------------------------------------------------
# Kernel facade
# ---------------------------------------------------------------------------


def test_cpp_kernel_is_built():
    # The colcon build always compiles _geom_kernel; a silent fallback to the
    # Euclidean path would invalidate every M3 benchmark.
    assert has_cpp_kernel()


@needs_cpp
def test_geodesic_detours_around_wall_and_path_is_collision_free():
    data = _walled_box(10.0, 6.0)
    h, w = data.shape
    wc = w // 2
    # Wall from the bottom up to y = 4.5 m: the only passage is the top strip.
    data[: int(4.5 / RES), wc - 2 : wc + 2] = 100
    occ = OccupancyView.from_numpy(data, 0.0, 0.0, RES)
    geo = GeodesicCache(occ, min_clearance_m=0.22, max_dist_m=0.0)

    a, b = (2.5, 1.5), (7.5, 1.5)
    d, path = geo.shortest_path(a, b)
    eucl = math.hypot(b[0] - a[0], b[1] - a[1])
    # Must go around the wall end at y ~ 4.5 m: lower bound is the two
    # straight legs a -> (5, 4.5) -> b = 2 * hypot(2.5, 3.0) ~ 7.8 m.
    assert math.isfinite(d)
    assert d > 2.0 * math.hypot(2.5, 3.0) - 0.1
    assert d < 2.0 * math.hypot(2.5, 3.0) + 2.0
    assert path.shape[0] >= 2 and path.shape[1] == 2
    for x, y in path:
        r, c = _cell(x, y)
        assert occ.data[r, c] <= 50, f"path crosses lethal cell at ({x:.2f}, {y:.2f})"

    # Batched matrix agrees with the A* distance and populates the cache.
    mat = geo.precompute_distances([a], [b, (2.5, 4.0)])
    assert mat.shape == (1, 2)
    assert abs(mat[0, 0] - d) < 0.25
    assert abs(mat[0, 1] - 2.5) < 0.25
    assert abs(geo.distance(a[0], a[1], b[0], b[1]) - mat[0, 0]) < 1e-9


@needs_cpp
def test_geodesic_unreachable_is_infinite():
    data = _walled_box(10.0, 6.0)
    h, w = data.shape
    wc = w // 2
    data[:, wc - 2 : wc + 2] = 100  # fully closed wall
    occ = OccupancyView.from_numpy(data, 0.0, 0.0, RES)
    geo = GeodesicCache(occ)
    mat = geo.precompute_distances([(2.5, 3.0)], [(7.5, 3.0)])
    assert not math.isfinite(mat[0, 0])


@needs_cpp
def test_geodesic_batch_speed_500x600():
    data = _walled_box(30.0, 25.0)  # 500 x 600 cells
    rng = np.random.default_rng(0)
    for _ in range(40):
        r = int(rng.integers(20, 480))
        c = int(rng.integers(20, 580))
        data[r : r + 6, c : c + 6] = 100
    occ = OccupancyView.from_numpy(data, 0.0, 0.0, RES)
    srcs = [(2.0, 2.0), (27.0, 22.0)]
    tgts = [(float(x), float(y)) for x, y in zip(np.linspace(3, 27, 16), np.linspace(22, 3, 16))]
    geo = GeodesicCache(occ, max_dist_m=40.0)
    geo.precompute_distances(srcs, tgts)  # warm-up
    t0 = time.perf_counter()
    for _ in range(5):
        GeodesicCache(occ, max_dist_m=40.0).precompute_distances(srcs, tgts)
    per_call_ms = (time.perf_counter() - t0) * 1000.0 / 5.0
    # 6 ms kernel target + Python wrapper/caching overhead.
    assert per_call_ms < 25.0, f"{per_call_ms:.2f} ms"


def test_line_of_sight_wall_blocks_and_matches_python_fallback():
    data = _walled_box(6.0, 4.0)
    h, w = data.shape
    data[10:50, 60:62] = 100  # short wall segment in the middle
    occ = OccupancyView.from_numpy(data, 0.0, 0.0, RES)
    # Ray straight through the wall segment.
    assert not has_line_of_sight_cpp(occ, 30, 20, 30, 100)
    # Ray along the top, above the wall segment.
    assert has_line_of_sight_cpp(occ, 60, 20, 60, 100)
    # Target cell on a wall is visible (we observe the wall surface).
    assert has_line_of_sight_cpp(occ, 30, 20, 30, 60)


# ---------------------------------------------------------------------------
# LoS + vertical FoV + submodular residual viewpoint optimizer
# ---------------------------------------------------------------------------


def _residual_columns(cx: float, cy: float, zmin: float, zmax: float, n: int = 9, cid: int = 1):
    xs = cx + np.linspace(-0.2, 0.2, int(math.sqrt(n)))
    ys = cy + np.linspace(-0.2, 0.2, int(math.sqrt(n)))
    cols = np.array([[x, y, zmin, zmax] for x in xs for y in ys], dtype=np.float64)
    return cols, np.full(len(cols), cid, dtype=np.int32)


@needs_cpp
def test_residual_viewpoint_respects_wall_occlusion():
    # Closed dividing wall: a viewpoint in the right room can never see the
    # residual cluster in the left room, however close it is.
    data = _walled_box(10.0, 6.0)
    h, w = data.shape
    wc = w // 2
    data[:, wc - 2 : wc + 2] = 100
    occ = OccupancyView.from_numpy(data, 0.0, 0.0, RES)
    cols, cids = _residual_columns(4.3, 3.0, 0.3, 1.2)
    views = optimize_residual_viewpoints_cpp(occ, cols, cids, min_clearance=0.35, max_viewpoints=4)
    assert views, "expected at least one viewpoint in the left room"
    for vp in views:
        assert float(vp["x"]) < 5.0 - 0.1, f"viewpoint behind wall: {vp}"
        assert occ.clearance_at_world(float(vp["x"]), float(vp["y"])) >= 0.35 - 1e-6
        # Yaw points roughly at the cluster.
        yaw_to = math.atan2(3.0 - float(vp["y"]), 4.3 - float(vp["x"]))
        dyaw = math.atan2(math.sin(yaw_to - float(vp["yaw"])), math.cos(yaw_to - float(vp["yaw"])))
        assert abs(dyaw) < math.radians(45.0)


@needs_cpp
def test_residual_viewpoint_rejects_targets_outside_vertical_fov():
    data = _walled_box(10.0, 6.0)
    occ = OccupancyView.from_numpy(data, 0.0, 0.0, RES)
    # A slab at 9-10 m height is above +55 deg from every ring sample (r <= 3.5 m).
    cols_hi, cids_hi = _residual_columns(5.0, 3.0, 9.0, 10.0)
    views_hi = optimize_residual_viewpoints_cpp(occ, cols_hi, cids_hi, r_max=3.5, max_viewpoints=2)
    assert all(float(v.get("coverage_ratio", 0.0)) <= 0.0 for v in views_hi)
    # Same cluster at wall height is fully covered.
    cols_lo, cids_lo = _residual_columns(5.0, 3.0, 0.3, 1.2)
    views_lo = optimize_residual_viewpoints_cpp(occ, cols_lo, cids_lo, r_max=3.5, max_viewpoints=2)
    assert views_lo and float(views_lo[0]["coverage_ratio"]) > 0.9


@needs_cpp
def test_submodular_cover_uses_one_view_for_two_close_clusters():
    data = _walled_box(10.0, 6.0)
    occ = OccupancyView.from_numpy(data, 0.0, 0.0, RES)
    c1, i1 = _residual_columns(4.0, 3.0, 0.3, 1.0, cid=1)
    c2, i2 = _residual_columns(6.0, 3.0, 0.3, 1.0, cid=2)
    cols = np.vstack([c1, c2])
    cids = np.concatenate([i1, i2])
    views = optimize_residual_viewpoints_cpp(
        occ, cols, cids, max_viewpoints=4, dedupe_radius=1.0, max_eff_range=5.0
    )
    assert len(views) >= 1
    # The first greedy pick already covers both clusters (marginal gain of the
    # second pick is zero, so it is not emitted or has ~0 coverage).
    assert len(views) == 1 or float(views[1].get("coverage_ratio", 0.0)) < 0.2


# ---------------------------------------------------------------------------
# Layer 1: teammate cloud
# ---------------------------------------------------------------------------


def test_sample_plan_horizon_truncates_and_skips_stale_prefix():
    plan = [(float(x), 0.0) for x in np.linspace(-2.0, 5.0, 71)]
    pts = sample_plan_horizon(plan, (0.0, 0.0), max_length_m=0.84, step_m=0.16)
    assert len(pts) == 5
    assert all(0.0 < x <= 0.84 + 1e-6 for x, _ in pts)


def test_teammate_cloud_footprint_plan_ego_guard_and_clearing_ring():
    cfg = TeammateCloudParams()
    poses = {"robot1": (0.0, 0.0, 0.0), "robot2": (1.2, 0.0, math.pi)}
    plans = {"robot1": [], "robot2": [(1.2, 0.0), (3.0, 0.0)]}
    pts = build_teammate_cloud_points("robot1", poses, plans, (0.0, 0.0, 0.0), cfg)
    assert pts.dtype == np.float32 and pts.shape[1] == 3
    assert np.allclose(pts[:, 2], cfg.cloud_z)
    d = np.hypot(pts[:, 0], pts[:, 1])
    # Never marks inside the ego safety disk.
    assert d.min() >= cfg.ego_safe_radius - 1e-5
    # Footprint ring around the teammate.
    dt = np.hypot(pts[:, 0] - 1.2, pts[:, 1])
    assert np.sum(np.abs(dt - cfg.footprint_radius) < 1e-4) >= cfg.footprint_samples - 1
    # Predicted trajectory points ahead of the teammate (x > 1.2).
    near_axis = (np.abs(pts[:, 1]) < 1e-4) & (pts[:, 0] > 1.25) & (d < 2.5)
    assert np.sum(near_axis) >= 3
    # Clearing ring strictly between obstacle_max_range (2.5) and raytrace (3.0).
    ring = np.abs(d - cfg.clear_ring_radius) < 1e-4
    assert np.sum(ring) == cfg.clear_ring_samples
    assert 2.5 < cfg.clear_ring_radius < 3.0


def test_teammate_cloud_far_teammate_only_clearing_ring_and_transform():
    cfg = TeammateCloudParams()
    poses = {"robot1": (0.0, 0.0, 0.0), "robot2": (20.0, 0.0, 0.0)}
    pts = build_teammate_cloud_points("robot1", poses, {}, (0.0, 0.0, 0.0), cfg)
    assert len(pts) == cfg.clear_ring_samples
    # World -> local transform: shift by (+1, -2) and rotate by 90 deg.
    tf = (1.0, -2.0, math.pi / 2.0)
    poses2 = {"robot1": (0.0, 0.0, 0.0), "robot2": (1.0, 0.0, 0.0)}
    pts_w = build_teammate_cloud_points("robot1", poses2, {}, (0.0, 0.0, 0.0), cfg)
    pts_l = build_teammate_cloud_points("robot1", poses2, {}, tf, cfg)
    exp_x = 1.0 - pts_w[:, 1]
    exp_y = -2.0 + pts_w[:, 0]
    assert np.allclose(pts_l[:, 0], exp_x, atol=1e-5)
    assert np.allclose(pts_l[:, 1], exp_y, atol=1e-5)


def test_pack_xyz32_cloud_layout():
    pytest.importorskip("sensor_msgs.msg")
    from hier_explore_core.deconfliction import pack_xyz32_cloud

    pts = np.array([[1.0, 2.0, 0.17], [-1.0, 0.5, 0.17]], dtype=np.float32)
    from builtin_interfaces.msg import Time as TimeMsg

    msg = pack_xyz32_cloud(pts, "map", TimeMsg(sec=1, nanosec=2))
    assert msg.header.frame_id == "map"
    assert msg.width == 2 and msg.point_step == 12 and msg.row_step == 24
    back = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, 3)
    assert np.allclose(back, pts)


# ---------------------------------------------------------------------------
# Layer 2: corridor mutex reservation
# ---------------------------------------------------------------------------


def _door_manager(door_w_m: float = 0.6):
    occ = OccupancyView.from_numpy(_two_rooms_with_door(door_w_m), 0.0, 0.0, RES)
    mgr = CorridorMutexManager(CorridorMutexParams())
    mgr.update_map(occ)
    return occ, mgr


@needs_cpp
def test_corridor_extracts_doorway_with_yield_points_on_both_sides():
    occ, mgr = _door_manager(0.6)
    assert len(mgr.segments) >= 1
    door = min(mgr.segments.values(), key=lambda s: math.hypot(s.centroid_x - 5.0, s.centroid_y - 3.0))
    assert math.hypot(door.centroid_x - 5.0, door.centroid_y - 3.0) < 0.6
    assert door.min_half_width_m < 0.22 + 0.35 / 2.0 + 0.1
    sides = {yp[0] < 5.0 for yp in door.yield_points}
    assert sides == {True, False}, door.yield_points
    for yx, yy, _clr in door.yield_points:
        assert occ.clearance_at_world(yx, yy) >= 0.4


@needs_cpp
def test_wide_opening_is_not_a_bottleneck():
    _occ, mgr = _door_manager(2.4)
    for s in mgr.segments.values():
        assert math.hypot(s.centroid_x - 5.0, s.centroid_y - 3.0) > 0.8


@needs_cpp
def test_corridor_priority_reservation_head_on_yield_and_branch_switch():
    occ, mgr = _door_manager(0.6)
    geo = GeodesicCache(occ, min_clearance_m=0.2)
    # robot1 is closer to the door (left side), robot2 approaches head-on.
    poses = {"robot1": (4.0, 3.0, 0.0), "robot2": (7.0, 3.0, math.pi)}
    _d1, p1 = geo.shortest_path((4.0, 3.0), (8.5, 3.0))
    _d2, p2 = geo.shortest_path((7.0, 3.0), (1.5, 3.0))
    paths = {"robot1": [tuple(p) for p in p1], "robot2": [tuple(p) for p in p2]}
    door_ids = mgr.corridors_on_path(paths["robot1"])
    assert door_ids, "door must lie on robot1's path"
    cid = door_ids[0]
    assert cid in mgr.corridors_on_path(paths["robot2"])

    res = mgr.update_reservations(poses, paths, now=10.0)
    assert res[cid] == "robot1"
    assert mgr.blocking_corridor_for_path("robot2", paths["robot2"]) == (cid, "robot1")
    assert mgr.blocking_corridor_for_path("robot1", paths["robot1"]) is None

    yielders = mgr.detect_head_on_yielders(poses, paths)
    assert [(y[0], y[1], y[2]) for y in yielders] == [("robot2", cid, "robot1")]
    yp = yielders[0][3]
    assert yp is not None and yp[0] > 5.0, "robot2 must wait on its own (right) side"
    assert occ.clearance_at_world(yp[0], yp[1]) >= 0.4

    # Branch switch: robot2 has a frontier in its own room and one through the door.
    cands = [SimpleNamespace(x=1.5, y=3.0), SimpleNamespace(x=8.5, y=1.2)]
    safe, conflict = filter_corridor_safe_candidates(
        "robot2", (7.0, 3.0), cands, mgr, geo.shortest_path
    )
    assert [(c.x, c.y) for c in safe] == [(8.5, 1.2)]
    assert conflict == (cid, "robot1")


@needs_cpp
def test_same_direction_convoy_does_not_yield():
    occ, mgr = _door_manager(0.6)
    geo = GeodesicCache(occ, min_clearance_m=0.2)
    poses = {"robot1": (4.2, 3.0, 0.0), "robot2": (1.5, 3.0, 0.0)}
    paths = {
        "robot1": [tuple(p) for p in geo.shortest_path((4.2, 3.0), (8.5, 3.0))[1]],
        "robot2": [tuple(p) for p in geo.shortest_path((1.5, 3.0), (8.0, 4.0))[1]],
    }
    mgr.update_reservations(poses, paths, now=1.0)
    assert mgr.detect_head_on_yielders(poses, paths) == []


@needs_cpp
def test_robot_inside_bottleneck_keeps_reservation():
    occ, mgr = _door_manager(0.6)
    poses = {"robot1": (2.0, 3.0, 0.0), "robot2": (5.0, 3.0, math.pi)}
    res = mgr.update_reservations(poses, {}, now=0.0)
    inside = mgr.corridors_near_point(5.0, 3.0)
    assert inside
    for cid in inside:
        assert res[cid] == "robot2"
