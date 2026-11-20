"""RViz MarkerArray builder supporting arbitrary N robots."""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .robot_tracker import RobotTracker

# Distinct colorblind-friendly palette cycling cleanly for N = 1..8+ robots.
ROBOT_PALETTE: Tuple[Tuple[float, float, float], ...] = (
    (0.90, 0.22, 0.22),  # robot1: crimson
    (0.18, 0.50, 0.95),  # robot2: azure
    (0.15, 0.72, 0.45),  # robot3: emerald
    (0.85, 0.45, 0.10),  # robot4: amber
    (0.62, 0.30, 0.85),  # robot5: violet
    (0.10, 0.75, 0.80),  # robot6: cyan
)


def robot_color(index: int) -> Tuple[float, float, float]:
    return ROBOT_PALETTE[index % len(ROBOT_PALETTE)]


def build_exploration_markers(
    stamp,
    robots: Dict[str, RobotTracker],
    available: Sequence[object],
    ownership: Sequence[object],
    residual: Sequence[object],
) -> MarkerArray:
    """Construct RViz markers for frontiers, pools, residuals, goals, and tours."""
    ma = MarkerArray()
    color_by_ns = {
        ns: robot_color(i) for i, ns in enumerate(robots.keys())
    }

    # 1. Candidate frontier points.
    frontier_marker = Marker()
    frontier_marker.header.frame_id = "world"
    frontier_marker.header.stamp = stamp
    frontier_marker.ns = "candidate_frontiers"
    frontier_marker.id = 0
    frontier_marker.type = Marker.SPHERE_LIST
    frontier_marker.action = Marker.ADD
    frontier_marker.scale.x = 0.2
    frontier_marker.scale.y = 0.2
    frontier_marker.scale.z = 0.2
    frontier_marker.color.r = 0.0
    frontier_marker.color.g = 1.0
    frontier_marker.color.b = 0.2
    frontier_marker.color.a = 0.8
    for c in available:
        if getattr(c, "kind", "frontier") != "frontier":
            continue
        p = Point(x=float(c.world_x), y=float(c.world_y), z=0.05)
        frontier_marker.points.append(p)
    ma.markers.append(frontier_marker)

    # 2. Local / Global pool overlays.
    pool_map = {
        (round(cand.world_x, 2), round(cand.world_y, 2)): own
        for cand, own in zip(available, ownership)
    }

    for ns in robots:
        local_marker = Marker()
        local_marker.header.frame_id = "world"
        local_marker.header.stamp = stamp
        local_marker.ns = f"local_pool_{ns}"
        local_marker.id = 0
        local_marker.type = Marker.SPHERE_LIST
        local_marker.action = Marker.ADD
        local_marker.scale.x = 0.14
        local_marker.scale.y = 0.14
        local_marker.scale.z = 0.14
        cr, cg, cb = color_by_ns[ns]
        local_marker.color.r = cr
        local_marker.color.g = cg
        local_marker.color.b = cb
        local_marker.color.a = 0.9
        for cand in available:
            own = pool_map.get((round(cand.world_x, 2), round(cand.world_y, 2)))
            if own is not None and own.pool == "local" and own.owner == ns:
                local_marker.points.append(
                    Point(x=float(cand.world_x), y=float(cand.world_y), z=0.08)
                )
        ma.markers.append(local_marker)

    global_marker = Marker()
    global_marker.header.frame_id = "world"
    global_marker.header.stamp = stamp
    global_marker.ns = "global_pool"
    global_marker.id = 0
    global_marker.type = Marker.SPHERE_LIST
    global_marker.action = Marker.ADD
    global_marker.scale.x = 0.16
    global_marker.scale.y = 0.16
    global_marker.scale.z = 0.16
    global_marker.color.r = 1.0
    global_marker.color.g = 0.9
    global_marker.color.b = 0.1
    global_marker.color.a = 0.85
    for cand in available:
        own = pool_map.get((round(cand.world_x, 2), round(cand.world_y, 2)))
        if own is not None and own.pool == "global":
            global_marker.points.append(
                Point(x=float(cand.world_x), y=float(cand.world_y), z=0.08)
            )
    ma.markers.append(global_marker)

    # 3. Residual blind spots.
    residual_marker = Marker()
    residual_marker.header.frame_id = "world"
    residual_marker.header.stamp = stamp
    residual_marker.ns = "residual_blindspots"
    residual_marker.id = 0
    residual_marker.type = Marker.CUBE_LIST
    residual_marker.action = Marker.ADD
    residual_marker.scale.x = 0.18
    residual_marker.scale.y = 0.18
    residual_marker.scale.z = 0.18
    residual_marker.color.r = 1.0
    residual_marker.color.g = 0.5
    residual_marker.color.b = 0.0
    residual_marker.color.a = 0.9
    for c in residual:
        residual_marker.points.append(
            Point(x=float(c.world_x), y=float(c.world_y), z=0.12)
        )
    ma.markers.append(residual_marker)

    # 4. Assigned goals and labels.
    for idx, (ns, robot) in enumerate(robots.items(), start=1):
        goal_sphere = Marker()
        goal_sphere.header.frame_id = "world"
        goal_sphere.header.stamp = stamp
        goal_sphere.ns = "assigned_goals"
        goal_sphere.id = idx

        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = stamp
        text_marker.ns = "assigned_labels"
        text_marker.id = idx

        active = (
            robot.state in ("DISPATCHING", "NAVIGATING")
            and robot.current_goal_world is not None
        )
        if active:
            gx, gy = robot.current_goal_world
            cr, cg, cb = color_by_ns[ns]
            goal_sphere.type = Marker.SPHERE
            goal_sphere.action = Marker.ADD
            goal_sphere.pose.position.x = gx
            goal_sphere.pose.position.y = gy
            goal_sphere.pose.position.z = 0.15
            goal_sphere.scale.x = 0.35
            goal_sphere.scale.y = 0.35
            goal_sphere.scale.z = 0.35
            goal_sphere.color.r = cr
            goal_sphere.color.g = cg
            goal_sphere.color.b = cb
            goal_sphere.color.a = 0.95

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = gx
            text_marker.pose.position.y = gy
            text_marker.pose.position.z = 0.4
            text_marker.scale.z = 0.2
            text_marker.text = f"{ns} target"
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
        else:
            goal_sphere.action = Marker.DELETE
            text_marker.action = Marker.DELETE

        ma.markers.append(goal_sphere)
        ma.markers.append(text_marker)

    # 5. Tour paths & remaining waypoints.
    for ns, robot in robots.items():
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.header.stamp = stamp
        path_marker.ns = f"tour_path_{ns}"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.04
        cr, cg, cb = color_by_ns[ns]
        path_marker.color.r = cr
        path_marker.color.g = cg
        path_marker.color.b = cb
        path_marker.color.a = 0.9

        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = "world"
        waypoint_marker.header.stamp = stamp
        waypoint_marker.ns = f"tour_waypoints_{ns}"
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.12
        waypoint_marker.scale.y = 0.12
        waypoint_marker.scale.z = 0.12
        waypoint_marker.color.r = cr
        waypoint_marker.color.g = cg
        waypoint_marker.color.b = cb
        waypoint_marker.color.a = 1.0

        if (
            robot.state in ("DISPATCHING", "NAVIGATING")
            and robot.active_waypoints_world
        ):
            path_marker.points.append(
                Point(x=float(robot.world_x), y=float(robot.world_y), z=0.1)
            )
            start = max(0, robot.current_waypoint_index)
            for wx, wy in robot.active_waypoints_world[start:]:
                path_marker.points.append(Point(x=float(wx), y=float(wy), z=0.1))
                waypoint_marker.points.append(Point(x=float(wx), y=float(wy), z=0.15))
        else:
            path_marker.action = Marker.DELETE
            waypoint_marker.action = Marker.DELETE

        ma.markers.append(path_marker)
        ma.markers.append(waypoint_marker)

    return ma
