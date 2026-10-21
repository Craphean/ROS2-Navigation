#!/usr/bin/env python3
"""
==============================================================================
Read-Only Live Exploration & Diagnostics Dashboard (with Permanent Warning Retention)
==============================================================================
A 100% PASSIVE OBSERVER designed for debugging autonomous exploration.

KEY FEATURES:
  - ZERO intrusion: Does NOT publish commands, does NOT alter any navigation/auction logic.
  - Read-only subscriber: Only listens to ROS 2 topics and parses coordinator logs.
  - PERMANENT INCIDENT RETENTION: Critical events like CANCEL, TIMEOUT, FAILED,
    STALL, and BLACKLIST stay permanently on the dashboard across refreshes so the
    user never misses why a robot stopped!
  - LAST STOP CAUSE MEMORY: Each robot retains its last stop / failure cause even
    after transitioning to IDLE.
==============================================================================
"""

import argparse
import math
import os
import re
import sys
import threading
import time
import traceback
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# ANSI styling
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
UNDERLINE = "\033[4m"

# Colors
FG_GREEN = "\033[32m"
FG_YELLOW = "\033[33m"
FG_RED = "\033[31m"
FG_CYAN = "\033[36m"
FG_BLUE = "\033[34m"
FG_MAGENTA = "\033[35m"
FG_WHITE = "\033[37m"

FG_BRIGHT_GREEN = "\033[92m"
FG_BRIGHT_YELLOW = "\033[93m"
FG_BRIGHT_BLUE = "\033[94m"
FG_BRIGHT_MAGENTA = "\033[95m"
FG_BRIGHT_RED = "\033[91m"
FG_BRIGHT_CYAN = "\033[96m"
FG_BRIGHT_WHITE = "\033[97m"

BG_RED = "\033[41m"
BG_YELLOW = "\033[43m"
BG_DARK = "\033[100m"

CLEAR_SCREEN = "\033[2J\033[H"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"


class CriticalIncident:
    """Represents a high-priority incident that must NEVER be discarded on refresh."""
    def __init__(self, time_str: str, category: str, robot: str, message: str):
        self.time_str = time_str
        self.category = category  # "CANCEL", "TIMEOUT", "FAILED", "STALL", "BLACKLIST"
        self.robot = robot
        self.message = message


class RobotState:
    def __init__(self, name: str):
        self.name = name
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.linear_vel: float = 0.0
        self.angular_vel: float = 0.0
        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.action_status: int = 0  # action_msgs/GoalStatus code
        self.action_status_text: str = "IDLE"
        # Latest status per action source: (header_stamp, goal_stamp, status).
        # ``status is None`` means that source published an empty status list.
        self.status_by_source: Dict[str, Tuple[float, float, Optional[int]]] = {}
        self.last_status_goal_stamp: float = -1.0
        self.last_move_time: float = time.time()
        self.last_pos: Tuple[float, float] = (0.0, 0.0)
        self.stall_duration: float = 0.0
        self.is_stalled: bool = False

        # Memory: Last known stop / incident cause (persists across status resets)
        self.last_stop_cause: str = "Initialized (Ready)"
        self.last_stop_time: str = "--:--:--"


class TelemetryCollector:
    """Passively collects data from ROS 2 topics and tails coordinator logs."""
    def __init__(self, namespaces: List[str], log_file: str):
        self.namespaces = namespaces
        self.log_file = log_file
        self.robots: Dict[str, RobotState] = {ns: RobotState(ns) for ns in namespaces}

        # Exploration stats
        self.candidate_frontiers: int = 0
        self.assigned_goals: int = 0
        self.blacklisted_goals: List[Tuple[float, float]] = []

        # Map stats
        self.map_known_cells: int = 0
        self.map_total_cells: int = 0
        self.map_progress_pct: float = 0.0
        self.map_res: float = 0.05
        self.map_width: int = 0
        self.map_height: int = 0

        # Permanent Critical Incidents log (Never wiped on refresh!)
        self.critical_incidents: List[CriticalIncident] = []
        self.incident_counts = {
            "CANCEL": 0,
            "TIMEOUT": 0,
            "FAILED": 0,
            "STALL": 0,
            "BLACKLIST": 0
        }

        # Routine events stream (regular auctions, goal dispatches)
        self.recent_events: List[str] = []
        self._lock = threading.Lock()
        self.ros_available = False

        # Pre-read existing log file on startup so past incidents are not missed
        self._initial_log_scan()

        # Background log reader thread
        self.log_thread = threading.Thread(target=self._log_reader_loop, daemon=True)
        self.log_thread.start()

        # Background ROS 2 listener thread
        self.ros_thread = threading.Thread(target=self._ros_listener_loop, daemon=True)
        self.ros_thread.start()

    def _initial_log_scan(self):
        """Scan historical log entries on startup to load past warnings/cancels."""
        if not os.path.exists(self.log_file):
            return
        try:
            with open(self.log_file, 'r', encoding='utf-8', errors='ignore') as f:
                for line in f:
                    self._parse_log_line(line.strip())
        except Exception:
            pass

    def _parse_log_line(self, line: str):
        """Parse coordinator log lines into either routine events or permanent critical incidents."""
        if not line:
            return

        now_str = datetime.now().strftime("%H:%M:%S")

        # 1. CANCEL: blocked by obstacle or cleared by exploration
        if "cleared by exploration" in line or "blocked by obstacle" in line:
            r_match = re.search(r'\[(robot\d+)\]', line)
            r_ns = r_match.group(1) if r_match else "robot"
            reason = "blocked by obstacle" if "blocked by obstacle" in line else "cleared by exploration"
            coord_m = re.search(r'Active goal \(([-\d.]+),\s*([-\d.]+)\)', line)
            g_str = f"({coord_m.group(1)}, {coord_m.group(2)})" if coord_m else "target"

            msg = f"[{r_ns}] Active goal {g_str} CANCELLED ({reason})"
            self._add_critical_incident(CriticalIncident(now_str, "CANCEL", r_ns, msg))

            # Update robot memory
            if r_ns in self.robots:
                self.robots[r_ns].last_stop_cause = f"Cancelled ({reason})"
                self.robots[r_ns].last_stop_time = now_str

        # 2. TIMEOUT: goal timed out
        elif "Goal timed out" in line:
            r_match = re.search(r'\[(robot\d+)\]', line)
            r_ns = r_match.group(1) if r_match else "robot"
            t_match = re.search(r'after\s+([\d.]+)s', line)
            limit_str = (
                f"{float(t_match.group(1)):.1f}s" if t_match else "the limit"
            )
            msg = f"[{r_ns}] Goal TIMED OUT after {limit_str}! Cancelling..."
            self._add_critical_incident(CriticalIncident(now_str, "TIMEOUT", r_ns, msg))
            if r_ns in self.robots:
                self.robots[r_ns].last_stop_cause = f"Timed out ({limit_str} limit)"
                self.robots[r_ns].last_stop_time = now_str

        # 3. FAILED: navigation failed
        elif "Navigation failed" in line or "Goal rejected" in line:
            r_match = re.search(r'\[(robot\d+)\]', line)
            r_ns = r_match.group(1) if r_match else "robot"
            st_match = re.search(r'status code (\d+)', line)
            st_code = st_match.group(1) if st_match else "ABORTED"
            msg = f"[{r_ns}] Navigation FAILED with status code {st_code}!"
            self._add_critical_incident(CriticalIncident(now_str, "FAILED", r_ns, msg))
            if r_ns in self.robots:
                self.robots[r_ns].last_stop_cause = f"Nav2 Failed (status {st_code})"
                self.robots[r_ns].last_stop_time = now_str

        # 4. BLACKLIST: area failed too many times
        elif "Adding to blacklisted goals" in line:
            coord_m = re.search(r'near \(([-\d.]+),\s*([-\d.]+)\)', line)
            if coord_m:
                bx, by = float(coord_m.group(1)), float(coord_m.group(2))
                if (bx, by) not in self.blacklisted_goals:
                    self.blacklisted_goals.append((bx, by))
                    msg = f"Area near ({bx:.2f}, {by:.2f}) failed >= 2 times -> BLACKLISTED (r=0.9m)"
                    self._add_critical_incident(CriticalIncident(now_str, "BLACKLIST", "all", msg))

        # 5. GOAL REACHED (Success memory)
        elif "Goal reached successfully!" in line:
            r_match = re.search(r'\[(robot\d+)\]', line)
            r_ns = r_match.group(1) if r_match else "robot"
            coord_m = re.search(r'Target:\s*(\([^)]+\))', line)
            target_str = coord_m.group(1) if coord_m else "frontier"
            if r_ns in self.robots:
                self.robots[r_ns].last_stop_cause = f"Succeeded ({target_str})"
                self.robots[r_ns].last_stop_time = now_str
            self._add_routine_event(f"[{r_ns}] Successfully reached {target_str}")

        # 6. Routine decision events (Auction / Dispatch)
        elif "won frontier at" in line:
            m = re.search(r'(★.*won frontier.*)', line)
            self._add_routine_event(m.group(1) if m else line)
        elif "Sending Nav2 goal:" in line:
            m = re.search(r'(\[robot\d\].*Sending Nav2 goal.*)', line)
            self._add_routine_event(m.group(1) if m else line)
        elif "No remaining reachable frontiers" in line:
            self._add_routine_event("No remaining reachable frontiers. Exploration complete or paused.")

    def _add_critical_incident(self, incident: CriticalIncident):
        """Append to permanent critical incidents with duplicate suppression."""
        # Avoid exact duplicate in a short window
        if self.critical_incidents:
            last = self.critical_incidents[-1]
            if last.category == incident.category and last.robot == incident.robot and last.message == incident.message:
                return

        self.critical_incidents.append(incident)
        if incident.category in self.incident_counts:
            self.incident_counts[incident.category] += 1
        # Keep up to 20 important incidents
        if len(self.critical_incidents) > 20:
            self.critical_incidents.pop(0)

    def _add_routine_event(self, text: str):
        now_str = datetime.now().strftime("%H:%M:%S")
        self.recent_events.append(f"[{now_str}] {text}")
        if len(self.recent_events) > 15:
            self.recent_events.pop(0)

    def _log_reader_loop(self):
        """Continuously tails /tmp/tb3_multi_coordinator.log for live decision events."""
        last_inode = None
        file_obj = None

        while True:
            try:
                if not os.path.exists(self.log_file):
                    time.sleep(1.0)
                    continue

                curr_inode = os.stat(self.log_file).st_ino
                if file_obj is None or curr_inode != last_inode:
                    if file_obj:
                        file_obj.close()
                    file_obj = open(self.log_file, 'r', encoding='utf-8', errors='ignore')
                    # Seek to end for live tailing
                    file_obj.seek(0, os.SEEK_END)
                    last_inode = curr_inode

                lines = file_obj.readlines()
                if lines:
                    with self._lock:
                        for line in lines:
                            self._parse_log_line(line.strip())

                time.sleep(0.5)
            except Exception:
                time.sleep(1.0)

    def _ros_listener_loop(self):
        """Pure read-only ROS 2 subscriber."""
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from action_msgs.msg import GoalStatusArray
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import OccupancyGrid, Odometry
            from visualization_msgs.msg import MarkerArray
        except ImportError:
            return

        try:
            rclpy.init()
            self.ros_available = True
        except Exception:
            return

        node = Node('exploration_dashboard_observer')

        def safe_callback(cb):
            """Keep one malformed message from killing the whole listener.

            rclpy's executor lets callback exceptions escape ``spin()``; the
            thread then exits silently, freezing pose/status telemetry at its
            startup values (which used to show both robots as stalled at 0,0
            even while Nav2 was driving them).
            """
            def wrapped(msg):
                try:
                    cb(msg)
                except Exception:
                    traceback.print_exc()
            return wrapped

        # 1. Subscriber: /exploration/markers
        def markers_cb(msg: MarkerArray):
            with self._lock:
                cand_count = 0
                assign_count = 0
                for m in msg.markers:
                    if m.ns == 'candidate_frontiers':
                        cand_count = len(m.points)
                    elif m.ns == 'assigned_goals' and m.action == 0:  # ADD
                        assign_count += 1
                self.candidate_frontiers = cand_count
                self.assigned_goals = assign_count

        node.create_subscription(
            MarkerArray, '/exploration/markers', safe_callback(markers_cb), 10
        )

        # 2. Subscriber: /map
        def map_cb(msg: OccupancyGrid):
            with self._lock:
                data = msg.data
                total = len(data)
                unknown = data.count(-1)
                known = total - unknown
                self.map_total_cells = total
                self.map_known_cells = known
                self.map_progress_pct = round((known / max(1, total)) * 100.0, 1)
                self.map_res = round(msg.info.resolution, 3)
                self.map_width = msg.info.width
                self.map_height = msg.info.height

        node.create_subscription(
            OccupancyGrid, '/map', safe_callback(map_cb), 1
        )

        # 3. Subscribers for each robot
        for ns in self.namespaces:
            # Odom
            def make_odom_cb(r_ns):
                def odom_cb(msg: Odometry):
                    with self._lock:
                        r = self.robots.get(r_ns)
                        if not r:
                            return
                        vx = msg.twist.twist.linear.x
                        vy = msg.twist.twist.linear.y
                        r.linear_vel = math.hypot(vx, vy)
                        r.angular_vel = msg.twist.twist.angular.z
                        r.x = msg.pose.pose.position.x
                        r.y = msg.pose.pose.position.y

                        q = msg.pose.pose.orientation
                        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                        r.yaw = math.atan2(siny_cosp, cosy_cosp)
                return odom_cb

            node.create_subscription(
                Odometry, f'/{ns}/odom', safe_callback(make_odom_cb(ns)), 10
            )

            # Goal pose (coordinator visualization topic; `goal_pose` is a
            # reserved Nav2 input and must not be used here).
            def make_goal_cb(r_ns):
                def goal_cb(msg: PoseStamped):
                    with self._lock:
                        r = self.robots.get(r_ns)
                        if r:
                            r.goal_x = msg.pose.position.x
                            r.goal_y = msg.pose.position.y
                return goal_cb

            node.create_subscription(
                PoseStamped,
                f'/{ns}/exploration_goal',
                safe_callback(make_goal_cb(ns)),
                10,
            )

            # Action status.  Both NavigateToPose and NavigateThroughPoses
            # servers publish independently; merge them without letting an
            # empty/idle status from one source hide an active tour on the
            # other.  A non-empty status is aged by its goal_info stamp (stable
            # across periodic republication), falling back to the header stamp.
            def make_status_cb(r_ns, source):
                def status_cb(msg: GoalStatusArray):
                    with self._lock:
                        r = self.robots.get(r_ns)
                        if not r:
                            return

                        # ROS 2's action_msgs/GoalStatusArray has no header
                        # (unlike ROS 1's actionlib_msgs), so age the source
                        # by its goal stamp and fall back to wall time.
                        if not msg.status_list:
                            now_stamp = time.time()
                            r.status_by_source[source] = (
                                now_stamp, 0.0, None
                            )
                        else:
                            latest = msg.status_list[-1]
                            goal_stamp = (
                                latest.goal_info.stamp.sec
                                + latest.goal_info.stamp.nanosec * 1e-9
                            )
                            if goal_stamp <= 0.0:
                                goal_stamp = time.time()
                            r.status_by_source[source] = (
                                goal_stamp, goal_stamp, int(latest.status)
                            )

                        active = [
                            (goal_stamp, source_name, st)
                            for source_name, (hdr, goal_stamp, st)
                            in r.status_by_source.items()
                            if st is not None
                        ]
                        if not active:
                            r.action_status = 0
                            r.action_status_text = "IDLE"
                            r.goal_x = None
                            r.goal_y = None
                            return

                        # Newest goal wins.  On an exact tie prefer a live
                        # status so an idle source cannot mask an active one.
                        goal_stamp, source_name, st = max(
                            active,
                            key=lambda e: (
                                e[0],
                                1 if e[2] in (1, 2, 3) else 0,
                            ),
                        )
                        del source_name
                        if goal_stamp < r.last_status_goal_stamp:
                            # Only a stale goal is still listed on the other
                            # source; keep the newest known display.
                            return
                        r.action_status = st
                        status_map = {
                            1: "ACCEPTED",
                            2: "NAVIGATING",
                            3: "CANCELING",
                            4: "SUCCEEDED",
                            5: "CANCELED",
                            6: "ABORTED",
                        }
                        r.action_status_text = status_map.get(
                            st, f"STATUS_{st}"
                        )

                        if goal_stamp >= r.last_status_goal_stamp:
                            r.last_status_goal_stamp = goal_stamp
                            if st == 4:
                                r.last_stop_cause = "Succeeded"
                                r.last_stop_time = datetime.now().strftime(
                                    "%H:%M:%S"
                                )
                            elif st == 5:
                                r.last_stop_cause = "Cancelled"
                                r.last_stop_time = datetime.now().strftime(
                                    "%H:%M:%S"
                                )
                            elif st == 6:
                                r.last_stop_cause = "Aborted / Failed"
                                r.last_stop_time = datetime.now().strftime(
                                    "%H:%M:%S"
                                )

                        if st in (4, 5, 6):
                            r.goal_x = None
                            r.goal_y = None
                return status_cb

            node.create_subscription(
                GoalStatusArray,
                f'/{ns}/navigate_to_pose/_action/status',
                safe_callback(make_status_cb(ns, 'to_pose')),
                10,
            )
            node.create_subscription(
                GoalStatusArray,
                f'/{ns}/navigate_through_poses/_action/status',
                safe_callback(make_status_cb(ns, 'through_poses')),
                10,
            )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except Exception:
            # ExternalShutdownException during Ctrl-C is expected; only
            # report exceptions that happen while the context is healthy.
            if rclpy.ok():
                traceback.print_exc()
        finally:
            node.destroy_node()

    def update_stall_diagnostics(self):
        """Passively compute stall state and append persistent warning if motionless >= 5s."""
        now = time.time()
        now_str = datetime.now().strftime("%H:%M:%S")

        with self._lock:
            for r in self.robots.values():
                is_navigating = (r.action_status_text in ("NAVIGATING", "ACCEPTED") or r.goal_x is not None)
                moved = math.hypot(r.x - r.last_pos[0], r.y - r.last_pos[1])

                if is_navigating:
                    if r.linear_vel < 0.02 and moved < 0.03:
                        r.stall_duration = now - r.last_move_time
                        if r.stall_duration >= 4.0:
                            if not r.is_stalled:
                                r.is_stalled = True
                                dist_str = f"{math.hypot(r.goal_x - r.x, r.goal_y - r.y):.2f}m" if r.goal_x is not None else "unknown"
                                warn_msg = f"[{r.name}] STALL DETECTED: linear_vel=0.00 m/s for >= 4s (dist to goal: {dist_str})"
                                self._add_critical_incident(CriticalIncident(now_str, "STALL", r.name, warn_msg))
                                r.last_stop_cause = f"Stalled near obstacle (dist: {dist_str})"
                                r.last_stop_time = now_str
                    else:
                        r.last_move_time = now
                        r.last_pos = (r.x, r.y)
                        r.stall_duration = 0.0
                        r.is_stalled = False
                else:
                    r.last_move_time = now
                    r.last_pos = (r.x, r.y)
                    r.stall_duration = 0.0
                    r.is_stalled = False


def format_badge(r: RobotState) -> str:
    """Colorized status badge."""
    if r.is_stalled:
        return f"{FG_BRIGHT_YELLOW}{BG_RED}{BOLD}[ STALLED ⚠️ ]{RESET}"
    elif r.action_status_text == "NAVIGATING":
        return f"{FG_BRIGHT_GREEN}{BOLD}[NAVIGATING]{RESET}"
    elif r.action_status_text in ("ACCEPTED", "PLANNING"):
        return f"{FG_BRIGHT_CYAN}{BOLD}[ PLANNING ]{RESET}"
    elif r.action_status_text == "SUCCEEDED":
        return f"{FG_BRIGHT_GREEN}{BOLD}[SUCCEEDED ]{RESET}"
    elif r.action_status_text in ("ABORTED", "CANCELED"):
        return f"{FG_RED}{BOLD}[ {r.action_status_text} ]{RESET}"
    else:
        return f"{FG_WHITE}{DIM}[   IDLE   ]{RESET}"


def make_progress_bar(pct: float, width: int = 24) -> str:
    pct = max(0.0, min(100.0, pct))
    filled_len = int(width * pct / 100.0)
    bar = "█" * filled_len + "░" * (width - filled_len)
    return f"{FG_BRIGHT_CYAN}{bar}{RESET} {pct:5.1f}%"


def render_dashboard(collector: TelemetryCollector):
    collector.update_stall_diagnostics()
    now_dt = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    out = []
    out.append(CLEAR_SCREEN)

    # 1. Header Box
    out.append(f"{BOLD}{FG_BRIGHT_CYAN}╔{'═'*84}╗{RESET}")
    out.append(
        f"{BOLD}{FG_BRIGHT_CYAN}║{FG_BRIGHT_WHITE}  🔍 ROS 2 EXPLORATION & ROBOT TELEMETRY MONITOR (READ-ONLY DEBUGGER)             {FG_BRIGHT_CYAN}║{RESET}"
    )
    ros_tag = f"{FG_GREEN}Active{RESET}" if collector.ros_available else f"{FG_YELLOW}Log Tail Mode{RESET}"
    out.append(
        f"{BOLD}{FG_BRIGHT_CYAN}║{RESET}  {DIM}Time: {now_dt} | ROS 2 Listener: {ros_tag} | Targets: {', '.join(collector.namespaces)}{RESET}{' '*(25-len(', '.join(collector.namespaces)))}{BOLD}{FG_BRIGHT_CYAN}║{RESET}"
    )
    out.append(f"{BOLD}{FG_BRIGHT_CYAN}╚{'═'*84}╝{RESET}\n")

    # 2. Exploration Progress & Map Info
    with collector._lock:
        total = collector.map_total_cells
        known = collector.map_known_cells
        pct = collector.map_progress_pct
        res = collector.map_res
        w, h = collector.map_width, collector.map_height

    out.append(f"{BOLD}{FG_BRIGHT_BLUE}┌── 🗺️  MERGED MAP & COVERAGE ({w}x{h} @ {res:.2f}m) {'─'*42}┐{RESET}")
    if total > 0:
        out.append(f"│  Exploration Progress: {make_progress_bar(pct, 26)} (Known: {known:,} / Total: {total:,} cells)")
    else:
        out.append(f"│  Exploration Progress: {DIM}Waiting for /map OccupancyGrid...{RESET}")
    out.append(f"{BOLD}{FG_BRIGHT_BLUE}└──{'─'*82}┘{RESET}\n")

    # 3. Frontier & Exploration Markers Section
    with collector._lock:
        c_count = collector.candidate_frontiers
        a_count = collector.assigned_goals
        b_goals = list(collector.blacklisted_goals)

    b_str = ", ".join([f"({x:.1f},{y:.1f})" for x, y in b_goals[-3:]]) if b_goals else "None"
    if len(b_goals) > 3:
        b_str += f" (+{len(b_goals)-3} more)"

    out.append(f"{BOLD}{FG_BRIGHT_MAGENTA}┌── 🎯  EXPLORATION MARKERS STATUS (Topic: /exploration/markers) {'─'*21}┐{RESET}")
    out.append(f"│  🟢 {BOLD}Candidate Frontiers (Green Spheres):{RESET} {c_count:2d} clusters detected on frontier edge")
    out.append(f"│  🔴🔵 {BOLD}Active Assigned Goals (Color Spheres):{RESET} {a_count:2d} robots locked on target")
    out.append(f"│  ⬛ {BOLD}Blacklisted Dead-Ends (Pruned):{RESET}       {len(b_goals):2d} locations [ {b_str} ]")
    out.append(f"{BOLD}{FG_BRIGHT_MAGENTA}└──{'─'*82}┘{RESET}\n")

    # 4. Robot Fleet Telemetry Table with Last Stop Cause Memory
    out.append(f"{BOLD}{FG_BRIGHT_WHITE}┌── 🤖  ROBOT FLEET TELEMETRY & LAST STOP CAUSE MEMORY {'─'*31}┐{RESET}")
    out.append(
        f"│ {BOLD}{'Robot':<8} {'State':<15} {'Pose (X, Y, Yaw)':<19} {'Speed (v/w)':<12} "
        f"{'Target (X, Y)':<15} {'Last Stop / Incident Cause'}{RESET}"
    )
    out.append(f"├{'─'*84}┤")

    with collector._lock:
        for ns in collector.namespaces:
            r = collector.robots[ns]
            badge = format_badge(r)
            pose_str = f"({r.x:5.2f},{r.y:5.2f},{r.yaw:4.1f})"
            vel_str = f"{r.linear_vel:4.2f}/{r.angular_vel:+4.2f}"

            if r.goal_x is not None:
                goal_str = f"({r.goal_x:5.2f},{r.goal_y:5.2f})"
            else:
                goal_str = "None"

            # Last stop cause with timestamp
            last_cause_str = f"{r.last_stop_cause}"
            if r.last_stop_time != "--:--:--":
                last_cause_str += f" @ {r.last_stop_time}"

            # Truncate if too long
            if len(last_cause_str) > 30:
                last_cause_str = last_cause_str[:27] + "..."

            out.append(
                f"│ {BOLD}{ns:<8}{RESET} {badge:<23} {pose_str:<19} {vel_str:<12} "
                f"{goal_str:<15} {FG_YELLOW}{last_cause_str}{RESET}"
            )

    out.append(f"{BOLD}{FG_BRIGHT_WHITE}└──{'─'*82}┘{RESET}\n")

    # 5. PERMANENT CRITICAL INCIDENTS & WARNINGS RETENTION LOG
    # THIS AREA NEVER GETS CLEARED ON REFRESH!
    counts = collector.incident_counts
    stats_hdr = f"Cancel: {counts['CANCEL']} | Timeout: {counts['TIMEOUT']} | Failed: {counts['FAILED']} | Stalls: {counts['STALL']} | Blacklisted: {counts['BLACKLIST']}"

    out.append(f"{BOLD}{FG_BRIGHT_RED}┌── ⚠️  CRITICAL INCIDENTS & WARNINGS LOG (PERMANENT RETENTION) {'─'*18}┐{RESET}")
    out.append(f"│  {DIM}Total Incidents: {len(collector.critical_incidents)} ({stats_hdr}){RESET}")
    out.append(f"├{'─'*84}┤")

    with collector._lock:
        incidents_to_show = list(collector.critical_incidents[-7:])

    if incidents_to_show:
        for inc in incidents_to_show:
            tag_color = FG_BRIGHT_RED
            if inc.category == "CANCEL":
                tag_color = FG_BRIGHT_YELLOW
            elif inc.category == "TIMEOUT":
                tag_color = FG_BRIGHT_RED
            elif inc.category == "FAILED":
                tag_color = FG_RED
            elif inc.category == "BLACKLIST":
                tag_color = FG_BRIGHT_MAGENTA
            elif inc.category == "STALL":
                tag_color = FG_YELLOW

            badge = f"{tag_color}{BOLD}[{inc.category:^9}]{RESET}"
            out.append(f"│  {DIM}[{inc.time_str}]{RESET} {badge} {inc.message}")
    else:
        out.append(f"│  {FG_GREEN}✓ No critical cancel, timeout, or failure incidents recorded yet.{RESET}")

    out.append(f"{BOLD}{FG_BRIGHT_RED}└──{'─'*82}┘{RESET}\n")

    # 6. Routine Decisions Stream (Auction / Dispatch)
    out.append(f"{BOLD}{FG_BRIGHT_YELLOW}┌── 📜  RECENT ROUTINE COORDINATION STREAM {'─'*42}┐{RESET}")
    with collector._lock:
        evs = list(collector.recent_events[-4:])

    if evs:
        for e in evs:
            colored_e = e
            if "won frontier" in colored_e:
                colored_e = f"{FG_BRIGHT_CYAN}{colored_e}{RESET}"
            elif "Sending Nav2 goal" in colored_e:
                colored_e = f"{FG_BRIGHT_BLUE}{colored_e}{RESET}"
            elif "reached" in colored_e:
                colored_e = f"{FG_BRIGHT_GREEN}{colored_e}{RESET}"
            out.append(f"│  {colored_e}")
    else:
        out.append(f"│  {DIM}Waiting for coordinator decision events...{RESET}")
    out.append(f"{BOLD}{FG_BRIGHT_YELLOW}└──{'─'*82}┘{RESET}")

    out.append(f"{DIM}Note: Pure observer. Important warnings remain visible permanently. [Ctrl+C] to exit.{RESET}")
    print("\n".join(out), flush=True)


def main():
    parser = argparse.ArgumentParser(description="Read-only Live Exploration Telemetry Dashboard")
    parser.add_argument("--robots", default="robot1,robot2", help="Comma-separated robot namespaces")
    parser.add_argument("--log", default="/tmp/tb3_multi_coordinator.log", help="Path to coordinator log file")
    parser.add_argument("--interval", type=float, default=0.5, help="Refresh interval in seconds")
    parser.add_argument("--stream", action="store_true", help="Append stream output mode")

    args = parser.parse_args()
    ns_list = [ns.strip() for ns in args.robots.split(",") if ns.strip()]

    collector = TelemetryCollector(namespaces=ns_list, log_file=args.log)

    if not args.stream:
        print(HIDE_CURSOR, end="")

    try:
        while True:
            if args.stream:
                collector.update_stall_diagnostics()
                now_str = datetime.now().strftime("%H:%M:%S")
                with collector._lock:
                    r_info = []
                    for ns in collector.namespaces:
                        r = collector.robots[ns]
                        r_info.append(f"{ns}:{r.action_status_text}(v={r.linear_vel:.2f},last={r.last_stop_cause})")
                    print(f"[{now_str}] {' | '.join(r_info)} | Incidents:{len(collector.critical_incidents)}", flush=True)
            else:
                render_dashboard(collector)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        if not args.stream:
            print(SHOW_CURSOR, end="")
            print(f"\n{FG_BRIGHT_CYAN}[Monitor dashboard exited cleanly]{RESET}")


if __name__ == '__main__':
    main()
