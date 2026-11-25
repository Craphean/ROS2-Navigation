"""Parameterized N=1..4 multi-robot exploration & 3D mapping simulation launcher.

Replaces the hardcoded 2-robot bash script with a single composable ROS 2 launch
file supporting:
  - num_robots: 1..4 (spawns /robot1 .. /robotN with staggered bringup)
  - world: house | hospital | mine | warehouse | cmu_indoor | cmu_tunnel | <path>
  - spawn_seed: 0 (fixed default formation) or >0 (deterministic seeded station pool)
  - odom_mode: ekf (noisy DiffDrive + 50 Hz IMU EKF) | gt (Gazebo GT odom ablation)
  - ablation_profile: legacy | e1_2d | e2_3dgain | e3_hier | e4_geodesic |
                      e5_deconflict | full | no_se2 | no_degen_gate
  - planner_backend: auto | coordinator | legacy | off
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hier_explore_bringup.profiles import resolve_profile
from hier_explore_bringup.worlds import (
    canonical_world_name,
    find_repo_root,
    resolve_world_file,
    sample_spawn_poses,
)


def _as_bool(val: str) -> bool:
    return str(val).strip().lower() in ("true", "1", "yes")


def _expand_xacro(xacro_path: Path, out_path: Path, args: Dict[str, str]) -> Path:
    cmd = ["xacro", "-o", str(out_path)]
    for k, v in args.items():
        cmd.append(f"{k}:={v}")
    cmd.append(str(xacro_path))
    subprocess.run(cmd, check=True)
    return out_path


def _prepare_nav2_params(src_params: Path, out_dir: Path, use_se2_merge: bool) -> Path:
    if use_se2_merge:
        return src_params
    dst = out_dir / "nav2_legacy_params.yaml"
    text = src_params.read_text(encoding="utf-8")
    dst.write_text(text.replace("map_topic: shared_map", "map_topic: map"), encoding="utf-8")
    return dst


def _build_launch_actions(context, *args, **kwargs):
    del args, kwargs
    repo_root = find_repo_root()
    sim_share = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    nav2_bringup_share = Path(get_package_share_directory("nav2_bringup"))

    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    if num_robots < 1 or num_robots > 4:
        raise ValueError(f"num_robots must be in 1..4, got {num_robots}")

    world_arg = LaunchConfiguration("world").perform(context)
    spawn_seed = int(LaunchConfiguration("spawn_seed").perform(context))
    odom_mode = LaunchConfiguration("odom_mode").perform(context)
    ablation_profile = LaunchConfiguration("ablation_profile").perform(context)
    planner_backend = LaunchConfiguration("planner_backend").perform(context)
    headless = _as_bool(LaunchConfiguration("headless").perform(context))
    use_rviz = _as_bool(LaunchConfiguration("use_rviz").perform(context))
    use_se2_merge_arg = LaunchConfiguration("use_se2_merge").perform(context)
    use_cpp_cloud_arg = LaunchConfiguration("use_cpp_cloud_merger").perform(context)
    enable_cameras = _as_bool(LaunchConfiguration("enable_cameras").perform(context))
    use_composition = LaunchConfiguration("use_composition").perform(context)
    robot_stagger_sec = float(LaunchConfiguration("robot_stagger_sec").perform(context))
    explorer_delay_arg = LaunchConfiguration("explorer_start_delay_sec").perform(context).strip().lower()
    lidar_v_samples = LaunchConfiguration("lidar_v_samples").perform(context).strip()
    voxel_resolution = float(LaunchConfiguration("voxel_resolution").perform(context))
    init_pose_noise_str = LaunchConfiguration("init_pose_noise").perform(context).strip()
    noise_parts = [float(v) for v in init_pose_noise_str.split(",") if v.strip()]
    if len(noise_parts) == 3:
        noise_dx, noise_dy, noise_dyaw_deg = noise_parts
    else:
        noise_dx, noise_dy, noise_dyaw_deg = 0.0, 0.0, 0.0
    import math
    noise_dyaw_rad = math.radians(noise_dyaw_deg)

    resolved = resolve_profile(
        ablation_profile=ablation_profile,
        planner_backend=planner_backend,
        odom_mode=odom_mode,
        use_se2_merge=use_se2_merge_arg,
        use_cpp_cloud_merger=use_cpp_cloud_arg,
    )
    use_gt_odom: bool = resolved["use_gt_odom"]
    use_se2_merge: bool = resolved["use_se2_merge"]
    use_cpp_cloud: bool = resolved["use_cpp_cloud_merger"]
    backend: str = resolved["planner_backend"]

    world_canon = canonical_world_name(world_arg)
    world_src = resolve_world_file(world_arg, repo_root)
    if not world_src.is_file():
        raise FileNotFoundError(f"World file not found: {world_src}")

    spawns = sample_spawn_poses(world_canon, num_robots=num_robots, spawn_seed=spawn_seed)
    namespaces = [f"robot{i + 1}" for i in range(num_robots)]

    tmp_dir = Path(f"/tmp/hier_explore_bringup_{os.getpid()}")
    tmp_dir.mkdir(parents=True, exist_ok=True)

    # Expand world SDF if .xacro
    if world_src.name.endswith(".xacro"):
        world_sdf = _expand_xacro(
            world_src,
            tmp_dir / f"{world_canon}_world.sdf",
            {"headless": "True" if headless else "False"},
        )
    else:
        world_sdf = world_src

    use_shared_map = use_se2_merge and bool(resolved["se2_merge_params"].get("publish_per_robot_shared_map", True))
    nav2_params = _prepare_nav2_params(
        repo_root / "config" / "nav2_multirobot_slam_params.yaml",
        tmp_dir,
        use_se2_merge=use_shared_map,
    )
    ekf_params = repo_root / "config" / "ekf_odom.yaml"
    se2_merge_params = repo_root / "config" / "se2_map_merge_params.yaml"
    octomap_params = repo_root / "config" / "octomap_params.yaml"
    coord_params = repo_root / "config" / "coordinator_params.yaml"
    explore_params = repo_root / "config" / "explore_params.yaml"
    legacy_merge_params = repo_root / "src" / "m-explore-ros2" / "map_merge" / "config" / "params.yaml"

    clock_bridge = repo_root / "config" / "tb3_clock_bridge.yaml"
    robot_bridge = repo_root / "config" / (
        "tb3_bridge_no_clock.yaml" if use_gt_odom else "tb3_bridge_ekf.yaml"
    )
    points_bridge = repo_root / "config" / "tb3_points_bridge.yaml"
    camera_bridge = repo_root / "config" / "tb3_camera_bridge.yaml"

    urdf_text = (sim_share / "urdf" / "turtlebot3_waffle.urdf").read_text(encoding="utf-8")
    robot_sdf_xacro = repo_root / "config" / "gz_waffle_3d_lidar.sdf.xacro"

    # Build GZ_SIM_RESOURCE_PATH combining TB3 models, Fuel/CMU cache (~/.gz/models), and worlds/models
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_paths = [
        p for p in [
            existing_gz_path,
            str(sim_share / "models"),
            str(sim_share.parent),
            str(Path.home() / ".gz" / "models"),
            str(repo_root / "worlds" / "models"),
        ]
        if p
    ]
    gz_resource_path = ":".join(gz_paths)

    spawn_summary = ", ".join(
        f"{ns}=({sp.x:.2f},{sp.y:.2f},{sp.yaw:.2f})" for ns, sp in zip(namespaces, spawns)
    )
    actions: List = [
        SetEnvironmentVariable("GZ_IP", os.environ.get("GZ_IP", "127.0.0.1")),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        LogInfo(
            msg=(
                f"[hier_explore_bringup] world={world_canon} ({world_sdf}) N={num_robots} "
                f"seed={spawn_seed} odom={resolved['odom_mode']} profile={resolved['profile'].name} "
                f"backend={backend} se2_merge={use_se2_merge} cpp_cloud={use_cpp_cloud} "
                f"spawns=[{spawn_summary}]"
            )
        ),
    ]

    # 1. Gazebo server + dedicated /clock bridge (use_sim_time=False)
    gz_cmd = ["gz", "sim", "-r", "-s"]
    if headless:
        gz_cmd.append("--headless-rendering")
    gz_cmd.append(str(world_sdf))
    actions.append(ExecuteProcess(cmd=gz_cmd, output="screen", name="gz_sim_server"))

    if not headless:
        actions.append(ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen", name="gz_sim_gui"))

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            output="screen",
            parameters=[{"config_file": str(clock_bridge), "use_sim_time": False}],
        )
    )

    # 2. C++ se2_map_merge_node starts before Nav2 so /<ns>/shared_map is available immediately
    if use_se2_merge:
        se2_dyn_params: Dict[str, object] = {
            "use_sim_time": True,
            "robot_namespaces": namespaces,
        }
        for idx_sp, (ns, sp) in enumerate(zip(namespaces, spawns)):
            base_pose = [0.0, 0.0, 0.0] if use_gt_odom else sp.as_list()
            if idx_sp >= 1 and (abs(noise_dx) > 1e-9 or abs(noise_dy) > 1e-9 or abs(noise_dyaw_rad) > 1e-9):
                sign = 1.0 if (idx_sp % 2 == 1) else -1.0
                base_pose = [
                    base_pose[0] + sign * noise_dx,
                    base_pose[1] + sign * noise_dy,
                    base_pose[2] + sign * noise_dyaw_rad,
                ]
            se2_dyn_params[f"{ns}_init_pose"] = base_pose
        se2_dyn_params.update(resolved["se2_merge_params"])
        actions.append(
            TimerAction(
                period=2.5,
                actions=[
                    Node(
                        package="se2_map_merge",
                        executable="se2_map_merge_node",
                        name="se2_map_merge",
                        output="screen",
                        parameters=[str(se2_merge_params), se2_dyn_params],
                    )
                ],
            )
        )

    # 3. Per-robot stacks (robot1 .. robotN), staggered to prevent startup CPU contention
    for idx, (ns, sp) in enumerate(zip(namespaces, spawns)):
        robot_sdf = _expand_xacro(
            robot_sdf_xacro,
            tmp_dir / f"{ns}.sdf",
            {
                "namespace": ns,
                "use_gt_odom": "true" if use_gt_odom else "false",
                "lidar_v_samples": lidar_v_samples,
            },
        )
        start_t = 3.0 + idx * robot_stagger_sec
        spawn_z = 0.02 + 0.01 * idx

        robot_nodes: List = [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=ns,
                output="screen",
                parameters=[{"use_sim_time": True, "robot_description": urdf_text}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                namespace=ns,
                output="screen",
                arguments=[
                    "-name", ns,
                    "-file", str(robot_sdf),
                    "-x", f"{sp.x:.4f}",
                    "-y", f"{sp.y:.4f}",
                    "-z", f"{spawn_z:.3f}",
                    "-R", "0.0",
                    "-P", "0.0",
                    "-Y", f"{sp.yaw:.4f}",
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{ns}_core_bridge",
                namespace=ns,
                output="screen",
                parameters=[
                    {
                        "config_file": str(robot_bridge),
                        "use_sim_time": True,
                        "expand_gz_topic_names": True,
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{ns}_points_bridge",
                namespace=ns,
                output="screen",
                parameters=[
                    {
                        "config_file": str(points_bridge),
                        "use_sim_time": True,
                        "expand_gz_topic_names": True,
                    }
                ],
            ),
        ]

        if not use_gt_odom:
            robot_nodes.append(
                Node(
                    package="robot_localization",
                    executable="ekf_node",
                    name="ekf_filter_node",
                    namespace=ns,
                    output="screen",
                    parameters=[str(ekf_params), {"use_sim_time": True}],
                    remappings=[
                        ("odometry/filtered", "odom"),
                        ("/tf", "tf"),
                        ("/tf_static", "tf_static"),
                    ],
                )
            )

        if enable_cameras:
            robot_nodes.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"{ns}_camera_bridge",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "config_file": str(camera_bridge),
                            "use_sim_time": True,
                            "expand_gz_topic_names": True,
                        }
                    ],
                )
            )

        # Nav2 + slam_toolbox launched 2 s after robot spawn & bridges
        nav2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_bringup_share / "launch" / "bringup_launch.py")),
            launch_arguments={
                "namespace": ns,
                "use_namespace": "True",
                "slam": "True",
                "use_keepout_zones": "False",
                "use_speed_zones": "False",
                "use_sim_time": "True",
                "params_file": str(nav2_params),
                "autostart": "True",
                "use_composition": use_composition,
                "use_respawn": "True",
            }.items(),
        )

        actions.append(TimerAction(period=start_t, actions=[GroupAction(robot_nodes)]))
        actions.append(TimerAction(period=start_t + 2.0, actions=[nav2_include]))

    # 4. Post-bringup global fusion & autonomous exploration backend
    if explorer_delay_arg in ("", "auto"):
        explorer_delay = 18.0 + (num_robots - 1) * max(robot_stagger_sec, 2.0)
    else:
        explorer_delay = float(explorer_delay_arg)

    post_actions: List = []

    if not use_se2_merge:
        legacy_params: Dict[str, object] = {
            "use_sim_time": True,
            "known_init_poses": True,
            "merging_rate": 1.5,
            "discovery_rate": 1.0,
            "world_frame": "world",
            "merged_map_topic": "map",
            "robot_map_topic": "map",
            "robot_map_updates_topic": "map_updates",
            "robot_namespace": "robot",
        }
        for ns, sp in zip(namespaces, spawns):
            ix, iy, iyaw = (0.0, 0.0, 0.0) if use_gt_odom else (sp.x, sp.y, sp.yaw)
            legacy_params[f"/{ns}/map_merge/init_pose_x"] = ix
            legacy_params[f"/{ns}/map_merge/init_pose_y"] = iy
            legacy_params[f"/{ns}/map_merge/init_pose_z"] = 0.0
            legacy_params[f"/{ns}/map_merge/init_pose_yaw"] = iyaw
        post_actions.append(
            Node(
                package="multirobot_map_merge",
                executable="map_merge",
                name="map_merge",
                output="screen",
                parameters=[str(legacy_merge_params), legacy_params],
            )
        )

    if use_cpp_cloud:
        post_actions.append(
            Node(
                package="se2_map_merge",
                executable="cloud_merger_node",
                name="cloud_merger",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robots": namespaces,
                        "world_frame": "world",
                        "voxel_size": 0.04,
                        "publish_rate": 1.0,
                    }
                ],
            )
        )
    else:
        post_actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(repo_root / "scripts" / "merge_pointclouds.py"),
                    "--ros-args",
                    "-p", "use_sim_time:=true",
                    "-p", f"robots:={namespaces!r}",
                    "-p", "world_frame:=world",
                    "-p", "voxel_size:=0.02",
                    "-p", "publish_rate:=2.0",
                ],
                output="screen",
                name="merge_pointclouds_py",
            )
        )

    post_actions.append(
        Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="octomap_server",
            output="screen",
            parameters=[
                str(octomap_params),
                {"use_sim_time": True, "resolution": voxel_resolution},
            ],
            remappings=[("cloud_in", "/points_merged")],
        )
    )

    if backend == "coordinator":
        coord_dyn: Dict[str, object] = {
            "use_sim_time": True,
            "robot_namespaces": namespaces,
            "voxel_index_resolution": voxel_resolution,
        }
        for ns, sp in zip(namespaces, spawns):
            coord_dyn[f"{ns}_init_pose"] = sp.as_list()
        coord_dyn.update(resolved["coordinator_params"])
        post_actions.append(
            Node(
                package="hier_explore_core",
                executable="coordinator_node",
                name="multi_robot_coordinator",
                output="screen",
                parameters=[str(coord_params), coord_dyn],
            )
        )
    elif backend == "legacy":
        for ns in namespaces:
            post_actions.append(
                Node(
                    package="explore_lite",
                    executable="explore",
                    name="explore_node",
                    namespace=ns,
                    output="screen",
                    parameters=[str(explore_params), {"use_sim_time": True}],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                )
            )

    if use_rviz:
        rviz_cfg = repo_root / "src" / "m-explore-ros2" / "map_merge" / "launch" / "map_merge.rviz"
        post_actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(rviz_cfg)],
                parameters=[{"use_sim_time": True}],
            )
        )

    actions.append(TimerAction(period=explorer_delay, actions=post_actions))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="2", description="Number of robots (1..4)."),
            DeclareLaunchArgument(
                "world",
                default_value="mine",
                description="World preset (house|hospital|mine|warehouse|cmu_indoor|cmu_tunnel) or SDF path.",
            ),
            DeclareLaunchArgument(
                "spawn_seed",
                default_value="0",
                description="0 for curated default formation; >0 for deterministic random station sampling.",
            ),
            DeclareLaunchArgument(
                "odom_mode",
                default_value="ekf",
                description="Odometry mode: 'ekf' (noisy wheel odom + IMU EKF) or 'gt' (Gazebo GT odom).",
            ),
            DeclareLaunchArgument(
                "ablation_profile",
                default_value="full",
                description="Profile: legacy|e1_2d|e2_3dgain|e3_hier|e4_geodesic|e5_deconflict|full|no_se2|no_degen_gate.",
            ),
            DeclareLaunchArgument(
                "planner_backend",
                default_value="auto",
                description="Exploration backend: auto|coordinator|legacy|off.",
            ),
            DeclareLaunchArgument("headless", default_value="true", description="Run Gazebo headless."),
            DeclareLaunchArgument("use_rviz", default_value="false", description="Launch RViz2."),
            DeclareLaunchArgument("use_se2_merge", default_value="auto", description="auto|true|false."),
            DeclareLaunchArgument("use_cpp_cloud_merger", default_value="auto", description="auto|true|false."),
            DeclareLaunchArgument("enable_cameras", default_value="false", description="Bridge RGB cameras."),
            DeclareLaunchArgument("use_composition", default_value="false", description="Nav2 component container."),
            DeclareLaunchArgument("robot_stagger_sec", default_value="3.0", description="Per-robot spawn stagger (s)."),
            DeclareLaunchArgument(
                "explorer_start_delay_sec",
                default_value="auto",
                description="Delay before starting cloud merger, octomap, and explorer ('auto' or seconds).",
            ),
            DeclareLaunchArgument(
                "lidar_v_samples",
                default_value="16",
                description="3D LiDAR vertical beam count (16 or 32 for sensitivity study).",
            ),
            DeclareLaunchArgument(
                "voxel_resolution",
                default_value="0.10",
                description="OctoMap & 2.5D VoxelColumnIndex resolution in meters (0.05|0.10|0.15).",
            ),
            DeclareLaunchArgument(
                "init_pose_noise",
                default_value="0.0,0.0,0.0",
                description="Perturbation dx_m,dy_m,dyaw_deg added to robot2..N map_merge init_pose prior.",
            ),
            OpaqueFunction(function=_build_launch_actions),
        ]
    )
