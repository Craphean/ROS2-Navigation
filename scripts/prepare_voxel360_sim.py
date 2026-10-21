#!/usr/bin/env python3
"""Generate matching Gazebo and position-only Nav2 assets for the 360 method."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from lidar360_observation import DEFAULT_PROFILE, Lidar360

ROOT = Path(__file__).resolve().parents[1]


def prepare(output, profile_path=DEFAULT_PROFILE, nav_path=None):
    profile = Lidar360.load(profile_path)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
    tree = ET.parse(ROOT/"config/gz_waffle_3d_lidar.sdf.xacro")
    spatial = tree.find(".//sensor[@name='tb3_3d_lidar']")
    original = np.fromstring(spatial.findtext("pose"), sep=" ")
    base = yaml.safe_load((ROOT/"config/voxel360_strong_baseline_params.yaml").read_text())["rolling_observation_planner"]["ros__parameters"]
    pose = np.array([0., 0., original[2]+(profile.height-base["observation_z"]), 0., 0., 0.])
    for name in ("hls_lfcd_lds", "tb3_3d_lidar"):
        sensor = tree.find(f".//sensor[@name='{name}']")
        sensor.find("pose").text = " ".join(map(str, pose))
        sensor.find("frame_id").text = "lidar360"
        sensor.find("update_rate").text = str(profile.scan_hz)
        horizontal = sensor.find("ray/scan/horizontal")
        horizontal.find("samples").text = str(profile.horizontal_samples)
        horizontal.find("min_angle").text = str(np.deg2rad(profile.azimuth_phase_deg))
        # SDF samples include both endpoints; exclude the duplicated seam.
        horizontal.find("max_angle").text = str(
            np.deg2rad(profile.azimuth_phase_deg)+2*np.pi*(profile.horizontal_samples-1)/profile.horizontal_samples)
        sensor.find("ray/range/min").text = str(profile.min_range)
        sensor.find("ray/range/max").text = str(profile.max_range)
    vertical = spatial.find("ray/scan/vertical")
    vertical.find("samples").text = str(profile.vertical_samples)
    vertical.find("min_angle").text = str(np.deg2rad(profile.vertical_min_deg))
    vertical.find("max_angle").text = str(np.deg2rad(profile.vertical_max_deg))
    ET.indent(tree, space="  ")
    tree.write(output/"robot.sdf.xacro", encoding="utf-8", xml_declaration=True)
    # base_scan's existing TF corresponds to the original 3D sensor pose.
    (output/"lidar_tf.xyz").write_text(" ".join(map(str, pose[:3]-original[:3]))+"\n")

    nav = yaml.safe_load(Path(nav_path or ROOT/"config/nav2_multirobot_slam_params.yaml").read_text())
    controller = nav["controller_server"]["ros__parameters"]
    for name in controller["goal_checker_plugins"]:
        controller[name]["plugin"] = "nav2_controller::PositionGoalChecker"
        # The inherited 25 cm tolerance can report success without driving to
        # the next 25 cm-spaced observation position. Resolve arrival within
        # half a planning voxel instead.
        controller[name]["xy_goal_tolerance"] = min(
            float(controller[name].get("xy_goal_tolerance", .25)), .5*float(base["voxel_resolution"]))
        controller[name].pop("yaw_goal_tolerance", None)
    controller["FollowPath"]["GoalAngleCritic"]["enabled"] = False
    nav["planner_server"]["ros__parameters"]["GridBased"]["use_final_approach_orientation"] = True
    (output/"nav2.yaml").write_text(yaml.safe_dump(nav, sort_keys=False))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--nav-params", type=Path, default=ROOT/"config/nav2_multirobot_slam_params.yaml")
    args = parser.parse_args()
    prepare(args.output_dir, args.profile, args.nav_params)


if __name__ == "__main__":
    main()
