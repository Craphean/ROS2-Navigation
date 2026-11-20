"""Unit tests validating M2 localization, EKF, bridge, and SE(2) map-merge configs."""

from __future__ import annotations

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
XACRO_NS = "http://www.ros.org/wiki/xacro"


def _eval_xacro_conditionals(path: Path, args: dict[str, str]) -> ET.Element:
    """Minimal xacro evaluator for $(arg ...), <xacro:if>, and <xacro:unless>."""
    text = path.read_text(encoding="utf-8")
    root = ET.fromstring(text)

    resolved = dict(args)
    for elem in list(root):
        if elem.tag == f"{{{XACRO_NS}}}arg":
            name = elem.attrib["name"]
            if name not in resolved:
                resolved[name] = elem.attrib.get("default", "")
            root.remove(elem)

    def _subst(s: str) -> str:
        for k, v in resolved.items():
            s = s.replace(f"$(arg {k})", v)
        return s

    def _is_truthy(val: str) -> bool:
        return val.strip().lower() in ("true", "1", "yes")

    def _expand(parent: ET.Element) -> None:
        i = 0
        while i < len(parent):
            child = parent[i]
            if child.tag in (f"{{{XACRO_NS}}}if", f"{{{XACRO_NS}}}unless"):
                cond = _is_truthy(_subst(child.attrib["value"]))
                keep = cond if child.tag.endswith("}if") else (not cond)
                parent.remove(child)
                if keep:
                    for offset, sub in enumerate(list(child)):
                        parent.insert(i + offset, sub)
                continue
            if child.text:
                child.text = _subst(child.text)
            for k, v in list(child.attrib.items()):
                child.attrib[k] = _subst(v)
            _expand(child)
            i += 1

    _expand(root)
    return root


def _plugins_by_name(root: ET.Element) -> dict[str, ET.Element]:
    return {p.attrib["name"]: p for p in root.findall(".//plugin")}


def test_robot_xacro_default_uses_noisy_diffdrive_and_eval_gt_pose():
    root = _eval_xacro_conditionals(
        REPO_ROOT / "config" / "gz_waffle_3d_lidar.sdf.xacro",
        {"namespace": "robot1"},
    )
    plugins = _plugins_by_name(root)
    diff = plugins["gz::sim::systems::DiffDrive"]
    assert diff.findtext("odom_publish_frequency") == "30"
    assert diff.findtext("odom_topic") == "robot1/wheel_odom"
    assert diff.findtext("tf_topic") == "robot1/wheel_tf_unused"
    assert diff.findtext("frame_id") == "odom"
    assert diff.findtext("child_frame_id") == "base_footprint"

    gt = plugins["gz::sim::systems::OdometryPublisher"]
    assert gt.findtext("odom_topic") == "robot1/gt_pose"
    assert gt.findtext("tf_topic") == "robot1/gt_tf_unused"
    assert gt.findtext("odom_frame") == "world"
    assert gt.findtext("odom_publish_frequency") == "10"


def test_robot_xacro_gt_odom_toggle_restores_gt_odom_and_tf():
    root = _eval_xacro_conditionals(
        REPO_ROOT / "config" / "gz_waffle_3d_lidar.sdf.xacro",
        {"namespace": "robot2", "use_gt_odom": "true"},
    )
    plugins = _plugins_by_name(root)
    diff = plugins["gz::sim::systems::DiffDrive"]
    assert diff.findtext("odom_publish_frequency") == "0"
    assert diff.find("odom_topic") is None

    gt = plugins["gz::sim::systems::OdometryPublisher"]
    assert gt.findtext("odom_topic") == "robot2/odom"
    assert gt.findtext("tf_topic") == "robot2/tf"
    assert gt.findtext("odom_frame") == "odom"
    assert gt.findtext("odom_publish_frequency") == "30"


def test_ekf_odom_yaml_schema():
    cfg = yaml.safe_load((REPO_ROOT / "config" / "ekf_odom.yaml").read_text(encoding="utf-8"))
    params = cfg["/**"]["ros__parameters"]
    assert params["two_d_mode"] is True
    assert params["publish_tf"] is True
    assert params["odom_frame"] == "odom"
    assert params["base_link_frame"] == "base_footprint"
    assert params["world_frame"] == "odom"
    # Relative topics so per-robot namespaces resolve cleanly
    assert params["odom0"] == "wheel_odom"
    assert params["imu0"] == "imu"
    assert len(params["odom0_config"]) == 15
    assert len(params["imu0_config"]) == 15
    assert len(params["process_noise_covariance"]) == 225
    assert params["imu0_remove_gravitational_acceleration"] is True
    # IMU owns vyaw (index 11); wheel odom owns vx, vy (indices 6, 7)
    assert params["odom0_config"][6] is True
    assert params["odom0_config"][7] is True
    assert params["odom0_config"][11] is False
    assert params["imu0_config"][11] is True


def test_tb3_bridge_ekf_yaml_excludes_tf_and_includes_wheel_odom_and_gt_pose():
    entries = yaml.safe_load(
        (REPO_ROOT / "config" / "tb3_bridge_ekf.yaml").read_text(encoding="utf-8")
    )
    ros_topics = {e["ros_topic_name"]: e for e in entries}
    assert "tf" not in ros_topics
    assert "odom" not in ros_topics
    for expected in ("cmd_vel", "wheel_odom", "gt_pose", "scan", "imu", "joint_states"):
        assert expected in ros_topics
    assert ros_topics["wheel_odom"]["ros_type_name"] == "nav_msgs/msg/Odometry"
    assert ros_topics["gt_pose"]["ros_type_name"] == "nav_msgs/msg/Odometry"


def test_se2_map_merge_params_yaml():
    cfg = yaml.safe_load(
        (REPO_ROOT / "config" / "se2_map_merge_params.yaml").read_text(encoding="utf-8")
    )
    params = cfg["/**"]["ros__parameters"]
    assert params["enable_online_alignment"] is True
    assert params["enable_phase_corr"] is True
    assert params["enable_degeneracy_gate"] is True
    assert 0.0 < params["max_step_xy_m"] <= 0.05
    assert params["min_trans_eigenvalue"] > 0.0
    assert params["max_trans_cond_number"] > 1.0


def test_shell_scripts_syntax():
    for rel in ("scripts/tb3_multi_explore_slam.sh", "scripts/kill_nav2_sim.sh"):
        proc = subprocess.run(
            ["bash", "-n", str(REPO_ROOT / rel)],
            capture_output=True,
            text=True,
            check=False,
        )
        assert proc.returncode == 0, f"bash -n failed for {rel}: {proc.stderr}"
