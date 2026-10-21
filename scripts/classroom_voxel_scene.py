"""Load the repository's actual classroom collision geometry into fine voxels."""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml

from voxel_exploration_core import FREE, OCCUPIED, GroundGrid, SensorModel, VoxelMap

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORLD = ROOT / "worlds/classroom.sdf.xacro"
DEFAULT_CONFIG = ROOT / "config/voxel360_strong_baseline_params.yaml"


def pose_matrix(element):
    pose = element.find("pose")
    values = np.zeros(6) if pose is None else np.fromstring(pose.text, sep=" ")
    if len(values) != 6 or (pose is not None and pose.get("relative_to")):
        raise ValueError("Expected a six-number parent-relative SDF pose")
    x, y, z, roll, pitch, yaw = values
    if abs(roll) > 1e-10 or abs(pitch) > 1e-10:
        raise ValueError("Classroom voxelizer currently supports yaw-only box poses")
    c, s = np.cos(yaw), np.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    transform[:3, 3] = [x, y, z]
    return transform


@dataclass
class Box:
    name: str
    center: np.ndarray
    rotation: np.ndarray
    size: np.ndarray


def load_boxes(path=DEFAULT_WORLD):
    # Expand exactly the xacro macros used by the Gazebo launcher.
    document = xacro.process_file(str(path), mappings={"headless": "true"})
    root = ET.fromstring(document.toxml())
    world = root.find("world")
    if world is None:
        raise ValueError("SDF contains no world")
    boxes = []

    def read_model(model, parent, prefix=""):
        transform = parent @ pose_matrix(model)
        name = prefix + model.get("name", "")
        for link in model.findall("link"):
            link_transform = transform @ pose_matrix(link)
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                if geometry.find("plane") is not None:
                    continue  # The ground plane is below the configured z ROI.
                size = geometry.find("box/size")
                if size is None:
                    raise ValueError(f"Unsupported collision geometry in {name}")
                collision_transform = link_transform @ pose_matrix(collision)
                boxes.append(Box(f"{name}/{collision.get('name')}",
                                 collision_transform[:3, 3], collision_transform[:3, :3],
                                 np.fromstring(size.text, sep=" ")))
        for child in model.findall("model"):
            read_model(child, transform, name + "/")

    for model in world.findall("model"):
        read_model(model, np.eye(4))
    return boxes


def voxelize_boxes(boxes, bounds, resolution):
    """Conservative box/cell overlap, including thin tabletops and 6 cm legs.

    Uses separating axes in XY and exact Z interval overlap, not center-only
    sampling (which would lose furniture thinner than one voxel).
    """
    result = VoxelMap(bounds, resolution)
    result.state[:] = FREE
    for box in boxes:
        half = box.size / 2
        aabb = np.abs(box.rotation) @ half
        lo = np.maximum(0, np.floor((box.center - aabb - result.origin) / resolution).astype(int))
        hi = np.minimum(result.shape_xyz, np.ceil((box.center + aabb - result.origin) / resolution).astype(int))
        if np.any(hi <= lo):
            continue
        x = result.origin[0] + (np.arange(lo[0], hi[0]) + .5) * resolution
        y = result.origin[1] + (np.arange(lo[1], hi[1]) + .5) * resolution
        xx, yy = np.meshgrid(x, y)
        dx, dy = xx - box.center[0], yy - box.center[1]
        r = box.rotation
        cell_half = resolution / 2
        # World X/Y axes; local box X/Y axes.
        overlap = (np.abs(dx) < aabb[0] + cell_half - 1e-10)
        overlap &= np.abs(dy) < aabb[1] + cell_half - 1e-10
        for axis in (0, 1):
            projected = r[0, axis] * dx + r[1, axis] * dy
            radius = half[axis] + cell_half * (abs(r[0, axis]) + abs(r[1, axis]))
            overlap &= np.abs(projected) < radius - 1e-10
        view = result.state[lo[2]:hi[2], lo[1]:hi[1], lo[0]:hi[0]]
        z = result.origin[2] + (np.arange(lo[2], hi[2]) + .5) * resolution
        z_overlap = np.abs(z - box.center[2]) < half[2] + cell_half - 1e-10
        view[z_overlap[:, None, None] & overlap[None]] = OCCUPIED
    return result


def scene(config_path=DEFAULT_CONFIG, world_path=DEFAULT_WORLD, resolution=None):
    config = yaml.safe_load(Path(config_path).read_text())["rolling_observation_planner"]["ros__parameters"]
    boxes = load_boxes(world_path)
    resolution = float(config["voxel_resolution"] if resolution is None else resolution)
    truth = voxelize_boxes(boxes, config["exploration_bounds"], resolution)
    known = truth.snapshot(empty=True)
    bottom = truth.origin[2] + np.arange(truth.state.shape[0]) * resolution
    body = ((bottom + resolution > config["robot_body_z_min"]) & (bottom < config["robot_body_z_max"]))
    ground = GroundGrid(np.where(np.any(truth.state[body] == OCCUPIED, axis=0), OCCUPIED, FREE),
                        resolution, truth.origin[:2], config["min_clearance"])
    # Same classroom spawn positions as tb3_multi_explore_slam.sh.
    starts = [tuple(config["robot1_init_pose"][:2]), tuple(config["robot2_init_pose"][:2])]
    sensor = SensorModel(config["observation_z"], config["sensor_min_range"],
                         config["max_obs_range"], config["sensor_vfov_min"],
                         config["sensor_vfov_max"], config["sensor_offset_x"], config["sensor_offset_y"])
    for xy in starts:
        if not ground.valid(ground.cell(xy), ground.safe):
            raise ValueError(f"Configured classroom spawn {xy} is not on safe ground")
    return truth, known, ground, starts, sensor, boxes, config
