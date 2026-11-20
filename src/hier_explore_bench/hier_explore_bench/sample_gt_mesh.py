#!/usr/bin/env python3
"""Ground-truth 3D surface point-cloud and 2D explorable-area sampler for Gazebo SDF/Xacro worlds.

Supports:
  - Native ROS ``xacro`` CLI expansion when available, with a built-in pure-Python
    Xacro macro expander fallback so SDF worlds (including recursive macros in
    ``worlds/mine.sdf.xacro``) can be sampled offline without ROS installed.
  - Primitive collision geometries: ``<box>``, ``<cylinder>``, ``<sphere>``, plus
    ``<mesh>`` (``.obj`` and binary/ASCII ``.stl``) for CMU/SubT benchmark worlds.
  - Flood-filled 2D interior explorable free-space rasterization from a seed pose,
    producing exact ``A_GT`` and 2D GT occupancy grids for IoU/coverage evaluation.

Usage:
  python3 -m hier_explore_bench.sample_gt_mesh \
      --world worlds/mine.sdf.xacro \
      --seed -19.0 0.0 \
      --out-dir maps/gt/mine
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import shutil
import struct
import subprocess
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .pcd_io import voxel_downsample, write_pcd

XACRO_NS = "http://www.ros.org/wiki/xacro"
_EXPR_RE = re.compile(r"\$\{([^}]+)\}")
_ARG_RE = re.compile(r"\$\(arg\s+([^)]+)\)")


# ---------------------------------------------------------------------------
# Pure-Python minimal Xacro expander (fallback when ROS xacro CLI is absent)
# ---------------------------------------------------------------------------


def _eval_expr(text: str, env: Dict[str, object], args: Dict[str, str]) -> str:
    if text is None:
        return ""
    s = _ARG_RE.sub(lambda m: str(args.get(m.group(1).strip(), "")), text)

    def _repl(match: re.Match) -> str:
        expr = match.group(1).strip()
        safe_globals = {
            "__builtins__": {},
            "int": int,
            "float": float,
            "str": str,
            "bool": bool,
            "abs": abs,
            "min": min,
            "max": max,
            "round": round,
            "pi": math.pi,
            "sin": math.sin,
            "cos": math.cos,
            "tan": math.tan,
            "sqrt": math.sqrt,
        }
        val = eval(expr, safe_globals, env)  # noqa: S307 - controlled xacro arithmetic
        return str(val)

    return _EXPR_RE.sub(_repl, s)


def _strip_ns(tag: str) -> Tuple[str, str]:
    if tag.startswith("{"):
        ns, local = tag[1:].split("}", 1)
        return ns, local
    return "", tag


def _truthy(val: str) -> bool:
    return str(val).strip().lower() not in ("", "0", "false", "no", "none")


def expand_xacro_xml(xml_text: str, extra_args: Optional[Dict[str, str]] = None) -> ET.Element:
    """Expand xacro:arg, xacro:property, xacro:if/unless, and recursive xacro:macro."""
    root = ET.fromstring(xml_text)
    args: Dict[str, str] = {}
    if extra_args:
        args.update({k: str(v) for k, v in extra_args.items()})
    macros: Dict[str, Tuple[List[str], List[ET.Element]]] = {}
    props: Dict[str, object] = {}

    def _expand_children(elem: ET.Element, env: Dict[str, object]) -> List[ET.Element]:
        out: List[ET.Element] = []
        for child in list(elem):
            ns, local = _strip_ns(child.tag)
            if ns == XACRO_NS:
                if local == "arg":
                    name = child.attrib["name"]
                    if name not in args:
                        args[name] = child.attrib.get("default", "")
                elif local == "property":
                    name = child.attrib["name"]
                    val_str = _eval_expr(child.attrib.get("value", ""), env, args)
                    try:
                        props[name] = float(val_str)
                    except ValueError:
                        props[name] = val_str
                    env[name] = props[name]
                elif local == "macro":
                    name = child.attrib["name"]
                    param_names = [
                        p.split(":=")[0].strip()
                        for p in child.attrib.get("params", "").split()
                        if p.strip()
                    ]
                    macros[name] = (param_names, [copy.deepcopy(c) for c in child])
                elif local == "if":
                    cond = _eval_expr(child.attrib.get("value", ""), env, args)
                    if _truthy(cond):
                        out.extend(_expand_children(child, dict(env)))
                elif local == "unless":
                    cond = _eval_expr(child.attrib.get("value", ""), env, args)
                    if not _truthy(cond):
                        out.extend(_expand_children(child, dict(env)))
                elif local in macros:
                    param_names, body = macros[local]
                    call_env = dict(env)
                    for pname in param_names:
                        raw_val = child.attrib.get(pname, "")
                        eval_val = _eval_expr(raw_val, env, args)
                        try:
                            call_env[pname] = float(eval_val) if "." in eval_val else int(eval_val)
                        except ValueError:
                            call_env[pname] = eval_val
                    holder = ET.Element("holder")
                    for b in body:
                        holder.append(copy.deepcopy(b))
                    out.extend(_expand_children(holder, call_env))
                continue

            new_elem = ET.Element(local)
            for k, v in child.attrib.items():
                new_elem.attrib[k] = _eval_expr(v, env, args)
            if child.text is not None:
                new_elem.text = _eval_expr(child.text, env, args)
            for expanded in _expand_children(child, env):
                new_elem.append(expanded)
            out.append(new_elem)
        return out

    result = ET.Element(_strip_ns(root.tag)[1], root.attrib)
    for expanded in _expand_children(root, dict(props)):
        result.append(expanded)
    return result


def load_sdf_root(world_path: Path) -> ET.Element:
    """Load an .sdf or .sdf.xacro file into an expanded XML ElementTree root."""
    text = world_path.read_text(encoding="utf-8")
    if world_path.suffix == ".xacro" or "xmlns:xacro" in text:
        if shutil.which("xacro"):
            proc = subprocess.run(
                ["xacro", str(world_path), "headless:=true"],
                capture_output=True,
                text=True,
                check=False,
            )
            if proc.returncode == 0 and proc.stdout.strip():
                return ET.fromstring(proc.stdout)
        return expand_xacro_xml(text, {"headless": "true"})
    return ET.fromstring(text)


# ---------------------------------------------------------------------------
# SE(3) transforms & primitive / mesh surface sampling
# ---------------------------------------------------------------------------


def _parse_pose(elem: Optional[ET.Element]) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    if elem is None or not (elem.text or "").strip():
        return T
    vals = [float(x) for x in elem.text.strip().split()]
    if len(vals) < 6:
        vals = vals + [0.0] * (6 - len(vals))
    x, y, z, roll, pitch, yaw = vals[:6]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]
    return T


def _transform_points(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    if pts.size == 0:
        return pts
    return (pts @ T[:3, :3].T) + T[:3, 3]


def sample_box_surface(sx: float, sy: float, sz: float, step: float) -> np.ndarray:
    """Uniformly sample all 6 faces of an axis-aligned box centered at the origin."""
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    nx = max(2, int(math.ceil(sx / step)) + 1)
    ny = max(2, int(math.ceil(sy / step)) + 1)
    nz = max(2, int(math.ceil(sz / step)) + 1)
    xs = np.linspace(-hx, hx, nx)
    ys = np.linspace(-hy, hy, ny)
    zs = np.linspace(-hz, hz, nz)

    faces = []
    # +/- X faces
    yy, zz = np.meshgrid(ys, zs, indexing="ij")
    for sign in (-1.0, 1.0):
        xx = np.full_like(yy, sign * hx)
        faces.append(np.column_stack((xx.ravel(), yy.ravel(), zz.ravel())))
    # +/- Y faces
    xx, zz = np.meshgrid(xs, zs, indexing="ij")
    for sign in (-1.0, 1.0):
        yy = np.full_like(xx, sign * hy)
        faces.append(np.column_stack((xx.ravel(), yy.ravel(), zz.ravel())))
    # +/- Z faces
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    for sign in (-1.0, 1.0):
        zz = np.full_like(xx, sign * hz)
        faces.append(np.column_stack((xx.ravel(), yy.ravel(), zz.ravel())))
    return np.vstack(faces)


def sample_cylinder_surface(radius: float, length: float, step: float) -> np.ndarray:
    """Sample lateral surface and caps of a Z-aligned cylinder centered at origin."""
    n_theta = max(12, int(math.ceil(2.0 * math.pi * radius / step)))
    n_z = max(2, int(math.ceil(length / step)) + 1)
    thetas = np.linspace(0.0, 2.0 * math.pi, n_theta, endpoint=False)
    zs = np.linspace(-0.5 * length, 0.5 * length, n_z)
    tt, zz = np.meshgrid(thetas, zs, indexing="ij")
    lat = np.column_stack(
        (radius * np.cos(tt).ravel(), radius * np.sin(tt).ravel(), zz.ravel())
    )
    return lat


def sample_sphere_surface(radius: float, step: float) -> np.ndarray:
    """Fibonacci-lattice uniform sampling on a sphere of given radius."""
    area = 4.0 * math.pi * radius * radius
    n = max(24, int(math.ceil(area / (step * step))))
    idx = np.arange(n, dtype=np.float64) + 0.5
    phi = np.arccos(1.0 - 2.0 * idx / n)
    theta = math.pi * (1.0 + math.sqrt(5.0)) * idx
    return np.column_stack(
        (radius * np.sin(phi) * np.cos(theta), radius * np.sin(phi) * np.sin(theta), radius * np.cos(phi))
    )


def sample_triangles(
    vertices: np.ndarray, faces: np.ndarray, step: float, rng: np.random.Generator
) -> np.ndarray:
    """Area-weighted uniform sampling on a triangle mesh."""
    if len(faces) == 0:
        return np.empty((0, 3), dtype=np.float64)
    v0 = vertices[faces[:, 0]]
    v1 = vertices[faces[:, 1]]
    v2 = vertices[faces[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    counts = np.maximum(1, np.ceil(areas / (step * step)).astype(np.int64))
    total = int(counts.sum())
    if total == 0:
        return np.empty((0, 3), dtype=np.float64)
    tri_idx = np.repeat(np.arange(len(faces)), counts)
    r1 = np.sqrt(rng.random(total))[:, None]
    r2 = rng.random(total)[:, None]
    return (1.0 - r1) * v0[tri_idx] + r1 * (1.0 - r2) * v1[tri_idx] + r1 * r2 * v2[tri_idx]


def load_obj_mesh(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    verts: List[List[float]] = []
    faces: List[List[int]] = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        if parts[0] == "v" and len(parts) >= 4:
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == "f" and len(parts) >= 4:
            idxs = [int(tok.split("/")[0]) - 1 for tok in parts[1:]]
            for k in range(1, len(idxs) - 1):
                faces.append([idxs[0], idxs[k], idxs[k + 1]])
    return np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def load_stl_mesh(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Load binary or ASCII STL into (vertices, faces)."""
    raw = path.read_bytes()
    if len(raw) >= 84:
        n_tri = struct.unpack_from("<I", raw, 80)[0]
        if 84 + n_tri * 50 == len(raw) and n_tri > 0:
            dt = np.dtype([("norm", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            arr = np.frombuffer(raw, dtype=dt, count=n_tri, offset=84)
            verts = arr["v"].reshape(-1, 3).astype(np.float64)
            faces = np.arange(len(verts), dtype=np.int64).reshape(-1, 3)
            return verts, faces
    # ASCII STL fallback
    verts_list: List[List[float]] = []
    for line in raw.decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) >= 4 and parts[0].lower() == "vertex":
            verts_list.append([float(parts[1]), float(parts[2]), float(parts[3])])
    n = (len(verts_list) // 3) * 3
    if n == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.int64)
    verts = np.asarray(verts_list[:n], dtype=np.float64)
    faces = np.arange(n, dtype=np.int64).reshape(-1, 3)
    return verts, faces


def load_dae_mesh(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Lightweight Collada (.dae) mesh loader supporting triangles and polylists."""
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.int64)

    unit_scale = 1.0
    up_axis = "Z_UP"
    for elem in root.iter():
        _, local = _strip_ns(elem.tag)
        if local == "unit" and "meter" in elem.attrib:
            try:
                unit_scale = float(elem.attrib["meter"])
            except ValueError:
                pass
        elif local == "up_axis" and elem.text:
            up_axis = elem.text.strip().upper()

    all_verts: List[np.ndarray] = []
    all_faces: List[np.ndarray] = []
    vert_offset = 0

    for geom in root.iter():
        if _strip_ns(geom.tag)[1] != "mesh":
            continue
        sources: Dict[str, np.ndarray] = {}
        vertices_map: Dict[str, str] = {}
        for child in geom:
            _, tag = _strip_ns(child.tag)
            if tag == "source":
                sid = child.attrib.get("id", "")
                for sub in child:
                    if _strip_ns(sub.tag)[1] == "float_array" and sub.text:
                        vals = np.fromstring(sub.text, sep=" ", dtype=np.float64)
                        if vals.size % 3 == 0:
                            sources[sid] = vals.reshape(-1, 3)
            elif tag == "vertices":
                vid = child.attrib.get("id", "")
                for inp in child:
                    if _strip_ns(inp.tag)[1] == "input" and inp.attrib.get("semantic") == "POSITION":
                        vertices_map[vid] = inp.attrib.get("source", "").lstrip("#")

        for prim in geom:
            _, tag = _strip_ns(prim.tag)
            if tag not in ("triangles", "polylist", "polygons"):
                continue
            v_off = 0
            stride = 1
            pos_id = ""
            p_elem = None
            vcount_elem = None
            for sub in prim:
                _, stag = _strip_ns(sub.tag)
                if stag == "input":
                    off = int(sub.attrib.get("offset", "0"))
                    stride = max(stride, off + 1)
                    if sub.attrib.get("semantic") == "VERTEX":
                        v_off = off
                        ref = sub.attrib.get("source", "").lstrip("#")
                        pos_id = vertices_map.get(ref, ref)
                elif stag == "p":
                    p_elem = sub
                elif stag == "vcount":
                    vcount_elem = sub
            if pos_id not in sources or p_elem is None or not p_elem.text:
                continue
            verts = sources[pos_id]
            p_vals = np.fromstring(p_elem.text, sep=" ", dtype=np.int64)
            v_indices = p_vals[v_off::stride]
            faces_local: List[List[int]] = []
            if tag == "triangles":
                n_tri = len(v_indices) // 3
                if n_tri > 0:
                    tri = v_indices[: n_tri * 3].reshape(-1, 3) + vert_offset
                    all_verts.append(verts)
                    all_faces.append(tri)
                    vert_offset += len(verts)
            elif vcount_elem is not None and vcount_elem.text:
                vcounts = np.fromstring(vcount_elem.text, sep=" ", dtype=np.int64)
                cursor = 0
                for vc in vcounts:
                    vc_i = int(vc)
                    if vc_i >= 3 and cursor + vc_i <= len(v_indices):
                        poly = v_indices[cursor : cursor + vc_i]
                        for k in range(1, vc_i - 1):
                            faces_local.append(
                                [int(poly[0]) + vert_offset, int(poly[k]) + vert_offset, int(poly[k + 1]) + vert_offset]
                            )
                    cursor += vc_i
                if faces_local:
                    all_verts.append(verts)
                    all_faces.append(np.asarray(faces_local, dtype=np.int64))
                    vert_offset += len(verts)

    if not all_verts or not all_faces:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.int64)

    V = np.vstack(all_verts) * unit_scale
    if up_axis == "Y_UP":
        # Rotate +90 deg around X: (x, y, z) -> (x, -z, y)
        V = np.column_stack((V[:, 0], -V[:, 2], V[:, 1]))
    F = np.vstack(all_faces)
    return V, F


def _load_mesh(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    suf = path.suffix.lower()
    if suf == ".obj":
        return load_obj_mesh(path)
    if suf == ".stl":
        return load_stl_mesh(path)
    if suf == ".dae":
        return load_dae_mesh(path)
    return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.int64)


def _resolve_fuel_model_sdf(uri: str) -> Optional[Path]:
    """Resolve a Gazebo Fuel URI to a cached local model.sdf if present."""
    clean = uri.strip()
    m = re.match(r"https?://[^/]+/[^/]+/([^/]+)/models/(.+?)(?:/(\d+|tip))?$", clean, re.IGNORECASE)
    if not m:
        return None
    owner = urllib.parse.unquote(m.group(1)).strip().lower()
    name = urllib.parse.unquote(m.group(2)).strip().lower()
    ver = m.group(3)

    candidates_roots = []
    env_cache = os.environ.get("GZ_FUEL_CACHE")
    if env_cache:
        candidates_roots.append(Path(env_cache))
    candidates_roots.append(Path.home() / ".gz" / "fuel")
    candidates_roots.append(Path("/root/.gz/fuel"))

    for root in candidates_roots:
        model_dir = root / "fuel.gazebosim.org" / owner / "models" / name
        if not model_dir.is_dir():
            continue
        if ver and ver.isdigit() and (model_dir / ver / "model.sdf").is_file():
            return model_dir / ver / "model.sdf"
        ver_dirs = sorted(
            [d for d in model_dir.iterdir() if d.is_dir() and d.name.isdigit()],
            key=lambda p: int(p.name),
            reverse=True,
        )
        for vd in ver_dirs:
            sdf = vd / "model.sdf"
            if sdf.is_file():
                return sdf
    return None


# ---------------------------------------------------------------------------
# World sampling + 2D GT explorable occupancy map generation
# ---------------------------------------------------------------------------


def sample_world_ground_truth(
    world_path: Path,
    sample_step: float = 0.05,
    voxel_size: float = 0.02,
    min_z: float = 0.08,
    max_z: float = 2.0,
    resolution_2d: float = 0.05,
    seed_xy: Tuple[float, float] = (0.0, 0.0),
    slice_z_min: float = 0.12,
    slice_z_max: float = 0.45,
) -> Dict[str, object]:
    """Sample 3D GT surface points and 2D interior explorable free-space grid."""
    root = load_sdf_root(world_path)
    world_elem = root.find("world") if root.tag != "world" else root
    if world_elem is None:
        world_elem = root

    rng = np.random.default_rng(42)
    chunks: List[np.ndarray] = []
    # Keep watertight 2D cross-section polygons (world XY) for primitives that
    # intersect the 2D LiDAR height slice [slice_z_min, slice_z_max].
    slice_polys_xy: List[np.ndarray] = []
    slice_mesh_pts: List[np.ndarray] = []
    primitive_count = 0

    def _sample_model(model_elem: ET.Element, T_parent: np.ndarray, base_dir: Path) -> None:
        nonlocal primitive_count
        T_model = T_parent @ _parse_pose(model_elem.find("pose"))
        for link in model_elem.findall("link"):
            T_link = T_model @ _parse_pose(link.find("pose"))
            for col in link.findall("collision"):
                geom = col.find("geometry")
                if geom is None:
                    continue
                if geom.find("plane") is not None:
                    continue  # skip infinite ground plane
                T_col = T_link @ _parse_pose(col.find("pose"))
                pts = None
                box = geom.find("box")
                cyl = geom.find("cylinder")
                sph = geom.find("sphere")
                mesh = geom.find("mesh")

                if box is not None and box.find("size") is not None:
                    sx, sy, sz = [float(x) for x in box.find("size").text.strip().split()[:3]]
                    pts = sample_box_surface(sx, sy, sz, sample_step)
                    corners_local = np.array(
                        [
                            [-0.5 * sx, -0.5 * sy, -0.5 * sz],
                            [0.5 * sx, -0.5 * sy, -0.5 * sz],
                            [0.5 * sx, 0.5 * sy, -0.5 * sz],
                            [-0.5 * sx, 0.5 * sy, -0.5 * sz],
                            [-0.5 * sx, -0.5 * sy, 0.5 * sz],
                            [0.5 * sx, -0.5 * sy, 0.5 * sz],
                            [0.5 * sx, 0.5 * sy, 0.5 * sz],
                            [-0.5 * sx, 0.5 * sy, 0.5 * sz],
                        ],
                        dtype=np.float64,
                    )
                    corners_w = _transform_points(corners_local, T_col)
                    if corners_w[:, 2].min() <= slice_z_max and corners_w[:, 2].max() >= slice_z_min:
                        base_xy = _transform_points(
                            np.array(
                                [
                                    [-0.5 * sx, -0.5 * sy, 0.0],
                                    [0.5 * sx, -0.5 * sy, 0.0],
                                    [0.5 * sx, 0.5 * sy, 0.0],
                                    [-0.5 * sx, 0.5 * sy, 0.0],
                                ],
                                dtype=np.float64,
                            ),
                            T_col,
                        )[:, :2]
                        slice_polys_xy.append(base_xy)
                elif cyl is not None:
                    r = float(cyl.findtext("radius", "0.1"))
                    l = float(cyl.findtext("length", "1.0"))
                    pts = sample_cylinder_surface(r, l, sample_step)
                    cz = T_col[2, 3]
                    if cz - 0.5 * l <= slice_z_max and cz + 0.5 * l >= slice_z_min:
                        ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
                        ring = np.column_stack((r * np.cos(ang), r * np.sin(ang), np.zeros_like(ang)))
                        slice_polys_xy.append(_transform_points(ring, T_col)[:, :2])
                elif sph is not None:
                    r = float(sph.findtext("radius", "0.1"))
                    pts = sample_sphere_surface(r, sample_step)
                    cz = T_col[2, 3]
                    if cz - r <= slice_z_max and cz + r >= slice_z_min:
                        ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
                        ring = np.column_stack((r * np.cos(ang), r * np.sin(ang), np.zeros_like(ang)))
                        slice_polys_xy.append(_transform_points(ring, T_col)[:, :2])
                elif mesh is not None:
                    uri = (mesh.findtext("uri") or "").strip()
                    scale_txt = mesh.findtext("scale", "1 1 1").strip().split()
                    scale = np.array([float(x) for x in scale_txt[:3]], dtype=np.float64)
                    mesh_path = _resolve_mesh_uri(uri, base_dir)
                    if mesh_path is not None:
                        verts, faces = _load_mesh(mesh_path)
                        if len(verts) and len(faces):
                            pts = sample_triangles(verts * scale, faces, sample_step, rng)

                if pts is not None and pts.size:
                    world_pts = _transform_points(pts, T_col)
                    chunks.append(world_pts)
                    if mesh is not None:
                        s_pts = world_pts[(world_pts[:, 2] >= slice_z_min) & (world_pts[:, 2] <= slice_z_max)]
                        if s_pts.size:
                            slice_mesh_pts.append(s_pts[:, :2])
                    primitive_count += 1

    eye4 = np.eye(4, dtype=np.float64)
    for model in world_elem.findall("model"):
        _sample_model(model, eye4, world_path.parent)

    for inc in world_elem.findall("include"):
        uri = (inc.findtext("uri") or "").strip()
        if not uri:
            continue
        model_sdf = _resolve_fuel_model_sdf(uri)
        if model_sdf is None:
            continue
        T_inc = _parse_pose(inc.find("pose"))
        try:
            inc_root = ET.parse(model_sdf).getroot()
        except Exception:
            continue
        for inc_model in inc_root.findall("model"):
            _sample_model(inc_model, T_inc, model_sdf.parent)

    if not chunks:
        empty = np.empty((0, 3), dtype=np.float32)
        return {
            "points_3d": empty,
            "grid_2d": np.full((1, 1), -1, dtype=np.int8),
            "origin_x": 0.0,
            "origin_y": 0.0,
            "resolution": resolution_2d,
            "explorable_area_m2": 0.0,
            "primitive_count": 0,
        }

    all_pts = np.vstack(chunks)
    z_mask = (all_pts[:, 2] >= min_z) & (all_pts[:, 2] <= max_z)
    pts_3d = voxel_downsample(all_pts[z_mask].astype(np.float32), voxel_size)

    # Rasterize watertight 2D cross-section at 2D-LiDAR height band
    slice_pts = all_pts[(all_pts[:, 2] >= slice_z_min) & (all_pts[:, 2] <= slice_z_max)]
    if slice_pts.size == 0:
        slice_pts = pts_3d

    margin = 1.0
    min_x = float(min(slice_pts[:, 0].min(), seed_xy[0])) - margin
    min_y = float(min(slice_pts[:, 1].min(), seed_xy[1])) - margin
    max_x = float(max(slice_pts[:, 0].max(), seed_xy[0])) + margin
    max_y = float(max(slice_pts[:, 1].max(), seed_xy[1])) + margin

    width = max(4, int(math.ceil((max_x - min_x) / resolution_2d)))
    height = max(4, int(math.ceil((max_y - min_y) / resolution_2d)))

    occ_mask = np.zeros((height, width), dtype=np.uint8)
    for poly_xy in slice_polys_xy:
        px = np.round((poly_xy[:, 0] - min_x) / resolution_2d).astype(np.int32)
        py = np.round((poly_xy[:, 1] - min_y) / resolution_2d).astype(np.int32)
        contour = np.column_stack((px, py)).reshape(-1, 1, 2)
        cv2.fillPoly(occ_mask, [contour], 1)

    if slice_mesh_pts:
        m_xy = np.vstack(slice_mesh_pts)
        cx = np.floor((m_xy[:, 0] - min_x) / resolution_2d).astype(np.int64)
        cy = np.floor((m_xy[:, 1] - min_y) / resolution_2d).astype(np.int64)
        inb = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
        mesh_mask = np.zeros((height, width), dtype=np.uint8)
        mesh_mask[cy[inb], cx[inb]] = 1
        k_mesh = max(3, int(math.ceil(sample_step / resolution_2d)) * 2 + 1)
        mesh_mask = cv2.dilate(mesh_mask, np.ones((k_mesh, k_mesh), dtype=np.uint8), iterations=1)
        occ_mask = np.maximum(occ_mask, mesh_mask)

    occ_closed = cv2.dilate(occ_mask, np.ones((3, 3), dtype=np.uint8), iterations=1)

    sr = int(math.floor((seed_xy[1] - min_y) / resolution_2d))
    sc = int(math.floor((seed_xy[0] - min_x) / resolution_2d))
    sr = max(0, min(height - 1, sr))
    sc = max(0, min(width - 1, sc))

    free_binary = (occ_closed == 0).astype(np.uint8)
    _n, labels = cv2.connectedComponents(free_binary, connectivity=4)
    seed_label = int(labels[sr, sc])

    grid_2d = np.full((height, width), -1, dtype=np.int8)
    if seed_label > 0:
        interior = labels == seed_label
        grid_2d[interior] = 0
        # Walls adjacent to reachable interior are marked occupied (100)
        interior_dilated = cv2.dilate(interior.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        grid_2d[interior_dilated & (occ_mask > 0)] = 100
        explorable_area = float(interior.sum()) * (resolution_2d ** 2)
    else:
        grid_2d[occ_mask > 0] = 100
        explorable_area = 0.0

    # Restrict 3D GT surface cloud to points near the reachable interior (+1.0 m wall thickness)
    if seed_label > 0 and pts_3d.size:
        reach_dil = cv2.dilate(
            (labels == seed_label).astype(np.uint8),
            np.ones((int(round(1.0 / resolution_2d)) * 2 + 1,) * 2, np.uint8),
        ) > 0
        px = np.floor((pts_3d[:, 0] - min_x) / resolution_2d).astype(np.int64)
        py = np.floor((pts_3d[:, 1] - min_y) / resolution_2d).astype(np.int64)
        keep = (px >= 0) & (px < width) & (py >= 0) & (py < height)
        keep_idx = np.zeros(len(pts_3d), dtype=bool)
        keep_idx[keep] = reach_dil[py[keep], px[keep]]
        pts_3d = pts_3d[keep_idx]

    return {
        "points_3d": pts_3d,
        "grid_2d": grid_2d,
        "origin_x": min_x,
        "origin_y": min_y,
        "resolution": resolution_2d,
        "explorable_area_m2": explorable_area,
        "primitive_count": primitive_count,
    }


def _resolve_mesh_uri(uri: str, base_dir: Path) -> Optional[Path]:
    if not uri:
        return None
    if uri.startswith("file://"):
        p = Path(uri[7:])
        return p if p.is_file() else None
    if uri.startswith("model://"):
        # model://<model_name>/<rel_path> -> check relative to included model's base_dir
        rest = uri[len("model://") :]
        parts = rest.split("/", 1)
        if len(parts) == 2:
            cand = base_dir / parts[1]
            if cand.is_file():
                return cand
    p = Path(uri)
    if p.is_file():
        return p
    rel = base_dir / uri
    return rel if rel.is_file() else None


def main() -> None:
    ap = argparse.ArgumentParser(description="Sample GT 3D surface cloud & 2D map from SDF/Xacro world.")
    ap.add_argument("--world", required=True, help="Path to .sdf or .sdf.xacro world file")
    ap.add_argument("--seed", nargs=2, type=float, default=[0.0, 0.0], help="Interior free seed (x y)")
    ap.add_argument("--out-dir", required=True, help="Directory for gt_surface.pcd, gt_2d.npz, gt_meta.json")
    ap.add_argument("--sample-step", type=float, default=0.05)
    ap.add_argument("--voxel-size", type=float, default=0.02)
    ap.add_argument("--min-z", type=float, default=0.08)
    ap.add_argument("--max-z", type=float, default=2.0)
    ap.add_argument("--resolution-2d", type=float, default=0.05)
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    res = sample_world_ground_truth(
        Path(args.world),
        sample_step=args.sample_step,
        voxel_size=args.voxel_size,
        min_z=args.min_z,
        max_z=args.max_z,
        resolution_2d=args.resolution_2d,
        seed_xy=(args.seed[0], args.seed[1]),
    )
    write_pcd(out_dir / "gt_surface.pcd", res["points_3d"], binary=True)
    np.savez_compressed(
        out_dir / "gt_2d.npz",
        grid=res["grid_2d"],
        origin_x=res["origin_x"],
        origin_y=res["origin_y"],
        resolution=res["resolution"],
        explorable_area_m2=res["explorable_area_m2"],
    )
    meta = {
        "world": str(args.world),
        "seed_xy": list(args.seed),
        "points_3d": int(len(res["points_3d"])),
        "explorable_area_m2": round(float(res["explorable_area_m2"]), 3),
        "primitive_count": int(res["primitive_count"]),
        "voxel_size": args.voxel_size,
        "resolution_2d": args.resolution_2d,
        "min_z": args.min_z,
        "max_z": args.max_z,
    }
    (out_dir / "gt_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
