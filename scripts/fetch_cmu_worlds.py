#!/usr/bin/env python3
"""Download and install official CMU AEDE meshes (cmu_indoor, cmu_tunnel) for Gazebo.

Fetches ``autonomous_exploration_environments.zip`` from the official CMU
Autonomous Exploration Development Environment mirror and extracts the
``indoor`` and ``tunnel`` meshes into ``~/.gz/models/cmu_indoor`` and
``~/.gz/models/cmu_tunnel`` so ``worlds/cmu_indoor.sdf.xacro`` and
``worlds/cmu_tunnel.sdf.xacro`` resolve ``model://cmu_indoor/...`` and
``model://cmu_tunnel/...`` offline.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path

CMU_GDRIVE_ID = "1V4MWCD3qYLr3RFLLATbqAh_lXKcCU39C"
CMU_DOWNLOAD_URL = f"https://drive.usercontent.google.com/download?id={CMU_GDRIVE_ID}&confirm=t"


def install_from_zip(zip_path: Path, models_dir: Path) -> None:
    models_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tmp_dir = Path(tmp)
        with zipfile.ZipFile(zip_path, "r") as zf:
            members = [
                m for m in zf.namelist()
                if m.startswith(("indoor/meshes/", "tunnel/meshes/"))
            ]
            zf.extractall(tmp_dir, members=members)

        for src_name, dst_name in (("indoor", "cmu_indoor"), ("tunnel", "cmu_tunnel")):
            src_meshes = tmp_dir / src_name / "meshes"
            dst_meshes = models_dir / dst_name / "meshes"
            if not src_meshes.is_dir():
                raise RuntimeError(f"Expected {src_meshes} in {zip_path}")
            dst_meshes.mkdir(parents=True, exist_ok=True)
            for item in src_meshes.iterdir():
                if item.is_file():
                    shutil.copy2(item, dst_meshes / item.name)
            print(f"[fetch_cmu_worlds] Installed {dst_meshes}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Fetch & install CMU indoor/tunnel meshes.")
    ap.add_argument(
        "--models-dir",
        default=str(Path.home() / ".gz" / "models"),
        help="Destination Gazebo models directory (default: ~/.gz/models)",
    )
    ap.add_argument(
        "--zip-path",
        default="",
        help="Optional already-downloaded autonomous_exploration_environments.zip",
    )
    ap.add_argument("--force", action="store_true", help="Re-install even if meshes exist")
    args = ap.parse_args()

    models_dir = Path(args.models_dir)
    indoor_dae = models_dir / "cmu_indoor" / "meshes" / "indoor.dae"
    tunnel_dae = models_dir / "cmu_tunnel" / "meshes" / "tunnel.dae"
    if not args.force and indoor_dae.is_file() and tunnel_dae.is_file():
        print(f"[fetch_cmu_worlds] CMU meshes already present in {models_dir}")
        return

    if args.zip_path:
        install_from_zip(Path(args.zip_path), models_dir)
        return

    with tempfile.TemporaryDirectory() as tmp:
        zip_path = Path(tmp) / "autonomous_exploration_environments.zip"
        print(f"[fetch_cmu_worlds] Downloading {CMU_DOWNLOAD_URL} -> {zip_path}")
        subprocess.run(
            ["curl", "-fL", "--retry", "3", CMU_DOWNLOAD_URL, "-o", str(zip_path)],
            check=True,
        )
        install_from_zip(zip_path, models_dir)


if __name__ == "__main__":
    main()
