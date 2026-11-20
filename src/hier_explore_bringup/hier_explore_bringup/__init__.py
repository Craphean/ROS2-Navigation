"""Parameterized N=1..4 multi-robot bringup and 6-world benchmark registry."""

from .profiles import (
    ABLATION_PROFILES,
    AblationProfile,
    resolve_profile,
)
from .worlds import (
    BENCHMARK_WORLDS,
    WORLD_SPECS,
    SpawnPose,
    WorldSpec,
    canonical_world_name,
    find_repo_root,
    resolve_world_file,
    sample_spawn_poses,
)

__all__ = [
    "ABLATION_PROFILES",
    "AblationProfile",
    "BENCHMARK_WORLDS",
    "WORLD_SPECS",
    "SpawnPose",
    "WorldSpec",
    "canonical_world_name",
    "find_repo_root",
    "resolve_profile",
    "resolve_world_file",
    "sample_spawn_poses",
]
