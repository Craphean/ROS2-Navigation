"""Ablation profile, planner backend, and odometry mode resolution for Lite-HMRE."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass(frozen=True)
class AblationProfile:
    name: str
    description: str
    default_planner_backend: str  # "coordinator" | "legacy" | "off"
    use_se2_merge: bool = True
    use_cpp_cloud_merger: bool = True
    coordinator_params: Dict[str, Any] = field(default_factory=dict)
    se2_merge_params: Dict[str, Any] = field(default_factory=dict)


ABLATION_PROFILES: Dict[str, AblationProfile] = {
    "legacy": AblationProfile(
        name="legacy",
        description="E0: m-explore-ros2 baseline (explore_lite + static multirobot_map_merge + Python cloud merger).",
        default_planner_backend="legacy",
        use_se2_merge=False,
        use_cpp_cloud_merger=False,
    ),
    "e0_legacy": AblationProfile(
        name="e0_legacy",
        description="Alias of legacy (m-explore-ros2 baseline).",
        default_planner_backend="legacy",
        use_se2_merge=False,
        use_cpp_cloud_merger=False,
    ),
    "e1_2d": AblationProfile(
        name="e1_2d",
        description="E1: Pure 2D utility auction (no 3D gain, no hierarchical TSP, no geodesic, no corridor mutex).",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": False,
            "local_scheduling_enable": False,
            "residual_enable": False,
            "convergence_enable": False,
            "use_geodesic_distance": False,
            "use_los_fov_optimizer": False,
            "teammate_deconfliction_enable": False,
            "corridor_mutex_enable": False,
        },
    ),
    "e2_3dgain": AblationProfile(
        name="e2_3dgain",
        description="E2: E1 + 2.5D VoxelColumnIndex structural 3D information gain.",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": False,
            "residual_enable": False,
            "convergence_enable": False,
            "use_geodesic_distance": False,
            "use_los_fov_optimizer": False,
            "teammate_deconfliction_enable": False,
            "corridor_mutex_enable": False,
        },
    ),
    "e3_hier": AblationProfile(
        name="e3_hier",
        description="E3: E2 + Local Open-TSP & Global ROI hierarchical scheduling + convergence.",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": False,
            "convergence_enable": True,
            "use_geodesic_distance": False,
            "use_los_fov_optimizer": False,
            "teammate_deconfliction_enable": False,
            "corridor_mutex_enable": False,
        },
    ),
    "e4_geodesic": AblationProfile(
        name="e4_geodesic",
        description="E4: E3 + C++ _geom_kernel geodesic distance matrix & shared global costmap.",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": False,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": False,
            "teammate_deconfliction_enable": False,
            "corridor_mutex_enable": False,
        },
    ),
    "e5_deconflict": AblationProfile(
        name="e5_deconflict",
        description="E5: E4 + two-layer deconfliction (5 Hz teammates_cloud + topological corridor mutex).",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": False,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": False,
            "teammate_deconfliction_enable": True,
            "corridor_mutex_enable": True,
        },
    ),
    "full": AblationProfile(
        name="full",
        description="Full Lite-HMRE (SE(2) alignment + geodesic + deconfliction + LoS/FoV submodular residual NBV).",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": True,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": True,
            "teammate_deconfliction_enable": True,
            "corridor_mutex_enable": True,
        },
    ),
    "e6_full": AblationProfile(
        name="e6_full",
        description="Alias of full (Full Lite-HMRE).",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": True,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": True,
            "teammate_deconfliction_enable": True,
            "corridor_mutex_enable": True,
        },
    ),
    "no_se2": AblationProfile(
        name="no_se2",
        description="Alignment ablation: Full coordinator with online SE(2) map alignment disabled.",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": True,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": True,
            "teammate_deconfliction_enable": True,
            "corridor_mutex_enable": True,
        },
        se2_merge_params={"enable_online_alignment": False},
    ),
    "no_phase_corr": AblationProfile(
        name="no_phase_corr",
        description="Alignment ablation: Full coordinator with 2D FFT coarse phase correlation disabled (ICP only).",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": True,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": True,
            "teammate_deconfliction_enable": True,
            "corridor_mutex_enable": True,
        },
        se2_merge_params={"enable_phase_corr": False},
    ),
    "no_degen_gate": AblationProfile(
        name="no_degen_gate",
        description="Alignment ablation: Full coordinator with Hessian corridor degeneracy gate disabled.",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": True,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": True,
            "teammate_deconfliction_enable": True,
            "corridor_mutex_enable": True,
        },
        se2_merge_params={"enable_degeneracy_gate": False},
    ),
    "no_shared_map": AblationProfile(
        name="no_shared_map",
        description="Ablation: Full coordinator without per-robot reprojected /<ns>/shared_map (Nav2 uses local map only).",
        default_planner_backend="coordinator",
        coordinator_params={
            "use_3d_gain": True,
            "local_scheduling_enable": True,
            "residual_enable": True,
            "convergence_enable": True,
            "use_geodesic_distance": True,
            "use_los_fov_optimizer": True,
            "teammate_deconfliction_enable": True,
            "corridor_mutex_enable": True,
        },
        se2_merge_params={"publish_per_robot_shared_map": False},
    ),
}


def resolve_profile(
    ablation_profile: str = "full",
    planner_backend: str = "auto",
    odom_mode: str = "ekf",
    use_se2_merge: str = "auto",
    use_cpp_cloud_merger: str = "auto",
) -> Dict[str, Any]:
    """Validate and resolve launch configuration flags into concrete parameters."""
    prof_key = ablation_profile.strip().lower()
    if prof_key not in ABLATION_PROFILES:
        raise ValueError(
            f"Unknown ablation_profile '{ablation_profile}'. Valid: {sorted(ABLATION_PROFILES)}"
        )
    prof = ABLATION_PROFILES[prof_key]

    odom_key = odom_mode.strip().lower()
    if odom_key in ("ekf", "noisy", "wheel_imu"):
        use_gt_odom = False
        odom_canonical = "ekf"
    elif odom_key in ("gt", "ground_truth", "ideal"):
        use_gt_odom = True
        odom_canonical = "gt"
    else:
        raise ValueError(f"Unknown odom_mode '{odom_mode}'. Expected 'ekf' or 'gt'.")

    backend_key = planner_backend.strip().lower()
    if backend_key in ("", "auto"):
        backend = prof.default_planner_backend
    elif backend_key in ("coordinator", "auction", "hier", "lite_hmre"):
        backend = "coordinator"
    elif backend_key in ("legacy", "explore_lite"):
        backend = "legacy"
    elif backend_key in ("off", "none", "manual"):
        backend = "off"
    else:
        raise ValueError(
            f"Unknown planner_backend '{planner_backend}'. Expected auto|coordinator|legacy|off."
        )

    def _tri_bool(val: str, default: bool) -> bool:
        v = str(val).strip().lower()
        if v in ("", "auto"):
            return default
        return v in ("true", "1", "yes")

    return {
        "profile": prof,
        "planner_backend": backend,
        "odom_mode": odom_canonical,
        "use_gt_odom": use_gt_odom,
        "use_se2_merge": _tri_bool(use_se2_merge, prof.use_se2_merge),
        "use_cpp_cloud_merger": _tri_bool(use_cpp_cloud_merger, prof.use_cpp_cloud_merger),
        "coordinator_params": dict(prof.coordinator_params),
        "se2_merge_params": dict(prof.se2_merge_params),
    }
