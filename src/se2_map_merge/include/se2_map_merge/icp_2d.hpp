#pragma once

#include <utility>
#include <vector>

#include "se2_map_merge/phase_corr.hpp"
#include "se2_map_merge/types.hpp"

namespace se2_map_merge
{

struct Se2AlignParams
{
  int occupied_threshold{65};
  int min_overlap_points{60};
  int max_icp_points{2500};
  int max_iterations{25};
  double max_correspondence_dist_m{0.80};
  double huber_delta_m{0.10};
  double convergence_trans_m{1e-4};
  double convergence_rot_rad{1e-4};

  // Corridor degeneracy gating (Zhang & Singh Solution Remapping)
  bool enable_degeneracy_gate{true};
  double min_trans_eigenvalue{12.0};     // min well-constrained translational info
  double max_trans_cond_number{25.0};    // eig_max / eig_min threshold for parallel walls
  double min_rot_eigenvalue{8.0};

  // Consistency & safety gates
  double max_total_delta_xy_m{3.5};
  double max_total_delta_yaw_rad{0.50};  // ~28.6 deg
  double min_inlier_ratio{0.45};
  double max_rmse_m{0.18};
  bool allow_large_slip_acquisition{true};
  double iou_tolerance{0.015};           // allow tiny discretization noise
  // High-IoU deadband: prevents random-walk jitter on already-aligned small rooms
  double deadband_iou_threshold{0.58};
  double deadband_min_improve_iou{0.015};
  double deadband_min_delta_xy_m{0.04};
  double deadband_min_delta_yaw_rad{0.015};

  // Temporal SE(2) exponential smoothing
  double smoothing_alpha{0.25};
  double max_step_xy_m{0.05};
  double max_step_yaw_rad{0.0174532925}; // 1 deg

  bool enable_phase_corr{true};
  PhaseCorrParams phase_corr{};
};

/// Compute 1-cell dilated occupied IoU between ``ref`` and ``src`` warped by ``pose``.
double compute_overlap_occupied_iou(
  const OccupancyGrid2D & ref,
  const OccupancyGrid2D & src,
  const Pose2D & pose,
  int occupied_threshold = 65);

/// Full two-stage online SE(2) alignment pipeline:
///   Stage 1: FFT phase correlation coarse translation (optional)
///   Stage 2: Point-to-line Huber ICP with eigenvalue degeneracy Solution Remapping
///   Stage 3: Overlap IoU monotonic gate + step-clamped SE(2) smoothing
RegistrationDiagnostics align_occupancy_grids_se2(
  const OccupancyGrid2D & ref,
  const OccupancyGrid2D & src,
  const Pose2D & current_pose,
  const Se2AlignParams & params = Se2AlignParams{});

/// Merge N occupancy grids into one world-frame grid using their current SE(2) poses.
/// Uses inverse nearest-neighbor warping so rotated grids have zero Moire holes.
OccupancyGrid2D merge_occupancy_grids(
  const std::vector<std::pair<const OccupancyGrid2D *, Pose2D>> & grids,
  double output_resolution = 0.05);

}  // namespace se2_map_merge
