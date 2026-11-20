#pragma once

#include <cstdint>
#include <vector>

namespace geom_kernel
{

struct ResidualColumn
{
  double x{0.0};
  double y{0.0};
  double z_min{0.25};
  double z_max{1.50};
  int32_t cluster_id{0};
};

struct VisibilityParams
{
  double r_min{0.8};
  double r_max{3.5};
  int num_samples{24};
  double min_clearance{0.55};
  double sensor_z{0.25};
  double vfov_min_rad{-0.5235987756};  // -30 deg
  double vfov_max_rad{0.9599310886};   // +55 deg
  double max_eff_range{5.0};
  int max_viewpoints{4};
  double dedupe_radius{1.5};
  int8_t lethal_threshold{50};
};

struct ViewpointCandidate
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double clearance_m{0.0};
  int32_t cluster_id{0};
  int32_t covered_count{0};
  int32_t marginal_gain{0};
  double coverage_ratio{0.0};
};

/// 2D Bresenham line-of-sight test on an int8 OccupancyGrid.
/// Returns true iff no intermediate cell strictly before (r1, c1) exceeds lethal_threshold.
bool has_line_of_sight(
  const int8_t * occ_data,
  int height,
  int width,
  int r0,
  int c0,
  int r1,
  int c1,
  int8_t lethal_threshold = 50);

/// Sample K safe viewpoints per residual cluster in [r_min, r_max], evaluate 2D Bresenham
/// wall occlusion + 3D vertical FoV [vfov_min_rad, vfov_max_rad] + max effective range,
/// and greedily maximize submodular coverage of residual columns.
std::vector<ViewpointCandidate> optimize_residual_viewpoints(
  const int8_t * occ_data,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<ResidualColumn> & columns,
  const VisibilityParams & params);

}  // namespace geom_kernel
