#pragma once

#include <vector>

#include "se2_map_merge/types.hpp"

namespace se2_map_merge
{

struct CloudFilterParams
{
  double min_range{0.10};
  double max_range{12.0};
  double min_z{0.05};
  double max_z{3.0};
  double voxel_size{0.02};

  // Self-reflection box filter in sensor frame (base_scan)
  double footprint_x_min{-0.22};
  double footprint_x_max{0.22};
  double footprint_y_min{-0.18};
  double footprint_y_max{0.18};
  double footprint_z_max{0.05};

  // Pose continuity guard (rejects TF jumps / clock stalls)
  double max_robot_speed_mps{0.65};
  double pose_step_margin_m{0.35};
};

inline bool pose_is_discontinuous(
  double dt,
  double dist,
  double max_speed = 0.65,
  double margin = 0.35)
{
  const double eff_dt = dt > 0.0 ? dt : 0.0;
  return dist > margin + max_speed * eff_dt;
}

/// Filter sensor-frame points (radial range + self-reflection box), transform
/// into world frame, and crop world Z into [min_z, max_z].
std::vector<Point3D> filter_and_transform_cloud(
  const std::vector<Point3D> & raw_sensor_points,
  const SE3Transform & tf_world_sensor,
  const CloudFilterParams & params);

/// Fast cache-friendly 1D packed-key voxel grid downsampling.
std::vector<Point3D> voxel_downsample_points(
  const std::vector<Point3D> & points,
  double voxel_size);

}  // namespace se2_map_merge
