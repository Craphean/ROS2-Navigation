#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace geom_kernel
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct ShortestPathResult
{
  double distance_m{0.0};
  bool reachable{false};
  std::vector<Point2D> path_xy;
};

/// Compute exact 8-connected bounded multi-source geodesic distances from each
/// source in `sources_xy` (S points) to every target in `targets_xy` (T points).
/// Returns a row-major S x T vector of distances in meters (infinity if unreachable
/// or beyond `max_dist_m` when `max_dist_m > 0`).
std::vector<double> compute_geodesic_distances(
  const uint8_t * traversable,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<Point2D> & sources_xy,
  const std::vector<Point2D> & targets_xy,
  double max_dist_m = 0.0,
  double min_clearance_cells = 0.0,
  double snap_radius_m = 0.65);

/// Compute symmetric pairwise M x M geodesic distance matrix among `points_xy`.
std::vector<double> compute_geodesic_matrix(
  const uint8_t * traversable,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<Point2D> & points_xy,
  double max_dist_m = 0.0,
  double min_clearance_cells = 0.0,
  double snap_radius_m = 0.65);

/// Compute exact 8-connected geodesic shortest path (A* with octile heuristic)
/// between `start_xy` and `goal_xy`, returning total length (m) and world path.
ShortestPathResult compute_geodesic_path(
  const uint8_t * traversable,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const Point2D & start_xy,
  const Point2D & goal_xy,
  double max_dist_m = 0.0,
  double min_clearance_cells = 0.0,
  double snap_radius_m = 0.65);

}  // namespace geom_kernel
