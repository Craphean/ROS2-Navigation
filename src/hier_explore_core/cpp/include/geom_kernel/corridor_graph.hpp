#pragma once

#include <cstdint>
#include <vector>

namespace geom_kernel
{

struct YieldPoint
{
  double x{0.0};
  double y{0.0};
  double clearance_m{0.0};
};

struct CorridorSegment
{
  int32_t id{0};
  int32_t cell_count{0};
  double min_half_width_m{0.0};
  double max_half_width_m{0.0};
  double length_m{0.0};
  double centroid_x{0.0};
  double centroid_y{0.0};
  std::vector<YieldPoint> yield_points;
};

struct CorridorGraphResult
{
  int height{0};
  int width{0};
  std::vector<int32_t> labels;  // row-major H x W; 0 = non-bottleneck, >0 = segment id
  std::vector<CorridorSegment> segments;
};

struct CorridorParams
{
  double robot_radius{0.22};
  double narrow_margin_m{0.35};       // bottleneck half-width threshold: 2*r + margin
  double min_passable_margin_m{0.02}; // ignore slivers narrower than robot_radius + margin
  int min_segment_cells{4};
  double yield_search_radius_m{3.0};
  double yield_min_separation_m{1.0};
  int max_yield_points{4};
};

/// Extract narrow-passage bottleneck segments E_narrow where channel half-width
/// w(p) < 2*robot_radius + narrow_margin_m using medial-axis ridge detection
/// (avoiding wide-room wall perimeter false positives) and locate spacious
/// yield points at each portal exit.
CorridorGraphResult extract_corridor_bottlenecks(
  const uint8_t * is_free,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const CorridorParams & params);

}  // namespace geom_kernel
