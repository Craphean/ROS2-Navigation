#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "se2_map_merge/cloud_filter.hpp"
#include "se2_map_merge/icp_2d.hpp"
#include "se2_map_merge/phase_corr.hpp"
#include "se2_map_merge/types.hpp"

using namespace se2_map_merge;

namespace
{

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

void draw_wall(
  OccupancyGrid2D & g, double x0, double y0, double x1, double y1, int8_t val = 100)
{
  const double len = std::hypot(x1 - x0, y1 - y0);
  const int steps = std::max(1, static_cast<int>(std::ceil(len / (g.resolution * 0.5))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double wx = x0 + t * (x1 - x0);
    const double wy = y0 + t * (y1 - y0);
    const int c = static_cast<int>(std::floor((wx - g.origin_x) / g.resolution));
    const int r = static_cast<int>(std::floor((wy - g.origin_y) / g.resolution));
    if (c >= 0 && c < g.width && r >= 0 && r < g.height) {
      g.data[static_cast<size_t>(r) * g.width + c] = val;
    }
  }
}

/// Build an asymmetric L-shaped room with shelves/pillars (well-constrained in SE(2)).
OccupancyGrid2D make_structured_room(double res = 0.05)
{
  OccupancyGrid2D g;
  g.resolution = res;
  g.origin_x = -6.0;
  g.origin_y = -6.0;
  g.width = static_cast<int>(std::round(16.0 / res));
  g.height = static_cast<int>(std::round(14.0 / res));
  g.data.assign(static_cast<size_t>(g.width) * g.height, 0);

  // Outer perimeter (non-symmetric L shape)
  draw_wall(g, -4.0, -4.0, 8.0, -4.0);
  draw_wall(g, 8.0, -4.0, 8.0, 2.0);
  draw_wall(g, 8.0, 2.0, 2.0, 2.0);
  draw_wall(g, 2.0, 2.0, 2.0, 6.0);
  draw_wall(g, 2.0, 6.0, -4.0, 6.0);
  draw_wall(g, -4.0, 6.0, -4.0, -4.0);

  // Interior shelves / partition walls
  draw_wall(g, -1.5, -1.5, 2.5, -1.5);
  draw_wall(g, 4.5, -2.5, 4.5, 0.5);
  draw_wall(g, -2.0, 2.5, -2.0, 4.5);
  return g;
}

/// Warp a reference grid into a drifted robot's local map frame via T_local_world = inv(T_true).
OccupancyGrid2D warp_grid_as_follower(const OccupancyGrid2D & ref, const Pose2D & true_world_from_src)
{
  OccupancyGrid2D src;
  src.resolution = ref.resolution;
  src.width = ref.width;
  src.height = ref.height;
  src.origin_x = ref.origin_x;
  src.origin_y = ref.origin_y;
  src.data.assign(static_cast<size_t>(src.width) * src.height, 0);

  for (int r = 0; r < src.height; ++r) {
    for (int c = 0; c < src.width; ++c) {
      const Point2D wp = apply_se2(true_world_from_src, src.cell_to_world(r, c));
      const int rc = static_cast<int>(std::floor((wp.x - ref.origin_x) / ref.resolution));
      const int rr = static_cast<int>(std::floor((wp.y - ref.origin_y) / ref.resolution));
      if (rc >= 0 && rc < ref.width && rr >= 0 && rr < ref.height) {
        src.data[static_cast<size_t>(r) * src.width + c] = ref.at(rr, rc);
      } else {
        src.data[static_cast<size_t>(r) * src.width + c] = -1;
      }
    }
  }
  return src;
}

/// Build a straight featureless mine corridor (parallel walls along X only -> X is degenerate!).
OccupancyGrid2D make_straight_corridor(double res = 0.05)
{
  OccupancyGrid2D g;
  g.resolution = res;
  g.origin_x = -12.0;
  g.origin_y = -4.0;
  g.width = static_cast<int>(std::round(24.0 / res));
  g.height = static_cast<int>(std::round(8.0 / res));
  g.data.assign(static_cast<size_t>(g.width) * g.height, 0);

  // Two long parallel horizontal walls (y = -1.6 m and y = +1.6 m), no end caps
  draw_wall(g, -10.0, -1.6, 10.0, -1.6);
  draw_wall(g, -10.0, +1.6, 10.0, +1.6);
  return g;
}

void test_large_shift_and_rotation_recovery()
{
  std::cout << "[TEST 1] Large slip + 5 deg rotation SE(2) recovery..." << std::endl;
  const auto ref = make_structured_room(0.05);
  // True drift: dx = +1.20 m, dy = -0.75 m, dyaw = +5.0 deg
  const Pose2D true_pose{1.20, -0.75, 5.0 * kDeg2Rad};
  const auto src = warp_grid_as_follower(ref, true_pose);

  Se2AlignParams params;
  params.smoothing_alpha = 1.0;       // verify raw recovered pose in single call
  params.max_step_xy_m = 5.0;
  params.max_step_yaw_rad = 1.0;

  const auto diag = align_occupancy_grids_se2(ref, src, Pose2D{0.0, 0.0, 0.0}, params);
  const double err_xy = std::hypot(diag.smoothed_pose.x - true_pose.x, diag.smoothed_pose.y - true_pose.y);
  const double err_yaw_deg = std::abs(wrap_angle(diag.smoothed_pose.yaw - true_pose.yaw)) / kDeg2Rad;

  std::cout << "  accepted=" << diag.accepted
            << " phase_used=" << diag.phase_corr_used
            << " iou=" << diag.iou_before << "->" << diag.iou_after
            << " err_xy=" << err_xy * 100.0 << "cm"
            << " err_yaw=" << err_yaw_deg << "deg"
            << " elapsed=" << diag.elapsed_ms << "ms" << std::endl;

  assert(diag.accepted);
  assert(diag.phase_corr_used);
  assert(!diag.degenerate_trans);
  assert(diag.iou_after > 0.85);
  assert(err_xy < 0.03);       // < 3 cm (< 1 grid cell!)
  assert(err_yaw_deg < 0.4);   // < 0.4 deg
}

void test_corridor_degeneracy_solution_remapping()
{
  std::cout << "[TEST 2] Straight mine corridor degeneracy gating (Solution Remapping)..." << std::endl;
  const auto ref = make_straight_corridor(0.05);
  // True lateral offset dy = +0.25 m and yaw = +3.5 deg, ZERO true longitudinal shift
  const Pose2D true_pose{0.0, 0.25, 3.5 * kDeg2Rad};
  const auto src = warp_grid_as_follower(ref, true_pose);

  Se2AlignParams params;
  params.smoothing_alpha = 1.0;
  params.max_step_xy_m = 2.0;
  params.max_step_yaw_rad = 0.5;
  params.enable_degeneracy_gate = true;

  const auto diag = align_occupancy_grids_se2(ref, src, Pose2D{0.0, 0.0, 0.0}, params);
  const double err_y = std::abs(diag.smoothed_pose.y - true_pose.y);
  const double err_yaw_deg = std::abs(wrap_angle(diag.smoothed_pose.yaw - true_pose.yaw)) / kDeg2Rad;
  const double longitudinal_drift_x = std::abs(diag.smoothed_pose.x);

  std::cout << "  accepted=" << diag.accepted
            << " degenerate_trans=" << diag.degenerate_trans
            << " eig_min=" << diag.eig_min_trans
            << " eig_max=" << diag.eig_max_trans
            << " cond=" << diag.cond_trans
            << " drift_x=" << longitudinal_drift_x * 100.0 << "cm"
            << " err_y=" << err_y * 100.0 << "cm"
            << " err_yaw=" << err_yaw_deg << "deg" << std::endl;

  assert(diag.accepted);
  assert(diag.degenerate_trans);          // MUST detect parallel-wall degeneracy!
  assert(longitudinal_drift_x < 0.01);    // X frozen (< 1 cm)
  assert(err_y < 0.03);                   // lateral Y recovered accurately
  assert(err_yaw_deg < 0.5);              // yaw recovered accurately
}

void test_step_clamped_smoothing_and_merge()
{
  std::cout << "[TEST 3] Step-clamped SE(2) smoothing & multi-grid fusion..." << std::endl;
  const Pose2D cur{0.0, 0.0, 0.0};
  const Pose2D target{1.0, 0.0, 10.0 * kDeg2Rad};
  const Pose2D step = smooth_se2(cur, target, 0.25, 0.05, 1.0 * kDeg2Rad);
  assert(std::abs(step.x - 0.05) < 1e-9);
  assert(std::abs(step.yaw - 1.0 * kDeg2Rad) < 1e-9);

  const auto ref = make_structured_room(0.05);
  const Pose2D pose2{0.30, -0.20, 3.0 * kDeg2Rad};
  const auto src = warp_grid_as_follower(ref, pose2);
  const auto merged = merge_occupancy_grids({{&ref, Pose2D{}}, {&src, pose2}}, 0.05);
  assert(!merged.empty());
  const double iou_aligned = compute_overlap_occupied_iou(ref, src, pose2);
  const double iou_unaligned = compute_overlap_occupied_iou(ref, src, Pose2D{});
  std::cout << "  IoU unaligned=" << iou_unaligned << " -> aligned=" << iou_aligned << std::endl;
  assert(iou_aligned > 0.90);
  assert(iou_aligned > iou_unaligned + 0.30);
}

void test_cloud_filter_and_speed_benchmark()
{
  std::cout << "[TEST 4] C++ cloud filter, self-reflection box, & voxel downsample benchmark..." << std::endl;
  assert(!pose_is_discontinuous(0.5, 0.25));
  assert(pose_is_discontinuous(0.5, 2.0));
  assert(pose_is_discontinuous(-0.1, 1.0));

  CloudFilterParams p;
  std::vector<Point3D> synthetic = {
    {0.05f, 0.0f, 0.0f},    // self-reflection inside footprint box -> reject
    {0.02f, 0.01f, 0.0f},   // below min_range -> reject
    {15.0f, 0.0f, 0.5f},    // beyond max_range -> reject
    {2.0f, 1.0f, 0.01f},    // below world min_z -> reject
    {2.0f, 1.0f, 3.5f},     // above world max_z -> reject
    {2.0f, 1.0f, 0.50f},    // valid
    {2.005f, 1.005f, 0.505f} // same 0.02m voxel -> merged by downsample
  };
  const auto filtered = filter_and_transform_cloud(synthetic, SE3Transform{}, p);
  assert(filtered.size() == 2);
  const auto ds = voxel_downsample_points(filtered, 0.02);
  assert(ds.size() == 1);

  // Throughput benchmark: 2 robots x 16x360 = 11,520 points/frame x 50 frames
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist_xy(-8.0f, 8.0f);
  std::uniform_real_distribution<float> dist_z(0.1f, 2.5f);
  std::vector<Point3D> frame(11520);
  for (auto & pt : frame) {
    pt = {dist_xy(rng), dist_xy(rng), dist_z(rng)};
  }
  const SE3Transform tf = SE3Transform::from_quat_trans(0, 0, std::sin(0.2), std::cos(0.2), 1.5, -2.0, 0.12);

  const auto t0 = std::chrono::steady_clock::now();
  size_t total_out = 0;
  for (int it = 0; it < 50; ++it) {
    auto f = filter_and_transform_cloud(frame, tf, p);
    auto d = voxel_downsample_points(f, 0.02);
    total_out += d.size();
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double ms_per_frame =
    std::chrono::duration<double, std::milli>(t1 - t0).count() / 50.0;
  std::cout << "  C++ cloud filter+transform+voxel(11,520 pts): "
            << ms_per_frame << " ms/frame (total_out=" << total_out << ")" << std::endl;
  assert(ms_per_frame < 5.0);  // must be well under 5 ms/frame!
}

}  // namespace

int main()
{
  test_large_shift_and_rotation_recovery();
  test_corridor_degeneracy_solution_remapping();
  test_step_clamped_smoothing_and_merge();
  test_cloud_filter_and_speed_benchmark();
  std::cout << "ALL se2_map_merge C++ TESTS PASSED!" << std::endl;
  return 0;
}
