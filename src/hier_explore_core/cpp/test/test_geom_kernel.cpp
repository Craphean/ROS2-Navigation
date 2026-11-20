// Tests must assert in every build type (Release defines NDEBUG).
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

#include "geom_kernel/corridor_graph.hpp"
#include "geom_kernel/geodesic_grid.hpp"
#include "geom_kernel/visibility_fov.hpp"

using namespace geom_kernel;

namespace
{

// Exact Euclidean distance transform (simple 2-pass Chamfer approx for C++ unit test)
std::vector<float> approx_dt(const std::vector<uint8_t> & is_free, int h, int w)
{
  const float kInf = 1e6f;
  std::vector<float> dt(static_cast<size_t>(h * w), kInf);
  for (int i = 0; i < h * w; ++i) {
    if (!is_free[static_cast<size_t>(i)]) {
      dt[static_cast<size_t>(i)] = 0.0f;
    }
  }
  const float s2 = 1.41421356f;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      float v = dt[static_cast<size_t>(r * w + c)];
      if (r > 0) v = std::min(v, dt[static_cast<size_t>((r - 1) * w + c)] + 1.0f);
      if (c > 0) v = std::min(v, dt[static_cast<size_t>(r * w + (c - 1))] + 1.0f);
      if (r > 0 && c > 0) v = std::min(v, dt[static_cast<size_t>((r - 1) * w + (c - 1))] + s2);
      if (r > 0 && c + 1 < w) v = std::min(v, dt[static_cast<size_t>((r - 1) * w + (c + 1))] + s2);
      dt[static_cast<size_t>(r * w + c)] = v;
    }
  }
  for (int r = h - 1; r >= 0; --r) {
    for (int c = w - 1; c >= 0; --c) {
      float v = dt[static_cast<size_t>(r * w + c)];
      if (r + 1 < h) v = std::min(v, dt[static_cast<size_t>((r + 1) * w + c)] + 1.0f);
      if (c + 1 < w) v = std::min(v, dt[static_cast<size_t>(r * w + (c + 1))] + 1.0f);
      if (r + 1 < h && c + 1 < w) v = std::min(v, dt[static_cast<size_t>((r + 1) * w + (c + 1))] + s2);
      if (r + 1 < h && c > 0) v = std::min(v, dt[static_cast<size_t>((r + 1) * w + (c - 1))] + s2);
      dt[static_cast<size_t>(r * w + c)] = v;
    }
  }
  return dt;
}

void test_geodesic_wall_detour_and_speed()
{
  std::cout << "[TEST 1] Geodesic Dijkstra wall detour & 500x600 speed benchmark...\n";
  // U-shaped wall separating two points that are 1 m apart in Euclidean space
  // but require a > 12 m detour around the wall.
  const int h = 200, w = 200;
  const double res = 0.05;  // 10m x 10m
  std::vector<uint8_t> free_g(static_cast<size_t>(h * w), 1);
  for (int r = 0; r < h; ++r) {
    free_g[static_cast<size_t>(r * w + 0)] = 0;
    free_g[static_cast<size_t>(r * w + (w - 1))] = 0;
  }
  for (int c = 0; c < w; ++c) {
    free_g[static_cast<size_t>(0 * w + c)] = 0;
    free_g[static_cast<size_t>((h - 1) * w + c)] = 0;
  }
  // Vertical partition wall at x = 5.0 m (col 100) from y = 0..8.5 m (rows 0..170)
  for (int r = 0; r <= 170; ++r) {
    free_g[static_cast<size_t>(r * w + 100)] = 0;
    free_g[static_cast<size_t>(r * w + 101)] = 0;
  }
  auto dt = approx_dt(free_g, h, w);

  Point2D a{4.5, 2.0};
  Point2D b{5.5, 2.0};
  auto sp = compute_geodesic_path(free_g.data(), dt.data(), h, w, res, 0.0, 0.0, a, b, 0.0, 2.0);
  assert(sp.reachable);
  // Euclidean is 1.0 m; detour around y=8.5 m is ~2 * 6.6 + 1.0 > 13.5 m
  assert(sp.distance_m > 13.0 && sp.distance_m < 16.0);
  assert(sp.path_xy.size() > 100);

  auto dmat = compute_geodesic_distances(
    free_g.data(), dt.data(), h, w, res, 0.0, 0.0, {a}, {b}, 0.0, 2.0);
  assert(std::abs(dmat[0] - sp.distance_m) < 0.15);

  // Speed benchmark on a 500x600 grid (target: < 6 ms for 2-robot multi-source query)
  const int bh = 500, bw = 600;
  std::vector<uint8_t> big_free(static_cast<size_t>(bh * bw), 0);
  // Carve a realistic 40% free corridor/room network (~120k free cells)
  for (int r = 100; r < 400; ++r) {
    for (int c = 100; c < 500; ++c) {
      big_free[static_cast<size_t>(r * bw + c)] = 1;
    }
  }
  auto big_dt = approx_dt(big_free, bh, bw);
  std::vector<Point2D> srcs = {{7.0, 7.0}, {22.0, 17.0}};
  std::vector<Point2D> tgts;
  for (int k = 0; k < 16; ++k) {
    tgts.push_back({6.0 + k * 1.1, 6.0 + (k % 4) * 3.0});
  }

  // Warmup + timed run
  auto warm = compute_geodesic_distances(
    big_free.data(), big_dt.data(), bh, bw, res, 0.0, 0.0, srcs, tgts, 12.0, 2.0);
  (void)warm;
  const auto t0 = std::chrono::steady_clock::now();
  const int iters = 10;
  for (int it = 0; it < iters; ++it) {
    auto res_d = compute_geodesic_distances(
      big_free.data(), big_dt.data(), bh, bw, res, 0.0, 0.0, srcs, tgts, 12.0, 2.0);
    assert(res_d.size() == srcs.size() * tgts.size());
  }
  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
    static_cast<double>(iters);
  std::cout << "  detour=" << sp.distance_m << "m | 500x600 2-srcx16-tgt bounded Dijkstra="
            << ms << " ms\n";
  assert(ms < 15.0);
}

void test_visibility_los_vfov_and_submodular()
{
  std::cout << "[TEST 2] 2D Bresenham LoS + 3D vertical FoV + submodular viewpoint cover...\n";
  // Room 10m x 10m at 0.05m with a solid wall at x=5.0m (col 100) from y=1..9m
  const int h = 200, w = 200;
  const double res = 0.05;
  std::vector<int8_t> occ(static_cast<size_t>(h * w), 0);
  std::vector<uint8_t> free_g(static_cast<size_t>(h * w), 1);
  for (int r = 0; r < h; ++r) {
    occ[static_cast<size_t>(r * w + 0)] = 100;
    occ[static_cast<size_t>(r * w + (w - 1))] = 100;
    free_g[static_cast<size_t>(r * w + 0)] = 0;
    free_g[static_cast<size_t>(r * w + (w - 1))] = 0;
  }
  for (int c = 0; c < w; ++c) {
    occ[static_cast<size_t>(0 * w + c)] = 100;
    occ[static_cast<size_t>((h - 1) * w + c)] = 100;
    free_g[static_cast<size_t>(0 * w + c)] = 0;
    free_g[static_cast<size_t>((h - 1) * w + c)] = 0;
  }
  for (int r = 20; r < 180; ++r) {
    occ[static_cast<size_t>(r * w + 100)] = 100;
    free_g[static_cast<size_t>(r * w + 100)] = 0;
  }
  auto dt = approx_dt(free_g, h, w);

  // Direct LoS checks
  assert(!has_line_of_sight(occ.data(), h, w, 100, 80, 100, 120));  // blocked by wall at col 100
  assert(has_line_of_sight(occ.data(), h, w, 100, 110, 100, 140));  // clear on right side

  // Residual cluster on the RIGHT side of the wall (x=5.4m, y=4.5..5.5m)
  std::vector<ResidualColumn> cols;
  for (int k = 0; k < 12; ++k) {
    cols.push_back({5.4, 4.4 + 0.1 * k, 0.3, 1.4, 1});
  }
  VisibilityParams vp;
  vp.r_min = 0.8;
  vp.r_max = 3.0;
  vp.num_samples = 24;
  vp.min_clearance = 0.40;
  auto views = optimize_residual_viewpoints(occ.data(), dt.data(), h, w, res, 0.0, 0.0, cols, vp);
  assert(!views.empty());
  // Chosen viewpoint MUST be on the right side of the wall (x > 5.0 m) so it has LoS!
  assert(views[0].x > 5.05);
  assert(views[0].covered_count == 12);

  // Overhead target outside narrow vertical FoV [-5 deg, +5 deg] at close range -> rejected
  std::vector<ResidualColumn> high_cols = {{6.0, 5.0, 3.5, 4.0, 1}};
  VisibilityParams narrow_fov = vp;
  narrow_fov.r_min = 0.8;
  narrow_fov.r_max = 1.2;
  narrow_fov.vfov_min_rad = -0.087;  // -5 deg
  narrow_fov.vfov_max_rad = 0.087;   // +5 deg
  auto no_views = optimize_residual_viewpoints(
    occ.data(), dt.data(), h, w, res, 0.0, 0.0, high_cols, narrow_fov);
  assert(no_views.empty());
  std::cout << "  best_vp=(" << views[0].x << "," << views[0].y << ") covered="
            << views[0].covered_count << "/12 | narrow_vfov_rejected=OK\n";
}

void test_corridor_bottleneck_and_yield_points()
{
  std::cout << "[TEST 3] Narrow doorway/corridor extraction & spacious yield points...\n";
  // Two wide 4m x 4m rooms connected by a 1.1m wide, 1.5m long doorway at x=4.5..6.0m, y=4.45..5.55m
  const int h = 200, w = 220;
  const double res = 0.05;
  std::vector<uint8_t> free_g(static_cast<size_t>(h * w), 0);
  // Left wide room: x in [0.5, 4.5], y in [2.5, 7.5] (rows 50..150, cols 10..90)
  for (int r = 50; r < 150; ++r) {
    for (int c = 10; c < 90; ++c) {
      free_g[static_cast<size_t>(r * w + c)] = 1;
    }
  }
  // Right wide room: x in [6.0, 10.0], y in [2.5, 7.5] (rows 50..150, cols 120..200)
  for (int r = 50; r < 150; ++r) {
    for (int c = 120; c < 200; ++c) {
      free_g[static_cast<size_t>(r * w + c)] = 1;
    }
  }
  // Narrow doorway connecting them: x in [4.5, 6.0] (cols 90..120), y in [4.45, 5.55] (rows 89..111 -> 1.1m wide)
  for (int r = 89; r < 111; ++r) {
    for (int c = 90; c < 120; ++c) {
      free_g[static_cast<size_t>(r * w + c)] = 1;
    }
  }
  auto dt = approx_dt(free_g, h, w);

  CorridorParams cp;
  cp.robot_radius = 0.22;
  cp.narrow_margin_m = 0.35;  // narrow half-width threshold = 0.79 m (full width < 1.58 m)
  const auto t0 = std::chrono::steady_clock::now();
  auto graph = extract_corridor_bottlenecks(free_g.data(), dt.data(), h, w, res, 0.0, 0.0, cp);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  std::cout << "  segments=" << graph.segments.size() << " elapsed=" << ms << "ms\n";
  // Must detect EXACTLY 1 bottleneck segment (the doorway) and ZERO wide-room walls/corners!
  assert(graph.segments.size() == 1);
  const auto & seg = graph.segments[0];
  std::cout << "  seg[0]: cells=" << seg.cell_count << " centroid=(" << seg.centroid_x << ","
            << seg.centroid_y << ") min_hw=" << seg.min_half_width_m << "m yields="
            << seg.yield_points.size() << "\n";
  assert(seg.centroid_x > 4.2 && seg.centroid_x < 6.3);
  assert(seg.centroid_y > 4.5 && seg.centroid_y < 5.5);
  assert(seg.yield_points.size() >= 2);
  // Yield points must sit on BOTH sides of the doorway (one < 4.5m, one > 6.0m) with generous clearance
  bool has_left = false, has_right = false;
  for (const auto & yp : seg.yield_points) {
    assert(yp.clearance_m >= 0.75);
    if (yp.x < 4.5) has_left = true;
    if (yp.x > 6.0) has_right = true;
  }
  assert(has_left && has_right);
}

}  // namespace

int main()
{
  test_geodesic_wall_detour_and_speed();
  test_visibility_los_vfov_and_submodular();
  test_corridor_bottleneck_and_yield_points();
  std::cout << "ALL geom_kernel C++ TESTS PASSED!\n";
  return 0;
}
