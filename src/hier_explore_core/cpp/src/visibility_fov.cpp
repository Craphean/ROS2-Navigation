#include "geom_kernel/visibility_fov.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace geom_kernel
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

inline bool is_free_cell(int8_t v)
{
  return v >= 0 && v < 20;
}

bool snap_safe_free_cell(
  const int8_t * occ_data,
  const float * obstacle_dt,
  int height,
  int width,
  int r0,
  int c0,
  float min_clearance_cells,
  int max_rad,
  int & out_r,
  int & out_c,
  float & out_clr)
{
  int best_r = -1;
  int best_c = -1;
  int best_d2 = max_rad * max_rad + 1;
  float best_clr = -1.0f;

  const int r_min = std::max(0, r0 - max_rad);
  const int r_max = std::min(height - 1, r0 + max_rad);
  const int c_min = std::max(0, c0 - max_rad);
  const int c_max = std::min(width - 1, c0 + max_rad);

  for (int r = r_min; r <= r_max; ++r) {
    const int dr = r - r0;
    const int dr2 = dr * dr;
    if (dr2 > best_d2) {
      continue;
    }
    const int row_off = r * width;
    for (int c = c_min; c <= c_max; ++c) {
      const int idx = row_off + c;
      if (!is_free_cell(occ_data[idx])) {
        continue;
      }
      const float clr = obstacle_dt ? obstacle_dt[idx] : min_clearance_cells;
      if (clr < min_clearance_cells) {
        continue;
      }
      const int dc = c - c0;
      const int d2 = dr2 + dc * dc;
      if (d2 < best_d2 || (d2 == best_d2 && clr > best_clr)) {
        best_d2 = d2;
        best_r = r;
        best_c = c;
        best_clr = clr;
      }
    }
  }
  if (best_r < 0) {
    return false;
  }
  out_r = best_r;
  out_c = best_c;
  out_clr = best_clr;
  return true;
}

}  // namespace

bool has_line_of_sight(
  const int8_t * occ_data,
  int height,
  int width,
  int r0,
  int c0,
  int r1,
  int c1,
  int8_t lethal_threshold)
{
  if (!occ_data || height <= 0 || width <= 0) {
    return false;
  }
  if (r0 < 0 || r0 >= height || c0 < 0 || c0 >= width ||
      r1 < 0 || r1 >= height || c1 < 0 || c1 >= width)
  {
    return false;
  }
  if (r0 == r1 && c0 == c1) {
    return occ_data[r0 * width + c0] <= lethal_threshold;
  }

  int r = r0;
  int c = c0;
  const int dr = std::abs(r1 - r0);
  const int dc = std::abs(c1 - c0);
  const int sr = (r0 < r1) ? 1 : -1;
  const int sc = (c0 < c1) ? 1 : -1;
  int err = dc - dr;

  while (true) {
    if (r == r1 && c == c1) {
      return true;
    }
    if (!(r == r0 && c == c0)) {
      if (occ_data[r * width + c] > lethal_threshold) {
        return false;
      }
    }
    const int e2 = 2 * err;
    if (e2 > -dr) {
      err -= dr;
      c += sc;
    }
    if (e2 < dc) {
      err += dc;
      r += sr;
    }
  }
}

std::vector<ViewpointCandidate> optimize_residual_viewpoints(
  const int8_t * occ_data,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<ResidualColumn> & columns,
  const VisibilityParams & params)
{
  std::vector<ViewpointCandidate> selected;
  if (!occ_data || height <= 0 || width <= 0 || resolution <= 0.0 || columns.empty() ||
      params.max_viewpoints <= 0)
  {
    return selected;
  }

  // Group columns by cluster_id and precompute each column's occupancy cell
  std::unordered_map<int32_t, std::vector<int>> clusters;
  std::vector<int> col_r(columns.size());
  std::vector<int> col_c(columns.size());
  for (size_t i = 0; i < columns.size(); ++i) {
    clusters[columns[i].cluster_id].push_back(static_cast<int>(i));
    col_r[i] = std::clamp(
      static_cast<int>(std::floor((columns[i].y - origin_y) / resolution)), 0, height - 1);
    col_c[i] = std::clamp(
      static_cast<int>(std::floor((columns[i].x - origin_x) / resolution)), 0, width - 1);
  }

  const float min_clr_cells = static_cast<float>(params.min_clearance / resolution);
  const int snap_rad = std::max(1, static_cast<int>(std::ceil(0.35 / resolution)));
  const int k_samples = std::max(3, params.num_samples);
  const double r_min = std::max(0.2, params.r_min);
  const double r_max = std::max(r_min, params.r_max);

  // Sample 3 concentric rings across [r_min, r_max] totaling ~num_samples viewpoints per cluster
  const int n_rings = (r_max - r_min > 0.25) ? 3 : 1;
  const int per_ring = std::max(4, (k_samples + n_rings - 1) / n_rings);

  struct EvalCandidate
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double clearance_m{0.0};
    double dist_to_cluster{0.0};
    int32_t cluster_id{0};
    int32_t cluster_total{1};
    std::vector<int> visible_cols;
  };

  std::vector<EvalCandidate> pool;
  std::unordered_set<uint64_t> seen_cells;

  // Stable cluster iteration order
  std::vector<int32_t> cluster_keys;
  cluster_keys.reserve(clusters.size());
  for (const auto & kv : clusters) {
    cluster_keys.push_back(kv.first);
  }
  std::sort(cluster_keys.begin(), cluster_keys.end());

  const double eff_r2 = params.max_eff_range * params.max_eff_range;

  for (int32_t cid : cluster_keys) {
    const auto & idxs = clusters[cid];
    if (idxs.empty()) {
      continue;
    }
    double cx = 0.0;
    double cy = 0.0;
    for (int idx : idxs) {
      cx += columns[static_cast<size_t>(idx)].x;
      cy += columns[static_cast<size_t>(idx)].y;
    }
    cx /= static_cast<double>(idxs.size());
    cy /= static_cast<double>(idxs.size());

    for (int ring = 0; ring < n_rings; ++ring) {
      const double frac = (n_rings == 1) ? 0.5 : static_cast<double>(ring) / static_cast<double>(n_rings - 1);
      const double rad = r_min + frac * (r_max - r_min);
      const double phase = (ring % 2 == 1) ? (0.5 * kTwoPi / static_cast<double>(per_ring)) : 0.0;

      for (int a = 0; a < per_ring; ++a) {
        const double theta = phase + kTwoPi * static_cast<double>(a) / static_cast<double>(per_ring);
        const double sx = cx + rad * std::cos(theta);
        const double sy = cy + rad * std::sin(theta);
        const int sr = static_cast<int>(std::floor((sy - origin_y) / resolution));
        const int sc = static_cast<int>(std::floor((sx - origin_x) / resolution));
        if (sr < 0 || sr >= height || sc < 0 || sc >= width) {
          continue;
        }
        int vr = -1, vc = -1;
        float vclr = 0.0f;
        if (!snap_safe_free_cell(
            occ_data, obstacle_dt, height, width, sr, sc, min_clr_cells, snap_rad, vr, vc, vclr))
        {
          continue;
        }
        const uint64_t cell_key = (static_cast<uint64_t>(vr) << 32) | static_cast<uint32_t>(vc);
        if (!seen_cells.insert(cell_key).second) {
          continue;
        }

        const double vx = origin_x + (static_cast<double>(vc) + 0.5) * resolution;
        const double vy = origin_y + (static_cast<double>(vr) + 0.5) * resolution;

        EvalCandidate cand;
        cand.x = vx;
        cand.y = vy;
        cand.clearance_m = static_cast<double>(vclr) * resolution;
        cand.dist_to_cluster = std::hypot(vx - cx, vy - cy);
        cand.cluster_id = cid;
        cand.cluster_total = static_cast<int32_t>(idxs.size());

        double vis_cx = 0.0;
        double vis_cy = 0.0;

        // Evaluate visibility across ALL residual columns within max_eff_range
        // so one well-placed viewpoint can simultaneously cover nearby clusters!
        for (size_t q = 0; q < columns.size(); ++q) {
          const double dx = columns[q].x - vx;
          const double dy = columns[q].y - vy;
          const double d2 = dx * dx + dy * dy;
          if (d2 > eff_r2 || d2 < 0.0225) {  // d < 0.15 m or d > max_eff_range
            continue;
          }
          const double d = std::sqrt(d2);

          // Vertical FoV interval check [vfov_min_rad, vfov_max_rad]
          double zlo = columns[q].z_min;
          double zhi = columns[q].z_max;
          if (!std::isfinite(zlo) || !std::isfinite(zhi) || zhi < zlo) {
            zlo = 0.25;
            zhi = 1.50;
          }
          const double elev_lo = std::atan2(zlo - params.sensor_z, d);
          const double elev_hi = std::atan2(zhi - params.sensor_z, d);
          if (elev_hi < params.vfov_min_rad || elev_lo > params.vfov_max_rad) {
            continue;
          }

          // 2D Bresenham wall-occlusion LoS check
          if (!has_line_of_sight(
              occ_data, height, width, vr, vc, col_r[q], col_c[q], params.lethal_threshold))
          {
            continue;
          }

          cand.visible_cols.push_back(static_cast<int>(q));
          vis_cx += columns[q].x;
          vis_cy += columns[q].y;
        }

        if (!cand.visible_cols.empty()) {
          const double inv_n = 1.0 / static_cast<double>(cand.visible_cols.size());
          cand.yaw = std::atan2(vis_cy * inv_n - vy, vis_cx * inv_n - vx);
          pool.push_back(std::move(cand));
        }
      }
    }
  }

  if (pool.empty()) {
    return selected;
  }

  // Greedy submodular set-cover selection maximizing marginal newly covered columns
  std::vector<uint8_t> covered(columns.size(), 0);
  const double dedupe_r2 = params.dedupe_radius * params.dedupe_radius;

  for (int pick = 0; pick < params.max_viewpoints; ++pick) {
    int best_idx = -1;
    int best_marginal = 0;
    double best_tie = -1e18;

    for (size_t i = 0; i < pool.size(); ++i) {
      const auto & cand = pool[i];
      bool too_close = false;
      for (const auto & chosen : selected) {
        const double dx = cand.x - chosen.x;
        const double dy = cand.y - chosen.y;
        if (dx * dx + dy * dy < dedupe_r2) {
          too_close = true;
          break;
        }
      }
      if (too_close) {
        continue;
      }

      int marginal = 0;
      for (int q : cand.visible_cols) {
        if (!covered[static_cast<size_t>(q)]) {
          ++marginal;
        }
      }
      if (marginal <= 0) {
        continue;
      }
      const double tie = 0.5 * cand.clearance_m - 0.1 * cand.dist_to_cluster;
      if (marginal > best_marginal || (marginal == best_marginal && tie > best_tie)) {
        best_marginal = marginal;
        best_tie = tie;
        best_idx = static_cast<int>(i);
      }
    }

    if (best_idx < 0) {
      break;
    }

    const auto & win = pool[static_cast<size_t>(best_idx)];
    for (int q : win.visible_cols) {
      covered[static_cast<size_t>(q)] = 1;
    }

    ViewpointCandidate out;
    out.x = win.x;
    out.y = win.y;
    out.yaw = win.yaw;
    out.clearance_m = win.clearance_m;
    out.cluster_id = win.cluster_id;
    out.covered_count = static_cast<int32_t>(win.visible_cols.size());
    out.marginal_gain = best_marginal;
    out.coverage_ratio = static_cast<double>(best_marginal) /
      static_cast<double>(std::max<int32_t>(1, win.cluster_total));
    selected.push_back(out);
  }

  return selected;
}

}  // namespace geom_kernel
