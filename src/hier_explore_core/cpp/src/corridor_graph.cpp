#include "geom_kernel/corridor_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace geom_kernel
{
namespace
{

constexpr double kSqrt2 = 1.4142135623730951;

inline bool in_bounds(int r, int c, int height, int width)
{
  return r >= 0 && r < height && c >= 0 && c < width;
}

// Raycast in free space along step (dr, dc) up to max_steps.
// Sets hit_lethal=true ONLY when the ray terminates on a true lethal obstacle
// (obstacle_dt < 0.5f), not when it merely reaches an unexplored UNKNOWN boundary.
int wall_dist_along(
  const uint8_t * is_free,
  const float * obstacle_dt,
  int height,
  int width,
  int r,
  int c,
  int dr,
  int dc,
  int max_steps,
  bool & hit_lethal,
  int & free_steps)
{
  hit_lethal = false;
  free_steps = 0;
  for (int s = 1; s <= max_steps; ++s) {
    const int nr = r + s * dr;
    const int nc = c + s * dc;
    if (!in_bounds(nr, nc, height, width)) {
      return s;
    }
    const int nidx = nr * width + nc;
    if (obstacle_dt[nidx] < 0.5f) {
      hit_lethal = true;
      return s;
    }
    if (is_free[nidx] == 0) {
      // Unexplored unknown cell: stops free travel, but is NOT a physical wall
      return s;
    }
    free_steps = s;
  }
  return max_steps;
}

}  // namespace

CorridorGraphResult extract_corridor_bottlenecks(
  const uint8_t * is_free,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const CorridorParams & params)
{
  CorridorGraphResult res;
  res.height = height;
  res.width = width;
  if (!is_free || !obstacle_dt || height <= 0 || width <= 0 || resolution <= 0.0) {
    return res;
  }

  const int n_cells = height * width;
  res.labels.assign(static_cast<size_t>(n_cells), 0);

  const double narrow_half_m = 2.0 * params.robot_radius + params.narrow_margin_m;
  const double min_pass_m = params.robot_radius + params.min_passable_margin_m;
  const float narrow_cells = static_cast<float>(narrow_half_m / resolution);
  const float min_pass_cells = static_cast<float>(min_pass_m / resolution);
  const double max_full_width_cells = 2.0 * static_cast<double>(narrow_cells);
  const int max_ray_cardinal = std::max(2, static_cast<int>(std::ceil(max_full_width_cells)) + 1);
  const int max_ray_diag = std::max(2, static_cast<int>(std::ceil(max_full_width_cells / kSqrt2)) + 1);
  const double min_perp_each_cells = std::max(2.0, static_cast<double>(min_pass_cells) * 0.8);

  // 4 cross-section axes and their perpendicular travel axes:
  //   axis 0: horizontal (0,1)  perp: vertical (1,0)
  //   axis 1: vertical (1,0)    perp: horizontal (0,1)
  //   axis 2: diag (1,1)        perp: anti-diag (1,-1)
  //   axis 3: anti-diag (1,-1)  perp: diag (1,1)
  constexpr int kAxDR[4] = {0, 1, 1, 1};
  constexpr int kAxDC[4] = {1, 0, 1, -1};
  constexpr int kPerpDR[4] = {1, 0, 1, 1};
  constexpr int kPerpDC[4] = {0, 1, -1, 1};
  constexpr double kStepLen[4] = {1.0, 1.0, kSqrt2, kSqrt2};

  // Step 1: Identify true narrow-passage centerline / interior cells (two opposing
  // walls within 2*narrow_half_m AND open travel along the perpendicular axis so
  // 90-degree corners and wide-room wall strips produce zero false positives).
  std::vector<uint8_t> core_mask(static_cast<size_t>(n_cells), 0);

  for (int r = 1; r < height - 1; ++r) {
    const int row_off = r * width;
    for (int c = 1; c < width - 1; ++c) {
      const int idx = row_off + c;
      if (is_free[idx] == 0) {
        continue;
      }
      const float dt = obstacle_dt[idx];
      if (dt < min_pass_cells || dt >= narrow_cells) {
        continue;
      }

      bool is_bottleneck = false;
      for (int ax = 0; ax < 4; ++ax) {
        const int max_steps = (ax < 2) ? max_ray_cardinal : max_ray_diag;
        bool hit_p = false, hit_n = false;
        int free_p = 0, free_n = 0;
        const int dp = wall_dist_along(
          is_free, obstacle_dt, height, width, r, c, kAxDR[ax], kAxDC[ax], max_steps, hit_p, free_p);
        if (!hit_p) {
          continue;
        }
        const int dn = wall_dist_along(
          is_free, obstacle_dt, height, width, r, c, -kAxDR[ax], -kAxDC[ax], max_steps, hit_n, free_n);
        if (!hit_n) {
          continue;
        }
        // Require the cell to sit near the medial centerline between the two opposing
        // physical walls, and reject recesses next to a closer third wall.
        if (std::abs(dp - dn) > 2) {
          continue;
        }
        const double chord_width = static_cast<double>(dp + dn) * kStepLen[ax];
        const double half_w = 0.5 * chord_width;
        if (chord_width >= max_full_width_cells || half_w > static_cast<double>(dt) + 2.0) {
          continue;
        }
        // Reject dead-end 90-deg / acute room corners and side-wall recesses:
        // in a genuine through-passage (doorway or corridor), BOTH perpendicular travel
        // directions stay open well beyond 0.75 * chord_width without hitting a wall.
        bool p_hit_p = false, p_hit_n = false;
        int p_free_p = 0, p_free_n = 0;
        const double perp_req_cells = std::max(min_perp_each_cells, 0.75 * chord_width);
        const int perp_need = std::max(2, static_cast<int>(std::ceil(perp_req_cells / kStepLen[ax])));
        const int pp = wall_dist_along(
          is_free, obstacle_dt, height, width, r, c, kPerpDR[ax], kPerpDC[ax], perp_need + 1, p_hit_p, p_free_p);
        const int pn = wall_dist_along(
          is_free, obstacle_dt, height, width, r, c, -kPerpDR[ax], -kPerpDC[ax], perp_need + 1, p_hit_n, p_free_n);
        if (static_cast<double>(pp) * kStepLen[ax] >= perp_req_cells &&
            static_cast<double>(pn) * kStepLen[ax] >= perp_req_cells &&
            static_cast<double>(p_free_p) * kStepLen[ax] >= min_perp_each_cells &&
            static_cast<double>(p_free_n) * kStepLen[ax] >= min_perp_each_cells)
        {
          is_bottleneck = true;
          break;
        }
      }
      if (is_bottleneck) {
        core_mask[static_cast<size_t>(idx)] = 1;
      }
    }
  }

  // Step 2: Light 1-cell morphological bridge inside narrow free space to connect
  // discrete centerline pixels across angled doorways cleanly.
  std::vector<uint8_t> narrow_mask = core_mask;
  constexpr int kN8R[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  constexpr int kN8C[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  for (int r = 1; r < height - 1; ++r) {
    const int row_off = r * width;
    for (int c = 1; c < width - 1; ++c) {
      const int idx = row_off + c;
      if (!core_mask[static_cast<size_t>(idx)]) {
        continue;
      }
      for (int k = 0; k < 8; ++k) {
        const int nr = r + kN8R[k];
        const int nc = c + kN8C[k];
        const int nidx = nr * width + nc;
        if (is_free[nidx] && obstacle_dt[nidx] >= min_pass_cells * 0.8f &&
            obstacle_dt[nidx] < narrow_cells)
        {
          narrow_mask[static_cast<size_t>(nidx)] = 1;
        }
      }
    }
  }

  // Step 3: 8-connected components labeling -> filter small noise fragments
  std::vector<int32_t> comp_id(static_cast<size_t>(n_cells), 0);
  std::vector<std::vector<int32_t>> comp_cells;
  std::queue<int32_t> q;

  for (int i = 0; i < n_cells; ++i) {
    if (!narrow_mask[static_cast<size_t>(i)] || comp_id[static_cast<size_t>(i)] != 0) {
      continue;
    }
    const int32_t cid = static_cast<int32_t>(comp_cells.size()) + 1;
    std::vector<int32_t> cells;
    comp_id[static_cast<size_t>(i)] = cid;
    q.push(i);
    while (!q.empty()) {
      const int32_t cur = q.front();
      q.pop();
      cells.push_back(cur);
      const int r = cur / width;
      const int c = cur - r * width;
      for (int k = 0; k < 8; ++k) {
        const int nr = r + kN8R[k];
        const int nc = c + kN8C[k];
        if (!in_bounds(nr, nc, height, width)) {
          continue;
        }
        const int nidx = nr * width + nc;
        if (narrow_mask[static_cast<size_t>(nidx)] && comp_id[static_cast<size_t>(nidx)] == 0) {
          comp_id[static_cast<size_t>(nidx)] = cid;
          q.push(nidx);
        }
      }
    }
    comp_cells.push_back(std::move(cells));
  }

  // Step 4: For each valid bottleneck segment, compute geometry stats and BFS
  // outward from its portals to find spacious Yield Points on each approach side.
  const int yield_max_steps = std::max(
    4, static_cast<int>(std::ceil(params.yield_search_radius_m / resolution)));
  const float yield_clr_pref = narrow_cells;
  const float yield_clr_min = static_cast<float>((params.robot_radius + 0.18) / resolution);
  const double sep_r2 = params.yield_min_separation_m * params.yield_min_separation_m;

  std::vector<int32_t> bfs_stamp(static_cast<size_t>(n_cells), 0);
  std::vector<int16_t> bfs_dist(static_cast<size_t>(n_cells), 0);
  int32_t stamp_gen = 0;

  int32_t out_id = 0;
  for (const auto & cells : comp_cells) {
    if (static_cast<int>(cells.size()) < params.min_segment_cells) {
      continue;
    }
    double min_dt = std::numeric_limits<double>::infinity();
    double max_dt = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    int r_min = height, r_max = 0, c_min = width, c_max = 0;

    for (int32_t cid : cells) {
      const int r = cid / width;
      const int c = cid - r * width;
      r_min = std::min(r_min, r);
      r_max = std::max(r_max, r);
      c_min = std::min(c_min, c);
      c_max = std::max(c_max, c);
      sum_x += origin_x + (static_cast<double>(c) + 0.5) * resolution;
      sum_y += origin_y + (static_cast<double>(r) + 0.5) * resolution;
      const double w_m = static_cast<double>(obstacle_dt[cid]) * resolution;
      min_dt = std::min(min_dt, w_m);
      max_dt = std::max(max_dt, w_m);
    }

    const double seg_min_half_w = std::isfinite(min_dt) ? min_dt : 0.0;
    const double seg_length_m = std::hypot(
      static_cast<double>(r_max - r_min + 1),
      static_cast<double>(c_max - c_min + 1)) * resolution;

    // Exempt short doorway frames / thin furniture gaps unless ultra-tight single-robot choke
    if (params.min_segment_length_m > 0.0 &&
        seg_length_m < params.min_segment_length_m &&
        seg_min_half_w >= params.robot_radius + 0.14)
    {
      continue;
    }

    ++out_id;

    CorridorSegment seg;
    seg.id = out_id;
    seg.cell_count = static_cast<int32_t>(cells.size());
    seg.min_half_width_m = seg_min_half_w;
    seg.max_half_width_m = max_dt;
    seg.centroid_x = sum_x / static_cast<double>(cells.size());
    seg.centroid_y = sum_y / static_cast<double>(cells.size());
    seg.length_m = seg_length_m;

    ++stamp_gen;
    std::queue<int32_t> yq;

    for (int32_t cid : cells) {
      res.labels[static_cast<size_t>(cid)] = out_id;
      bfs_stamp[static_cast<size_t>(cid)] = stamp_gen;
      bfs_dist[static_cast<size_t>(cid)] = 0;
      yq.push(cid);
    }

    struct YieldCand
    {
      double x;
      double y;
      double clearance_m;
      double score;
    };
    std::vector<YieldCand> yield_cands;

    constexpr int kC4R[4] = {-1, 1, 0, 0};
    constexpr int kC4C[4] = {0, 0, -1, 1};
    while (!yq.empty()) {
      const int32_t cur = yq.front();
      yq.pop();
      const int16_t d = bfs_dist[static_cast<size_t>(cur)];
      if (d >= yield_max_steps) {
        continue;
      }
      const int r = cur / width;
      const int c = cur - r * width;
      for (int k = 0; k < 4; ++k) {
        const int nr = r + kC4R[k];
        const int nc = c + kC4C[k];
        if (!in_bounds(nr, nc, height, width)) {
          continue;
        }
        const int nidx = nr * width + nc;
        if (bfs_stamp[static_cast<size_t>(nidx)] == stamp_gen || is_free[nidx] == 0) {
          continue;
        }
        bfs_stamp[static_cast<size_t>(nidx)] = stamp_gen;
        bfs_dist[static_cast<size_t>(nidx)] = static_cast<int16_t>(d + 1);
        yq.push(nidx);

        // Candidate yield cell must be outside the narrow bottleneck and reasonably clear
        if (narrow_mask[static_cast<size_t>(nidx)] == 0 && obstacle_dt[nidx] >= yield_clr_min) {
          const double clr_m = static_cast<double>(obstacle_dt[nidx]) * resolution;
          const double dist_m = static_cast<double>(d + 1) * resolution;
          // Prefer spacious cells (>= narrow_half_m) close to the portal (not too far away)
          const double bonus = (obstacle_dt[nidx] >= yield_clr_pref) ? 2.0 : 0.0;
          const double sc = bonus + std::min(clr_m, 1.8) - 0.35 * dist_m;
          yield_cands.push_back({
            origin_x + (static_cast<double>(nc) + 0.5) * resolution,
            origin_y + (static_cast<double>(nr) + 0.5) * resolution,
            clr_m,
            sc
          });
        }
      }
    }

    std::sort(
      yield_cands.begin(), yield_cands.end(),
      [](const YieldCand & a, const YieldCand & b) { return a.score > b.score; });

    for (const auto & yc : yield_cands) {
      if (static_cast<int>(seg.yield_points.size()) >= params.max_yield_points) {
        break;
      }
      bool too_close = false;
      for (const auto & chosen : seg.yield_points) {
        const double dx = yc.x - chosen.x;
        const double dy = yc.y - chosen.y;
        if (dx * dx + dy * dy < sep_r2) {
          too_close = true;
          break;
        }
      }
      if (!too_close) {
        seg.yield_points.push_back({yc.x, yc.y, yc.clearance_m});
      }
    }

    res.segments.push_back(std::move(seg));
  }

  return res;
}

}  // namespace geom_kernel
