#include "geom_kernel/geodesic_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace geom_kernel
{
namespace
{

constexpr double kInfD = std::numeric_limits<double>::infinity();
constexpr uint32_t kInfU = std::numeric_limits<uint32_t>::max();
constexpr float kInfF = std::numeric_limits<float>::infinity();
constexpr float kSqrt2 = 1.4142135623730951f;

// 8-connected offsets: first 4 are cardinal, last 4 are diagonal.
// Integer step scale 12 (cardinal) / 17 (diagonal): 17/12 = 1.41667 (0.17% of sqrt(2)),
// enabling an O(1) 32-bucket Dial's queue with bitmask indexing.
constexpr int kDR[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
constexpr int kDC[8] = {0, 0, -1, 1, -1, 1, -1, 1};
constexpr uint32_t kUnit = 12u;
constexpr uint32_t kStepU[8] = {12u, 12u, 12u, 12u, 17u, 17u, 17u, 17u};
constexpr float kStepF[8] = {1.0f, 1.0f, 1.0f, 1.0f, kSqrt2, kSqrt2, kSqrt2, kSqrt2};

struct HeapNode
{
  float prio;
  int32_t idx;
  bool operator>(const HeapNode & o) const { return prio > o.prio; }
};

inline bool in_bounds(int r, int c, int height, int width)
{
  return r >= 0 && r < height && c >= 0 && c < width;
}

inline int world_to_cell_clamped(double val, double origin, double res, int limit)
{
  const int idx = static_cast<int>(std::floor((val - origin) / res));
  return std::clamp(idx, 0, limit - 1);
}

int snap_to_passable(
  const uint8_t * passable,
  const uint8_t * traversable,
  int height,
  int width,
  int r0,
  int c0,
  int max_radius)
{
  if (!in_bounds(r0, c0, height, width)) {
    return -1;
  }
  const int idx0 = r0 * width + c0;
  if (passable[idx0]) {
    return idx0;
  }

  int best_idx = -1;
  int best_d2 = max_radius * max_radius + 1;
  const int r_min = std::max(0, r0 - max_radius);
  const int r_max = std::min(height - 1, r0 + max_radius);
  const int c_min = std::max(0, c0 - max_radius);
  const int c_max = std::min(width - 1, c0 + max_radius);
  for (int r = r_min; r <= r_max; ++r) {
    const int dr = r - r0;
    const int dr2 = dr * dr;
    if (dr2 >= best_d2) {
      continue;
    }
    const int row_off = r * width;
    for (int c = c_min; c <= c_max; ++c) {
      const int idx = row_off + c;
      if (!passable[idx]) {
        continue;
      }
      const int dc = c - c0;
      const int d2 = dr2 + dc * dc;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = idx;
      }
    }
  }
  if (best_idx >= 0) {
    return best_idx;
  }

  if (traversable[idx0]) {
    return idx0;
  }
  best_d2 = max_radius * max_radius + 1;
  for (int r = r_min; r <= r_max; ++r) {
    const int dr = r - r0;
    const int dr2 = dr * dr;
    if (dr2 >= best_d2) {
      continue;
    }
    const int row_off = r * width;
    for (int c = c_min; c <= c_max; ++c) {
      const int idx = row_off + c;
      if (!traversable[idx]) {
        continue;
      }
      const int dc = c - c0;
      const int d2 = dr2 + dc * dc;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = idx;
      }
    }
  }
  return best_idx;
}

void fill_passable_mask(
  const uint8_t * traversable,
  const float * obstacle_dt,
  int n_cells,
  double min_clearance_cells,
  std::vector<uint8_t> & passable)
{
  passable.resize(static_cast<size_t>(n_cells));
  const float clr = static_cast<float>(std::max(0.0, min_clearance_cells));
  if (obstacle_dt && clr > 0.0f) {
    for (int i = 0; i < n_cells; ++i) {
      passable[static_cast<size_t>(i)] =
        (traversable[i] != 0 && obstacle_dt[i] >= clr) ? 1 : 0;
    }
  } else {
    for (int i = 0; i < n_cells; ++i) {
      passable[static_cast<size_t>(i)] = (traversable[i] != 0) ? 1 : 0;
    }
  }
}

inline float octile_heuristic(int r, int c, int gr, int gc)
{
  const int dr = std::abs(r - gr);
  const int dc = std::abs(c - gc);
  const int dmin = std::min(dr, dc);
  const int dmax = std::max(dr, dc);
  return static_cast<float>(dmax) + (kSqrt2 - 1.0f) * static_cast<float>(dmin);
}

struct DialScratch
{
  std::vector<uint8_t> passable;
  std::vector<uint8_t> is_target;
  std::vector<uint32_t> dist;
  std::vector<uint32_t> stamp;
  std::array<std::vector<int32_t>, 32> buckets;
  uint32_t gen{0};

  void ensure_size(size_t n)
  {
    if (dist.size() < n) {
      dist.assign(n, kInfU);
      stamp.assign(n, 0);
      gen = 0;
    }
    if (is_target.size() < n) {
      is_target.assign(n, 0);
    }
  }
};

}  // namespace

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
  double max_dist_m,
  double min_clearance_cells,
  double snap_radius_m)
{
  const size_t n_src = sources_xy.size();
  const size_t n_tgt = targets_xy.size();
  std::vector<double> out(n_src * n_tgt, kInfD);
  if (n_src == 0 || n_tgt == 0 || height <= 0 || width <= 0 || resolution <= 0.0 || !traversable) {
    return out;
  }

  const int n_cells = height * width;
  const int snap_cells = std::max(1, static_cast<int>(std::ceil(snap_radius_m / resolution)));
  const uint32_t max_dist_u = (max_dist_m > 0.0)
    ? static_cast<uint32_t>(std::ceil((max_dist_m / resolution) * static_cast<double>(kUnit)))
    : (kInfU - 64u);

  thread_local DialScratch sc;
  sc.ensure_size(static_cast<size_t>(n_cells));
  fill_passable_mask(traversable, obstacle_dt, n_cells, min_clearance_cells, sc.passable);

  std::vector<int> src_cells(n_src, -1);
  for (size_t s = 0; s < n_src; ++s) {
    const int r = world_to_cell_clamped(sources_xy[s].y, origin_y, resolution, height);
    const int c = world_to_cell_clamped(sources_xy[s].x, origin_x, resolution, width);
    src_cells[s] = snap_to_passable(sc.passable.data(), traversable, height, width, r, c, snap_cells);
  }

  std::vector<int> tgt_cells(n_tgt, -1);
  std::vector<int> touched_targets;
  touched_targets.reserve(n_tgt);
  for (size_t t = 0; t < n_tgt; ++t) {
    const int r = world_to_cell_clamped(targets_xy[t].y, origin_y, resolution, height);
    const int c = world_to_cell_clamped(targets_xy[t].x, origin_x, resolution, width);
    const int cid = snap_to_passable(sc.passable.data(), traversable, height, width, r, c, snap_cells);
    tgt_cells[t] = cid;
    if (cid >= 0 && !sc.is_target[static_cast<size_t>(cid)]) {
      sc.is_target[static_cast<size_t>(cid)] = 1;
      touched_targets.push_back(cid);
    }
  }

  const int unique_valid_targets = static_cast<int>(touched_targets.size());
  if (unique_valid_targets == 0) {
    return out;
  }

  const double u_to_m = resolution / static_cast<double>(kUnit);
  uint32_t * __restrict__ dist = sc.dist.data();
  uint32_t * __restrict__ stamp = sc.stamp.data();
  uint8_t * __restrict__ passable = sc.passable.data();
  const uint8_t * __restrict__ is_target = sc.is_target.data();

  for (size_t s = 0; s < n_src; ++s) {
    const int s_cell = src_cells[s];
    if (s_cell < 0) {
      continue;
    }

    ++sc.gen;
    if (sc.gen == 0) {
      std::fill(sc.stamp.begin(), sc.stamp.begin() + n_cells, 0);
      sc.gen = 1;
    }
    const uint32_t gen = sc.gen;

    for (auto & b : sc.buckets) {
      b.clear();
    }

    const uint8_t saved_start = passable[s_cell];
    passable[s_cell] = 1;

    dist[s_cell] = 0u;
    stamp[s_cell] = gen;
    sc.buckets[0].push_back(s_cell);
    size_t queued = 1;
    uint32_t cur_d = 0u;
    int remaining = unique_valid_targets;

    while (queued > 0 && cur_d <= max_dist_u) {
      auto & cur_bucket = sc.buckets[cur_d & 31u];
      while (!cur_bucket.empty()) {
        const int32_t idx = cur_bucket.back();
        cur_bucket.pop_back();
        --queued;

        if (dist[idx] != cur_d) {
          continue;
        }
        if (is_target[idx]) {
          if (--remaining == 0) {
            queued = 0;
            break;
          }
        }

        const int r = idx / width;
        const int c = idx - r * width;

        // Cardinal neighbors (0..3)
        if (r > 0) {
          const int nidx = idx - width;
          if (passable[nidx] || (is_target[nidx] && traversable[nidx])) {
            const uint32_t nd = cur_d + kStepU[0];
            if (nd <= max_dist_u && (stamp[nidx] != gen || nd < dist[nidx])) {
              stamp[nidx] = gen;
              dist[nidx] = nd;
              sc.buckets[nd & 31u].push_back(nidx);
              ++queued;
            }
          }
        }
        if (r + 1 < height) {
          const int nidx = idx + width;
          if (passable[nidx] || (is_target[nidx] && traversable[nidx])) {
            const uint32_t nd = cur_d + kStepU[1];
            if (nd <= max_dist_u && (stamp[nidx] != gen || nd < dist[nidx])) {
              stamp[nidx] = gen;
              dist[nidx] = nd;
              sc.buckets[nd & 31u].push_back(nidx);
              ++queued;
            }
          }
        }
        if (c > 0) {
          const int nidx = idx - 1;
          if (passable[nidx] || (is_target[nidx] && traversable[nidx])) {
            const uint32_t nd = cur_d + kStepU[2];
            if (nd <= max_dist_u && (stamp[nidx] != gen || nd < dist[nidx])) {
              stamp[nidx] = gen;
              dist[nidx] = nd;
              sc.buckets[nd & 31u].push_back(nidx);
              ++queued;
            }
          }
        }
        if (c + 1 < width) {
          const int nidx = idx + 1;
          if (passable[nidx] || (is_target[nidx] && traversable[nidx])) {
            const uint32_t nd = cur_d + kStepU[3];
            if (nd <= max_dist_u && (stamp[nidx] != gen || nd < dist[nidx])) {
              stamp[nidx] = gen;
              dist[nidx] = nd;
              sc.buckets[nd & 31u].push_back(nidx);
              ++queued;
            }
          }
        }

        // Diagonal neighbors (4..7) with corner-cut guard
        for (int dir = 4; dir < 8; ++dir) {
          const int nr = r + kDR[dir];
          const int nc = c + kDC[dir];
          if (nr < 0 || nr >= height || nc < 0 || nc >= width) {
            continue;
          }
          const int nidx = nr * width + nc;
          if (!passable[nidx] && !(is_target[nidx] && traversable[nidx])) {
            continue;
          }
          if (!traversable[nr * width + c] || !traversable[r * width + nc]) {
            continue;
          }
          const uint32_t nd = cur_d + kStepU[dir];
          if (nd <= max_dist_u && (stamp[nidx] != gen || nd < dist[nidx])) {
            stamp[nidx] = gen;
            dist[nidx] = nd;
            sc.buckets[nd & 31u].push_back(nidx);
            ++queued;
          }
        }
      }
      ++cur_d;
    }

    passable[s_cell] = saved_start;

    for (size_t t = 0; t < n_tgt; ++t) {
      const int tc = tgt_cells[t];
      if (tc >= 0 && stamp[tc] == gen) {
        out[s * n_tgt + t] = static_cast<double>(dist[tc]) * u_to_m;
      }
    }
  }

  for (int cid : touched_targets) {
    sc.is_target[static_cast<size_t>(cid)] = 0;
  }

  return out;
}

std::vector<double> compute_geodesic_matrix(
  const uint8_t * traversable,
  const float * obstacle_dt,
  int height,
  int width,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<Point2D> & points_xy,
  double max_dist_m,
  double min_clearance_cells,
  double snap_radius_m)
{
  auto mat = compute_geodesic_distances(
    traversable, obstacle_dt, height, width, resolution, origin_x, origin_y,
    points_xy, points_xy, max_dist_m, min_clearance_cells, snap_radius_m);
  const size_t n = points_xy.size();
  for (size_t i = 0; i < n; ++i) {
    mat[i * n + i] = 0.0;
    for (size_t j = i + 1; j < n; ++j) {
      const double d = std::min(mat[i * n + j], mat[j * n + i]);
      mat[i * n + j] = d;
      mat[j * n + i] = d;
    }
  }
  return mat;
}

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
  double max_dist_m,
  double min_clearance_cells,
  double snap_radius_m)
{
  ShortestPathResult res;
  res.distance_m = kInfD;
  res.reachable = false;
  if (height <= 0 || width <= 0 || resolution <= 0.0 || !traversable) {
    return res;
  }

  const int n_cells = height * width;
  const int snap_cells = std::max(1, static_cast<int>(std::ceil(snap_radius_m / resolution)));
  const float max_dist_cells = (max_dist_m > 0.0)
    ? static_cast<float>(max_dist_m / resolution)
    : kInfF;

  std::vector<uint8_t> passable;
  fill_passable_mask(traversable, obstacle_dt, n_cells, min_clearance_cells, passable);

  const int sr = world_to_cell_clamped(start_xy.y, origin_y, resolution, height);
  const int sc = world_to_cell_clamped(start_xy.x, origin_x, resolution, width);
  const int gr_in = world_to_cell_clamped(goal_xy.y, origin_y, resolution, height);
  const int gc_in = world_to_cell_clamped(goal_xy.x, origin_x, resolution, width);

  const int s_cell = snap_to_passable(passable.data(), traversable, height, width, sr, sc, snap_cells);
  const int g_cell = snap_to_passable(passable.data(), traversable, height, width, gr_in, gc_in, snap_cells);
  if (s_cell < 0 || g_cell < 0) {
    return res;
  }

  if (s_cell == g_cell) {
    res.distance_m = 0.0;
    res.reachable = true;
    const int r = s_cell / width;
    const int c = s_cell - r * width;
    res.path_xy.push_back({origin_x + (c + 0.5) * resolution, origin_y + (r + 0.5) * resolution});
    return res;
  }

  const int gr = g_cell / width;
  const int gc = g_cell - gr * width;

  passable[static_cast<size_t>(s_cell)] = 1;
  passable[static_cast<size_t>(g_cell)] = 1;

  std::vector<float> g_score(static_cast<size_t>(n_cells), kInfF);
  std::vector<int32_t> parent(static_cast<size_t>(n_cells), -1);
  std::vector<uint8_t> closed(static_cast<size_t>(n_cells), 0);
  std::vector<HeapNode> heap;
  heap.reserve(4096);

  g_score[static_cast<size_t>(s_cell)] = 0.0f;
  const int sr0 = s_cell / width;
  const int sc0 = s_cell - sr0 * width;
  heap.push_back({octile_heuristic(sr0, sc0, gr, gc), s_cell});

  bool found = false;
  while (!heap.empty()) {
    std::pop_heap(heap.begin(), heap.end(), std::greater<HeapNode>{});
    const HeapNode cur = heap.back();
    heap.pop_back();

    if (closed[static_cast<size_t>(cur.idx)]) {
      continue;
    }
    closed[static_cast<size_t>(cur.idx)] = 1;

    const float cg = g_score[static_cast<size_t>(cur.idx)];
    if (cg > max_dist_cells) {
      continue;
    }
    if (cur.idx == g_cell) {
      found = true;
      break;
    }

    const int r = cur.idx / width;
    const int c = cur.idx - r * width;
    for (int dir = 0; dir < 8; ++dir) {
      const int nr = r + kDR[dir];
      const int nc = c + kDC[dir];
      if (!in_bounds(nr, nc, height, width)) {
        continue;
      }
      const int nidx = nr * width + nc;
      if (closed[static_cast<size_t>(nidx)] || !passable[static_cast<size_t>(nidx)]) {
        continue;
      }
      if (dir >= 4) {
        const int adj1 = (r + kDR[dir]) * width + c;
        const int adj2 = r * width + (c + kDC[dir]);
        if (!traversable[adj1] || !traversable[adj2]) {
          continue;
        }
      }
      const float ng = cg + kStepF[dir];
      if (ng > max_dist_cells) {
        continue;
      }
      if (ng < g_score[static_cast<size_t>(nidx)]) {
        g_score[static_cast<size_t>(nidx)] = ng;
        parent[static_cast<size_t>(nidx)] = cur.idx;
        const float f = ng + octile_heuristic(nr, nc, gr, gc);
        heap.push_back({f, nidx});
        std::push_heap(heap.begin(), heap.end(), std::greater<HeapNode>{});
      }
    }
  }

  if (!found) {
    return res;
  }

  res.reachable = true;
  res.distance_m = static_cast<double>(g_score[static_cast<size_t>(g_cell)]) * resolution;

  std::vector<int32_t> chain;
  for (int32_t cur = g_cell; cur >= 0; cur = parent[static_cast<size_t>(cur)]) {
    chain.push_back(cur);
    if (cur == s_cell) {
      break;
    }
  }
  std::reverse(chain.begin(), chain.end());

  res.path_xy.reserve(chain.size());
  for (int32_t cid : chain) {
    const int r = cid / width;
    const int c = cid - r * width;
    res.path_xy.push_back({
      origin_x + (static_cast<double>(c) + 0.5) * resolution,
      origin_y + (static_cast<double>(r) + 0.5) * resolution
    });
  }
  return res;
}

}  // namespace geom_kernel
