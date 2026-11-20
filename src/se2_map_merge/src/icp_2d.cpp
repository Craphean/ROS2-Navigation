#include "se2_map_merge/icp_2d.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace se2_map_merge
{
namespace
{

// ---------------------------------------------------------------------------
// Compact balanced 2D KD-Tree
// ---------------------------------------------------------------------------

struct KdNode
{
  Point2D pt{};
  int idx{-1};
  int axis{0};
  int left{-1};
  int right{-1};
};

class KdTree2D
{
public:
  explicit KdTree2D(const std::vector<Point2D> & points)
  {
    if (points.empty()) {
      return;
    }
    nodes_.reserve(points.size());
    std::vector<int> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);
    root_ = build(points, indices, 0, static_cast<int>(indices.size()), 0);
  }

  bool empty() const { return root_ < 0; }

  int nearest(const Point2D & query, double & best_dist2) const
  {
    best_dist2 = std::numeric_limits<double>::infinity();
    int best_idx = -1;
    if (root_ >= 0) {
      search_nearest(root_, query, best_idx, best_dist2);
    }
    return best_idx;
  }

  void k_nearest(
    const Point2D & query, int k,
    std::vector<std::pair<double, int>> & results) const
  {
    results.clear();
    if (root_ < 0 || k <= 0) {
      return;
    }
    search_knn(root_, query, static_cast<size_t>(k), results);
  }

private:
  int build(
    const std::vector<Point2D> & points,
    std::vector<int> & indices, int begin, int end, int depth)
  {
    if (begin >= end) {
      return -1;
    }
    const int axis = depth & 1;
    const int mid = begin + (end - begin) / 2;
    std::nth_element(
      indices.begin() + begin,
      indices.begin() + mid,
      indices.begin() + end,
      [&](int a, int b) {
        return axis == 0 ? (points[a].x < points[b].x) : (points[a].y < points[b].y);
      });

    const int node_id = static_cast<int>(nodes_.size());
    nodes_.push_back({points[indices[mid]], indices[mid], axis, -1, -1});
    const int left = build(points, indices, begin, mid, depth + 1);
    const int right = build(points, indices, mid + 1, end, depth + 1);
    nodes_[static_cast<size_t>(node_id)].left = left;
    nodes_[static_cast<size_t>(node_id)].right = right;
    return node_id;
  }

  void search_nearest(int node_id, const Point2D & q, int & best_idx, double & best_d2) const
  {
    const KdNode & n = nodes_[static_cast<size_t>(node_id)];
    const double dx = q.x - n.pt.x;
    const double dy = q.y - n.pt.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = n.idx;
    }
    const double diff = (n.axis == 0) ? dx : dy;
    const int first = diff < 0.0 ? n.left : n.right;
    const int second = diff < 0.0 ? n.right : n.left;
    if (first >= 0) {
      search_nearest(first, q, best_idx, best_d2);
    }
    if (second >= 0 && diff * diff < best_d2) {
      search_nearest(second, q, best_idx, best_d2);
    }
  }

  void search_knn(
    int node_id, const Point2D & q, size_t k,
    std::vector<std::pair<double, int>> & heap) const
  {
    const KdNode & n = nodes_[static_cast<size_t>(node_id)];
    const double dx = q.x - n.pt.x;
    const double dy = q.y - n.pt.y;
    const double d2 = dx * dx + dy * dy;
    if (heap.size() < k) {
      heap.push_back({d2, n.idx});
      std::push_heap(heap.begin(), heap.end());
    } else if (d2 < heap.front().first) {
      std::pop_heap(heap.begin(), heap.end());
      heap.back() = {d2, n.idx};
      std::push_heap(heap.begin(), heap.end());
    }

    const double diff = (n.axis == 0) ? dx : dy;
    const int first = diff < 0.0 ? n.left : n.right;
    const int second = diff < 0.0 ? n.right : n.left;
    if (first >= 0) {
      search_knn(first, q, k, heap);
    }
    if (second >= 0 && (heap.size() < k || diff * diff < heap.front().first)) {
      search_knn(second, q, k, heap);
    }
  }

  std::vector<KdNode> nodes_;
  int root_{-1};
};

// ---------------------------------------------------------------------------
// Exact 3x3 symmetric Jacobi eigendecomposition & 2x2 translational eigenvalues
// ---------------------------------------------------------------------------

struct Eigen3
{
  std::array<double, 3> values{};
  // Columns v[col][row] are orthonormal eigenvectors
  std::array<std::array<double, 3>, 3> vectors{};
};

Eigen3 jacobi_eigen_3x3(const std::array<std::array<double, 3>, 3> & A_in)
{
  auto A = A_in;
  std::array<std::array<double, 3>, 3> V = {{
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  }};

  for (int iter = 0; iter < 50; ++iter) {
    int p = 0, q = 1;
    double max_off = std::abs(A[0][1]);
    if (std::abs(A[0][2]) > max_off) { p = 0; q = 2; max_off = std::abs(A[0][2]); }
    if (std::abs(A[1][2]) > max_off) { p = 1; q = 2; max_off = std::abs(A[1][2]); }
    if (max_off < 1e-13) {
      break;
    }

    const double app = A[p][p];
    const double aqq = A[q][q];
    const double apq = A[p][q];
    const double tau = (aqq - app) / (2.0 * apq);
    const double t = (tau >= 0.0)
      ? (1.0 / (tau + std::sqrt(1.0 + tau * tau)))
      : (-1.0 / (-tau + std::sqrt(1.0 + tau * tau)));
    const double c = 1.0 / std::sqrt(1.0 + t * t);
    const double s = t * c;

    A[p][p] = app - t * apq;
    A[q][q] = aqq + t * apq;
    A[p][q] = A[q][p] = 0.0;
    for (int r = 0; r < 3; ++r) {
      if (r != p && r != q) {
        const double arp = A[r][p];
        const double arq = A[r][q];
        A[r][p] = A[p][r] = c * arp - s * arq;
        A[r][q] = A[q][r] = s * arp + c * arq;
      }
      const double vrp = V[r][p];
      const double vrq = V[r][q];
      V[r][p] = c * vrp - s * vrq;
      V[r][q] = s * vrp + c * vrq;
    }
  }

  Eigen3 out;
  for (int col = 0; col < 3; ++col) {
    out.values[col] = A[col][col];
    out.vectors[col] = {V[0][col], V[1][col], V[2][col]};
  }
  return out;
}

struct TransEigen2
{
  double eig_min{0.0};
  double eig_max{0.0};
  double cond{1.0};
  Point2D vec_max{1.0, 0.0};  // well-constrained direction
  Point2D vec_min{0.0, 1.0};  // weakest / potentially degenerate direction
};

TransEigen2 eigen_2x2_sym(double h00, double h01, double h11)
{
  TransEigen2 out;
  const double tr = h00 + h11;
  const double diff = h00 - h11;
  const double disc = std::sqrt(diff * diff * 0.25 + h01 * h01);
  out.eig_max = 0.5 * tr + disc;
  out.eig_min = std::max(0.0, 0.5 * tr - disc);
  out.cond = (out.eig_min > 1e-12) ? (out.eig_max / out.eig_min) : 1e12;

  if (std::abs(h01) > 1e-12) {
    const double vx = out.eig_max - h11;
    const double vy = h01;
    const double norm = std::hypot(vx, vy);
    out.vec_max = {vx / norm, vy / norm};
  } else if (h00 >= h11) {
    out.vec_max = {1.0, 0.0};
  } else {
    out.vec_max = {0.0, 1.0};
  }
  out.vec_min = {-out.vec_max.y, out.vec_max.x};
  return out;
}

std::vector<Point2D> extract_occupied_points(
  const OccupancyGrid2D & grid, int threshold, int max_points)
{
  std::vector<Point2D> pts;
  for (int r = 0; r < grid.height; ++r) {
    for (int c = 0; c < grid.width; ++c) {
      if (grid.at(r, c) > threshold) {
        pts.push_back(grid.cell_to_world(r, c));
      }
    }
  }
  if (max_points > 0 && static_cast<int>(pts.size()) > max_points) {
    std::vector<Point2D> sub;
    sub.reserve(static_cast<size_t>(max_points));
    const double stride = static_cast<double>(pts.size()) / static_cast<double>(max_points);
    for (int i = 0; i < max_points; ++i) {
      sub.push_back(pts[static_cast<size_t>(i * stride)]);
    }
    return sub;
  }
  return pts;
}

struct Normal2D
{
  double nx{0.0};
  double ny{0.0};
  bool planar{false};  // true when local points lie along a clean wall segment
};

std::vector<Normal2D> estimate_normals(
  const std::vector<Point2D> & pts, const KdTree2D & tree, double max_radius)
{
  std::vector<Normal2D> normals(pts.size());
  std::vector<std::pair<double, int>> knn;
  const double r2 = max_radius * max_radius;
  for (size_t i = 0; i < pts.size(); ++i) {
    tree.k_nearest(pts[i], 7, knn);
    double mx = 0.0, my = 0.0;
    int cnt = 0;
    for (const auto & item : knn) {
      if (item.first <= r2) {
        mx += pts[static_cast<size_t>(item.second)].x;
        my += pts[static_cast<size_t>(item.second)].y;
        ++cnt;
      }
    }
    if (cnt < 3) {
      continue;
    }
    mx /= cnt;
    my /= cnt;
    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (const auto & item : knn) {
      if (item.first <= r2) {
        const double dx = pts[static_cast<size_t>(item.second)].x - mx;
        const double dy = pts[static_cast<size_t>(item.second)].y - my;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
      }
    }
    const auto eig = eigen_2x2_sym(cxx, cxy, cyy);
    if (eig.eig_max > 1e-6 && (eig.eig_min < 1e-9 || eig.eig_max / eig.eig_min >= 3.0)) {
      normals[i] = {eig.vec_min.x, eig.vec_min.y, true};
    }
  }
  return normals;
}

}  // namespace

double compute_overlap_occupied_iou(
  const OccupancyGrid2D & ref,
  const OccupancyGrid2D & src,
  const Pose2D & pose,
  int occupied_threshold)
{
  if (ref.empty() || src.empty()) {
    return 0.0;
  }

  // Build ref occupied mask (and 1-cell dilated mask) in ref's own cell grid
  const size_t total = static_cast<size_t>(ref.width) * static_cast<size_t>(ref.height);
  std::vector<uint8_t> ref_occ(total, 0);
  std::vector<uint8_t> ref_dil(total, 0);
  for (int r = 0; r < ref.height; ++r) {
    for (int c = 0; c < ref.width; ++c) {
      if (ref.at(r, c) > occupied_threshold) {
        ref_occ[static_cast<size_t>(r) * ref.width + c] = 1;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            const int nr = r + dr, nc = c + dc;
            if (nr >= 0 && nr < ref.height && nc >= 0 && nc < ref.width) {
              ref_dil[static_cast<size_t>(nr) * ref.width + nc] = 1;
            }
          }
        }
      }
    }
  }

  std::vector<uint8_t> src_occ(total, 0);
  std::vector<uint8_t> src_dil(total, 0);
  std::vector<uint8_t> overlap_mask(total, 0);

  // Mark cells in ref that fall within src's known bounding box
  const Pose2D inv_pose = inverse_se2(pose);
  for (int r = 0; r < ref.height; ++r) {
    for (int c = 0; c < ref.width; ++c) {
      const Point2D sp = apply_se2(inv_pose, ref.cell_to_world(r, c));
      const int sc = static_cast<int>(std::floor((sp.x - src.origin_x) / src.resolution));
      const int sr = static_cast<int>(std::floor((sp.y - src.origin_y) / src.resolution));
      if (sc >= 0 && sc < src.width && sr >= 0 && sr < src.height) {
        const int8_t sv = src.at(sr, sc);
        if (sv >= 0) {
          const size_t idx = static_cast<size_t>(r) * ref.width + c;
          overlap_mask[idx] = 1;
          if (sv > occupied_threshold) {
            src_occ[idx] = 1;
            for (int dr = -1; dr <= 1; ++dr) {
              for (int dc = -1; dc <= 1; ++dc) {
                const int nr = r + dr, nc = c + dc;
                if (nr >= 0 && nr < ref.height && nc >= 0 && nc < ref.width) {
                  src_dil[static_cast<size_t>(nr) * ref.width + nc] = 1;
                }
              }
            }
          }
        }
      }
    }
  }

  int inter = 0;
  int uni = 0;
  for (size_t i = 0; i < total; ++i) {
    if (!overlap_mask[i]) {
      continue;
    }
    const bool r_o = ref_occ[i] != 0;
    const bool s_o = src_occ[i] != 0;
    if (r_o || s_o) {
      ++uni;
      if ((r_o && src_dil[i]) || (s_o && ref_dil[i])) {
        ++inter;
      }
    }
  }
  return uni > 0 ? static_cast<double>(inter) / static_cast<double>(uni) : 0.0;
}

RegistrationDiagnostics align_occupancy_grids_se2(
  const OccupancyGrid2D & ref,
  const OccupancyGrid2D & src,
  const Pose2D & current_pose,
  const Se2AlignParams & params)
{
  const auto t_start = std::chrono::steady_clock::now();
  RegistrationDiagnostics diag;
  diag.smoothed_pose = current_pose;

  if (ref.empty() || src.empty()) {
    diag.reject_reason = "empty_grid";
    return diag;
  }

  const auto ref_pts = extract_occupied_points(ref, params.occupied_threshold, params.max_icp_points * 2);
  const auto src_pts = extract_occupied_points(src, params.occupied_threshold, params.max_icp_points);
  diag.overlap_ref_points = static_cast<int>(ref_pts.size());
  diag.overlap_src_points = static_cast<int>(src_pts.size());

  if (static_cast<int>(ref_pts.size()) < params.min_overlap_points ||
      static_cast<int>(src_pts.size()) < params.min_overlap_points)
  {
    diag.reject_reason = "insufficient_occupied_points";
    return diag;
  }

  diag.iou_before = compute_overlap_occupied_iou(ref, src, current_pose, params.occupied_threshold);

  // Stage 1: Coarse translation via Hanning-windowed 2D FFT phase correlation
  Pose2D working_pose = current_pose;
  if (params.enable_phase_corr) {
    const auto pc = estimate_coarse_translation(ref, src, current_pose, params.phase_corr);
    diag.phase_corr_response = pc.response;
    if (pc.valid) {
      const Pose2D coarse_candidate{
        current_pose.x + pc.dx,
        current_pose.y + pc.dy,
        current_pose.yaw
      };
      // Only adopt coarse shift if it does not degrade overlap IoU significantly
      const double coarse_iou =
        compute_overlap_occupied_iou(ref, src, coarse_candidate, params.occupied_threshold);
      if (coarse_iou >= diag.iou_before - 0.02) {
        working_pose = coarse_candidate;
        diag.coarse_delta = {pc.dx, pc.dy, 0.0};
        diag.phase_corr_used = true;
      }
    }
  }

  // Build KD-Tree & local surface normals on reference occupied edge points
  KdTree2D ref_tree(ref_pts);
  const auto ref_normals = estimate_normals(ref_pts, ref_tree, std::max(0.35, 5.0 * ref.resolution));

  // Stage 2: Point-to-line Huber ICP with eigenvalue degeneracy gating (Solution Remapping)
  const double max_d2 = params.max_correspondence_dist_m * params.max_correspondence_dist_m;
  TransEigen2 last_trans_eig;
  bool ever_degenerate_trans = false;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    diag.icp_iterations = iter + 1;

    struct Corr
    {
      Point2D p_w;
      Point2D q_w;
      Normal2D n;
      double dist;
    };
    std::vector<Corr> corrs;
    corrs.reserve(src_pts.size());
    double mean_x = 0.0, mean_y = 0.0;

    for (const auto & p_local : src_pts) {
      const Point2D p_w = apply_se2(working_pose, p_local);
      double d2 = 0.0;
      const int idx = ref_tree.nearest(p_w, d2);
      if (idx >= 0 && d2 <= max_d2) {
        corrs.push_back({p_w, ref_pts[static_cast<size_t>(idx)], ref_normals[static_cast<size_t>(idx)], std::sqrt(d2)});
        mean_x += p_w.x;
        mean_y += p_w.y;
      }
    }

    diag.icp_inliers = static_cast<int>(corrs.size());
    diag.inlier_ratio = static_cast<double>(corrs.size()) / static_cast<double>(src_pts.size());
    if (static_cast<int>(corrs.size()) < params.min_overlap_points) {
      diag.reject_reason = "too_few_icp_inliers";
      return diag;
    }

    mean_x /= static_cast<double>(corrs.size());
    mean_y /= static_cast<double>(corrs.size());

    std::array<std::array<double, 3>, 3> H = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
    std::array<double, 3> b = {0.0, 0.0, 0.0};
    double sq_err_sum = 0.0;

    auto add_obs = [&](double j0, double j1, double j2, double residual, double weight) {
      const double w = weight;
      H[0][0] += w * j0 * j0; H[0][1] += w * j0 * j1; H[0][2] += w * j0 * j2;
      H[1][0] += w * j1 * j0; H[1][1] += w * j1 * j1; H[1][2] += w * j1 * j2;
      H[2][0] += w * j2 * j0; H[2][1] += w * j2 * j1; H[2][2] += w * j2 * j2;
      b[0] -= w * j0 * residual;
      b[1] -= w * j1 * residual;
      b[2] -= w * j2 * residual;
    };

    for (const auto & c : corrs) {
      const double tx = c.p_w.x - mean_x;
      const double ty = c.p_w.y - mean_y;
      const double ex = c.p_w.x - c.q_w.x;
      const double ey = c.p_w.y - c.q_w.y;
      sq_err_sum += ex * ex + ey * ey;

      if (c.n.planar) {
        const double r = c.n.nx * ex + c.n.ny * ey;
        const double abs_r = std::abs(r);
        const double w = (abs_r <= params.huber_delta_m) ? 1.0 : (params.huber_delta_m / abs_r);
        const double j_yaw = -c.n.nx * ty + c.n.ny * tx;
        add_obs(c.n.nx, c.n.ny, j_yaw, r, w);
      } else {
        // Point-to-point anchor for isolated corners / pillars
        const double w_pt = 0.5 * (
          c.dist <= params.huber_delta_m ? 1.0 : (params.huber_delta_m / std::max(c.dist, 1e-9)));
        add_obs(1.0, 0.0, -ty, ex, w_pt);
        add_obs(0.0, 1.0, tx, ey, w_pt);
      }
    }

    diag.icp_rmse_m = std::sqrt(sq_err_sum / static_cast<double>(corrs.size()));

    // Inspect 2x2 translational block for parallel-wall corridor degeneracy
    last_trans_eig = eigen_2x2_sym(H[0][0], H[0][1], H[1][1]);
    diag.eig_min_trans = last_trans_eig.eig_min;
    diag.eig_max_trans = last_trans_eig.eig_max;
    diag.cond_trans = last_trans_eig.cond;

    const bool is_deg =
      (last_trans_eig.eig_min < params.min_trans_eigenvalue) ||
      (last_trans_eig.cond > params.max_trans_cond_number);
    if (is_deg) {
      ever_degenerate_trans = true;
    }

    // Solve via 3x3 symmetric Jacobi eigendecomposition + Solution Remapping
    const Eigen3 eig3 = jacobi_eigen_3x3(H);
    std::array<double, 3> delta = {0.0, 0.0, 0.0};
    for (int m = 0; m < 3; ++m) {
      const double lam = eig3.values[m];
      const auto & v = eig3.vectors[m];
      if (lam <= 1e-9) {
        continue;
      }
      const double trans_frac = v[0] * v[0] + v[1] * v[1];
      bool keep = true;
      if (params.enable_degeneracy_gate) {
        if (trans_frac >= 0.5) {
          if (lam < params.min_trans_eigenvalue ||
              (last_trans_eig.eig_max / lam) > params.max_trans_cond_number)
          {
            keep = false;
          }
        } else {
          if (lam < params.min_rot_eigenvalue) {
            keep = false;
          }
        }
      }
      if (!keep) {
        continue;
      }
      const double coeff = (v[0] * b[0] + v[1] * b[1] + v[2] * b[2]) / lam;
      delta[0] += coeff * v[0];
      delta[1] += coeff * v[1];
      delta[2] += coeff * v[2];
    }

    // Convert centroid-centered perturbation to world-frame left SE(2) increment
    const double dtheta = std::clamp(delta[2], -0.25, 0.25);
    const double c = std::cos(dtheta);
    const double s = std::sin(dtheta);
    const Pose2D inc{
      mean_x - (c * mean_x - s * mean_y) + delta[0],
      mean_y - (s * mean_x + c * mean_y) + delta[1],
      dtheta
    };
    working_pose = compose_se2(inc, working_pose);

    if (std::hypot(delta[0], delta[1]) < params.convergence_trans_m &&
        std::abs(dtheta) < params.convergence_rot_rad)
    {
      break;
    }
  }

  diag.degenerate_trans = ever_degenerate_trans;

  // If translational degeneracy fired (e.g., straight mine corridor), project
  // total translational delta onto the well-constrained eigenvector so neither
  // coarse phase-corr nor numerical noise can inject longitudinal slip.
  if (params.enable_degeneracy_gate && ever_degenerate_trans) {
    const double total_dx = working_pose.x - current_pose.x;
    const double total_dy = working_pose.y - current_pose.y;
    const double proj = total_dx * last_trans_eig.vec_max.x + total_dy * last_trans_eig.vec_max.y;
    working_pose.x = current_pose.x + proj * last_trans_eig.vec_max.x;
    working_pose.y = current_pose.y + proj * last_trans_eig.vec_max.y;
  }

  diag.refined_delta = {
    working_pose.x - current_pose.x,
    working_pose.y - current_pose.y,
    wrap_angle(working_pose.yaw - current_pose.yaw)
  };

  // Stage 3: Safety, RMSE, and monotonic overlap IoU consistency gates
  if (std::hypot(diag.refined_delta.x, diag.refined_delta.y) > params.max_total_delta_xy_m ||
      std::abs(diag.refined_delta.yaw) > params.max_total_delta_yaw_rad)
  {
    diag.reject_reason = "delta_exceeds_bounds";
  } else if (diag.inlier_ratio < params.min_inlier_ratio) {
    diag.reject_reason = "low_inlier_ratio";
  } else if (diag.icp_rmse_m > params.max_rmse_m) {
    diag.reject_reason = "high_icp_rmse";
  } else {
    diag.iou_after = compute_overlap_occupied_iou(ref, src, working_pose, params.occupied_threshold);
    if (diag.iou_after + params.iou_tolerance < diag.iou_before) {
      diag.reject_reason = "iou_degraded";
    } else {
      diag.accepted = true;
      diag.smoothed_pose = smooth_se2(
        current_pose, working_pose,
        params.smoothing_alpha, params.max_step_xy_m, params.max_step_yaw_rad);
    }
  }

  const auto t_end = std::chrono::steady_clock::now();
  diag.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  return diag;
}

OccupancyGrid2D merge_occupancy_grids(
  const std::vector<std::pair<const OccupancyGrid2D *, Pose2D>> & grids,
  double output_resolution)
{
  OccupancyGrid2D out;
  out.resolution = output_resolution;
  if (grids.empty()) {
    return out;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  std::vector<Pose2D> inv_poses;
  inv_poses.reserve(grids.size());
  for (const auto & item : grids) {
    const auto * g = item.first;
    if (!g || g->empty()) {
      inv_poses.push_back(Pose2D{});
      continue;
    }
    inv_poses.push_back(inverse_se2(item.second));
    const double w = static_cast<double>(g->width) * g->resolution;
    const double h = static_cast<double>(g->height) * g->resolution;
    const Point2D corners[4] = {
      {g->origin_x, g->origin_y},
      {g->origin_x + w, g->origin_y},
      {g->origin_x, g->origin_y + h},
      {g->origin_x + w, g->origin_y + h}
    };
    for (const auto & p : corners) {
      const Point2D wp = apply_se2(item.second, p);
      min_x = std::min(min_x, wp.x);
      min_y = std::min(min_y, wp.y);
      max_x = std::max(max_x, wp.x);
      max_y = std::max(max_y, wp.y);
    }
  }

  if (!std::isfinite(min_x) || max_x <= min_x || max_y <= min_y) {
    return out;
  }

  out.origin_x = min_x;
  out.origin_y = min_y;
  out.width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / output_resolution)));
  out.height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / output_resolution)));
  out.data.assign(static_cast<size_t>(out.width) * static_cast<size_t>(out.height), -1);

  // Inverse nearest-neighbor sampling prevents Moire holes on rotated maps.
  // Priority: occupied (>50) > free (0..50) > unknown (-1).
  for (int r = 0; r < out.height; ++r) {
    const double wy = out.origin_y + (static_cast<double>(r) + 0.5) * output_resolution;
    for (int c = 0; c < out.width; ++c) {
      const double wx = out.origin_x + (static_cast<double>(c) + 0.5) * output_resolution;
      int8_t fused = -1;
      for (size_t k = 0; k < grids.size(); ++k) {
        const auto * g = grids[k].first;
        if (!g || g->empty()) {
          continue;
        }
        const Point2D lp = apply_se2(inv_poses[k], {wx, wy});
        const int lc = static_cast<int>(std::floor((lp.x - g->origin_x) / g->resolution));
        const int lr = static_cast<int>(std::floor((lp.y - g->origin_y) / g->resolution));
        if (lc >= 0 && lc < g->width && lr >= 0 && lr < g->height) {
          const int8_t val = g->at(lr, lc);
          if (val >= 0 && val > fused) {
            fused = val;
          }
        }
      }
      out.data[static_cast<size_t>(r) * static_cast<size_t>(out.width) + static_cast<size_t>(c)] =
        fused;
    }
  }
  return out;
}

}  // namespace se2_map_merge
