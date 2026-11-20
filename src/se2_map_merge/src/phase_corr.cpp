#include "se2_map_merge/phase_corr.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <vector>

namespace se2_map_merge
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

int next_pow2(int v)
{
  int p = 1;
  while (p < v) {
    p <<= 1;
  }
  return p;
}

void fft1d(std::vector<std::complex<double>> & a, bool inverse)
{
  const size_t n = a.size();
  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      std::swap(a[i], a[j]);
    }
  }

  for (size_t len = 2; len <= n; len <<= 1) {
    const double ang = 2.0 * kPi / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
    const std::complex<double> wlen(std::cos(ang), std::sin(ang));
    for (size_t i = 0; i < n; i += len) {
      std::complex<double> w(1.0, 0.0);
      const size_t half = len >> 1;
      for (size_t j = 0; j < half; ++j) {
        const auto u = a[i + j];
        const auto v = a[i + j + half] * w;
        a[i + j] = u + v;
        a[i + j + half] = u - v;
        w *= wlen;
      }
    }
  }

  if (inverse) {
    const double inv_n = 1.0 / static_cast<double>(n);
    for (auto & x : a) {
      x *= inv_n;
    }
  }
}

void fft2d(std::vector<std::complex<double>> & grid, int rows, int cols, bool inverse)
{
  std::vector<std::complex<double>> row_buf(static_cast<size_t>(cols));
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      row_buf[static_cast<size_t>(c)] = grid[static_cast<size_t>(r) * cols + c];
    }
    fft1d(row_buf, inverse);
    for (int c = 0; c < cols; ++c) {
      grid[static_cast<size_t>(r) * cols + c] = row_buf[static_cast<size_t>(c)];
    }
  }

  std::vector<std::complex<double>> col_buf(static_cast<size_t>(rows));
  for (int c = 0; c < cols; ++c) {
    for (int r = 0; r < rows; ++r) {
      col_buf[static_cast<size_t>(r)] = grid[static_cast<size_t>(r) * cols + c];
    }
    fft1d(col_buf, inverse);
    for (int r = 0; r < rows; ++r) {
      grid[static_cast<size_t>(r) * cols + c] = col_buf[static_cast<size_t>(r)];
    }
  }
}

void blur3x3(std::vector<double> & img, int rows, int cols)
{
  std::vector<double> tmp(img.size(), 0.0);
  for (int r = 1; r < rows - 1; ++r) {
    for (int c = 1; c < cols - 1; ++c) {
      const double sum =
        img[(r - 1) * cols + (c - 1)] + 2.0 * img[(r - 1) * cols + c] + img[(r - 1) * cols + (c + 1)] +
        2.0 * img[r * cols + (c - 1)] + 4.0 * img[r * cols + c] + 2.0 * img[r * cols + (c + 1)] +
        img[(r + 1) * cols + (c - 1)] + 2.0 * img[(r + 1) * cols + c] + img[(r + 1) * cols + (c + 1)];
      tmp[r * cols + c] = sum * (1.0 / 16.0);
    }
  }
  img.swap(tmp);
}

void grid_world_bounds(
  const OccupancyGrid2D & g, const Pose2D & pose,
  double & min_x, double & min_y, double & max_x, double & max_y)
{
  const double w = static_cast<double>(g.width) * g.resolution;
  const double h = static_cast<double>(g.height) * g.resolution;
  const Point2D corners[4] = {
    {g.origin_x, g.origin_y},
    {g.origin_x + w, g.origin_y},
    {g.origin_x, g.origin_y + h},
    {g.origin_x + w, g.origin_y + h}
  };
  min_x = std::numeric_limits<double>::infinity();
  min_y = std::numeric_limits<double>::infinity();
  max_x = -std::numeric_limits<double>::infinity();
  max_y = -std::numeric_limits<double>::infinity();
  for (const auto & p : corners) {
    const Point2D wp = apply_se2(pose, p);
    min_x = std::min(min_x, wp.x);
    min_y = std::min(min_y, wp.y);
    max_x = std::max(max_x, wp.x);
    max_y = std::max(max_y, wp.y);
  }
}

}  // namespace

PhaseCorrResult estimate_coarse_translation(
  const OccupancyGrid2D & ref,
  const OccupancyGrid2D & src,
  const Pose2D & init_pose,
  const PhaseCorrParams & params)
{
  PhaseCorrResult out;
  if (ref.empty() || src.empty()) {
    return out;
  }

  double r_min_x, r_min_y, r_max_x, r_max_y;
  double s_min_x, s_min_y, s_max_x, s_max_y;
  grid_world_bounds(ref, Pose2D{}, r_min_x, r_min_y, r_max_x, r_max_y);
  grid_world_bounds(src, init_pose, s_min_x, s_min_y, s_max_x, s_max_y);

  const double min_x = std::max(r_min_x, s_min_x) - params.overlap_margin_m;
  const double min_y = std::max(r_min_y, s_min_y) - params.overlap_margin_m;
  const double max_x = std::min(r_max_x, s_max_x) + params.overlap_margin_m;
  const double max_y = std::min(r_max_y, s_max_y) + params.overlap_margin_m;
  if (max_x <= min_x || max_y <= min_y) {
    return out;
  }

  const double span_x = max_x - min_x;
  const double span_y = max_y - min_y;
  const double base_res = std::max(ref.resolution, src.resolution);
  const double max_span = std::max(span_x, span_y);
  const int cap = std::max(32, next_pow2(params.max_fft_dim));
  const double cell_res = std::max(base_res, max_span / static_cast<double>(cap));

  const int raw_cols = std::max(16, static_cast<int>(std::ceil(span_x / cell_res)));
  const int raw_rows = std::max(16, static_cast<int>(std::ceil(span_y / cell_res)));
  const int cols = std::min(cap, next_pow2(raw_cols));
  const int rows = std::min(cap, next_pow2(raw_rows));

  std::vector<double> img_ref(static_cast<size_t>(rows) * cols, 0.0);
  std::vector<double> img_src(static_cast<size_t>(rows) * cols, 0.0);

  int ref_count = 0;
  for (int r = 0; r < ref.height; ++r) {
    for (int c = 0; c < ref.width; ++c) {
      if (ref.at(r, c) <= params.occupied_threshold) {
        continue;
      }
      const Point2D wp = ref.cell_to_world(r, c);
      const int ic = static_cast<int>(std::floor((wp.x - min_x) / cell_res));
      const int ir = static_cast<int>(std::floor((wp.y - min_y) / cell_res));
      if (ic >= 0 && ic < cols && ir >= 0 && ir < rows) {
        if (img_ref[static_cast<size_t>(ir) * cols + ic] == 0.0) {
          ++ref_count;
        }
        img_ref[static_cast<size_t>(ir) * cols + ic] = 1.0;
      }
    }
  }

  int src_count = 0;
  for (int r = 0; r < src.height; ++r) {
    for (int c = 0; c < src.width; ++c) {
      if (src.at(r, c) <= params.occupied_threshold) {
        continue;
      }
      const Point2D wp = apply_se2(init_pose, src.cell_to_world(r, c));
      const int ic = static_cast<int>(std::floor((wp.x - min_x) / cell_res));
      const int ir = static_cast<int>(std::floor((wp.y - min_y) / cell_res));
      if (ic >= 0 && ic < cols && ir >= 0 && ir < rows) {
        if (img_src[static_cast<size_t>(ir) * cols + ic] == 0.0) {
          ++src_count;
        }
        img_src[static_cast<size_t>(ir) * cols + ic] = 1.0;
      }
    }
  }

  out.overlap_ref_cells = ref_count;
  out.overlap_src_cells = src_count;
  if (ref_count < params.min_occupied_cells || src_count < params.min_occupied_cells) {
    return out;
  }

  // Light 3x3 smoothing makes phase correlation robust to 5-10 deg yaw offset
  blur3x3(img_ref, rows, cols);
  blur3x3(img_src, rows, cols);

  // Separable Hanning window to suppress wrap-around edge artifacts
  std::vector<double> win_r(static_cast<size_t>(rows));
  std::vector<double> win_c(static_cast<size_t>(cols));
  for (int r = 0; r < rows; ++r) {
    win_r[static_cast<size_t>(r)] =
      0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(r) / static_cast<double>(rows - 1)));
  }
  for (int c = 0; c < cols; ++c) {
    win_c[static_cast<size_t>(c)] =
      0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(c) / static_cast<double>(cols - 1)));
  }

  const size_t total = static_cast<size_t>(rows) * static_cast<size_t>(cols);
  std::vector<std::complex<double>> f_ref(total);
  std::vector<std::complex<double>> f_src(total);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const size_t idx = static_cast<size_t>(r) * cols + c;
      const double w = win_r[static_cast<size_t>(r)] * win_c[static_cast<size_t>(c)];
      f_ref[idx] = std::complex<double>(img_ref[idx] * w, 0.0);
      f_src[idx] = std::complex<double>(img_src[idx] * w, 0.0);
    }
  }

  fft2d(f_ref, rows, cols, false);
  fft2d(f_src, rows, cols, false);

  std::vector<std::complex<double>> cps(total);
  for (size_t i = 0; i < total; ++i) {
    const auto prod = f_ref[i] * std::conj(f_src[i]);
    const double mag = std::abs(prod);
    cps[i] = (mag > 1e-9) ? (prod / mag) : std::complex<double>(0.0, 0.0);
  }
  fft2d(cps, rows, cols, true);

  // Locate peak within legal max_shift_m window
  const int max_shift_cells = static_cast<int>(std::ceil(params.max_shift_m / cell_res));
  int best_r = 0;
  int best_c = 0;
  double best_val = -1.0;
  for (int r = 0; r < rows; ++r) {
    const int sr = (r <= rows / 2) ? r : (r - rows);
    if (std::abs(sr) > max_shift_cells) {
      continue;
    }
    for (int c = 0; c < cols; ++c) {
      const int sc = (c <= cols / 2) ? c : (c - cols);
      if (std::abs(sc) > max_shift_cells) {
        continue;
      }
      const double val = cps[static_cast<size_t>(r) * cols + c].real();
      if (val > best_val) {
        best_val = val;
        best_r = r;
        best_c = c;
      }
    }
  }

  // Parabolic subpixel refinement
  auto at_wrap = [&](int r, int c) -> double {
    const int rr = (r % rows + rows) % rows;
    const int cc = (c % cols + cols) % cols;
    return cps[static_cast<size_t>(rr) * cols + cc].real();
  };

  auto subpixel = [](double ym, double y0, double yp) -> double {
    const double denom = ym - 2.0 * y0 + yp;
    if (std::abs(denom) < 1e-12) {
      return 0.0;
    }
    return std::clamp(0.5 * (ym - yp) / denom, -0.5, 0.5);
  };

  const double sub_r = subpixel(at_wrap(best_r - 1, best_c), best_val, at_wrap(best_r + 1, best_c));
  const double sub_c = subpixel(at_wrap(best_r, best_c - 1), best_val, at_wrap(best_r, best_c + 1));

  double shift_r = (best_r <= rows / 2) ? static_cast<double>(best_r) : static_cast<double>(best_r - rows);
  double shift_c = (best_c <= cols / 2) ? static_cast<double>(best_c) : static_cast<double>(best_c - cols);
  shift_r += sub_r;
  shift_c += sub_c;

  // Response: sum of 3x3 energy around peak (matches OpenCV phaseCorrelate convention)
  double peak_mass = 0.0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      peak_mass += std::max(0.0, at_wrap(best_r + dr, best_c + dc));
    }
  }

  out.dx = shift_c * cell_res;
  out.dy = shift_r * cell_res;
  out.response = peak_mass;
  out.valid = (
    peak_mass >= params.min_response &&
    std::hypot(out.dx, out.dy) <= params.max_shift_m
  );
  return out;
}

}  // namespace se2_map_merge
