#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "geom_kernel/corridor_graph.hpp"
#include "geom_kernel/geodesic_grid.hpp"
#include "geom_kernel/visibility_fov.hpp"

namespace py = pybind11;

namespace
{

std::vector<geom_kernel::Point2D> array_to_points2d(
  const py::array_t<double, py::array::c_style | py::array::forcecast> & arr)
{
  auto buf = arr.request();
  if (buf.ndim == 1 && buf.shape[0] == 0) {
    return {};
  }
  if (buf.ndim != 2 || buf.shape[1] < 2) {
    throw std::invalid_argument("points array must have shape (N, 2)");
  }
  const auto n = static_cast<size_t>(buf.shape[0]);
  const auto stride = static_cast<size_t>(buf.shape[1]);
  const auto * ptr = static_cast<const double *>(buf.ptr);
  std::vector<geom_kernel::Point2D> pts;
  pts.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    pts.push_back({ptr[i * stride], ptr[i * stride + 1]});
  }
  return pts;
}

py::array_t<double> py_geodesic_distances(
  const py::array_t<uint8_t, py::array::c_style | py::array::forcecast> & traversable,
  const py::array_t<float, py::array::c_style | py::array::forcecast> & obstacle_dt,
  double resolution,
  double origin_x,
  double origin_y,
  const py::array_t<double, py::array::c_style | py::array::forcecast> & sources_xy,
  const py::array_t<double, py::array::c_style | py::array::forcecast> & targets_xy,
  double max_dist_m,
  double min_clearance_cells,
  double snap_radius_m)
{
  auto t_buf = traversable.request();
  if (t_buf.ndim != 2) {
    throw std::invalid_argument("traversable must be a 2D uint8 array");
  }
  const int h = static_cast<int>(t_buf.shape[0]);
  const int w = static_cast<int>(t_buf.shape[1]);

  const float * dt_ptr = nullptr;
  auto dt_buf = obstacle_dt.request();
  if (dt_buf.size > 0) {
    if (dt_buf.ndim != 2 || dt_buf.shape[0] != h || dt_buf.shape[1] != w) {
      throw std::invalid_argument("obstacle_dt must match traversable shape");
    }
    dt_ptr = static_cast<const float *>(dt_buf.ptr);
  }

  auto srcs = array_to_points2d(sources_xy);
  auto tgts = array_to_points2d(targets_xy);

  std::vector<double> dists;
  {
    py::gil_scoped_release release;
    dists = geom_kernel::compute_geodesic_distances(
      static_cast<const uint8_t *>(t_buf.ptr), dt_ptr,
      h, w, resolution, origin_x, origin_y,
      srcs, tgts, max_dist_m, min_clearance_cells, snap_radius_m);
  }

  py::array_t<double> out({static_cast<py::ssize_t>(srcs.size()), static_cast<py::ssize_t>(tgts.size())});
  if (!dists.empty()) {
    std::copy(dists.begin(), dists.end(), out.mutable_data());
  }
  return out;
}

py::array_t<double> py_geodesic_matrix(
  const py::array_t<uint8_t, py::array::c_style | py::array::forcecast> & traversable,
  const py::array_t<float, py::array::c_style | py::array::forcecast> & obstacle_dt,
  double resolution,
  double origin_x,
  double origin_y,
  const py::array_t<double, py::array::c_style | py::array::forcecast> & points_xy,
  double max_dist_m,
  double min_clearance_cells,
  double snap_radius_m)
{
  auto t_buf = traversable.request();
  if (t_buf.ndim != 2) {
    throw std::invalid_argument("traversable must be a 2D uint8 array");
  }
  const int h = static_cast<int>(t_buf.shape[0]);
  const int w = static_cast<int>(t_buf.shape[1]);

  const float * dt_ptr = nullptr;
  auto dt_buf = obstacle_dt.request();
  if (dt_buf.size > 0) {
    if (dt_buf.ndim != 2 || dt_buf.shape[0] != h || dt_buf.shape[1] != w) {
      throw std::invalid_argument("obstacle_dt must match traversable shape");
    }
    dt_ptr = static_cast<const float *>(dt_buf.ptr);
  }

  auto pts = array_to_points2d(points_xy);
  std::vector<double> mat;
  {
    py::gil_scoped_release release;
    mat = geom_kernel::compute_geodesic_matrix(
      static_cast<const uint8_t *>(t_buf.ptr), dt_ptr,
      h, w, resolution, origin_x, origin_y,
      pts, max_dist_m, min_clearance_cells, snap_radius_m);
  }

  py::array_t<double> out({static_cast<py::ssize_t>(pts.size()), static_cast<py::ssize_t>(pts.size())});
  if (!mat.empty()) {
    std::copy(mat.begin(), mat.end(), out.mutable_data());
  }
  return out;
}

py::tuple py_geodesic_path(
  const py::array_t<uint8_t, py::array::c_style | py::array::forcecast> & traversable,
  const py::array_t<float, py::array::c_style | py::array::forcecast> & obstacle_dt,
  double resolution,
  double origin_x,
  double origin_y,
  double start_x,
  double start_y,
  double goal_x,
  double goal_y,
  double max_dist_m,
  double min_clearance_cells,
  double snap_radius_m)
{
  auto t_buf = traversable.request();
  if (t_buf.ndim != 2) {
    throw std::invalid_argument("traversable must be a 2D uint8 array");
  }
  const int h = static_cast<int>(t_buf.shape[0]);
  const int w = static_cast<int>(t_buf.shape[1]);

  const float * dt_ptr = nullptr;
  auto dt_buf = obstacle_dt.request();
  if (dt_buf.size > 0) {
    if (dt_buf.ndim != 2 || dt_buf.shape[0] != h || dt_buf.shape[1] != w) {
      throw std::invalid_argument("obstacle_dt must match traversable shape");
    }
    dt_ptr = static_cast<const float *>(dt_buf.ptr);
  }

  geom_kernel::ShortestPathResult res;
  {
    py::gil_scoped_release release;
    res = geom_kernel::compute_geodesic_path(
      static_cast<const uint8_t *>(t_buf.ptr), dt_ptr,
      h, w, resolution, origin_x, origin_y,
      {start_x, start_y}, {goal_x, goal_y},
      max_dist_m, min_clearance_cells, snap_radius_m);
  }

  py::array_t<double> path_arr({static_cast<py::ssize_t>(res.path_xy.size()), static_cast<py::ssize_t>(2)});
  auto * dst = path_arr.mutable_data();
  for (size_t i = 0; i < res.path_xy.size(); ++i) {
    dst[2 * i] = res.path_xy[i].x;
    dst[2 * i + 1] = res.path_xy[i].y;
  }
  return py::make_tuple(res.distance_m, path_arr);
}

bool py_has_line_of_sight(
  const py::array_t<int8_t, py::array::c_style | py::array::forcecast> & occ_data,
  int r0,
  int c0,
  int r1,
  int c1,
  int8_t lethal_threshold)
{
  auto buf = occ_data.request();
  if (buf.ndim != 2) {
    throw std::invalid_argument("occ_data must be a 2D int8 array");
  }
  return geom_kernel::has_line_of_sight(
    static_cast<const int8_t *>(buf.ptr),
    static_cast<int>(buf.shape[0]),
    static_cast<int>(buf.shape[1]),
    r0, c0, r1, c1, lethal_threshold);
}

py::list py_optimize_residual_viewpoints(
  const py::array_t<int8_t, py::array::c_style | py::array::forcecast> & occ_data,
  const py::array_t<float, py::array::c_style | py::array::forcecast> & obstacle_dt,
  double resolution,
  double origin_x,
  double origin_y,
  const py::array_t<double, py::array::c_style | py::array::forcecast> & columns_xyzz,
  const py::array_t<int32_t, py::array::c_style | py::array::forcecast> & cluster_ids,
  double r_min,
  double r_max,
  int num_samples,
  double min_clearance,
  double sensor_z,
  double vfov_min_rad,
  double vfov_max_rad,
  double max_eff_range,
  int max_viewpoints,
  double dedupe_radius,
  int8_t lethal_threshold)
{
  auto occ_buf = occ_data.request();
  if (occ_buf.ndim != 2) {
    throw std::invalid_argument("occ_data must be a 2D int8 array");
  }
  const int h = static_cast<int>(occ_buf.shape[0]);
  const int w = static_cast<int>(occ_buf.shape[1]);

  const float * dt_ptr = nullptr;
  auto dt_buf = obstacle_dt.request();
  if (dt_buf.size > 0) {
    if (dt_buf.ndim != 2 || dt_buf.shape[0] != h || dt_buf.shape[1] != w) {
      throw std::invalid_argument("obstacle_dt must match occ_data shape");
    }
    dt_ptr = static_cast<const float *>(dt_buf.ptr);
  }

  auto col_buf = columns_xyzz.request();
  auto cid_buf = cluster_ids.request();
  if (col_buf.ndim != 2 || col_buf.shape[1] < 4) {
    throw std::invalid_argument("columns_xyzz must have shape (C, 4) [x, y, z_min, z_max]");
  }
  if (cid_buf.ndim != 1 || cid_buf.shape[0] != col_buf.shape[0]) {
    throw std::invalid_argument("cluster_ids must have shape (C,)");
  }

  const auto n = static_cast<size_t>(col_buf.shape[0]);
  const auto * cptr = static_cast<const double *>(col_buf.ptr);
  const auto * idptr = static_cast<const int32_t *>(cid_buf.ptr);
  std::vector<geom_kernel::ResidualColumn> cols;
  cols.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    cols.push_back({
      cptr[4 * i + 0],
      cptr[4 * i + 1],
      cptr[4 * i + 2],
      cptr[4 * i + 3],
      idptr[i]
    });
  }

  geom_kernel::VisibilityParams vp;
  vp.r_min = r_min;
  vp.r_max = r_max;
  vp.num_samples = num_samples;
  vp.min_clearance = min_clearance;
  vp.sensor_z = sensor_z;
  vp.vfov_min_rad = vfov_min_rad;
  vp.vfov_max_rad = vfov_max_rad;
  vp.max_eff_range = max_eff_range;
  vp.max_viewpoints = max_viewpoints;
  vp.dedupe_radius = dedupe_radius;
  vp.lethal_threshold = lethal_threshold;

  std::vector<geom_kernel::ViewpointCandidate> cands;
  {
    py::gil_scoped_release release;
    cands = geom_kernel::optimize_residual_viewpoints(
      static_cast<const int8_t *>(occ_buf.ptr), dt_ptr,
      h, w, resolution, origin_x, origin_y, cols, vp);
  }

  py::list out;
  for (const auto & c : cands) {
    py::dict d;
    d["x"] = c.x;
    d["y"] = c.y;
    d["yaw"] = c.yaw;
    d["clearance_m"] = c.clearance_m;
    d["cluster_id"] = c.cluster_id;
    d["covered_count"] = c.covered_count;
    d["marginal_gain"] = c.marginal_gain;
    d["coverage_ratio"] = c.coverage_ratio;
    out.append(std::move(d));
  }
  return out;
}

py::dict py_extract_corridor_bottlenecks(
  const py::array_t<uint8_t, py::array::c_style | py::array::forcecast> & is_free,
  const py::array_t<float, py::array::c_style | py::array::forcecast> & obstacle_dt,
  double resolution,
  double origin_x,
  double origin_y,
  double robot_radius,
  double narrow_margin_m,
  double min_passable_margin_m,
  int min_segment_cells,
  double yield_search_radius_m,
  double yield_min_separation_m,
  int max_yield_points)
{
  auto f_buf = is_free.request();
  auto dt_buf = obstacle_dt.request();
  if (f_buf.ndim != 2 || dt_buf.ndim != 2 ||
      f_buf.shape[0] != dt_buf.shape[0] || f_buf.shape[1] != dt_buf.shape[1])
  {
    throw std::invalid_argument("is_free and obstacle_dt must be 2D arrays of matching shape");
  }
  const int h = static_cast<int>(f_buf.shape[0]);
  const int w = static_cast<int>(f_buf.shape[1]);

  geom_kernel::CorridorParams cp;
  cp.robot_radius = robot_radius;
  cp.narrow_margin_m = narrow_margin_m;
  cp.min_passable_margin_m = min_passable_margin_m;
  cp.min_segment_cells = min_segment_cells;
  cp.yield_search_radius_m = yield_search_radius_m;
  cp.yield_min_separation_m = yield_min_separation_m;
  cp.max_yield_points = max_yield_points;

  geom_kernel::CorridorGraphResult graph;
  {
    py::gil_scoped_release release;
    graph = geom_kernel::extract_corridor_bottlenecks(
      static_cast<const uint8_t *>(f_buf.ptr),
      static_cast<const float *>(dt_buf.ptr),
      h, w, resolution, origin_x, origin_y, cp);
  }

  py::array_t<int32_t> labels({static_cast<py::ssize_t>(h), static_cast<py::ssize_t>(w)});
  if (!graph.labels.empty()) {
    std::copy(graph.labels.begin(), graph.labels.end(), labels.mutable_data());
  }

  py::list segs;
  for (const auto & s : graph.segments) {
    py::dict sd;
    sd["id"] = s.id;
    sd["cell_count"] = s.cell_count;
    sd["min_half_width_m"] = s.min_half_width_m;
    sd["max_half_width_m"] = s.max_half_width_m;
    sd["length_m"] = s.length_m;
    sd["centroid_x"] = s.centroid_x;
    sd["centroid_y"] = s.centroid_y;
    py::list yps;
    for (const auto & yp : s.yield_points) {
      yps.append(py::make_tuple(yp.x, yp.y, yp.clearance_m));
    }
    sd["yield_points"] = std::move(yps);
    segs.append(std::move(sd));
  }

  py::dict out;
  out["labels"] = std::move(labels);
  out["segments"] = std::move(segs);
  return out;
}

}  // namespace

PYBIND11_MODULE(_geom_kernel, m)
{
  m.doc() = "C++17 geometry kernel for Lite-HMRE: bounded multi-source geodesic Dijkstra, "
            "2D Bresenham LoS + 3D vertical-FoV submodular viewpoint optimizer, "
            "and medial-axis corridor bottleneck / yield-point extractor.";

  m.def(
    "geodesic_distances", &py_geodesic_distances,
    py::arg("traversable"),
    py::arg("obstacle_dt"),
    py::arg("resolution"),
    py::arg("origin_x"),
    py::arg("origin_y"),
    py::arg("sources_xy"),
    py::arg("targets_xy"),
    py::arg("max_dist_m") = 0.0,
    py::arg("min_clearance_cells") = 0.0,
    py::arg("snap_radius_m") = 0.65);

  m.def(
    "geodesic_matrix", &py_geodesic_matrix,
    py::arg("traversable"),
    py::arg("obstacle_dt"),
    py::arg("resolution"),
    py::arg("origin_x"),
    py::arg("origin_y"),
    py::arg("points_xy"),
    py::arg("max_dist_m") = 0.0,
    py::arg("min_clearance_cells") = 0.0,
    py::arg("snap_radius_m") = 0.65);

  m.def(
    "geodesic_path", &py_geodesic_path,
    py::arg("traversable"),
    py::arg("obstacle_dt"),
    py::arg("resolution"),
    py::arg("origin_x"),
    py::arg("origin_y"),
    py::arg("start_x"),
    py::arg("start_y"),
    py::arg("goal_x"),
    py::arg("goal_y"),
    py::arg("max_dist_m") = 0.0,
    py::arg("min_clearance_cells") = 0.0,
    py::arg("snap_radius_m") = 0.65);

  m.def(
    "has_line_of_sight", &py_has_line_of_sight,
    py::arg("occ_data"),
    py::arg("r0"),
    py::arg("c0"),
    py::arg("r1"),
    py::arg("c1"),
    py::arg("lethal_threshold") = static_cast<int8_t>(50));

  m.def(
    "optimize_residual_viewpoints", &py_optimize_residual_viewpoints,
    py::arg("occ_data"),
    py::arg("obstacle_dt"),
    py::arg("resolution"),
    py::arg("origin_x"),
    py::arg("origin_y"),
    py::arg("columns_xyzz"),
    py::arg("cluster_ids"),
    py::arg("r_min") = 0.8,
    py::arg("r_max") = 3.5,
    py::arg("num_samples") = 24,
    py::arg("min_clearance") = 0.55,
    py::arg("sensor_z") = 0.25,
    py::arg("vfov_min_rad") = -0.5235987756,
    py::arg("vfov_max_rad") = 0.9599310886,
    py::arg("max_eff_range") = 5.0,
    py::arg("max_viewpoints") = 4,
    py::arg("dedupe_radius") = 1.5,
    py::arg("lethal_threshold") = static_cast<int8_t>(50));

  m.def(
    "extract_corridor_bottlenecks", &py_extract_corridor_bottlenecks,
    py::arg("is_free"),
    py::arg("obstacle_dt"),
    py::arg("resolution"),
    py::arg("origin_x"),
    py::arg("origin_y"),
    py::arg("robot_radius") = 0.22,
    py::arg("narrow_margin_m") = 0.35,
    py::arg("min_passable_margin_m") = 0.02,
    py::arg("min_segment_cells") = 4,
    py::arg("yield_search_radius_m") = 3.0,
    py::arg("yield_min_separation_m") = 1.0,
    py::arg("max_yield_points") = 4);
}
