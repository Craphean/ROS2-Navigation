#include "se2_map_merge/cloud_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace se2_map_merge
{

std::vector<Point3D> filter_and_transform_cloud(
  const std::vector<Point3D> & raw_sensor_points,
  const SE3Transform & tf,
  const CloudFilterParams & p)
{
  std::vector<Point3D> out;
  out.reserve(raw_sensor_points.size());

  const float min_r2 = static_cast<float>(p.min_range * p.min_range);
  const float max_r2 = static_cast<float>(p.max_range * p.max_range);
  const float fx0 = static_cast<float>(p.footprint_x_min);
  const float fx1 = static_cast<float>(p.footprint_x_max);
  const float fy0 = static_cast<float>(p.footprint_y_min);
  const float fy1 = static_cast<float>(p.footprint_y_max);
  const float fz1 = static_cast<float>(p.footprint_z_max);
  const float min_z = static_cast<float>(p.min_z);
  const float max_z = static_cast<float>(p.max_z);

  const float r0 = static_cast<float>(tf.r[0]), r1 = static_cast<float>(tf.r[1]), r2 = static_cast<float>(tf.r[2]);
  const float r3 = static_cast<float>(tf.r[3]), r4 = static_cast<float>(tf.r[4]), r5 = static_cast<float>(tf.r[5]);
  const float r6 = static_cast<float>(tf.r[6]), r7 = static_cast<float>(tf.r[7]), r8 = static_cast<float>(tf.r[8]);
  const float tx = static_cast<float>(tf.t[0]), ty = static_cast<float>(tf.t[1]), tz = static_cast<float>(tf.t[2]);

  for (const auto & pt : raw_sensor_points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    const float range2 = pt.x * pt.x + pt.y * pt.y;
    if (range2 < min_r2 || range2 > max_r2) {
      continue;
    }
    // Reject self-reflections on robot chassis / wheels / camera housing
    if (pt.x >= fx0 && pt.x <= fx1 && pt.y >= fy0 && pt.y <= fy1 && pt.z <= fz1) {
      continue;
    }

    const float wx = r0 * pt.x + r1 * pt.y + r2 * pt.z + tx;
    const float wy = r3 * pt.x + r4 * pt.y + r5 * pt.z + ty;
    const float wz = r6 * pt.x + r7 * pt.y + r8 * pt.z + tz;

    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
      continue;
    }
    if (wz < min_z || wz > max_z) {
      continue;
    }
    out.push_back({wx, wy, wz});
  }
  return out;
}

std::vector<Point3D> voxel_downsample_points(
  const std::vector<Point3D> & points,
  double voxel_size)
{
  if (points.empty() || voxel_size <= 0.0) {
    return points;
  }

  const float inv_v = static_cast<float>(1.0 / voxel_size);
  constexpr int64_t kBias = 1LL << 20;
  constexpr int64_t kMaxCoord = (1LL << 21) - 1;

  std::vector<std::pair<uint64_t, uint32_t>> keys;
  keys.reserve(points.size());

  for (uint32_t i = 0; i < static_cast<uint32_t>(points.size()); ++i) {
    const auto & pt = points[i];
    int64_t ix = static_cast<int64_t>(std::floor(pt.x * inv_v)) + kBias;
    int64_t iy = static_cast<int64_t>(std::floor(pt.y * inv_v)) + kBias;
    int64_t iz = static_cast<int64_t>(std::floor(pt.z * inv_v)) + kBias;
    ix = std::clamp<int64_t>(ix, 0, kMaxCoord);
    iy = std::clamp<int64_t>(iy, 0, kMaxCoord);
    iz = std::clamp<int64_t>(iz, 0, kMaxCoord);
    const uint64_t packed =
      (static_cast<uint64_t>(ix) << 42) |
      (static_cast<uint64_t>(iy) << 21) |
      static_cast<uint64_t>(iz);
    keys.emplace_back(packed, i);
  }

  std::sort(
    keys.begin(), keys.end(),
    [](const auto & a, const auto & b) { return a.first < b.first; });

  std::vector<Point3D> out;
  out.reserve(points.size());
  uint64_t prev = UINT64_MAX;
  for (const auto & item : keys) {
    if (item.first != prev) {
      out.push_back(points[item.second]);
      prev = item.first;
    }
  }
  return out;
}

}  // namespace se2_map_merge
