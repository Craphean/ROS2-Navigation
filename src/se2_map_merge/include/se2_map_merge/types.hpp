#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace se2_map_merge
{

inline double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Point3D
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct SE3Transform
{
  // Row-major 3x3 rotation + translation
  double r[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double t[3]{0, 0, 0};

  static SE3Transform from_quat_trans(
    double qx, double qy, double qz, double qw,
    double tx, double ty, double tz)
  {
    SE3Transform out;
    const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    const double wx = qw * qx, wy = qw * qy, wz = qw * qz;
    out.r[0] = 1.0 - 2.0 * (yy + zz);
    out.r[1] = 2.0 * (xy - wz);
    out.r[2] = 2.0 * (xz + wy);
    out.r[3] = 2.0 * (xy + wz);
    out.r[4] = 1.0 - 2.0 * (xx + zz);
    out.r[5] = 2.0 * (yz - wx);
    out.r[6] = 2.0 * (xz - wy);
    out.r[7] = 2.0 * (yz + wx);
    out.r[8] = 1.0 - 2.0 * (xx + yy);
    out.t[0] = tx;
    out.t[1] = ty;
    out.t[2] = tz;
    return out;
  }
};

inline Point2D apply_se2(const Pose2D & tf, const Point2D & p)
{
  const double c = std::cos(tf.yaw);
  const double s = std::sin(tf.yaw);
  return {tf.x + c * p.x - s * p.y, tf.y + s * p.x + c * p.y};
}

inline Pose2D compose_se2(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {
    a.x + c * b.x - s * b.y,
    a.y + s * b.x + c * b.y,
    wrap_angle(a.yaw + b.yaw)
  };
}

inline Pose2D inverse_se2(const Pose2D & a)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {
    -(c * a.x + s * a.y),
    s * a.x - c * a.y,
    wrap_angle(-a.yaw)
  };
}

/// Step-clamped SE(2) exponential smoothing to avoid violent jumps in Nav2 StaticLayer.
inline Pose2D smooth_se2(
  const Pose2D & current,
  const Pose2D & target,
  double alpha,
  double max_step_xy = 0.05,
  double max_step_yaw = 0.0174532925)  // 1 deg
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  double dx = alpha * (target.x - current.x);
  double dy = alpha * (target.y - current.y);
  double dyaw = alpha * wrap_angle(target.yaw - current.yaw);

  const double d_norm = std::hypot(dx, dy);
  if (max_step_xy > 0.0 && d_norm > max_step_xy) {
    const double scale = max_step_xy / d_norm;
    dx *= scale;
    dy *= scale;
  }
  if (max_step_yaw > 0.0) {
    dyaw = std::clamp(dyaw, -max_step_yaw, max_step_yaw);
  }
  return {
    current.x + dx,
    current.y + dy,
    wrap_angle(current.yaw + dyaw)
  };
}

/// Pivot-centered step-clamped SE(2) exponential smoothing (eliminates lever-arm shear
/// when rotating maps whose occupied centroid sits far from the local frame origin).
inline Pose2D smooth_se2_about_pivot(
  const Pose2D & current,
  const Pose2D & target,
  const Point2D & pivot_local,
  double alpha,
  double max_step_xy = 0.05,
  double max_step_yaw = 0.0174532925)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  const Point2D c_cur = apply_se2(current, pivot_local);
  const Point2D c_tgt = apply_se2(target, pivot_local);
  double dx = alpha * (c_tgt.x - c_cur.x);
  double dy = alpha * (c_tgt.y - c_cur.y);
  double dyaw = alpha * wrap_angle(target.yaw - current.yaw);

  const double d_norm = std::hypot(dx, dy);
  if (max_step_xy > 0.0 && d_norm > max_step_xy) {
    const double scale = max_step_xy / d_norm;
    dx *= scale;
    dy *= scale;
  }
  if (max_step_yaw > 0.0) {
    dyaw = std::clamp(dyaw, -max_step_yaw, max_step_yaw);
  }
  const double yaw_next = wrap_angle(current.yaw + dyaw);
  const double c = std::cos(yaw_next);
  const double s = std::sin(yaw_next);
  return {
    (c_cur.x + dx) - (c * pivot_local.x - s * pivot_local.y),
    (c_cur.y + dy) - (s * pivot_local.x + c * pivot_local.y),
    yaw_next
  };
}

struct OccupancyGrid2D
{
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;

  bool empty() const { return width <= 0 || height <= 0 || data.empty(); }

  int8_t at(int r, int c) const
  {
    return data[static_cast<size_t>(r) * static_cast<size_t>(width) + static_cast<size_t>(c)];
  }

  Point2D cell_to_world(int r, int c) const
  {
    return {
      origin_x + (static_cast<double>(c) + 0.5) * resolution,
      origin_y + (static_cast<double>(r) + 0.5) * resolution
    };
  }
};

struct RegistrationDiagnostics
{
  bool accepted{false};
  std::string reject_reason;
  Pose2D coarse_delta{};
  Pose2D refined_delta{};
  Pose2D smoothed_pose{};
  double phase_corr_response{0.0};
  bool phase_corr_used{false};
  int overlap_ref_points{0};
  int overlap_src_points{0};
  int icp_iterations{0};
  int icp_inliers{0};
  double inlier_ratio{0.0};
  double icp_rmse_m{0.0};
  double eig_min_trans{0.0};
  double eig_max_trans{0.0};
  double cond_trans{0.0};
  bool degenerate_trans{false};
  double iou_before{0.0};
  double iou_after{0.0};
  double elapsed_ms{0.0};
};

}  // namespace se2_map_merge
