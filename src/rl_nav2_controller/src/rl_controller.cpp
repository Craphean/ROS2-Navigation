#include "rl_nav2_controller/rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include <stdexcept>

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/qos_profiles.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace rl_nav2_controller
{

void RlController::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name,
  nav2::TransformBuffer::SharedPtr tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in RlController::configure");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(1.5));
  nav2::declare_parameter_if_not_declared(
    node, plugin_name_ + ".free_range_fill", rclcpp::ParameterValue(7.0));
  nav2::declare_parameter_if_not_declared(
    node, plugin_name_ + ".vx_max", rclcpp::ParameterValue(0.26));
  nav2::declare_parameter_if_not_declared(
    node, plugin_name_ + ".wz_max", rclcpp::ParameterValue(1.0));
  nav2::declare_parameter_if_not_declared(
    node, plugin_name_ + ".state_bins", rclcpp::ParameterValue(20));
  nav2::declare_parameter_if_not_declared(
    node, plugin_name_ + ".scan_topic", rclcpp::ParameterValue(std::string("scan")));
  nav2::declare_parameter_if_not_declared(
    node, plugin_name_ + ".weights_file", rclcpp::ParameterValue(std::string("")));

  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".free_range_fill", free_range_fill_);
  node->get_parameter(plugin_name_ + ".vx_max", vx_max_);
  node->get_parameter(plugin_name_ + ".wz_max", wz_max_);
  node->get_parameter(plugin_name_ + ".state_bins", state_bins_);
  node->get_parameter(plugin_name_ + ".scan_topic", scan_topic_);
  node->get_parameter(plugin_name_ + ".weights_file", weights_file_);

  if (state_bins_ <= 0 || state_bins_ != Td3Actor::kStateDim - 5) {
    throw std::runtime_error(
            "RlController state_bins must be " +
            std::to_string(Td3Actor::kStateDim - 5) + " for this TD3 actor");
  }

  if (weights_file_.empty()) {
    const auto share = ament_index_cpp::get_package_share_path("rl_nav2_controller");
    weights_file_ = (share / "models" / "td3_actor.bin").string();
  }

  actor_.load(weights_file_);
  RCLCPP_INFO(logger_, "Loaded TD3 actor weights from %s", weights_file_.c_str());

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    std::bind(&RlController::scanCallback, this, std::placeholders::_1),
    nav2::qos::SensorDataQoS());

  carrot_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "rl_carrot", nav2::qos::StandardTopicQoS());

  RCLCPP_INFO(
    logger_,
    "Configured RlController plugin %s (lookahead=%.2f vx_max=%.2f)",
    plugin_name_.c_str(), lookahead_dist_, vx_max_);
}

void RlController::cleanup()
{
  scan_sub_.reset();
  carrot_pub_.reset();
}

void RlController::activate()
{
  if (carrot_pub_) {
    carrot_pub_->on_activate();
  }
}

void RlController::deactivate()
{
  if (carrot_pub_) {
    carrot_pub_->on_deactivate();
  }
}

void RlController::newPathReceived(const nav_msgs::msg::Path & /*raw_global_path*/)
{
  last_vx_ = 0.0;
  last_wz_ = 0.0;
}

void RlController::reset()
{
  last_vx_ = 0.0;
  last_wz_ = 0.0;
}

bool RlController::cancel()
{
  last_vx_ = 0.0;
  last_wz_ = 0.0;
  return true;
}

void RlController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ratio_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else if (vx_max_ > 1e-6) {
    speed_limit_ratio_ = std::clamp(speed_limit / vx_max_, 0.0, 1.0);
  } else {
    speed_limit_ratio_ = 1.0;
  }
}

void RlController::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = *msg;
  has_scan_ = true;

  if (!logged_scan_meta_) {
    RCLCPP_INFO(
      logger_,
      "LaserScan meta: n=%zu angle_min=%.4f angle_max=%.4f range_max=%.2f",
      latest_scan_.ranges.size(), latest_scan_.angle_min, latest_scan_.angle_max,
      latest_scan_.range_max);
    logged_scan_meta_ = true;
  }
}

std::vector<float> RlController::binScanLocked() const
{
  std::vector<float> bins(static_cast<size_t>(state_bins_), static_cast<float>(free_range_fill_));
  if (!has_scan_ || latest_scan_.ranges.empty()) {
    return bins;
  }

  const size_t n = latest_scan_.ranges.size();
  const size_t bin_size = static_cast<size_t>(
    std::ceil(static_cast<double>(n) / static_cast<double>(state_bins_)));

  size_t bin_idx = 0;
  for (size_t i = 0; i < n && bin_idx < bins.size(); i += bin_size, ++bin_idx) {
    float min_v = static_cast<float>(free_range_fill_);
    const size_t end = std::min(n, i + bin_size);
    for (size_t j = i; j < end; ++j) {
      float r = latest_scan_.ranges[j];
      if (!std::isfinite(r) || r >= latest_scan_.range_max) {
        r = static_cast<float>(free_range_fill_);
      } else if (r > free_range_fill_) {
        r = static_cast<float>(free_range_fill_);
      }
      if (r < latest_scan_.range_min) {
        r = latest_scan_.range_min;
      }
      min_v = std::min(min_v, r);
    }
    bins[bin_idx] = min_v;
  }
  return bins;
}

geometry_msgs::msg::PoseStamped RlController::selectLookahead(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & plan) const
{
  geometry_msgs::msg::PoseStamped carrot;
  if (plan.poses.empty()) {
    carrot.header = plan.header;
    return carrot;
  }

  for (const auto & pose : plan.poses) {
    const double dx = pose.pose.position.x - robot_pose.pose.position.x;
    const double dy = pose.pose.position.y - robot_pose.pose.position.y;
    if (std::hypot(dx, dy) >= lookahead_dist_) {
      return pose;
    }
  }
  return plan.poses.back();
}

void RlController::publishCarrot(const geometry_msgs::msg::PoseStamped & carrot)
{
  if (!carrot_pub_ || !carrot_pub_->is_activated()) {
    return;
  }
  visualization_msgs::msg::Marker marker;
  marker.header = carrot.header;
  if (marker.header.frame_id.empty() && costmap_ros_) {
    marker.header.frame_id = costmap_ros_->getBaseFrameID();
  }
  marker.header.stamp = clock_->now();
  marker.ns = "rl_carrot";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = carrot.pose;
  marker.pose.position.z = 0.15;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.a = 1.0;
  marker.color.r = 0.1;
  marker.color.g = 0.9;
  marker.color.b = 0.2;
  carrot_pub_->publish(marker);
}

geometry_msgs::msg::TwistStamped RlController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/,
  const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & /*global_goal*/)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in RlController::computeVelocityCommands");
  }

  if (transformed_global_plan.poses.empty()) {
    throw std::runtime_error("RlController received empty transformed global plan");
  }

  std::vector<float> bins;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!has_scan_) {
      throw std::runtime_error("RlController has not received LaserScan yet");
    }
    bins = binScanLocked();
  }

  // PathHandler provides the plan in the local costmap frame (odom). Convert the
  // lookahead point into the robot base frame before building the TD3 state.
  const auto carrot = selectLookahead(pose, transformed_global_plan);
  publishCarrot(carrot);

  const double dx_map = carrot.pose.position.x - pose.pose.position.x;
  const double dy_map = carrot.pose.position.y - pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  const double dx = cos_y * dx_map + sin_y * dy_map;
  const double dy = -sin_y * dx_map + cos_y * dy_map;
  const double distance = std::hypot(dx, dy);
  double cos_yaw = 1.0;
  double sin_yaw = 0.0;
  if (distance > 1e-6) {
    cos_yaw = dx / distance;
    sin_yaw = dy / distance;
  }

  std::array<float, Td3Actor::kStateDim> state{};
  for (int i = 0; i < state_bins_; ++i) {
    state[static_cast<size_t>(i)] = bins[static_cast<size_t>(i)];
  }
  state[static_cast<size_t>(state_bins_)] = static_cast<float>(distance);
  state[static_cast<size_t>(state_bins_ + 1)] = static_cast<float>(cos_yaw);
  state[static_cast<size_t>(state_bins_ + 2)] = static_cast<float>(sin_yaw);
  state[static_cast<size_t>(state_bins_ + 3)] = static_cast<float>(last_vx_);
  state[static_cast<size_t>(state_bins_ + 4)] = static_cast<float>(last_wz_);

  const auto action = actor_.act(state);
  double vx = (static_cast<double>(action[0]) + 1.0) / 2.0;
  double wz = static_cast<double>(action[1]);

  vx = std::clamp(vx, 0.0, vx_max_ * speed_limit_ratio_);
  wz = std::clamp(wz, -wz_max_, wz_max_);

  last_vx_ = vx;
  last_wz_ = wz;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = costmap_ros_ ? costmap_ros_->getBaseFrameID() : "base_link";
  cmd.twist.linear.x = vx;
  cmd.twist.angular.z = wz;
  return cmd;
}

}  // namespace rl_nav2_controller

PLUGINLIB_EXPORT_CLASS(rl_nav2_controller::RlController, nav2_core::Controller)
