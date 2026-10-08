#ifndef RL_NAV2_CONTROLLER__RL_CONTROLLER_HPP_
#define RL_NAV2_CONTROLLER__RL_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rl_nav2_controller/td3_actor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rl_nav2_controller
{

class RlController : public nav2_core::Controller
{
public:
  RlController() = default;
  ~RlController() override = default;

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name,
    nav2::TransformBuffer::SharedPtr tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker,
    const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;

  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

protected:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  std::vector<float> binScanLocked() const;
  geometry_msgs::msg::PoseStamped selectLookahead(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & plan) const;
  void publishCarrot(const geometry_msgs::msg::PoseStamped & carrot);

  nav2::LifecycleNode::WeakPtr node_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("RlController")};
  rclcpp::Clock::SharedPtr clock_;

  Td3Actor actor_;
  nav2::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  nav2::Publisher<visualization_msgs::msg::Marker>::SharedPtr carrot_pub_;

  mutable std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_scan_{false};
  bool logged_scan_meta_{false};

  double lookahead_dist_{1.5};
  double free_range_fill_{7.0};
  double vx_max_{0.26};
  double wz_max_{1.0};
  int state_bins_{20};
  std::string scan_topic_{"scan"};
  std::string weights_file_;

  double speed_limit_ratio_{1.0};
  double last_vx_{0.0};
  double last_wz_{0.0};
};

}  // namespace rl_nav2_controller

#endif  // RL_NAV2_CONTROLLER__RL_CONTROLLER_HPP_
