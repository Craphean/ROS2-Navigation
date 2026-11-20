#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#endif
#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif

#include "se2_map_merge/icp_2d.hpp"
#include "se2_map_merge/types.hpp"

namespace se2_map_merge
{
namespace
{

std::string strip_slashes(std::string s)
{
  while (!s.empty() && s.front() == '/') {
    s.erase(0, 1);
  }
  while (!s.empty() && s.back() == '/') {
    s.pop_back();
  }
  return s;
}

std::string prefix_frame(const std::string & ns, const std::string & frame)
{
  const std::string clean = strip_slashes(frame);
  if (ns.empty() || clean.rfind(ns + "/", 0) == 0) {
    return clean;
  }
  return ns + "/" + clean;
}

OccupancyGrid2D from_ros_grid(const nav_msgs::msg::OccupancyGrid & msg)
{
  OccupancyGrid2D g;
  g.width = static_cast<int>(msg.info.width);
  g.height = static_cast<int>(msg.info.height);
  g.resolution = static_cast<double>(msg.info.resolution);
  g.origin_x = msg.info.origin.position.x;
  g.origin_y = msg.info.origin.position.y;
  g.data = msg.data;
  return g;
}

nav_msgs::msg::OccupancyGrid to_ros_grid(
  const OccupancyGrid2D & g, const std::string & frame_id, const rclcpp::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.info.map_load_time = stamp;
  msg.info.resolution = static_cast<float>(g.resolution);
  msg.info.width = static_cast<uint32_t>(g.width);
  msg.info.height = static_cast<uint32_t>(g.height);
  msg.info.origin.position.x = g.origin_x;
  msg.info.origin.position.y = g.origin_y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data = g.data;
  return msg;
}

geometry_msgs::msg::TransformStamped make_tf_stamped(
  const std::string & parent, const std::string & child,
  const Pose2D & pose, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = pose.x;
  tf.transform.translation.y = pose.y;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = std::sin(0.5 * pose.yaw);
  tf.transform.rotation.w = std::cos(0.5 * pose.yaw);
  return tf;
}

}  // namespace

class Se2MapMergeNode : public rclcpp::Node
{
public:
  Se2MapMergeNode()
  : Node("se2_map_merge")
  {
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", true);
    }
    this->declare_parameter<std::vector<std::string>>(
      "robot_namespaces", std::vector<std::string>{"robot1", "robot2"});
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<std::string>("merged_map_topic", "/map");
    this->declare_parameter<std::string>("robot_map_topic", "map");
    this->declare_parameter<double>("merging_rate", 1.5);
    this->declare_parameter<double>("alignment_rate", 0.5);
    this->declare_parameter<double>("tf_broadcast_rate", 20.0);
    this->declare_parameter<double>("tf_future_offset", 0.05);
    this->declare_parameter<double>("output_resolution", 0.05);
    this->declare_parameter<bool>("enable_online_alignment", true);
    this->declare_parameter<bool>("enable_phase_corr", true);
    this->declare_parameter<bool>("enable_degeneracy_gate", true);
    this->declare_parameter<bool>("publish_per_robot_shared_map", false);
    this->declare_parameter<double>("smoothing_alpha", 0.25);
    this->declare_parameter<double>("max_step_xy_m", 0.05);
    this->declare_parameter<double>("max_step_yaw_rad", 0.0174532925);
    this->declare_parameter<double>("min_trans_eigenvalue", 12.0);
    this->declare_parameter<double>("max_trans_cond_number", 25.0);
    this->declare_parameter<double>("max_correspondence_dist_m", 0.80);

    world_frame_ = this->get_parameter("world_frame").as_string();
    merged_map_topic_ = this->get_parameter("merged_map_topic").as_string();
    robot_map_topic_ = this->get_parameter("robot_map_topic").as_string();
    output_resolution_ = this->get_parameter("output_resolution").as_double();
    tf_future_offset_ = std::max(0.0, this->get_parameter("tf_future_offset").as_double());
    enable_online_alignment_ = this->get_parameter("enable_online_alignment").as_bool();
    publish_shared_maps_ = this->get_parameter("publish_per_robot_shared_map").as_bool();

    align_params_.enable_phase_corr = this->get_parameter("enable_phase_corr").as_bool();
    align_params_.enable_degeneracy_gate = this->get_parameter("enable_degeneracy_gate").as_bool();
    align_params_.smoothing_alpha = this->get_parameter("smoothing_alpha").as_double();
    align_params_.max_step_xy_m = this->get_parameter("max_step_xy_m").as_double();
    align_params_.max_step_yaw_rad = this->get_parameter("max_step_yaw_rad").as_double();
    align_params_.min_trans_eigenvalue = this->get_parameter("min_trans_eigenvalue").as_double();
    align_params_.max_trans_cond_number = this->get_parameter("max_trans_cond_number").as_double();
    align_params_.max_correspondence_dist_m =
      this->get_parameter("max_correspondence_dist_m").as_double();

    auto raw_ns = this->get_parameter("robot_namespaces").as_string_array();
    if (raw_ns.empty()) {
      raw_ns = {"robot1", "robot2"};
    }

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    if (world_frame_ != "map") {
      static_tf_broadcaster_->sendTransform(
        make_tf_stamped(world_frame_, "map", Pose2D{}, this->now()));
    }

    const auto map_pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    merged_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      merged_map_topic_, map_pub_qos);
    diag_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/map_merge/alignment_diagnostics", 10);

    const auto map_sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    const auto tf_static_qos = rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable();

    for (const auto & item : raw_ns) {
      const std::string ns = strip_slashes(item);
      if (ns.empty()) {
        continue;
      }
      const std::string pose_param = ns + "_init_pose";
      if (!this->has_parameter(pose_param)) {
        this->declare_parameter<std::vector<double>>(pose_param, {0.0, 0.0, 0.0});
      }
      auto init_vec = this->get_parameter(pose_param).as_double_array();
      Pose2D init_pose{};
      if (init_vec.size() >= 2) {
        init_pose.x = init_vec[0];
        init_pose.y = init_vec[1];
      }
      if (init_vec.size() >= 3) {
        init_pose.yaw = init_vec[2];
      }

      RobotSlot slot;
      slot.ns = ns;
      slot.pose = init_pose;
      robots_.push_back(std::move(slot));
    }

    // Create subscriptions after vector storage is finalized
    for (size_t i = 0; i < robots_.size(); ++i) {
      auto & slot = robots_[i];
      const std::string ns = slot.ns;

      const std::string map_topic = "/" + ns + "/" + strip_slashes(robot_map_topic_);
      slot.map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, map_sub_qos,
        [this, i](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          robots_[i].grid = from_ros_grid(*msg);
          robots_[i].has_grid = true;
        });

      // Relay per-robot namespaced /<ns>/tf and /<ns>/tf_static into global /tf
      slot.tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/" + ns + "/tf", rclcpp::QoS(rclcpp::KeepLast(100)),
        [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          if (!dynamic_tf_broadcaster_ || msg->transforms.empty()) {
            return;
          }
          std::vector<geometry_msgs::msg::TransformStamped> relayed;
          relayed.reserve(msg->transforms.size());
          for (auto tf : msg->transforms) {
            tf.header.frame_id = prefix_frame(ns, tf.header.frame_id);
            tf.child_frame_id = prefix_frame(ns, tf.child_frame_id);
            relayed.push_back(std::move(tf));
          }
          dynamic_tf_broadcaster_->sendTransform(relayed);
        });

      slot.tf_static_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/" + ns + "/tf_static", tf_static_qos,
        [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          if (!static_tf_broadcaster_ || msg->transforms.empty()) {
            return;
          }
          std::vector<geometry_msgs::msg::TransformStamped> relayed;
          relayed.reserve(msg->transforms.size());
          for (auto tf : msg->transforms) {
            tf.header.frame_id = prefix_frame(ns, tf.header.frame_id);
            tf.child_frame_id = prefix_frame(ns, tf.child_frame_id);
            relayed.push_back(std::move(tf));
          }
          static_tf_broadcaster_->sendTransform(relayed);
        });

      if (publish_shared_maps_) {
        slot.shared_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/" + ns + "/shared_map", map_pub_qos);
      }

      RCLCPP_INFO(
        this->get_logger(), "Registered %s: map=%s init=(%.2f, %.2f, %.2f)",
        ns.c_str(), map_topic.c_str(), slot.pose.x, slot.pose.y, slot.pose.yaw);
    }

    const double merging_rate = std::max(0.2, this->get_parameter("merging_rate").as_double());
    const double align_rate = std::max(0.1, this->get_parameter("alignment_rate").as_double());
    const double tf_rate = std::max(1.0, this->get_parameter("tf_broadcast_rate").as_double());

    merge_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(1000.0 / merging_rate)),
      [this]() { merge_and_publish(); });
    align_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(1000.0 / align_rate)),
      [this]() { run_online_alignment(); });
    tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(1000.0 / tf_rate)),
      [this]() { broadcast_map_transforms(); });
  }

private:
  struct RobotSlot
  {
    std::string ns;
    Pose2D pose{};
    OccupancyGrid2D grid{};
    bool has_grid{false};
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr shared_map_pub;
  };

  void broadcast_map_transforms()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!dynamic_tf_broadcaster_) {
      return;
    }
    rclcpp::Time stamp = this->now();
    if (stamp.nanoseconds() > 0 && tf_future_offset_ > 0.0) {
      stamp = stamp + rclcpp::Duration::from_seconds(tf_future_offset_);
    }
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.reserve(robots_.size());
    for (const auto & slot : robots_) {
      tfs.push_back(make_tf_stamped(world_frame_, slot.ns + "/map", slot.pose, stamp));
    }
    dynamic_tf_broadcaster_->sendTransform(tfs);
  }

  void run_online_alignment()
  {
    if (!enable_online_alignment_) {
      return;
    }

    struct Snapshot
    {
      size_t idx;
      std::string ns;
      OccupancyGrid2D grid;
      Pose2D pose;
      bool has_grid;
    };

    std::vector<Snapshot> snaps;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (robots_.size() < 2 || !robots_[0].has_grid) {
        return;
      }
      snaps.reserve(robots_.size());
      for (size_t i = 0; i < robots_.size(); ++i) {
        snaps.push_back(
          {i, robots_[i].ns, robots_[i].grid, robots_[i].pose, robots_[i].has_grid});
      }
    }

    // Progressive DAG alignment anchored at robot[0]: robot[i] aligns against the
    // world-frame fusion of robots[0..i-1], avoiding circular drift feedback while
    // supporting chains of N=2..4+ robots.
    for (size_t i = 1; i < snaps.size(); ++i) {
      if (!snaps[i].has_grid || snaps[i].grid.empty()) {
        continue;
      }
      std::vector<std::pair<const OccupancyGrid2D *, Pose2D>> ref_inputs;
      ref_inputs.reserve(i);
      for (size_t j = 0; j < i; ++j) {
        if (snaps[j].has_grid && !snaps[j].grid.empty()) {
          ref_inputs.emplace_back(&snaps[j].grid, snaps[j].pose);
        }
      }
      if (ref_inputs.empty()) {
        continue;
      }
      const OccupancyGrid2D ref_grid = merge_occupancy_grids(ref_inputs, output_resolution_);
      if (ref_grid.empty()) {
        continue;
      }

      const auto diag = align_occupancy_grids_se2(
        ref_grid, snaps[i].grid, snaps[i].pose, align_params_);
      if (diag.accepted) {
        snaps[i].pose = diag.smoothed_pose;
        std::lock_guard<std::mutex> lock(mutex_);
        robots_[snaps[i].idx].pose = diag.smoothed_pose;
      }

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4)
          << "{\"robot\":\"" << snaps[i].ns << "\""
          << ",\"accepted\":" << (diag.accepted ? "true" : "false")
          << ",\"reject_reason\":\"" << diag.reject_reason << "\""
          << ",\"phase_corr_used\":" << (diag.phase_corr_used ? "true" : "false")
          << ",\"phase_corr_response\":" << diag.phase_corr_response
          << ",\"degenerate_trans\":" << (diag.degenerate_trans ? "true" : "false")
          << ",\"eig_min\":" << diag.eig_min_trans
          << ",\"eig_max\":" << diag.eig_max_trans
          << ",\"cond\":" << diag.cond_trans
          << ",\"inliers\":" << diag.icp_inliers
          << ",\"inlier_ratio\":" << diag.inlier_ratio
          << ",\"rmse_m\":" << diag.icp_rmse_m
          << ",\"iou_before\":" << diag.iou_before
          << ",\"iou_after\":" << diag.iou_after
          << ",\"pose\":[" << diag.smoothed_pose.x << "," << diag.smoothed_pose.y
          << "," << diag.smoothed_pose.yaw << "]"
          << ",\"elapsed_ms\":" << diag.elapsed_ms << "}";
      std_msgs::msg::String msg;
      msg.data = oss.str();
      diag_pub_->publish(msg);

      if (diag.accepted) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "[SE2:%s] accepted (deg=%s, iou %.3f->%.3f, rmse=%.3fm, %.1fms) pose=(%.3f,%.3f,%.3frad)",
          snaps[i].ns.c_str(), diag.degenerate_trans ? "yes" : "no",
          diag.iou_before, diag.iou_after, diag.icp_rmse_m, diag.elapsed_ms,
          diag.smoothed_pose.x, diag.smoothed_pose.y, diag.smoothed_pose.yaw);
      }
    }
  }

  void merge_and_publish()
  {
    std::vector<std::pair<OccupancyGrid2D, Pose2D>> copies;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & slot : robots_) {
        if (slot.has_grid && !slot.grid.empty()) {
          copies.emplace_back(slot.grid, slot.pose);
        }
      }
    }
    if (copies.empty()) {
      return;
    }

    std::vector<std::pair<const OccupancyGrid2D *, Pose2D>> refs;
    refs.reserve(copies.size());
    for (const auto & item : copies) {
      refs.emplace_back(&item.first, item.second);
    }

    const auto merged = merge_occupancy_grids(refs, output_resolution_);
    if (merged.empty()) {
      return;
    }

    const auto stamp = this->now();
    merged_map_pub_->publish(to_ros_grid(merged, world_frame_, stamp));

    // Optional per-robot local-frame reprojected shared map for TODO #12 fallback
    if (publish_shared_maps_) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & slot : robots_) {
        if (!slot.shared_map_pub) {
          continue;
        }
        const Pose2D inv_pose = inverse_se2(slot.pose);
        const auto local_shared = merge_occupancy_grids({{&merged, inv_pose}}, output_resolution_);
        if (!local_shared.empty()) {
          slot.shared_map_pub->publish(to_ros_grid(local_shared, "map", stamp));
        }
      }
    }
  }

  std::mutex mutex_;
  std::string world_frame_;
  std::string merged_map_topic_;
  std::string robot_map_topic_;
  double output_resolution_{0.05};
  double tf_future_offset_{0.05};
  bool enable_online_alignment_{true};
  bool publish_shared_maps_{false};
  Se2AlignParams align_params_{};

  std::vector<RobotSlot> robots_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr merge_timer_;
  rclcpp::TimerBase::SharedPtr align_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace se2_map_merge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<se2_map_merge::Se2MapMergeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
