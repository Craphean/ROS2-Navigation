#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#if __has_include(<tf2/exceptions.hpp>)
#include <tf2/exceptions.hpp>
#else
#include <tf2/exceptions.h>
#endif
#if __has_include(<tf2/time.hpp>)
#include <tf2/time.hpp>
#else
#include <tf2/time.h>
#endif
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif
#if __has_include(<tf2_ros/transform_listener.hpp>)
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/transform_listener.h>
#endif

#include "se2_map_merge/cloud_filter.hpp"
#include "se2_map_merge/types.hpp"

namespace se2_map_merge
{
namespace
{

std::string strip_leading_slash(std::string s)
{
  while (!s.empty() && s.front() == '/') {
    s.erase(0, 1);
  }
  return s;
}

bool extract_xyz_float32(
  const sensor_msgs::msg::PointCloud2 & msg,
  std::vector<Point3D> & out)
{
  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto & f : msg.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }
    if (f.name == "x") {
      off_x = static_cast<int>(f.offset);
    } else if (f.name == "y") {
      off_y = static_cast<int>(f.offset);
    } else if (f.name == "z") {
      off_z = static_cast<int>(f.offset);
    }
  }
  if (off_x < 0 || off_y < 0 || off_z < 0 || msg.point_step < 12) {
    return false;
  }

  const size_t count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  const size_t step = static_cast<size_t>(msg.point_step);
  if (count == 0 || msg.data.size() < count * step) {
    return false;
  }

  out.resize(count);
  const uint8_t * base = msg.data.data();
  for (size_t i = 0; i < count; ++i) {
    const uint8_t * row = base + i * step;
    std::memcpy(&out[i].x, row + off_x, sizeof(float));
    std::memcpy(&out[i].y, row + off_y, sizeof(float));
    std::memcpy(&out[i].z, row + off_z, sizeof(float));
  }
  return true;
}

sensor_msgs::msg::PointCloud2 make_xyz32_cloud(
  const std::vector<Point3D> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(pts.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 12;
  msg.row_step = msg.point_step * msg.width;

  msg.fields.resize(3);
  const char * names[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<uint32_t>(i * 4);
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }

  msg.data.resize(static_cast<size_t>(msg.row_step));
  if (!pts.empty()) {
    static_assert(sizeof(Point3D) == 12, "Point3D must be tightly packed 12 bytes");
    std::memcpy(msg.data.data(), pts.data(), pts.size() * sizeof(Point3D));
  }
  return msg;
}

}  // namespace

class CloudMergerNode : public rclcpp::Node
{
public:
  CloudMergerNode()
  : Node("cloud_merger_node")
  {
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", true);
    }
    this->declare_parameter<std::vector<std::string>>(
      "robots", std::vector<std::string>{"robot1", "robot2"});
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<std::string>("output_topic", "/points_merged");
    this->declare_parameter<double>("voxel_size", 0.02);
    this->declare_parameter<double>("publish_rate", 2.0);
    this->declare_parameter<double>("min_z", 0.05);
    this->declare_parameter<double>("max_z", 3.0);
    this->declare_parameter<double>("min_range", 0.10);
    this->declare_parameter<double>("max_range", 12.0);
    this->declare_parameter<double>("tf_timeout", 0.05);
    this->declare_parameter<double>("max_tf_skew", 0.15);
    this->declare_parameter<double>("footprint_x_min", -0.22);
    this->declare_parameter<double>("footprint_x_max", 0.22);
    this->declare_parameter<double>("footprint_y_min", -0.18);
    this->declare_parameter<double>("footprint_y_max", 0.18);
    this->declare_parameter<double>("footprint_z_max", 0.05);

    robots_ = this->get_parameter("robots").as_string_array();
    world_frame_ = this->get_parameter("world_frame").as_string();
    const std::string output_topic = this->get_parameter("output_topic").as_string();
    tf_timeout_ = this->get_parameter("tf_timeout").as_double();
    max_tf_skew_ = this->get_parameter("max_tf_skew").as_double();

    params_.voxel_size = this->get_parameter("voxel_size").as_double();
    params_.min_z = this->get_parameter("min_z").as_double();
    params_.max_z = this->get_parameter("max_z").as_double();
    params_.min_range = this->get_parameter("min_range").as_double();
    params_.max_range = this->get_parameter("max_range").as_double();
    params_.footprint_x_min = this->get_parameter("footprint_x_min").as_double();
    params_.footprint_x_max = this->get_parameter("footprint_x_max").as_double();
    params_.footprint_y_min = this->get_parameter("footprint_y_min").as_double();
    params_.footprint_y_max = this->get_parameter("footprint_y_max").as_double();
    params_.footprint_z_max = this->get_parameter("footprint_z_max").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(30.0));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    for (const auto & r : robots_) {
      const std::string ns = strip_leading_slash(r);
      const std::string topic = "/" + ns + "/points";
      subs_.push_back(
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, sensor_qos,
          [this, ns](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_[ns] = msg;
          }));
      RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic.c_str());
    }

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 5);

    const double rate = std::max(0.2, this->get_parameter("publish_rate").as_double());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate)),
      [this]() { publish_merged(); });

    RCLCPP_INFO(
      this->get_logger(),
      "C++ CloudMergerNode active for %zu robot(s) -> %s (voxel=%.3fm, rate=%.1fHz)",
      robots_.size(), output_topic.c_str(), params_.voxel_size, rate);
  }

private:
  struct AcceptedPose
  {
    double stamp_sec{0.0};
    double x{0.0};
    double y{0.0};
    bool valid{false};
  };

  std::string normalized_frame(const std::string & robot, const std::string & raw_frame) const
  {
    const std::string frame = strip_leading_slash(raw_frame);
    if (frame == world_frame_ || frame.rfind(robot + "/", 0) == 0) {
      return frame;
    }
    return robot + "/" + frame;
  }

  void publish_merged()
  {
    std::vector<std::pair<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>> batch;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & r : robots_) {
        const std::string ns = strip_leading_slash(r);
        auto it = latest_.find(ns);
        if (it != latest_.end() && it->second) {
          batch.emplace_back(ns, it->second);
        }
      }
    }
    if (batch.empty()) {
      return;
    }

    std::vector<Point3D> merged;
    std::vector<Point3D> raw_buf;

    for (const auto & [robot, msg] : batch) {
      if (!extract_xyz_float32(*msg, raw_buf) || raw_buf.empty()) {
        continue;
      }

      const std::string src_frame = normalized_frame(robot, msg->header.frame_id);
      const bool has_stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0);
      geometry_msgs::msg::TransformStamped tf_msg;
      bool got_tf = false;
      bool used_latest = false;

      const tf2::TimePoint lookup_time = has_stamp
        ? tf2::TimePoint(
            std::chrono::seconds(msg->header.stamp.sec) +
            std::chrono::nanoseconds(msg->header.stamp.nanosec))
        : tf2::TimePointZero;

      try {
        tf_msg = tf_buffer_->lookupTransform(
          world_frame_, src_frame, lookup_time, tf2::durationFromSec(tf_timeout_));
        got_tf = true;
      } catch (const tf2::TransformException &) {
        if (has_stamp) {
          try {
            tf_msg = tf_buffer_->lookupTransform(
              world_frame_, src_frame, tf2::TimePointZero);
            got_tf = true;
            used_latest = true;
          } catch (const tf2::TransformException & exc) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "TF %s -> %s unavailable: %s", world_frame_.c_str(), src_frame.c_str(), exc.what());
          }
        }
      }
      if (!got_tf) {
        continue;
      }

      const double cloud_sec =
        static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

      if (used_latest && has_stamp) {
        const double tf_sec =
          static_cast<double>(tf_msg.header.stamp.sec) +
          static_cast<double>(tf_msg.header.stamp.nanosec) * 1e-9;
        const double skew = std::abs(tf_sec - cloud_sec);
        if (skew > max_tf_skew_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "%s: dropping cloud (TF skew %.2fs > %.2fs)",
            robot.c_str(), skew, max_tf_skew_);
          continue;
        }
      }

      const auto & t = tf_msg.transform.translation;
      if (has_stamp) {
        auto & prev = accepted_poses_[robot];
        if (prev.valid) {
          const double dist = std::hypot(t.x - prev.x, t.y - prev.y);
          if (pose_is_discontinuous(
              cloud_sec - prev.stamp_sec, dist,
              params_.max_robot_speed_mps, params_.pose_step_margin_m))
          {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "%s: dropping cloud, pose jumped %.2fm in %.2fs",
              robot.c_str(), dist, cloud_sec - prev.stamp_sec);
            continue;
          }
        }
        prev = {cloud_sec, t.x, t.y, true};
      }

      const auto & q = tf_msg.transform.rotation;
      const SE3Transform se3 = SE3Transform::from_quat_trans(q.x, q.y, q.z, q.w, t.x, t.y, t.z);
      auto filtered = filter_and_transform_cloud(raw_buf, se3, params_);
      merged.insert(merged.end(), filtered.begin(), filtered.end());
    }

    if (merged.empty()) {
      return;
    }

    const auto downsampled = voxel_downsample_points(merged, params_.voxel_size);
    if (downsampled.empty()) {
      return;
    }
    pub_->publish(make_xyz32_cloud(downsampled, world_frame_, this->now()));
  }

  std::mutex mutex_;
  std::vector<std::string> robots_;
  std::string world_frame_;
  double tf_timeout_{0.05};
  double max_tf_skew_{0.15};
  CloudFilterParams params_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> latest_;
  std::unordered_map<std::string, AcceptedPose> accepted_poses_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace se2_map_merge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<se2_map_merge::CloudMergerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
