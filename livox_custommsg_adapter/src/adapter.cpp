#include "livox_custommsg_adapter/adapter.hpp"

namespace livox_custommsg_adapter {

static inline builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & t) {
  builtin_interfaces::msg::Time bt;
  const int64_t ns = t.nanoseconds();
  bt.sec     = static_cast<int32_t>(ns / 1000000000LL);
  bt.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return bt;
}

AdapterNode::AdapterNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("livox_custommsg_adapter", options) {
  frame_id_ = this->declare_parameter<std::string>("frame_id", "livox_frame");
  out_topic_ = this->declare_parameter<std::string>("out_topic", "/lio_sam/points");
  time_scale_ = this->declare_parameter<double>("time_scale", 1e-6);
  use_msg_header_time_ = this->declare_parameter<bool>("use_msg_header_time", true);

  auto in_topic = this->declare_parameter<std::string>("in_topic", "/livox/lidar");

  // 구독: RELIABLE
  rclcpp::QoS sub_qos(rclcpp::KeepLast(100));
  sub_qos.reliable();
  sub_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(
      in_topic, sub_qos,
      std::bind(&AdapterNode::cb, this, std::placeholders::_1));

  // 퍼블: RELIABLE
  rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
  pub_qos.reliable();
  // pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_, pub_qos);

  RCLCPP_INFO(get_logger(), "Adapter started. in: %s → out: %s, frame: %s, time_scale: %.3e",
              in_topic.c_str(), out_topic_.c_str(), frame_id_.c_str(), time_scale_);
}

void AdapterNode::cb(const livox_interfaces::msg::CustomMsg::SharedPtr msg) {
  const auto &pts = msg->points;
  const size_t n = pts.size();
  if (n == 0) return;

  sensor_msgs::msg::PointCloud2 cloud;
  if (use_msg_header_time_) {
    cloud.header.stamp = msg->header.stamp;
  } else {
    cloud.header.stamp = to_builtin_time(this->now());
  }
  cloud.header.frame_id = frame_id_;

  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(n);
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(6,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32,
    "ring", 1, sensor_msgs::msg::PointField::UINT16
  );
  mod.resize(n);

  sensor_msgs::PointCloud2Iterator<float>     ix(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float>     iy(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float>     iz(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float>     it(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<float>     itime(cloud, "time");
  sensor_msgs::PointCloud2Iterator<uint16_t>  iring(cloud, "ring");

  for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz, ++it, ++itime, ++iring) {
    const auto &p = pts[i];
    *ix = p.x;
    *iy = p.y;
    *iz = p.z;
    *it = static_cast<float>(p.reflectivity);
    *itime = static_cast<float>(p.offset_time) * static_cast<float>(time_scale_);
    *iring = static_cast<uint16_t>(p.line);
  }

  // pub_->publish(cloud);
}

} // namespace livox_custommsg_adapter