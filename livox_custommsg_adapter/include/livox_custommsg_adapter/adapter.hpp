#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_msgs/msg/custom_msg.hpp>

namespace livox_custommsg_adapter {

class AdapterNode : public rclcpp::Node {
public:
  explicit AdapterNode(const rclcpp::NodeOptions &options);

private:
  void cb(const livox_msgs::msg::CustomMsg::SharedPtr msg);

  // params
  std::string frame_id_;
  std::string out_topic_;
  double time_scale_; // scale offset_time to seconds (e.g., 1e-6 for µs, 1e-9 for ns)
  bool use_msg_header_time_;

  rclcpp::Subscription<livox_msgs::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

} // namespace livox_custommsg_adapter