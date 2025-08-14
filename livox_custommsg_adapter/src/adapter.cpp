#include "livox_custommsg_adapter/adapter.hpp"

namespace livox_custommsg_adapter {

AdapterNode::AdapterNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("livox_custommsg_adapter", options) {
  frame_id_ = this->declare_parameter<std::string>("frame_id", "livox_frame");
  out_topic_ = this->declare_parameter<std::string>("out_topic", "/lio_sam/points");
  time_scale_ = this->declare_parameter<double>("time_scale", 1e-6); // default µs → s
  use_msg_header_time_ = this->declare_parameter<bool>("use_msg_header_time", true);

  // Subscribe to Livox CustomMsg
  auto in_topic = this->declare_parameter<std::string>("in_topic", "/livox/lidar");
  rclcpp::SensorDataQoS qos; // best-effort, low-latency
  sub_ = this->create_subscription<livox_msgs::msg::CustomMsg>(
      in_topic, qos,
      std::bind(&AdapterNode::cb, this, std::placeholders::_1));

  // Publisher PointCloud2
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_, qos);

  RCLCPP_INFO(get_logger(), "Adapter started. in: %s → out: %s, frame: %s, time_scale: %.3e",
              in_topic.c_str(), out_topic_.c_str(), frame_id_.c_str(), time_scale_);
}

void AdapterNode::cb(const livox_msgs::msg::CustomMsg::SharedPtr msg) {
  const auto &pts = msg->points;
  const size_t n = pts.size();
  if (n == 0) return;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = use_msg_header_time_ ? msg->header.stamp : this->now();
  cloud.header.frame_id = frame_id_;

  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(n);
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(2, "xyz", "intensity");
  // Add custom fields: time (float32), ring (uint16)
  mod.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32
  );
  // Append ring at the end (uint16)
  sensor_msgs::PointCloud2Modifier mod2(cloud);
  mod2.resize(n);

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_time(cloud, "time");

  // Manually compute ring field offset and write as uint16
  // Add ring field at the end (after "time")
  sensor_msgs::msg::PointField ring_field;
  ring_field.name = "ring";
  ring_field.offset = cloud.point_step; // current end
  ring_field.datatype = sensor_msgs::msg::PointField::UINT16;
  ring_field.count = 1;
  cloud.fields.push_back(ring_field);
  cloud.point_step += sizeof(uint16_t);
  cloud.row_step = cloud.point_step * cloud.width;

  // Resize data buffer accordingly
  std::vector<uint8_t> data(cloud.row_step * cloud.height);
  cloud.data.swap(data);

  // Recreate iterators after data change
  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<float> itime(cloud, "time");

  // pointer to ring field
  auto ring_offset = ring_field.offset;
  for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz, ++it, ++itime) {
    const auto &p = pts[i];
    *ix = p.x;
    *iy = p.y;
    *iz = p.z;
    *it = static_cast<float>(p.reflectivity);

    // Convert offset_time to seconds with scaling (float)
    const float t = static_cast<float>(p.offset_time) * static_cast<float>(time_scale_);
    *itime = t;

    // ring from livox line
    uint16_t ring = static_cast<uint16_t>(p.line);

    // write ring
    uint8_t *ptr = &cloud.data[i * cloud.point_step + ring_offset];
    *reinterpret_cast<uint16_t*>(ptr) = ring;
  }

  pub_->publish(cloud);
}

} // namespace livox_custommsg_adapter