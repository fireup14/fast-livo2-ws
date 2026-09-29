#ifndef TOP_PKG__TF_LIDAR_DATA_HPP_
#define TOP_PKG__TF_LIDAR_DATA_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace top_pkg
{

class TfLidarDataNode : public rclcpp::Node
{
public:
  explicit TfLidarDataNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  bool convertPointTimestamp(
    double point_timestamp, int64_t header_time_ns, uint32_t & offset_time_ns) const;

  std::string input_topic_;
  std::string output_topic_;
  std::string timestamp_mode_;
  uint8_t lidar_id_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
};

}  // namespace top_pkg

#endif  // TOP_PKG__TF_LIDAR_DATA_HPP_
