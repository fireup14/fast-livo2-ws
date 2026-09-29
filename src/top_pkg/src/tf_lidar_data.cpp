#include "top_pkg/tf_lidar_data.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"

namespace
{

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto iterator = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
  return iterator == cloud.fields.end() ? nullptr : &(*iterator);
}

bool fieldIsValid(
  const sensor_msgs::msg::PointField * field, uint8_t expected_datatype,
  std::size_t value_size, uint32_t point_step)
{
  return field != nullptr && field->datatype == expected_datatype && field->count == 1 &&
         field->offset + value_size <= point_step;
}

template<typename ValueT>
ValueT readValue(const uint8_t * point_data, uint32_t offset)
{
  ValueT value{};
  std::memcpy(&value, point_data + offset, sizeof(ValueT));
  return value;
}

}  // namespace

namespace top_pkg
{

TfLidarDataNode::TfLidarDataNode(const rclcpp::NodeOptions & options)
: Node("tf_lidar_data", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar_points");
  output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar");
  timestamp_mode_ = declare_parameter<std::string>("timestamp_mode", "absolute_ns");
  const int64_t lidar_id = declare_parameter<int64_t>("lidar_id", 0);

  if (input_topic_ == output_topic_) {
    throw std::invalid_argument(
            "input_topic and output_topic must be different because their ROS message types differ");
  }
  if (lidar_id < 0 || lidar_id > std::numeric_limits<uint8_t>::max()) {
    throw std::invalid_argument("lidar_id must be in the range [0, 255]");
  }
  const std::vector<std::string> supported_timestamp_modes = {
    "absolute_ns", "relative_ns", "absolute_sec", "relative_sec"};
  if (std::find(
      supported_timestamp_modes.begin(), supported_timestamp_modes.end(), timestamp_mode_) ==
    supported_timestamp_modes.end())
  {
    throw std::invalid_argument(
            "timestamp_mode must be absolute_ns, relative_ns, absolute_sec, or relative_sec");
  }
  lidar_id_ = static_cast<uint8_t>(lidar_id);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic_, qos);
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, qos,
    std::bind(&TfLidarDataNode::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Converting Livox PointCloud2 '%s' to CustomMsg '%s' (timestamp_mode=%s)",
    input_topic_.c_str(), output_topic_.c_str(), timestamp_mode_.c_str());
}

bool TfLidarDataNode::convertPointTimestamp(
  double point_timestamp, int64_t header_time_ns, uint32_t & offset_time_ns) const
{
  long double offset_ns = 0.0L;
  if (timestamp_mode_ == "absolute_ns") {
    offset_ns = static_cast<long double>(point_timestamp) -
      static_cast<long double>(header_time_ns);
  } else if (timestamp_mode_ == "relative_ns") {
    offset_ns = static_cast<long double>(point_timestamp);
  } else if (timestamp_mode_ == "absolute_sec") {
    offset_ns =
      (static_cast<long double>(point_timestamp) -
      static_cast<long double>(header_time_ns) / 1.0e9L) * 1.0e9L;
  } else {
    offset_ns = static_cast<long double>(point_timestamp) * 1.0e9L;
  }

  // A double storing an epoch-scale nanosecond timestamp loses a few low bits.
  // Accept a small negative rounding error for the first point of a scan.
  constexpr long double rounding_tolerance_ns = 10000.0L;
  if (offset_ns < 0.0L && offset_ns >= -rounding_tolerance_ns) {
    offset_ns = 0.0L;
  }
  if (!std::isfinite(static_cast<double>(offset_ns)) || offset_ns < 0.0L ||
    offset_ns > static_cast<long double>(std::numeric_limits<uint32_t>::max()))
  {
    return false;
  }

  offset_time_ns = static_cast<uint32_t>(std::llround(offset_ns));
  return true;
}

void TfLidarDataNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->is_bigendian) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Big-endian PointCloud2 is not supported");
    return;
  }

  const auto * field_x = findField(*msg, "x");
  const auto * field_y = findField(*msg, "y");
  const auto * field_z = findField(*msg, "z");
  const auto * field_intensity = findField(*msg, "intensity");
  const auto * field_tag = findField(*msg, "tag");
  const auto * field_line = findField(*msg, "line");
  const auto * field_timestamp = findField(*msg, "timestamp");

  const bool fields_valid =
    fieldIsValid(
    field_x, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), msg->point_step) &&
    fieldIsValid(
    field_y, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), msg->point_step) &&
    fieldIsValid(
    field_z, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), msg->point_step) &&
    fieldIsValid(
    field_intensity, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), msg->point_step) &&
    fieldIsValid(
    field_tag, sensor_msgs::msg::PointField::UINT8, sizeof(uint8_t), msg->point_step) &&
    fieldIsValid(
    field_line, sensor_msgs::msg::PointField::UINT8, sizeof(uint8_t), msg->point_step) &&
    fieldIsValid(
    field_timestamp, sensor_msgs::msg::PointField::FLOAT64, sizeof(double), msg->point_step);

  if (!fields_valid) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Input must be Livox PointXYZRTLT PointCloud2 with fields: "
      "x/y/z/intensity(float32), tag/line(uint8), timestamp(float64)");
    return;
  }

  const std::size_t expected_data_size =
    msg->height == 0 ? 0 :
    static_cast<std::size_t>(msg->row_step) * (msg->height - 1) +
    static_cast<std::size_t>(msg->point_step) * msg->width;
  if (msg->height == 0 || msg->width == 0 || msg->data.size() < expected_data_size) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received an empty or malformed PointCloud2 message");
    return;
  }

  const int64_t header_time_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (header_time_ns < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Negative PointCloud2 header timestamp is not supported");
    return;
  }

  livox_ros_driver2::msg::CustomMsg output;
  output.header = msg->header;
  output.timebase = static_cast<uint64_t>(header_time_ns);
  output.lidar_id = lidar_id_;
  output.rsvd = {0, 0, 0};
  output.points.reserve(static_cast<std::size_t>(msg->width) * msg->height);

  std::size_t rejected_points = 0;
  for (uint32_t row = 0; row < msg->height; ++row) {
    for (uint32_t column = 0; column < msg->width; ++column) {
      const std::size_t point_offset =
        static_cast<std::size_t>(row) * msg->row_step +
        static_cast<std::size_t>(column) * msg->point_step;
      const uint8_t * point_data = msg->data.data() + point_offset;

      const float x = readValue<float>(point_data, field_x->offset);
      const float y = readValue<float>(point_data, field_y->offset);
      const float z = readValue<float>(point_data, field_z->offset);
      const float intensity = readValue<float>(point_data, field_intensity->offset);
      const double timestamp = readValue<double>(point_data, field_timestamp->offset);

      uint32_t offset_time_ns = 0;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(intensity) || !std::isfinite(timestamp) ||
        !convertPointTimestamp(timestamp, header_time_ns, offset_time_ns))
      {
        ++rejected_points;
        continue;
      }

      livox_ros_driver2::msg::CustomPoint point;
      point.offset_time = offset_time_ns;
      point.x = x;
      point.y = y;
      point.z = z;
      point.reflectivity = static_cast<uint8_t>(
        std::clamp(std::lround(intensity), 0L, 255L));
      point.tag = readValue<uint8_t>(point_data, field_tag->offset);
      point.line = readValue<uint8_t>(point_data, field_line->offset);
      output.points.push_back(point);
    }
  }

  if (output.points.empty()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "PointCloud2 conversion rejected every point; check timestamp_mode and field contents");
    return;
  }

  std::stable_sort(
    output.points.begin(), output.points.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.offset_time < rhs.offset_time;});
  output.point_num = static_cast<uint32_t>(output.points.size());

  if (rejected_points > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected %zu invalid points while converting PointCloud2", rejected_points);
  }
  custom_pub_->publish(std::move(output));
}

}  // namespace top_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<top_pkg::TfLidarDataNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("tf_lidar_data"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
