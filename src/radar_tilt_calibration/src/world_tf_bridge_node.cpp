#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include "radar_tilt_calibration/tilt_angle_io.hpp"
#include "radar_tilt_calibration/tilt_angles.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace radar_tilt_calibration
{
namespace
{

constexpr const char * kPackageName = "radar_tilt_calibration";

struct ParsedArguments
{
  std::string config_path;
  std::vector<std::string> ros_arguments;
};

std::string default_config_path()
{
  return (
    std::filesystem::path(
      ament_index_cpp::get_package_share_directory(kPackageName)) /
    "config" / "calibrate_tilt_param.yaml").string();
}

ParsedArguments parse_arguments(int argc, char ** argv)
{
  ParsedArguments parsed;
  parsed.config_path = default_config_path();
  parsed.ros_arguments.emplace_back(argv[0]);

  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--config") {
      if (index + 1 >= argc) {
        throw std::invalid_argument("--config requires a file path");
      }
      parsed.config_path = argv[++index];
      continue;
    }

    constexpr const char * kConfigPrefix = "--config=";
    if (argument.rfind(kConfigPrefix, 0) == 0) {
      parsed.config_path = argument.substr(std::char_traits<char>::length(kConfigPrefix));
      if (parsed.config_path.empty()) {
        throw std::invalid_argument("--config= requires a non-empty file path");
      }
      continue;
    }
    parsed.ros_arguments.push_back(argument);
  }
  return parsed;
}

std::filesystem::path resolve_config_path(const std::string & value)
{
  const std::filesystem::path path(value);
  if (path.is_absolute()) {
    return path;
  }
  if (std::filesystem::exists(path)) {
    return std::filesystem::absolute(path);
  }
  return std::filesystem::path(
    ament_index_cpp::get_package_share_directory(kPackageName)) / path;
}

template<typename ValueT>
ValueT require_yaml_value(
  const YAML::Node & section, const std::string & section_name,
  const std::string & key)
{
  if (!section || !section.IsMap() || !section[key]) {
    throw std::runtime_error(
            "Missing YAML key '" + section_name + "." + key + "'");
  }
  try {
    return section[key].as<ValueT>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid YAML value for '" + section_name + "." + key + "': " +
            exception.what());
  }
}

rclcpp::QoS make_qos(int depth, const std::string & reliability)
{
  if (depth <= 0) {
    throw std::invalid_argument("QoS depth must be greater than zero");
  }
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth)));
  if (reliability == "reliable") {
    return qos.reliable();
  }
  if (reliability == "best_effort") {
    return qos.best_effort();
  }
  throw std::invalid_argument(
          "QoS reliability must be 'reliable' or 'best_effort', got '" +
          reliability + "'");
}

void validate_frame_id(const std::string & parameter_name, const std::string & frame_id)
{
  if (frame_id.empty()) {
    throw std::invalid_argument(parameter_name + " must not be empty");
  }
  if (frame_id.front() == '/') {
    throw std::invalid_argument(
            parameter_name + " must not start with '/'; got '" + frame_id + "'");
  }
}

}  // namespace

class WorldTfBridgeNode : public rclcpp::Node
{
public:
  explicit WorldTfBridgeNode(const std::string & initial_config_path)
  : Node("world_tf_bridge_node")
  {
    const std::string requested_config =
      declare_parameter<std::string>("config_file", initial_config_path);
    config_path_ = resolve_config_path(requested_config);

    YAML::Node root;
    try {
      root = YAML::LoadFile(config_path_.string());
    } catch (const YAML::Exception & exception) {
      throw std::runtime_error(
              "Failed to load configuration file '" + config_path_.string() +
              "': " + exception.what());
    }

    const YAML::Node section = root["tf_bridge"];
    enabled_ = declare_parameter<bool>(
      "tf_bridge.enable",
      require_yaml_value<bool>(section, "tf_bridge", "enable"));
    original_root_frame_ = declare_parameter<std::string>(
      "tf_bridge.original_root_frame",
      require_yaml_value<std::string>(section, "tf_bridge", "original_root_frame"));
    corrected_root_frame_ = declare_parameter<std::string>(
      "tf_bridge.corrected_root_frame",
      require_yaml_value<std::string>(section, "tf_bridge", "corrected_root_frame"));
    corrected_child_frame_ = declare_parameter<std::string>(
      "tf_bridge.corrected_child_frame",
      require_yaml_value<std::string>(section, "tf_bridge", "corrected_child_frame"));
    corrected_odom_topic_ = declare_parameter<std::string>(
      "tf_bridge.corrected_odom_topic",
      require_yaml_value<std::string>(section, "tf_bridge", "corrected_odom_topic"));
    angle_file_ = declare_parameter<std::string>(
      "tf_bridge.angle_file",
      require_yaml_value<std::string>(section, "tf_bridge", "angle_file"));
    const int qos_depth = declare_parameter<int>(
      "tf_bridge.qos_depth",
      require_yaml_value<int>(section, "tf_bridge", "qos_depth"));
    const std::string qos_reliability = declare_parameter<std::string>(
      "tf_bridge.qos_reliability",
      require_yaml_value<std::string>(section, "tf_bridge", "qos_reliability"));

    validate_configuration();
    if (!enabled_) {
      RCLCPP_WARN(
        get_logger(), "TF bridge is disabled by tf_bridge.enable. Config: %s",
        config_path_.c_str());
      return;
    }

    load_correction();

    static_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(this);

    publish_static_root_transform();

    corrected_odometry_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      corrected_odom_topic_, make_qos(qos_depth, qos_reliability),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        broadcast_corrected_odometry(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "TF bridge ready: static '%s' -> '%s'; dynamic '%s' -> '%s' from '%s'. "
      "Correction axis=%s, angle=%.9f deg. Config: %s",
      corrected_root_frame_.c_str(), original_root_frame_.c_str(),
      corrected_root_frame_.c_str(), corrected_child_frame_.c_str(),
      corrected_odom_topic_.c_str(), correction_.axis.c_str(),
      correction_.angle_deg, config_path_.c_str());
  }

private:
  void validate_configuration() const
  {
    if (!enabled_) {
      return;
    }
    validate_frame_id("tf_bridge.original_root_frame", original_root_frame_);
    validate_frame_id("tf_bridge.corrected_root_frame", corrected_root_frame_);
    validate_frame_id("tf_bridge.corrected_child_frame", corrected_child_frame_);
    if (original_root_frame_ == corrected_root_frame_ ||
      original_root_frame_ == corrected_child_frame_ ||
      corrected_root_frame_ == corrected_child_frame_)
    {
      throw std::invalid_argument("TF bridge frame IDs must be three distinct names");
    }
    if (corrected_odom_topic_.empty()) {
      throw std::invalid_argument("tf_bridge.corrected_odom_topic must not be empty");
    }
    if (angle_file_.empty()) {
      throw std::invalid_argument("tf_bridge.angle_file must not be empty");
    }
  }

  void load_correction()
  {
    const std::filesystem::path angle_path = resolve_tilt_angle_path(angle_file_);
    angles_deg_ = load_tilt_angles_deg(angle_path);
    correction_ = select_dominant_tilt(angles_deg_);
    correction_rotation_ = make_correction_rotation(correction_);
    correction_quaternion_ = Eigen::Quaterniond(correction_rotation_).normalized();
  }

  void publish_static_root_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = corrected_root_frame_;
    transform.child_frame_id = original_root_frame_;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = correction_quaternion_.x();
    transform.transform.rotation.y = correction_quaternion_.y();
    transform.transform.rotation.z = correction_quaternion_.z();
    transform.transform.rotation.w = correction_quaternion_.w();
    static_broadcaster_->sendTransform(transform);
  }

  void broadcast_corrected_odometry(const nav_msgs::msg::Odometry & message)
  {
    if (message.header.frame_id != corrected_root_frame_ ||
      message.child_frame_id != corrected_child_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring '%s': expected frame pair '%s' -> '%s', got '%s' -> '%s'",
        corrected_odom_topic_.c_str(), corrected_root_frame_.c_str(),
        corrected_child_frame_.c_str(), message.header.frame_id.c_str(),
        message.child_frame_id.c_str());
      return;
    }

    const auto & position = message.pose.pose.position;
    const auto & orientation = message.pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(quaternion_norm) ||
      quaternion_norm <= 1.0e-12)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring '%s' message with invalid pose", corrected_odom_topic_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message.header.stamp;
    transform.header.frame_id = corrected_root_frame_;
    transform.child_frame_id = corrected_child_frame_;
    transform.transform.translation.x = position.x;
    transform.transform.translation.y = position.y;
    transform.transform.translation.z = position.z;
    transform.transform.rotation.x = orientation.x / quaternion_norm;
    transform.transform.rotation.y = orientation.y / quaternion_norm;
    transform.transform.rotation.z = orientation.z / quaternion_norm;
    transform.transform.rotation.w = orientation.w / quaternion_norm;
    dynamic_broadcaster_->sendTransform(transform);
  }

  std::filesystem::path config_path_;
  bool enabled_{true};
  std::string original_root_frame_;
  std::string corrected_root_frame_;
  std::string corrected_child_frame_;
  std::string corrected_odom_topic_;
  std::string angle_file_;

  TiltAnglesDeg angles_deg_;
  DominantTilt correction_;
  Eigen::Matrix3d correction_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond correction_quaternion_{Eigen::Quaterniond::Identity()};

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    corrected_odometry_subscription_;
};

}  // namespace radar_tilt_calibration

int main(int argc, char ** argv)
{
  try {
    const radar_tilt_calibration::ParsedArguments parsed =
      radar_tilt_calibration::parse_arguments(argc, argv);

    std::vector<char *> ros_argv;
    ros_argv.reserve(parsed.ros_arguments.size());
    for (const std::string & argument : parsed.ros_arguments) {
      ros_argv.push_back(const_cast<char *>(argument.c_str()));
    }
    int ros_argc = static_cast<int>(ros_argv.size());
    rclcpp::init(ros_argc, ros_argv.data());

    auto node = std::make_shared<radar_tilt_calibration::WorldTfBridgeNode>(
      parsed.config_path);
    rclcpp::spin(node);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "world_tf_bridge_node fatal error: " << exception.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
