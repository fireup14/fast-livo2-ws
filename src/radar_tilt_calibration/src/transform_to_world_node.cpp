#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/header.hpp>
#include <yaml-cpp/yaml.h>

#include "radar_tilt_calibration/tilt_angle_io.hpp"
#include "radar_tilt_calibration/tilt_angles.hpp"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
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

}  // namespace

class TransformToWorldNode : public rclcpp::Node
{
public:
  explicit TransformToWorldNode(const std::string & initial_config_path)
  : Node("transform_to_world_node")
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

    const YAML::Node section = root["transform"];
    input_topic_ = declare_parameter<std::string>(
      "transform.input_topic",
      require_yaml_value<std::string>(section, "transform", "input_topic"));
    input_type_ = declare_parameter<std::string>(
      "transform.input_type",
      require_yaml_value<std::string>(section, "transform", "input_type"));
    output_topic_ = declare_parameter<std::string>(
      "transform.output_topic",
      require_yaml_value<std::string>(section, "transform", "output_topic"));
    output_type_ = declare_parameter<std::string>(
      "transform.output_type",
      require_yaml_value<std::string>(section, "transform", "output_type"));
    output_frame_id_ = declare_parameter<std::string>(
      "transform.output_frame_id",
      require_yaml_value<std::string>(section, "transform", "output_frame_id"));
    output_child_frame_id_ = declare_parameter<std::string>(
      "transform.output_child_frame_id",
      require_yaml_value<std::string>(section, "transform", "output_child_frame_id"));
    angle_file_ = declare_parameter<std::string>(
      "transform.angle_file",
      require_yaml_value<std::string>(section, "transform", "angle_file"));
    const int input_qos_depth = declare_parameter<int>(
      "transform.input_qos_depth",
      require_yaml_value<int>(section, "transform", "input_qos_depth"));
    const std::string input_qos_reliability = declare_parameter<std::string>(
      "transform.input_qos_reliability",
      require_yaml_value<std::string>(section, "transform", "input_qos_reliability"));
    const int output_qos_depth = declare_parameter<int>(
      "transform.output_qos_depth",
      require_yaml_value<int>(section, "transform", "output_qos_depth"));
    const std::string output_qos_reliability = declare_parameter<std::string>(
      "transform.output_qos_reliability",
      require_yaml_value<std::string>(section, "transform", "output_qos_reliability"));
    preserve_odometry_twist_ = declare_parameter<bool>(
      "transform.preserve_odometry_twist",
      require_yaml_value<bool>(section, "transform", "preserve_odometry_twist"));

    validate_configuration();
    load_angle_file();

    const rclcpp::QoS input_qos = make_qos(input_qos_depth, input_qos_reliability);
    const rclcpp::QoS output_qos = make_qos(output_qos_depth, output_qos_reliability);

    if (output_type_ == "PoseStamped") {
      pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        output_topic_, output_qos);
    } else {
      odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        output_topic_, output_qos);
    }

    if (input_type_ == "PoseStamped") {
      pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        input_topic_, input_qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          process_pose(message->header, message->pose, nullptr);
        });
    } else {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, input_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          process_pose(message->header, message->pose.pose, message.get());
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "Transform ready: %s '%s' -> %s '%s', output frame '%s'. "
      "Loaded angles: x=%.9f deg, y=%.9f deg, z=%.9f deg; "
      "selected axis=%s, angle=%.9f deg (%.12f rad). Config: %s",
      input_type_.c_str(), input_topic_.c_str(), output_type_.c_str(),
      output_topic_.c_str(), output_frame_id_.c_str(), angles_deg_.x,
      angles_deg_.y, angles_deg_.z, dominant_tilt_.axis.c_str(),
      dominant_tilt_.angle_deg, dominant_tilt_.angle_rad,
      config_path_.c_str());
  }

private:
  void validate_configuration() const
  {
    if (input_topic_.empty() || output_topic_.empty()) {
      throw std::invalid_argument("transform input_topic and output_topic must not be empty");
    }
    if (output_frame_id_.empty()) {
      throw std::invalid_argument("transform.output_frame_id must not be empty");
    }
    if (angle_file_.empty()) {
      throw std::invalid_argument("transform.angle_file must not be empty");
    }
    if (input_type_ != "PoseStamped" && input_type_ != "Odometry") {
      throw std::invalid_argument(
              "transform.input_type must be 'PoseStamped' or 'Odometry', got '" +
              input_type_ + "'");
    }
    if (output_type_ != "PoseStamped" && output_type_ != "Odometry") {
      throw std::invalid_argument(
              "transform.output_type must be 'PoseStamped' or 'Odometry', got '" +
              output_type_ + "'");
    }
  }

  void load_angle_file()
  {
    const std::filesystem::path angle_path = resolve_tilt_angle_path(angle_file_);
    angles_deg_ = load_tilt_angles_deg(angle_path);
    dominant_tilt_ = select_dominant_tilt(angles_deg_);
    rotation_ = make_correction_rotation(dominant_tilt_);

    const Eigen::Quaterniond correction(rotation_);
    correction_quaternion_.setValue(
      correction.x(), correction.y(), correction.z(), correction.w());
    correction_quaternion_.normalize();
  }

  std::optional<geometry_msgs::msg::Pose> transform_pose(
    const geometry_msgs::msg::Pose & input_pose)
  {
    const Eigen::Vector3d input_position(
      input_pose.position.x, input_pose.position.y, input_pose.position.z);
    if (!input_position.allFinite()) {
      RCLCPP_ERROR(get_logger(), "Skipping input pose with non-finite position");
      return std::nullopt;
    }

    tf2::Quaternion input_quaternion;
    tf2::fromMsg(input_pose.orientation, input_quaternion);
    if (!std::isfinite(input_quaternion.length2()) || input_quaternion.length2() <= 1.0e-12) {
      RCLCPP_ERROR(get_logger(), "Skipping input pose with an invalid quaternion");
      return std::nullopt;
    }
    input_quaternion.normalize();

    const Eigen::Vector3d output_position = rotation_ * input_position;
    tf2::Quaternion output_quaternion = correction_quaternion_ * input_quaternion;
    output_quaternion.normalize();

    geometry_msgs::msg::Pose output_pose;
    output_pose.position.x = output_position.x();
    output_pose.position.y = output_position.y();
    output_pose.position.z = output_position.z();
    output_pose.orientation = tf2::toMsg(output_quaternion);
    return output_pose;
  }

  std::array<double, 36> transform_pose_covariance(
    const std::array<double, 36> & input_covariance) const
  {
    Eigen::Matrix<double, 6, 6> covariance;
    for (Eigen::Index row = 0; row < 6; ++row) {
      for (Eigen::Index column = 0; column < 6; ++column) {
        covariance(row, column) = input_covariance[static_cast<size_t>(row * 6 + column)];
      }
    }

    Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
    transform.block<3, 3>(0, 0) = rotation_;
    transform.block<3, 3>(3, 3) = rotation_;
    const Eigen::Matrix<double, 6, 6> output =
      transform * covariance * transform.transpose();

    std::array<double, 36> result{};
    for (Eigen::Index row = 0; row < 6; ++row) {
      for (Eigen::Index column = 0; column < 6; ++column) {
        result[static_cast<size_t>(row * 6 + column)] = output(row, column);
      }
    }
    return result;
  }

  void process_pose(
    const std_msgs::msg::Header & input_header,
    const geometry_msgs::msg::Pose & input_pose,
    const nav_msgs::msg::Odometry * input_odometry)
  {
    const auto output_pose = transform_pose(input_pose);
    if (!output_pose.has_value()) {
      return;
    }

    if (output_type_ == "PoseStamped") {
      geometry_msgs::msg::PoseStamped output;
      output.header.stamp = input_header.stamp;
      output.header.frame_id = output_frame_id_;
      output.pose = *output_pose;
      pose_publisher_->publish(output);
      return;
    }

    nav_msgs::msg::Odometry output;
    output.header.stamp = input_header.stamp;
    output.header.frame_id = output_frame_id_;
    output.child_frame_id = output_child_frame_id_;
    output.pose.pose = *output_pose;

    if (input_odometry != nullptr) {
      if (output.child_frame_id.empty()) {
        output.child_frame_id = input_odometry->child_frame_id;
      }
      output.pose.covariance = transform_pose_covariance(input_odometry->pose.covariance);

      // nav_msgs/Odometry defines twist in child_frame_id, so preserving it is
      // consistent while the child frame remains unchanged. If disabled, or if
      // the input is PoseStamped, the default-initialized twist is all zeros.
      if (preserve_odometry_twist_) {
        output.twist = input_odometry->twist;
      }
    }
    odometry_publisher_->publish(output);
  }

  std::filesystem::path config_path_;
  std::string input_topic_;
  std::string input_type_;
  std::string output_topic_;
  std::string output_type_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  std::string angle_file_;
  TiltAnglesDeg angles_deg_;
  DominantTilt dominant_tilt_;
  bool preserve_odometry_twist_{true};

  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  tf2::Quaternion correction_quaternion_{0.0, 0.0, 0.0, 1.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
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

    auto node = std::make_shared<radar_tilt_calibration::TransformToWorldNode>(
      parsed.config_path);
    rclcpp::spin(node);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "transform_to_world_node fatal error: " << exception.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
