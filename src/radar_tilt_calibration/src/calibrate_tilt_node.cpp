#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
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

// Remove the package-specific --config option before rclcpp parses ROS arguments.
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

// Result files intentionally resolve relative to the installed package share directory,
// as required by the package interface. An absolute path remains unchanged.
std::filesystem::path resolve_package_output_path(const std::string & value)
{
  const std::filesystem::path path(value);
  if (path.is_absolute()) {
    return path;
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

class CalibrateTiltNode : public rclcpp::Node
{
public:
  explicit CalibrateTiltNode(const std::string & initial_config_path)
  : Node("calibrate_tilt_node")
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

    const YAML::Node section = root["calibrate"];
    input_topic_ = declare_parameter<std::string>(
      "calibrate.input_topic",
      require_yaml_value<std::string>(section, "calibrate", "input_topic"));
    input_type_ = declare_parameter<std::string>(
      "calibrate.input_type",
      require_yaml_value<std::string>(section, "calibrate", "input_type"));
    duration_seconds_ = declare_parameter<double>(
      "calibrate.duration",
      require_yaml_value<double>(section, "calibrate", "duration"));
    output_file_ = declare_parameter<std::string>(
      "calibrate.output_file",
      require_yaml_value<std::string>(section, "calibrate", "output_file"));
    min_points_ = declare_parameter<int>(
      "calibrate.min_points",
      require_yaml_value<int>(section, "calibrate", "min_points"));
    const int qos_depth = declare_parameter<int>(
      "calibrate.qos_depth",
      require_yaml_value<int>(section, "calibrate", "qos_depth"));
    const std::string qos_reliability = declare_parameter<std::string>(
      "calibrate.qos_reliability",
      require_yaml_value<std::string>(section, "calibrate", "qos_reliability"));

    if (input_topic_.empty()) {
      throw std::invalid_argument("calibrate.input_topic must not be empty");
    }
    if (!std::isfinite(duration_seconds_) || duration_seconds_ <= 0.0) {
      throw std::invalid_argument("calibrate.duration must be a finite positive value");
    }
    if (min_points_ < 3) {
      throw std::invalid_argument("calibrate.min_points must be at least 3");
    }
    if (output_file_.empty()) {
      throw std::invalid_argument("calibrate.output_file must not be empty");
    }

    const rclcpp::QoS qos = make_qos(qos_depth, qos_reliability);
    if (input_type_ == "PoseStamped") {
      pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        input_topic_, qos,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          add_point(
            message->pose.position.x,
            message->pose.position.y,
            message->pose.position.z);
        });
    } else if (input_type_ == "Odometry") {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          add_point(
            message->pose.pose.position.x,
            message->pose.pose.position.y,
            message->pose.pose.position.z);
        });
    } else {
      throw std::invalid_argument(
              "calibrate.input_type must be 'PoseStamped' or 'Odometry', got '" +
              input_type_ + "'");
    }

    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(duration_seconds_));
    completion_timer_ = create_wall_timer(
      duration, std::bind(&CalibrateTiltNode::finish_calibration, this));

    RCLCPP_INFO(
      get_logger(),
      "Collecting %s positions from '%s' for %.3f seconds. Config: %s",
      input_type_.c_str(), input_topic_.c_str(), duration_seconds_,
      config_path_.c_str());
  }

  bool succeeded() const
  {
    return calibration_succeeded_;
  }

private:
  void add_point(double x, double y, double z)
  {
    if (calibration_finished_) {
      return;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      RCLCPP_WARN(get_logger(), "Skipping a localization sample with non-finite position");
      return;
    }
    points_.emplace_back(x, y, z);
  }

  void finish_calibration()
  {
    if (calibration_finished_) {
      return;
    }
    calibration_finished_ = true;
    completion_timer_->cancel();

    RCLCPP_INFO(get_logger(), "Collection finished with %zu valid points", points_.size());
    if (points_.size() < static_cast<size_t>(min_points_)) {
      RCLCPP_ERROR(
        get_logger(), "Calibration requires at least %d points, but only %zu were received; "
        "no result file was written", min_points_, points_.size());
      rclcpp::shutdown();
      return;
    }

    try {
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      for (const Eigen::Vector3d & point : points_) {
        centroid += point;
      }
      centroid /= static_cast<double>(points_.size());

      Eigen::MatrixXd centered_points(points_.size(), 3);
      for (size_t index = 0; index < points_.size(); ++index) {
        centered_points.row(static_cast<Eigen::Index>(index)) =
          (points_[index] - centroid).transpose();
      }

      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        centered_points, Eigen::ComputeThinV);
      if (svd.matrixV().cols() != 3) {
        throw std::runtime_error("SVD did not produce a three-dimensional basis");
      }

      Eigen::Vector3d normal = svd.matrixV().col(2);
      const double normal_length = normal.norm();
      if (!normal.allFinite() || normal_length <= 1.0e-12) {
        throw std::runtime_error("Plane normal is invalid; collect a less degenerate trajectory");
      }
      normal /= normal_length;
      if (normal.z() < 0.0) {
        normal = -normal;
      }

      std::string axis;
      double angle = 0.0;
      if (std::abs(normal.x()) < std::abs(normal.y())) {
        axis = "x";
        angle = std::atan2(normal.y(), normal.z());
      } else {
        axis = "y";
        angle = std::atan2(normal.x(), normal.z());
      }

      const std::filesystem::path result_path =
        resolve_package_output_path(output_file_);
      if (!result_path.parent_path().empty()) {
        std::filesystem::create_directories(result_path.parent_path());
      }

      YAML::Emitter emitter;
      emitter.SetDoublePrecision(17);
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "axis" << YAML::Value << YAML::DoubleQuoted << axis;
      emitter << YAML::Key << "angle" << YAML::Value << angle;
      emitter << YAML::EndMap;
      if (!emitter.good()) {
        throw std::runtime_error("yaml-cpp failed to serialize the calibration result");
      }

      std::ofstream output(result_path, std::ios::out | std::ios::trunc);
      if (!output.is_open()) {
        throw std::runtime_error(
                "Unable to open result file for writing: " + result_path.string());
      }
      output << emitter.c_str() << '\n';
      output.close();
      if (!output) {
        throw std::runtime_error(
                "Failed while writing result file: " + result_path.string());
      }

      calibration_succeeded_ = true;
      RCLCPP_INFO(
        get_logger(), "Plane normal in radar frame: [%.9f, %.9f, %.9f]",
        normal.x(), normal.y(), normal.z());
      RCLCPP_INFO(
        get_logger(), "Tilt calibration succeeded: axis=%s, angle=%.12f rad (%.6f deg)",
        axis.c_str(), angle, angle * 180.0 / std::acos(-1.0));
      RCLCPP_INFO(get_logger(), "Calibration result written to: %s", result_path.c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Tilt calibration failed: %s", exception.what());
    }

    rclcpp::shutdown();
  }

  std::filesystem::path config_path_;
  std::string input_topic_;
  std::string input_type_;
  std::string output_file_;
  double duration_seconds_{0.0};
  int min_points_{0};
  bool calibration_finished_{false};
  bool calibration_succeeded_{false};
  std::vector<Eigen::Vector3d> points_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr completion_timer_;
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

    auto node = std::make_shared<radar_tilt_calibration::CalibrateTiltNode>(
      parsed.config_path);
    rclcpp::spin(node);
    const bool succeeded = node->succeeded();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return succeeded ? 0 : 1;
  } catch (const std::exception & exception) {
    std::cerr << "calibrate_tilt_node fatal error: " << exception.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
