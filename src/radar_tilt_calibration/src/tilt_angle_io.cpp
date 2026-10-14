#include "radar_tilt_calibration/tilt_angle_io.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace radar_tilt_calibration
{
namespace
{

constexpr const char * kPackageName = "radar_tilt_calibration";

std::filesystem::path find_source_package_directory()
{
  std::filesystem::path current =
    ament_index_cpp::get_package_prefix(kPackageName);

  while (!current.empty()) {
    const std::filesystem::path candidate = current / "src" / kPackageName;
    if (std::filesystem::is_regular_file(candidate / "package.xml")) {
      return candidate;
    }

    const std::filesystem::path parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }

  throw std::runtime_error(
          "Unable to locate source package 'src/radar_tilt_calibration'. "
          "Use an absolute tilt angle file path when running without the source workspace.");
}

double require_finite_angle(const YAML::Node & angles, const std::string & axis)
{
  if (!angles[axis]) {
    throw std::runtime_error("Missing YAML key 'angles_deg." + axis + "'");
  }

  double value = 0.0;
  try {
    value = angles[axis].as<double>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid YAML value for 'angles_deg." + axis + "': " + exception.what());
  }

  if (!std::isfinite(value)) {
    throw std::runtime_error("Angle 'angles_deg." + axis + "' is not finite");
  }
  return value;
}

}  // namespace

std::filesystem::path resolve_tilt_angle_path(const std::string & value)
{
  if (value.empty()) {
    throw std::invalid_argument("Tilt angle file path must not be empty");
  }

  const std::filesystem::path path(value);
  if (path.is_absolute()) {
    return path.lexically_normal();
  }
  return (find_source_package_directory() / path).lexically_normal();
}

TiltAnglesDeg load_tilt_angles_deg(const std::filesystem::path & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Failed to load tilt angle file '" + path.string() + "': " + exception.what());
  }

  const YAML::Node angles = root["angles_deg"];
  if (!angles || !angles.IsMap()) {
    throw std::runtime_error(
            "Missing YAML map 'angles_deg' in tilt angle file '" + path.string() + "'");
  }

  return TiltAnglesDeg{
    require_finite_angle(angles, "x"),
    require_finite_angle(angles, "y"),
    require_finite_angle(angles, "z")};
}

void write_tilt_angles_deg(
  const std::filesystem::path & path,
  const TiltAnglesDeg & angles)
{
  // 复用选择函数对三个角做有限值检查；返回的主轴在写文件阶段无需使用。
  static_cast<void>(select_dominant_tilt(angles));

  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }

  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "angles_deg" << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "x" << YAML::Value << angles.x;
  emitter << YAML::Key << "y" << YAML::Value << angles.y;
  emitter << YAML::Key << "z" << YAML::Value << angles.z;
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;
  if (!emitter.good()) {
    throw std::runtime_error("yaml-cpp failed to serialize the tilt angles");
  }

  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("Unable to open tilt angle file for writing: " + path.string());
  }

  output << emitter.c_str() << '\n';
  output.close();
  if (!output) {
    throw std::runtime_error("Failed while writing tilt angle file: " + path.string());
  }
}

}  // namespace radar_tilt_calibration
