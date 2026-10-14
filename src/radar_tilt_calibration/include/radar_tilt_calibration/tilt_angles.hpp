#ifndef RADAR_TILT_CALIBRATION__TILT_ANGLES_HPP_
#define RADAR_TILT_CALIBRATION__TILT_ANGLES_HPP_

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace radar_tilt_calibration
{

// 标定文件中保存的三个轴向角，单位统一为度。
struct TiltAnglesDeg
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// 转换节点最终选中的单轴校正量，同时保存度和弧度，避免单位混用。
struct DominantTilt
{
  std::string axis{"x"};
  double angle_deg{0.0};
  double angle_rad{0.0};
};

inline double radians_to_degrees(double radians)
{
  return radians * 180.0 / std::acos(-1.0);
}

inline double degrees_to_radians(double degrees)
{
  return degrees * std::acos(-1.0) / 180.0;
}

// 由输入坐标系中的世界 Z 轴方向计算三个轴向角。
//
// 对水平运动平面而言，法向量能够观测绕 X、Y 轴的倾斜，但无法观测绕 Z 轴的
// 偏航。因此 z 明确输出为 0 deg，而不是给出一个没有物理意义的估计值。
inline TiltAnglesDeg calculate_tilt_angles_deg(const Eigen::Vector3d & plane_normal)
{
  if (!plane_normal.allFinite() || plane_normal.norm() <= 1.0e-12) {
    throw std::invalid_argument("Plane normal must be finite and non-zero");
  }

  Eigen::Vector3d normal = plane_normal.normalized();
  if (normal.z() < 0.0) {
    normal = -normal;
  }

  TiltAnglesDeg angles;
  angles.x = radians_to_degrees(std::atan2(normal.y(), normal.z()));
  // 对 p_world = R_y(beta) * p_input，有
  // n_input = R_y(beta)^T * e_z = (-sin(beta), 0, cos(beta))。
  angles.y = radians_to_degrees(std::atan2(-normal.x(), normal.z()));
  angles.z = 0.0;
  return angles;
}

// 按绝对值选择倾斜最大的轴，但保留角度原本的正负号。
inline DominantTilt select_dominant_tilt(const TiltAnglesDeg & angles)
{
  const std::array<std::pair<const char *, double>, 3> candidates{{
    {"x", angles.x},
    {"y", angles.y},
    {"z", angles.z},
  }};

  for (const auto & candidate : candidates) {
    if (!std::isfinite(candidate.second)) {
      throw std::invalid_argument("Tilt angle for axis '" +
              std::string(candidate.first) + "' is not finite");
    }
  }

  const auto * selected = &candidates.front();
  for (const auto & candidate : candidates) {
    if (std::abs(candidate.second) > std::abs(selected->second)) {
      selected = &candidate;
    }
  }

  return DominantTilt{
    selected->first,
    selected->second,
    degrees_to_radians(selected->second)};
}

// 由选中的单轴校正角构造从原始初始坐标系到校正后初始坐标系的旋转矩阵。
inline Eigen::Matrix3d make_correction_rotation(const DominantTilt & tilt)
{
  if (!std::isfinite(tilt.angle_rad)) {
    throw std::invalid_argument("Dominant tilt angle is not finite");
  }

  if (tilt.axis == "x") {
    return Eigen::AngleAxisd(
      tilt.angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  if (tilt.axis == "y") {
    return Eigen::AngleAxisd(
      tilt.angle_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
  }
  if (tilt.axis == "z") {
    return Eigen::AngleAxisd(
      tilt.angle_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  throw std::invalid_argument("Tilt axis must be 'x', 'y', or 'z'");
}

}  // namespace radar_tilt_calibration

#endif  // RADAR_TILT_CALIBRATION__TILT_ANGLES_HPP_
