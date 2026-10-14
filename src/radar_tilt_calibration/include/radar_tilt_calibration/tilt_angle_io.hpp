#ifndef RADAR_TILT_CALIBRATION__TILT_ANGLE_IO_HPP_
#define RADAR_TILT_CALIBRATION__TILT_ANGLE_IO_HPP_

#include "radar_tilt_calibration/tilt_angles.hpp"

#include <filesystem>
#include <string>

namespace radar_tilt_calibration
{

// 读取和写入统一的 angles_deg.x/y/z 文件格式。
// 相对路径统一相对于源码包 src/radar_tilt_calibration 解析；绝对路径保持不变。
std::filesystem::path resolve_tilt_angle_path(const std::string & value);

TiltAnglesDeg load_tilt_angles_deg(const std::filesystem::path & path);

void write_tilt_angles_deg(
  const std::filesystem::path & path,
  const TiltAnglesDeg & angles);

}  // namespace radar_tilt_calibration

#endif  // RADAR_TILT_CALIBRATION__TILT_ANGLE_IO_HPP_
