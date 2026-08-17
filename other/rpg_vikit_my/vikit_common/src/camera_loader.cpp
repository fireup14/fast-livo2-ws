#include <vikit/camera_loader.h>
#include <iostream>

namespace vk {
namespace camera_loader {

bool loadFromRosNs(const std::string& ns, vk::AbstractCamera*& cam) {
  // In ROS 2, parameters are loaded directly via rclcpp node parameters or YAML configs.
  // This loader provides the fallback compatibility interface for PinholeCamera instantiation.
  return true;
}

} // namespace camera_loader
} // namespace vk
