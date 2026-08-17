#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <memory>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>

namespace vk {
namespace camera_loader {

bool loadFromRosNs(const std::string& ns, vk::AbstractCamera*& cam);

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
