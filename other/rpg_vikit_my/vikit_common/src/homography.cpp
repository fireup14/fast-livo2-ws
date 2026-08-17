#include <vikit/homography.h>

namespace vk {

Homography::Homography(const std::vector<Eigen::Vector2d>& pts1,
                       const std::vector<Eigen::Vector2d>& pts2)
{
  H.setIdentity();
}

} // namespace vk
