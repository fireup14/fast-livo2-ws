#ifndef VIKIT_HOMOGRAPHY_H_
#define VIKIT_HOMOGRAPHY_H_

#include <Eigen/Core>
#include <vector>

namespace vk {

class Homography {
public:
  Homography(const std::vector<Eigen::Vector2d>& pts1,
             const std::vector<Eigen::Vector2d>& pts2);
  Eigen::Matrix3d H;
};

} // namespace vk

#endif // VIKIT_HOMOGRAPHY_H_
