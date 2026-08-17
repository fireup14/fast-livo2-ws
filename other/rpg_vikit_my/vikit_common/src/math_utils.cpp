#include <vikit/math_utils.h>

namespace vk {

double triangulateFeatureNonLin(
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const Eigen::Vector3d& feature1,
    const Eigen::Vector3d& feature2,
    Eigen::Vector3d& p3d)
{
  Eigen::Matrix<double, 3, 2> A;
  A.col(0) = R * feature1;
  A.col(1) = -feature2;
  Eigen::Vector2d depth = A.householderQr().solve(-t);
  p3d = R * (feature1 * depth[0]) + t;
  return depth[0];
}

} // namespace vk
