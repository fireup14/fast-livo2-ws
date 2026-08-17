#ifndef VIKIT_MATH_UTILS_H_
#define VIKIT_MATH_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vk {

inline Eigen::Vector2d project2d(const Eigen::Vector3d& v) {
  return Eigen::Vector2d(v[0]/v[2], v[1]/v[2]);
}

inline Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
  return Eigen::Vector3d(v[0], v[1], 1.0);
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v[2], v[1],
       v[2], 0.0, -v[0],
      -v[1], v[0], 0.0;
  return m;
}

double triangulateFeatureNonLin(
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const Eigen::Vector3d& feature1,
    const Eigen::Vector3d& feature2,
    Eigen::Vector3d& p3d);

} // namespace vk

#endif // VIKIT_MATH_UTILS_H_
