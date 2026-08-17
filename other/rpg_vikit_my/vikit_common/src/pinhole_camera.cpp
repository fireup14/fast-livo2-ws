#include <vikit/pinhole_camera.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace vk {

PinholeCamera::PinholeCamera(double width, double height,
                             double fx, double fy,
                             double cx, double cy,
                             double d0, double d1, double d2, double d3, double d4)
  : AbstractCamera(width, height),
    fx_(fx), fy_(fy), cx_(cx), cy_(cy),
    distortion_(d0 != 0.0 || d1 != 0.0 || d2 != 0.0 || d3 != 0.0 || d4 != 0.0)
{
  d_[0] = d0; d_[1] = d1; d_[2] = d2; d_[3] = d3; d_[4] = d4;
  cvK_ = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cvD_ = (cv::Mat_<double>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
}

PinholeCamera::~PinholeCamera() {}

Eigen::Vector3d PinholeCamera::cam2world(const double u, const double v) const {
  Eigen::Vector3d xyz;
  if (!distortion_) {
    xyz[0] = (u - cx_) / fx_;
    xyz[1] = (v - cy_) / fy_;
    xyz[2] = 1.0;
  } else {
    cv::Mat pt(1, 1, CV_64FC2);
    pt.at<cv::Vec2d>(0, 0) = cv::Vec2d(u, v);
    cv::Mat pt_undist;
    cv::undistortPoints(pt, pt_undist, cvK_, cvD_);
    xyz[0] = pt_undist.at<cv::Vec2d>(0, 0)[0];
    xyz[1] = pt_undist.at<cv::Vec2d>(0, 0)[1];
    xyz[2] = 1.0;
  }
  return xyz.normalized();
}

Eigen::Vector3d PinholeCamera::cam2world(const Eigen::Vector2d& px) const {
  return cam2world(px[0], px[1]);
}

Eigen::Vector2d PinholeCamera::world2cam(const Eigen::Vector3d& xyz) const {
  Eigen::Vector2d uv;
  if (xyz[2] <= 0) return Eigen::Vector2d(-1, -1);
  double x = xyz[0] / xyz[2];
  double y = xyz[1] / xyz[2];
  if (!distortion_) {
    uv[0] = fx_ * x + cx_;
    uv[1] = fy_ * y + cy_;
  } else {
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double cdist = 1.0 + d_[0] * r2 + d_[1] * r4 + d_[4] * r6;
    double xd = x * cdist + 2.0 * d_[2] * x * y + d_[3] * (r2 + 2.0 * x * x);
    double yd = y * cdist + d_[2] * (r2 + 2.0 * y * y) + 2.0 * d_[3] * x * y;
    uv[0] = fx_ * xd + cx_;
    uv[1] = fy_ * yd + cy_;
  }
  return uv;
}

Eigen::Vector2d PinholeCamera::world2cam(const Eigen::Vector2d& uv) const {
  return uv;
}

void PinholeCamera::undistortImage(const cv::Mat& raw, cv::Mat& rect) const {
  if (distortion_) {
    cv::undistort(raw, rect, cvK_, cvD_);
  } else {
    rect = raw.clone();
  }
}

} // namespace vk
