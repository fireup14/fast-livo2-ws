#ifndef VIKIT_PINHOLE_CAMERA_H_
#define VIKIT_PINHOLE_CAMERA_H_

#include <vikit/abstract_camera.h>

namespace vk {

class PinholeCamera : public AbstractCamera {
public:
  PinholeCamera(double width, double height,
                double fx, double fy,
                double cx, double cy,
                double d0=0.0, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);
  virtual ~PinholeCamera();

  virtual Eigen::Vector3d cam2world(const double u, const double v) const override;
  virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const override;
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz) const override;
  virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const override;

  virtual double errorMultiplier2() const override { return fx_; }
  virtual double errorMultiplier1() const override { return fx_; }

  virtual double fx() const override { return fx_; }
  virtual double fy() const override { return fy_; }
  virtual double cx() const override { return cx_; }
  virtual double cy() const override { return cy_; }
  virtual double scale() const override { return 1.0; }

  virtual void undistortImage(const cv::Mat& raw, cv::Mat& rect) const override;

  inline double d0() const { return d_[0]; }
  inline double d1() const { return d_[1]; }
  inline double d2() const { return d_[2]; }
  inline double d3() const { return d_[3]; }
  inline double d4() const { return d_[4]; }

  inline const cv::Mat& cvK() const { return cvK_; }
  inline const cv::Mat& cvD() const { return cvD_; }

private:
  double fx_, fy_, cx_, cy_;
  double d_[5];
  cv::Mat cvK_;
  cv::Mat cvD_;
  bool distortion_;

  void initUnistortionMap();
};

} // namespace vk

#endif // VIKIT_PINHOLE_CAMERA_H_
