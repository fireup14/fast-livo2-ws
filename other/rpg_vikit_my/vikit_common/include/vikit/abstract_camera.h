#ifndef VIKIT_ABSTRACT_CAMERA_H_
#define VIKIT_ABSTRACT_CAMERA_H_

#include <memory>
#include <string>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace vk {

class AbstractCamera {
public:
  AbstractCamera() : width_(0), height_(0) {}
  AbstractCamera(int width, int height) : width_(width), height_(height) {}
  virtual ~AbstractCamera() {}

  virtual Eigen::Vector3d cam2world(const double u, const double v) const = 0;
  virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const = 0;
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz) const = 0;
  virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const = 0;

  virtual double errorMultiplier2() const = 0;
  virtual double errorMultiplier1() const = 0;

  virtual double fx() const { return 0.0; }
  virtual double fy() const { return 0.0; }
  virtual double cx() const { return 0.0; }
  virtual double cy() const { return 0.0; }
  virtual double scale() const { return 1.0; }

  inline int width() const { return width_; }
  inline int height() const { return height_; }

  inline bool isInFrame(const Eigen::Vector2i& obs, int boundary=0) const
  {
    return obs[0] >= boundary && obs[0] < width_ - boundary &&
           obs[1] >= boundary && obs[1] < height_ - boundary;
  }

  inline bool isInFrame(const Eigen::Vector2i& obs, int boundary, int level) const
  {
    return obs[0] >= boundary && obs[0] < (width_ >> level) - boundary &&
           obs[1] >= boundary && obs[1] < (height_ >> level) - boundary;
  }

  virtual void undistortImage(const cv::Mat& raw, cv::Mat& rect) const {}

protected:
  int width_;
  int height_;
};

typedef std::shared_ptr<AbstractCamera> AbstractCameraPtr;

} // namespace vk

#endif // VIKIT_ABSTRACT_CAMERA_H_
