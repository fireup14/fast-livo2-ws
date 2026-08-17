#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "Eigen/Core"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

class LidarCameraProjectionCheck : public rclcpp::Node
{
public:
  LidarCameraProjectionCheck()
  : Node("lidar_camera_projection_check")
  {
    lidar_topic_ = declare_parameter<std::string>("common.lid_topic", "");
    image_topic_ = declare_parameter<std::string>("common.img_topic", "");
    const auto rcl = declare_parameter<std::vector<double>>("extrin_calib.Rcl", std::vector<double>{});
    const auto pcl = declare_parameter<std::vector<double>>("extrin_calib.Pcl", std::vector<double>{});
    width_ = declare_parameter<int>("camera.cam_width", 0);
    height_ = declare_parameter<int>("camera.cam_height", 0);
    fx_ = declare_parameter<double>("camera.cam_fx", 0.0);
    fy_ = declare_parameter<double>("camera.cam_fy", 0.0);
    cx_ = declare_parameter<double>("camera.cam_cx", 0.0);
    cy_ = declare_parameter<double>("camera.cam_cy", 0.0);
    d0_ = declare_parameter<double>("camera.cam_d0", 0.0);
    d1_ = declare_parameter<double>("camera.cam_d1", 0.0);
    d2_ = declare_parameter<double>("camera.cam_d2", 0.0);
    d3_ = declare_parameter<double>("camera.cam_d3", 0.0);
    min_range_ = declare_parameter<double>("projection.min_range", 0.6);
    max_range_ = declare_parameter<double>("projection.max_range", 30.0);
    point_radius_ = declare_parameter<int>("projection.point_radius", 2);
    point_step_ = declare_parameter<int>("projection.point_step", 3);
    test_img_time_offset_ = declare_parameter<double>("projection.img_time_offset", 0.0);

    if (rcl.size() != 9 || pcl.size() != 3 || fx_ <= 0.0 || fy_ <= 0.0 ||
      width_ != 1280 || height_ != 720 || min_range_ < 0.0 || max_range_ <= min_range_ ||
      point_step_ <= 0 || point_radius_ <= 0)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid calibration or projection parameters: Rcl=%zu Pcl=%zu width=%d height=%d fx=%f fy=%f",
        rcl.size(), pcl.size(), width_, height_, fx_, fy_);
      throw std::runtime_error("Invalid LiDAR-to-D405 projection parameters");
    }

    Rcl_ << rcl[0], rcl[1], rcl[2], rcl[3], rcl[4], rcl[5], rcl[6], rcl[7], rcl[8];
    Pcl_ << pcl[0], pcl[1], pcl[2];
    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    distortion_ = (cv::Mat_<double>(1, 5) << d0_, d1_, d2_, d3_, 0.0);

    RCLCPP_INFO(get_logger(), "LiDAR topic: %s", lidar_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Camera: %dx%d fx=%.12f fy=%.12f cx=%.12f cy=%.12f", width_, height_, fx_, fy_, cx_, cy_);
    RCLCPP_INFO(get_logger(), "Distortion: [%.12f, %.12f, %.12f, %.12f, 0.0]", d0_, d1_, d2_, d3_);
    RCLCPP_INFO(get_logger(), "Rcl:\n%.12f %.12f %.12f\n%.12f %.12f %.12f\n%.12f %.12f %.12f", rcl[0], rcl[1], rcl[2], rcl[3], rcl[4], rcl[5], rcl[6], rcl[7], rcl[8]);
    RCLCPP_INFO(get_logger(), "Pcl: [%.12f, %.12f, %.12f]", pcl[0], pcl[1], pcl[2]);
    RCLCPP_INFO(get_logger(), "projection.img_time_offset: %.3f s", test_img_time_offset_);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/lidar_projection/image", rclcpp::SensorDataQoS());
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarCameraProjectionCheck::lidarCallback, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarCameraProjectionCheck::imageCallback, this, std::placeholders::_1));
  }

private:
  void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_lidar_ = msg;
    latest_lidar_stamp_ = msg->header.stamp;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    livox_ros_driver2::msg::CustomMsg::SharedPtr lidar;
    rclcpp::Time lidar_stamp;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      lidar = latest_lidar_;
      lidar_stamp = latest_lidar_stamp_;
    }
    if (!lidar) {
      return;
    }
    projectAndPublish(msg, lidar, lidar_stamp);
  }

  void projectAndPublish(
    const sensor_msgs::msg::Image::SharedPtr &image_msg,
    const livox_ros_driver2::msg::CustomMsg::SharedPtr &lidar_msg,
    const rclcpp::Time &lidar_stamp)
  {
    cv_bridge::CvImagePtr input;
    try {
      input = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &exception) {
      RCLCPP_ERROR(get_logger(), "Unable to convert image to BGR8: %s", exception.what());
      return;
    }

    cv::Mat overlay = input->image;
    const size_t points_in = lidar_msg->points.size();
    size_t points_projected = 0;
    const cv::Mat zero_rotation = cv::Mat::zeros(3, 1, CV_64F);
    const cv::Mat zero_translation = cv::Mat::zeros(3, 1, CV_64F);

    for (size_t index = 0; index < points_in; index += static_cast<size_t>(point_step_)) {
      const auto &point = lidar_msg->points[index];
      // Retain the raw Livox fields for later time/intensity diagnostics.
      const auto reflectivity = point.reflectivity;
      const auto offset_time = point.offset_time;
      static_cast<void>(reflectivity);
      static_cast<void>(offset_time);
      const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
      const double range = point_lidar.norm();
      if (range < min_range_ || range > max_range_) {
        continue;
      }

      const Eigen::Vector3d point_camera = Rcl_ * point_lidar + Pcl_;
      if (point_camera.z() <= 0.0) {
        continue;
      }

      std::vector<cv::Point3d> object_points = {{point_camera.x(), point_camera.y(), point_camera.z()}};
      std::vector<cv::Point2d> image_points;
      cv::projectPoints(object_points, zero_rotation, zero_translation, camera_matrix_, distortion_, image_points);
      const int u = static_cast<int>(std::lround(image_points.front().x));
      const int v = static_cast<int>(std::lround(image_points.front().y));
      if (u < 0 || u >= overlay.cols || v < 0 || v >= overlay.rows) {
        continue;
      }

      const double normalized_depth = std::clamp(
        (point_camera.z() - min_range_) / (max_range_ - min_range_), 0.0, 1.0);
      const cv::Scalar color(
        255.0 * normalized_depth,
        255.0 * (1.0 - std::abs(2.0 * normalized_depth - 1.0)),
        255.0 * (1.0 - normalized_depth));
      cv::circle(overlay, cv::Point(u, v), point_radius_, color, cv::FILLED, cv::LINE_AA);
      ++points_projected;
    }

    const rclcpp::Time image_stamp(image_msg->header.stamp);
    const double dt_raw_ms = (image_stamp - lidar_stamp).seconds() * 1000.0;
    const double corrected_image_time = image_stamp.seconds() + test_img_time_offset_;
    const std::vector<std::string> labels = {
      "LiDAR->D405 Projection Check",
      "points_in: " + std::to_string(points_in),
      "points_projected: " + std::to_string(points_projected),
      "image_stamp: " + std::to_string(image_stamp.seconds()),
      "lidar_stamp: " + std::to_string(lidar_stamp.seconds()),
      "dt_raw: " + std::to_string(dt_raw_ms) + " ms",
      "offset_test: " + std::to_string(test_img_time_offset_ * 1000.0) + " ms",
      "corrected_image_time: " + std::to_string(corrected_image_time),
      "Rcl/Pcl loaded: OK", "camera params loaded: OK"};
    for (size_t index = 0; index < labels.size(); ++index) {
      cv::putText(overlay, labels[index], cv::Point(15, 30 + static_cast<int>(24 * index)),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      cv::putText(overlay, labels[index], cv::Point(15, 30 + static_cast<int>(24 * index)),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    image_pub_->publish(*cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg());

    const auto now = std::chrono::steady_clock::now();
    if (last_log_time_.time_since_epoch().count() == 0 || now - last_log_time_ >= std::chrono::seconds(1)) {
      RCLCPP_INFO(get_logger(), "input lidar points=%zu projected points=%zu dt_raw=%.1f ms", points_in, points_projected, dt_raw_ms);
      last_log_time_ = now;
    }
  }

  std::string lidar_topic_;
  std::string image_topic_;
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double d0_;
  double d1_;
  double d2_;
  double d3_;
  Eigen::Matrix3d Rcl_;
  Eigen::Vector3d Pcl_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  int point_step_;
  int point_radius_;
  double min_range_;
  double max_range_;
  double test_img_time_offset_;
  livox_ros_driver2::msg::CustomMsg::SharedPtr latest_lidar_;
  rclcpp::Time latest_lidar_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex data_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::chrono::steady_clock::time_point last_log_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCameraProjectionCheck>());
  rclcpp::shutdown();
  return 0;
}
