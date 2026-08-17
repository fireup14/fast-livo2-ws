#ifndef VIKIT_VISION_H_
#define VIKIT_VISION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>

namespace vk {

inline float interpolateMat_8u(const cv::Mat& mat, float u, float v) {
  int x = (int)u;
  int y = (int)v;
  float subpix_x = u - x;
  float subpix_y = v - y;
  float wx0 = 1.0f - subpix_x;
  float wy0 = 1.0f - subpix_y;

  if (x < 0 || x >= mat.cols - 1 || y < 0 || y >= mat.rows - 1)
    return 0.0f;

  const uint8_t* ptr = mat.ptr<uint8_t>(y) + x;
  return wy0 * (wx0 * ptr[0] + subpix_x * ptr[1]) +
         subpix_y * (wx0 * ptr[mat.step] + subpix_x * ptr[mat.step + 1]);
}

inline float shiTomasiScore(const cv::Mat& img, int u, int v, int halfbox_size = 4) {
  if (u - halfbox_size < 0 || u + halfbox_size >= img.cols ||
      v - halfbox_size < 0 || v + halfbox_size >= img.rows)
    return 0.0f;

  float dXX = 0.0f, dYY = 0.0f, dXY = 0.0f;

  for (int y = v - halfbox_size; y <= v + halfbox_size; ++y) {
    const uint8_t* ptr = img.ptr<uint8_t>(y);
    for (int x = u - halfbox_size; x <= u + halfbox_size; ++x) {
      float dx = (float)(ptr[x + 1] - ptr[x - 1]) * 0.5f;
      float dy = (float)(img.ptr<uint8_t>(y + 1)[x] - img.ptr<uint8_t>(y - 1)[x]) * 0.5f;
      dXX += dx * dx;
      dYY += dy * dy;
      dXY += dx * dy;
    }
  }

  float trace = dXX + dYY;
  float det = dXX * dYY - dXY * dXY;
  return (trace - std::sqrt(std::max(0.0f, trace * trace - 4.0f * det))) * 0.5f;
}

void halfSample(const cv::Mat& in, cv::Mat& out);

} // namespace vk

#endif // VIKIT_VISION_H_
