#include <vikit/vision.h>

namespace vk {

void halfSample(const cv::Mat& in, cv::Mat& out) {
  cv::pyrDown(in, out);
}

} // namespace vk
