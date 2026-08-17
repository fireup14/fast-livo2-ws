#ifndef VIKIT_PERFORMANCE_MONITOR_H_
#define VIKIT_PERFORMANCE_MONITOR_H_

#include <chrono>
#include <string>
#include <unordered_map>

namespace vk {

class Timer {
public:
  void start() { start_time_ = std::chrono::high_resolution_clock::now(); }
  void stop() { stop_time_ = std::chrono::high_resolution_clock::now(); }
  double getTimeSec() const {
    return std::chrono::duration<double>(stop_time_ - start_time_).count();
  }
  double getTimeMilliSec() const {
    return std::chrono::duration<double, std::milli>(stop_time_ - start_time_).count();
  }
private:
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::high_resolution_clock::time_point stop_time_;
};

} // namespace vk

#endif // VIKIT_PERFORMANCE_MONITOR_H_
