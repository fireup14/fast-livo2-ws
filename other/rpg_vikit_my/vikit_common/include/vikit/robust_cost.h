#ifndef VIKIT_ROBUST_COST_H_
#define VIKIT_ROBUST_COST_H_

#include <cmath>

namespace vk {
namespace robust_cost {

class HuberWeightFunction {
public:
  static inline double weight(double e, double k) {
    double abs_e = std::abs(e);
    if (abs_e <= k)
      return 1.0;
    return k / abs_e;
  }
};

class TukeyWeightFunction {
public:
  static inline double weight(double e, double k) {
    double abs_e = std::abs(e);
    if (abs_e > k)
      return 0.0;
    double d = 1.0 - (e / k) * (e / k);
    return d * d;
  }
};

} // namespace robust_cost
} // namespace vk

#endif // VIKIT_ROBUST_COST_H_
