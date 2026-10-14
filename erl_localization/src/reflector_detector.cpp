#include "erl_localization/reflector_detector.hpp"

namespace erl_localization {

std::vector<ReflectorCluster> ReflectorDetector::detect(
    const PointCloudFrame& frame) const {
  (void)frame;
  return {};
}

}  // namespace erl_localization
