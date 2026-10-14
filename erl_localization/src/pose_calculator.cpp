#include "erl_localization/pose_calculator.hpp"

namespace erl_localization {

ErlPoseResult PoseCalculator::calculate(
    const std::vector<FittedReflector>& reflectors) const {
  (void)reflectors;
  return ErlPoseResult{};
}

}  // namespace erl_localization
