#pragma once

#include <string>
#include <vector>

#include "erl_localization/cylinder_fitter.hpp"

namespace erl_localization {

struct ErlPoseResult {
  double x_ref{0.0};
  double y_ref{0.0};
  double yaw_ref{0.0};
  bool valid{false};
  bool heading_resolved{false};
  std::string status{"NOT_IMPLEMENTED"};
};

class PoseCalculator {
 public:
  ErlPoseResult calculate(
      const std::vector<FittedReflector>& reflectors) const;
};

}  // namespace erl_localization
