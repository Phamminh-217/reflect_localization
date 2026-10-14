#pragma once

#include <cstddef>
#include <vector>

#include "erl_localization/reflector_detector.hpp"

namespace erl_localization {

struct FittedReflector {
  std::size_t candidate_id{0};
  Point3D center_world;
  double fitted_radius{0.0};
  double band_lower{0.0};
  double band_upper{0.0};
  double band_extent{0.0};
  bool geometry_valid{false};
  bool height_evidence_valid{false};
};

class CylinderFitter {
 public:
  std::vector<FittedReflector> fit(
      const std::vector<ReflectorCluster>& clusters) const;
};

}  // namespace erl_localization
