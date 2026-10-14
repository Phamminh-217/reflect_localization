#include "erl_localization/cylinder_fitter.hpp"

namespace erl_localization {

std::vector<FittedReflector> CylinderFitter::fit(
    const std::vector<ReflectorCluster>& clusters) const {
  (void)clusters;
  return {};
}

}  // namespace erl_localization
