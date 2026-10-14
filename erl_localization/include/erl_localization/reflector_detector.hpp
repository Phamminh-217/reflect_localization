#pragma once

#include <cstddef>
#include <vector>

namespace erl_localization {

struct Point3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PointCloudFrame {
  std::vector<Point3D> points;
};

struct ReflectorCluster {
  std::size_t candidate_id{0};
  std::vector<Point3D> points_world;
};

class ReflectorDetector {
 public:
  std::vector<ReflectorCluster> detect(const PointCloudFrame& frame) const;
};

}  // namespace erl_localization
