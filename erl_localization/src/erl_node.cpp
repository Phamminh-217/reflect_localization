#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "erl_localization/cylinder_fitter.hpp"
#include "erl_localization/pose_calculator.hpp"
#include "erl_localization/reflector_detector.hpp"

namespace {

bool loadConfigFile(const std::string& path, std::string& content) {
  std::ifstream input(path);
  if (!input.is_open()) {
    return false;
  }

  std::ostringstream buffer;
  buffer << input.rdbuf();
  content = buffer.str();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--config") {
    std::cerr << "Usage: erl_node --config <path-to-config/erl.yaml>\n";
    return 2;
  }

  const std::string config_path = argv[2];
  std::string config_content;
  if (!loadConfigFile(config_path, config_content)) {
    std::cerr << "erl_node: failed to load configuration: " << config_path << "\n";
    return 3;
  }

  erl_localization::ReflectorDetector detector;
  erl_localization::CylinderFitter fitter;
  erl_localization::PoseCalculator pose_calculator;

  const erl_localization::PointCloudFrame frame{};
  const auto clusters = detector.detect(frame);
  const auto fitted_reflectors = fitter.fit(clusters);
  const auto pose_result = pose_calculator.calculate(fitted_reflectors);

  std::cout << "erl_node skeleton ready"
            << " | config=" << config_path
            << " | config_bytes=" << config_content.size()
            << " | clusters=" << clusters.size()
            << " | fitted_reflectors=" << fitted_reflectors.size()
            << " | pose_valid=" << (pose_result.valid ? "true" : "false")
            << " | status=" << pose_result.status << '\n';

  return 0;
}
