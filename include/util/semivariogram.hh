#pragma once

#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace util {
/**
 * @brief This is a utility semivariogram class that computes semivariogram
 * values at certain distances using config parameters.
 *
 */
class Semivariogram {
 public:
  Semivariogram() = delete;
  Semivariogram(const YAML::Node& config);

  double Calculate(const double& distance_m) const;

 private:
  // Parameters for the semivariogram
  double nugget_;
  double sill_;
  double range_m_;
};
}  // namespace util
