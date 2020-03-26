#pragma once

#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace util
{
  class Semivariogram {
   public:
    Semivariogram() = delete;
    Semivariogram(const YAML::Node& config);
    
    double Calculate(const double& distance_m) const;
   
   private:
    double nugget_;
    double sill_;
    double range_m_;

  };
} // namespace util
