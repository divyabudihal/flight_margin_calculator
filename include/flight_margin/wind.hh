#pragma once

#include <eigen3/Eigen/Core>

namespace flight_margin {

class Wind {
 public:
  Wind() = default;
  Wind(const Eigen::Vector2d& origin, const double& speed_ms, const double& direction_deg);

  Eigen::Vector2d origin() const { return origin_; }
  Eigen::Vector2d vector() const { return vector_; }

 private:
  Eigen::Vector2d origin_;
  Eigen::Vector2d vector_;

};

}  // namespace flight_margin