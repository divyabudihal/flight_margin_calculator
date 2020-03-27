#pragma once

#include <eigen3/Eigen/Core>

namespace flight_margin {

/**
 * @brief This is a container class for a wind measurement. It holds the origin
 * of a measurement and its velocity vector.
 * 
 */
class Wind {
 public:
  Wind() = default;
  Wind(const Eigen::Vector2d& origin, const double& speed_ms,
       const double& direction_deg);

  Eigen::Vector2d origin() const { return origin_; }
  Eigen::Vector2d velocity() const { return velocity_; }

 private:
  Eigen::Vector2d origin_;
  Eigen::Vector2d velocity_;
};

}  // namespace flight_margin
