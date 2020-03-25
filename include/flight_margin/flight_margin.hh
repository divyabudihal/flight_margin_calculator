#pragma once

#include <flight_margin/wind.hh>

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>
#include <vector>

namespace flight_margin {

class FlightMargin {
 public:
  FlightMargin() = delete;
  FlightMargin(const YAML::Node& config);

  double RemainingBattery(const double& initial_battery_Wh,
                          const std::vector<Eigen::Vector2d>& waypoints,
                          const std::vector<Wind> winds,
                          const double& airspeed_mag_ms,
                          const double& required_power_Wh) const;

 private:
  double discretization_frequency_hz_;

  double PathFlightTime(const Eigen::Vector2d& start,
                        const Eigen::Vector2d& end, const double& airspeed,
                        const std::vector<Wind> winds) const;
  Eigen::Vector2d CalculateWindVector(const Eigen::Vector2d& position,
                                      const std::vector<Wind>& winds) const;
};

}  // namespace flight_margin
