#pragma once

#include <flight_margin/flight_plotter.hh>
#include <flight_margin/wind.hh>
#include <util/semivariogram.hh>

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>

#include <vector>

namespace flight_margin {

/**
 * Flight Margin Calculator Class
 *
 * @brief This class uses calculates remaining battery life for a set of
 * waypoints using the desired airspeed and wind vectors
 *
 */
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
  // Config variables
  double discretization_frequency_hz_;
  double wind_radius_m_;
  unsigned int wind_max_neighbours_;

  // Utility semivariogram class
  util::Semivariogram semivariogram_;

  // Calculations
  double PathFlightTime(const Eigen::Vector2d& start,
                        const Eigen::Vector2d& end, const double& airspeed,
                        const std::vector<Wind> winds,
                        const Eigen::MatrixXd& semivariogram_winds,
                        FlightPlotter& flight_plotter) const;
  Eigen::Vector2d CalculateWindVector(
      const Eigen::Vector2d& position, const std::vector<Wind>& winds,
      const Eigen::MatrixXd& semivariogram_winds) const;
  Eigen::MatrixXd WindSemivariogramMatrix(const std::vector<Wind> winds) const;
};

}  // namespace flight_margin
