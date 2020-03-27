#pragma once

#include <flight_margin/wind.hh>
#include <matplotlib/matplotlib.hh>

#include <eigen3/Eigen/Core>

namespace flight_margin {
/**
 * @brief This class plots flight data such as ground speed, wind speed, air
 * speed, and waypoints in a mission.
 *
 */

class FlightPlotter {
 public:
  FlightPlotter();
  FlightPlotter(const std::vector<Eigen::Vector2d>& waypoints,
                const std::vector<Wind> winds);

  void AddData(const Eigen::Vector2d& start_segment,
               const Eigen::Vector2d& end_segment,
               const Eigen::Vector2d& segment_increment,
               const Eigen::Vector2d& wind_travel_m,
               const Eigen::Vector2d& ground_travel_m,
               const Eigen::Vector2d& windspeed_ms);

  void Plot() const;

 private:
  // Flight Plotting
  std::vector<double> x_waypoints_;
  std::vector<double> y_waypoints_;

  // Wind data plotting
  std::vector<double> x_wind_data_origin_;
  std::vector<double> y_wind_data_origin_;
  std::vector<double> u_wind_data_velocity_;
  std::vector<double> v_wind_data_velocity_;

  // Segment travel plotting
  std::vector<double> x_segment_start_;
  std::vector<double> y_segment_start_;
  std::vector<double> x_segment_end_;
  std::vector<double> y_segment_end_;

  std::vector<double> u_segment_travel_;
  std::vector<double> v_segment_travel_;

  // Wind travel plotting
  std::vector<double> u_wind_travel_;
  std::vector<double> v_wind_travel_;

  // Ground travel plotting
  std::vector<double> u_ground_travel_;
  std::vector<double> v_ground_travel_;

  // Wind velocity plotting
  std::vector<double> u_wind_velocity_;
  std::vector<double> v_wind_velocity_;
};

}  // namespace flight_margin
