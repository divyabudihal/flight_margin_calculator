#include <gtest/gtest.h>

#include <flight_margin/flight_margin.hh>
#include <flight_margin/wind.hh>

#include <matplotlib/matplotlib.hh>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>

#define ERROR_TOL 0.0001

using namespace flight_margin;

TEST(FlightMarginTest, OnePath_NoWind) {
  auto config_file = "config/flight_margin.yaml";
  auto config = YAML::LoadFile(config_file);

  FlightMargin flight_margin(config);

  double initial_battery_Wh = 100;
  std::vector<Eigen::Vector2d> waypoints{
      Eigen::Vector2d(0, 0),
      Eigen::Vector2d(50.0 * sqrt(3.0) / 2.0, 50.0 * 1.0 / 2.0)};
  std::vector<Wind> winds;  // Empty wind vector
  double airspeed_mag_ms = 30;
  double required_power_Wh = 10;

  ASSERT_NEAR(
      flight_margin.RemainingBattery(initial_battery_Wh, waypoints, winds,
                                     airspeed_mag_ms, required_power_Wh),
      99.7222222, ERROR_TOL);
}

TEST(FlightMarginTest, OnePath_OnePerpendicularWind) {
  auto config_file = "config/flight_margin.yaml";
  auto config = YAML::LoadFile(config_file);

  FlightMargin flight_margin(config);

  double initial_battery_Wh = 100;
  std::vector<Eigen::Vector2d> waypoints{
      Eigen::Vector2d(0, 0),
      Eigen::Vector2d(50.0 * sqrt(3.0) / 2.0, 50.0 * 1.0 / 2.0)};
  std::vector<Wind> winds{
      Wind(Eigen::Vector2d(25, 25), 10, 210)};  // Empty wind vector
  double airspeed_mag_ms = 50;
  double required_power_Wh = 10;

  ASSERT_NEAR(
      flight_margin.RemainingBattery(initial_battery_Wh, waypoints, winds,
                                     airspeed_mag_ms, required_power_Wh),
      99.83333, ERROR_TOL);
}

TEST(FlightMarginTest, OnePath_OneWind) {
  auto config_file = "config/flight_margin.yaml";
  auto config = YAML::LoadFile(config_file);

  FlightMargin flight_margin(config);

  double initial_battery_Wh = 100;
  std::vector<Eigen::Vector2d> waypoints{
      Eigen::Vector2d(0, 0),
      Eigen::Vector2d(50.0 * sqrt(3.0) / 2.0, 50.0 * 1.0 / 2.0)};
  std::vector<Wind> winds{
      Wind(Eigen::Vector2d(25, 25), 10, 20)};  // Empty wind vector
  double airspeed_mag_ms = 10;
  double required_power_Wh = 10;

//   matplotlibcpp::quiver(std::vector<double>{1}, std::vector<double>{2}, std::vector<double>{1}, std::vector<double>{2});

  ASSERT_NEAR(
      flight_margin.RemainingBattery(initial_battery_Wh, waypoints, winds,
                                     airspeed_mag_ms, required_power_Wh),
      99.1666666, ERROR_TOL);
}