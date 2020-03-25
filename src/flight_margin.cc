#include <flight_margin/flight_margin.hh>

#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace flight_margin;

FlightMargin::FlightMargin(const YAML::Node& config) {

  try {
    discretization_frequency_hz_ = config["discretization_frequency_hz"].as<double>();
  }
  catch (const YAML::Exception& e) { 
    // LOG(ERROR) << "Flight Margin Calculator constructor failed with exception: " << e.what(); 
    std::cout << "Flight Margin Calculator constructor failed with exception: " << e.what(); 
  }
}

Eigen::Vector2d FlightMargin::CalculateWindVector(const Eigen::Vector2d& position, const std::vector<Wind>& winds) const {
  return Eigen::Vector2d(5, 6);
}

double FlightMargin::PathFlightTime(const Eigen::Vector2d& start_path, const Eigen::Vector2d& end_path, const double& airspeed_mag_ms, const std::vector<Wind> winds) const {
  // Approximate discretization of path into segments using airspeed an discretization frequency
  Eigen::Vector2d path = end_path - start_path;
  double path_distance = path.norm();
  unsigned int num_segments = (path_distance / airspeed_mag_ms) / discretization_frequency_hz_;
  auto segment_increment =  path / num_segments;
  double segment_distance = segment_increment.norm();
  auto unit_segment = segment_increment / segment_distance;

  double path_flight_time = 0;
  auto start_segment = start_path;
  for (int i = 0; i < num_segments; i++) {
    auto end_segment = start_segment + segment_increment;
    auto windspeed_ms = CalculateWindVector(end_segment, winds);

    // ground speed = air speed + wind speed
    auto groundspeed_ms = airspeed_mag_ms * unit_segment + windspeed_ms;

    // Use sine law to calculate groundspeed distance magnitude
    // Let A = ground , B = segment, C = wind
    // Calculate angle betwee airspeed vector and windspeed vector

    auto unit_groundspeed_ms = groundspeed_ms / groundspeed_ms.norm();
    auto unit_windspeed_ms = windspeed_ms / windspeed_ms.norm();

    double angle_segment_rad = acos(unit_groundspeed_ms.dot(unit_windspeed_ms));
    double angle_ground_rad = acos(unit_segment.dot(unit_windspeed_ms));

    double ground_distance_m = angle_ground_rad * segment_distance / angle_segment_rad;

    // Calculate angle between groundspeed vector and airspeed vector
    path_flight_time += ground_distance_m / groundspeed_ms.norm();
  }




}

double FlightMargin::RemainingBattery(const double& initial_battery_Wh,
                          const std::vector<Eigen::Vector2d>& waypoints,
                          const std::vector<Wind> winds, const double& airspeed_mag_ms,
                          const double& required_power_Wh) const {
                            
                            double flight_time = 0;

                            // Discretize paths between waypoints based on discretization frequency
                            for (int index = 1; index < waypoints.size(); index++) {
                              flight_time += PathFlightTime(waypoints[index - 1] , waypoints[index], airspeed_mag_ms, winds);
                            }



                          }