#include <flight_margin/flight_margin.hh>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <matplotlib/matplotlib.hh>

namespace plt = matplotlibcpp;

using namespace flight_margin;

FlightMargin::FlightMargin(const YAML::Node& config)
    : semivariogram_(config["semivariogram"]) {
  try {
    discretization_frequency_hz_ =
        config["discretization_frequency_hz"].as<double>();
    wind_radius_m_ = config["wind_radius_km"].as<double>() * 1000;
    wind_max_neighbours_ = config["wind_max_neighbours"].as<int>();
  } catch (const YAML::Exception& e) {
    std::cout
        << "Flight Margin Calculator constructor failed with an exception: "
        << e.what();
  }
}

Eigen::Vector2d FlightMargin::CalculateWindVector(
    const Eigen::Vector2d& position, const std::vector<Wind>& winds) const {
  // Assume no duplicated wind points

  // Use Kriging model to compute wind vector
  std::map<double, Wind> distance_to_wind;
  for (auto const& wind : winds) {
    double distance = (wind.origin() - position).norm();
    if (distance <= wind_radius_m_) {
      distance_to_wind[distance] = wind;
    }
  }

  if (distance_to_wind.empty()) {
    // Return zero vector because there are no winds in the search radius
    return Eigen::Vector2d(0, 0);
  } else if (distance_to_wind.size() == 1) {
    // Return the only wind vector as there is no way to approximate the
    // distribution
    auto wind = distance_to_wind.begin()->second;
    return wind.vector();
  }

  // Ensure only configured maximum number of neighbours are considered
  if (distance_to_wind.size() > wind_max_neighbours_) {
    auto it = distance_to_wind.begin();
    std::advance(it, wind_max_neighbours_);
    distance_to_wind.erase(it, distance_to_wind.end());
  }

  unsigned int N_winds = distance_to_wind.size();

  // For the kriging model, C * W = D where:
  // C is an (N + 1) x (N + 1) matrix of semivariogram values corresponding to
  // distances between wind vectors with a Lagrangian multiplier W is a weight
  // matrix (to be solved for) D is a column vector of semivariogram values
  // corresponding to the position and distances to wind vectors For more
  // information, visit:
  // spatial-analyst.net/ILWIS/htm/ilwisapp/kriging_algorithm.htm

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_winds + 1, N_winds + 1);
  C(N_winds) = 1;  // Lagrangian multiplier

  // Fill in C matrix
  int i = 0;
  for (auto w_i = distance_to_wind.begin(); w_i != distance_to_wind.end();
       w_i++) {
    int j = i + 1;
    for (auto w_j = std::next(w_i); w_j != distance_to_wind.end(); w_j++) {
      double distance_ij = (w_i->second.origin() - w_j->second.origin()).norm();
      // Calculate semivariogram value for this pair
      double sv_value = semivariogram_.Calculate(distance_ij);
      C(i, j) = sv_value;
      C(j, i) = sv_value;
      j++;
    }
    i++;
  }

  // Fill in D column vector
  Eigen::VectorXd D(N_winds + 1);
  D(N_winds) = 1;

  i = 0;
  for (auto& [distance, wind] : distance_to_wind) {
    D[i] = semivariogram_.Calculate(distance);
    i++;
  }

  // Calculate Weights
  Eigen::VectorXd W = C.colPivHouseholderQr().solve(D);

  // Apply Weights
  Eigen::Vector2d wind_speed_ms(0, 0);

  i = 0;
  for (auto& [distance, wind] : distance_to_wind) {
    wind_speed_ms += W[i] * wind.vector();
    i++;
  }
  plt::quiver(
      std::vector<double>{position[0]}, std::vector<double>{position[1]},
      std::vector<double>{position[0]}, std::vector<double>{position[1]});
  return wind_speed_ms;
}

double FlightMargin::PathFlightTime(const Eigen::Vector2d& start_path,
                                    const Eigen::Vector2d& end_path,
                                    const double& airspeed_mag_ms,
                                    const std::vector<Wind> winds) const {
  // Approximate discretization of path into segments using airspeed an
  // discretization frequency
  Eigen::Vector2d path = end_path - start_path;
  double path_distance = path.norm();
  unsigned int num_segments =
      (path_distance / airspeed_mag_ms) / discretization_frequency_hz_;
  auto segment_increment = path / num_segments;
  double segment_distance = segment_increment.norm();
  auto unit_segment = segment_increment / segment_distance;

  double path_flight_time_s = 0;
  auto start_segment = start_path;
  for (int i = 0; i < num_segments; i++) {
    auto end_segment = start_segment + segment_increment;
    auto windspeed_ms = CalculateWindVector(end_segment, winds);

    // ground speed = air speed + wind speed
    auto groundspeed_ms = airspeed_mag_ms * unit_segment + windspeed_ms;

    if (windspeed_ms.norm() <= 0.001) {
      // Windspeed vector is 0
      // Groundspeed = airspeed
      path_flight_time_s += segment_distance / airspeed_mag_ms;
    } else {
      auto unit_groundspeed_ms = groundspeed_ms / groundspeed_ms.norm();
      auto unit_windspeed_ms = windspeed_ms / windspeed_ms.norm();

      // Create a triangle with the segment, groundspeed, and windspeed vectors
      // Calculate angles opposite to the segment and groundspeed vectors
      double angle_segment_rad =
          acos(unit_groundspeed_ms.dot(unit_windspeed_ms));
      double angle_ground_rad = acos(unit_segment.dot(unit_windspeed_ms));

      // Use angles to calculate ground distance
      double ground_distance_m;
      if (abs(angle_segment_rad - M_PI / 2.0) < 0.001) {
        // Segment is the hypotenuse of a right triangle
        ground_distance_m = segment_distance * sin(angle_ground_rad);
      } else if (abs(angle_ground_rad - M_PI / 2.0) < 0.001) {
        // Ground distance in the hypotenuse of a right triangle
        ground_distance_m = segment_distance / sin(angle_segment_rad);
      } else {
        // Use sine law to calculate groundspeed distance magnitude
        ground_distance_m =
            sin(angle_ground_rad) * segment_distance / sin(angle_segment_rad);
      }

      // Calculate angle between groundspeed vector and airspeed vector
      path_flight_time_s += ground_distance_m / groundspeed_ms.norm();
    }
  }
  return path_flight_time_s;
}

double FlightMargin::RemainingBattery(
    const double& initial_battery_Wh,
    const std::vector<Eigen::Vector2d>& waypoints,
    const std::vector<Wind> winds, const double& airspeed_mag_ms,
    const double& required_power_Wh) const {
  double flight_time_s = 0;

  for (int index = 1; index < waypoints.size(); index++) {
    flight_time_s += PathFlightTime(waypoints[index - 1], waypoints[index],
                                    airspeed_mag_ms, winds);
  }

  double energy_usage_Wh = required_power_Wh * flight_time_s * 60.0 / 3600.0;
  plt::show();
  return initial_battery_Wh - energy_usage_Wh;
}