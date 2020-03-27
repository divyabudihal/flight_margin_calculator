#include <flight_margin/flight_margin.hh>

#include <eigen3/Eigen/Dense>

#include <iostream>

namespace plt = matplotlibcpp;
using namespace flight_margin;

/**
 * @brief Flight margin constructor
 *
 * @param config a YAML config node with the follow parameters:
 * discretization_frequency_hz, wind_radius_km, wind_max_neighbours
 *
 */
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

/**
 * @brief This function computes the remaining battery level after a flight
 * through the user set waypoints.
 *
 * @param inital_battery_Wh The initial battery level in Wh
 * @param waypoints The set of waypoints in the flight
 * @param winds A vector of wind objects representing winds in the are
 * @param airspeed_mag_ms Magnitude of the desired airspeed for flight in m/s
 * @param required_power_Wh Power required to fly at desired airspeed in Wh
 *
 * @return Remaining battery power in Wh after flight
 *
 */
double FlightMargin::RemainingBattery(
    const double& initial_battery_Wh,
    const std::vector<Eigen::Vector2d>& waypoints,
    const std::vector<Wind> winds, const double& airspeed_mag_ms,
    const double& required_power_Wh) const {
  double flight_time_s = 0;
  // Declare flight plotting tool
  FlightPlotter flight_plotter(waypoints, winds);

  // Pre-calculate the semivariogram values for the winds needed or the Ordinary
  // Kriging Model to be used to calculate wind vectors for each segment
  auto semivariogram_winds = WindSemivariogramMatrix(winds);

  for (unsigned int index = 1; index < waypoints.size(); index++) {
    flight_time_s += PathFlightTime(waypoints[index - 1], waypoints[index],
                                    airspeed_mag_ms, winds, semivariogram_winds, flight_plotter);
  }

  // Calculate energy usage based on time
  double energy_usage_Wh = required_power_Wh * flight_time_s * 60.0 / 3600.0;
  flight_plotter.Plot();
  return initial_battery_Wh - energy_usage_Wh;
}

/**
 * @brief This function calculates the flight time between a start and end
 * waypoint of a path. The discretization frequency is used to split the path
 * into segments and calculate the flight time for each segment.
 *
 * @param start_path Waypoint at start of path
 * @param end_path Waypoint at end of path
 * @param airspeed_mag_ms Desired airspeed magnitude in m/s
 * @param winds Vector of Wind objects representing winds in the area
 * @param semivariogram_winds A matrix of semivariogram values for all of the winds
 *
 * @return Flight time required ofr this path
 *
 */
double FlightMargin::PathFlightTime(const Eigen::Vector2d& start_path,
                                    const Eigen::Vector2d& end_path,
                                    const double& airspeed_mag_ms,
                                    const std::vector<Wind> winds,
                                    const Eigen::MatrixXd& semivariogram_winds,
                                    FlightPlotter& flight_plotter) const {
  // Approximate discretization of path into segments using airspeed an
  // discretization frequency

  // Calculate path vector and its distance
  Eigen::Vector2d path = end_path - start_path;
  double path_distance = path.norm();

  // Split path into segments using discretization frequency
  unsigned int N_segments =
      (path_distance / airspeed_mag_ms) * discretization_frequency_hz_;
  auto segment_increment = path / N_segments;
  double segment_distance = segment_increment.norm();
  auto unit_segment = segment_increment / segment_distance;
  auto segment_velocity = airspeed_mag_ms * unit_segment;

  // Initalize path flight time and starting segment position
  double path_flight_time_s = 0;
  auto start_segment = start_path;

  for (unsigned int i = 0; i < N_segments; i++) {
    auto end_segment = start_segment + segment_increment;

    // Declare distance travel vectors for plotting
    Eigen::Vector2d wind_travel_m, ground_travel_m;
    // Declare segment_flight_time
    double segment_flight_time;

    // Calculate windspeed vector
    auto windspeed_ms = CalculateWindVector(end_segment, winds, semivariogram_winds);

    // In order to keep airspeed constant,
    // Ground speed = segment air speed - wind speed
    auto groundspeed_ms = segment_velocity - windspeed_ms;

    if (windspeed_ms.norm() <= 0.001) {
      // Windspeed vector is 0
      // Groundspeed = airspeed
      wind_travel_m = Eigen::Vector2d(0, 0);
      ground_travel_m = segment_increment;

      segment_flight_time = segment_distance / airspeed_mag_ms;
    } else {
      // Calculate groundspeed and windspeed unit vectors
      auto unit_groundspeed_ms = groundspeed_ms / groundspeed_ms.norm();
      auto unit_windspeed_ms = windspeed_ms / windspeed_ms.norm();

      // Create a triangle with the segment, groundspeed, and windspeed vectors
      // Calculate angles opposite to the segment and groundspeed vectors
      double angle_segment_rad =
          acos(unit_groundspeed_ms.dot(unit_windspeed_ms));
      double angle_ground_rad = acos(unit_segment.dot(unit_windspeed_ms));

      // Use angles to calculate distance values
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

      // Calculate segment flight time
      segment_flight_time = ground_distance_m / groundspeed_ms.norm();

      // Calculate distance travel vectors
      wind_travel_m = segment_flight_time * windspeed_ms;
      ground_travel_m = ground_distance_m * unit_groundspeed_ms;
    }
    // Update path flight time
    path_flight_time_s += segment_flight_time;

    // Update start of next segment
    start_segment = end_segment;

    // Add flight plotting data
    flight_plotter.AddData(start_segment, end_segment, segment_increment,
                           wind_travel_m, ground_travel_m, windspeed_ms);
  }

  return path_flight_time_s;
}

/**
 * @brief This function calculates the equivalent wind vector at a given
 * position based on winds using the Ordinary Kriging model.
 *
 * @param position x and y position at which to calculate a wind vector
 * @param winds A vector of Wind objects representing the measured winds in the
 * area
 * @param semivariogram_winds A matrix of semivariogram values for all of the winds
 *
 * @return The computed wind vector for that location based on the Ordinary
 * Kriging model. Returns a 0 vector if no winds are present in the radius, and
 * the same wind vector if there is only one wind present.
 *
 */
Eigen::Vector2d FlightMargin::CalculateWindVector(
    const Eigen::Vector2d& position, const std::vector<Wind>& winds, const Eigen::MatrixXd& semivariogram_winds) const {
  // Assume no duplicated wind points
  // Use Kriging model to compute wind vector

  // Find all wind vectors within the wind radius and record the distance to
  // them. Use a tuple to store the distance, wind object, and index of wind object
  std::vector<std::tuple<double, Wind, unsigned int>> distance_to_wind;
  for (unsigned int i = 0; i < winds.size(); i++) {
    double distance = (winds[i].origin() - position).norm();
    if (distance <= wind_radius_m_) {
      distance_to_wind.push_back(std::make_tuple(distance, winds[i], i));
    }
  }

  // Check number of winds found in radius
  if (distance_to_wind.empty()) {
    // Return zero vector because there are no winds in the search radius
    return Eigen::Vector2d(0, 0);
  } else if (distance_to_wind.size() == 1) {
    // Return the only wind vector as there is no way to approximate the spatial
    // distribution of the winds
    auto wind = std::get<Wind>(distance_to_wind[0]);
    return wind.velocity();
  }

  // Sort the winds based on closest distance
  std::sort(distance_to_wind.begin(), distance_to_wind.end(),
            [](const auto& a, const auto& b) { return std::get<double>(a) < std::get<double>(b); });

  // Ensure only configured maximum number of neighbours are considered
  if (distance_to_wind.size() > wind_max_neighbours_) {
    // Erase additional wind vector neighbours so they are not considered
    distance_to_wind.erase(distance_to_wind.begin() + wind_max_neighbours_,
                           distance_to_wind.end());
  }

  unsigned int N_winds =
      distance_to_wind.size();  // Number of wind vectors to be considered

  // For the ordinary kriging model, C * W = D where:
  // C is an (N + 1) x (N + 1) matrix of semivariogram values corresponding to
  // distances between wind vectors with a Lagrangian multiplier W is a weight
  // matrix (to be solved for) D is a column vector of semivariogram values
  // corresponding to the position and distances to wind vectors For more
  // information, visit:
  // spatial-analyst.net/ILWIS/htm/ilwisapp/kriging_algorithm.htm
  
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_winds + 1, N_winds + 1);
  // Set last row and column to ones for Lagrangian multiplier
  C.row(N_winds).setOnes();
  C.col(N_winds).setOnes();

  // Fill in C matrix using semivariogram_winds matrix
  for (auto i = 0; i < N_winds; i++) {
    for (auto j = i + 1; j < N_winds; j++) {
      auto wi_index = std::get<unsigned int>(distance_to_wind[i]);
      auto wj_index = std::get<unsigned int>(distance_to_wind[j]);
      C(i, j) = semivariogram_winds(wi_index, wj_index);
      C(j, i) =  C(i, j);  // C is a symmetrical matrix
    }
  }

  // Fill in D column vector
  Eigen::VectorXd D(N_winds + 1);
  D(N_winds) = 1;

  int i = 0;
  for (auto& [distance, wind, index] : distance_to_wind) {
    D[i] = semivariogram_.Calculate(distance);
    i++;
  }

  // Calculate Weights
  Eigen::VectorXd W = C.colPivHouseholderQr().solve(D);

  // Apply Weights to each wind velocity and sum
  Eigen::Vector2d windspeed_ms(0, 0);
  i = 0;  // weight index
  for (auto& [distance, wind, index] : distance_to_wind) {
    windspeed_ms += W[i] * wind.velocity();
    i++;
  }

  return windspeed_ms;
}

/**
 * @brief This function calculates a symmetrical matrix of semivariogram values for the given winds.
 * 
 * @param winds A vector of Wind objects representing the winds in the area.
 * 
 * @return An NxN matrix where N is the number of winds with all of the semivariogram values
 * 
 */ 
Eigen::MatrixXd FlightMargin::WindSemivariogramMatrix(
    const std::vector<Wind> winds) const {
  auto N_winds = winds.size();
  Eigen::MatrixXd semivariogram_winds = Eigen::MatrixXd::Zero(N_winds, N_winds);

  // Fill matrix with semivariogram values
  for (unsigned int i = 0; i < N_winds; i++) {
    for (unsigned int j = i + 1; j < N_winds; j++) {
      double distance_ij = (winds[i].origin() - winds[j].origin()).norm();
      // Calculate semivariogram value for this pair
      double sv_value = semivariogram_.Calculate(distance_ij);
      semivariogram_winds(i, j) = sv_value;
      semivariogram_winds(j, i) = sv_value;
    }
  }
  return semivariogram_winds;
}
