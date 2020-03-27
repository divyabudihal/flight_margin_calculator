#include <flight_margin/flight_plotter.hh>

namespace plt = matplotlibcpp;
using namespace flight_margin;

/**
 * @brief This constructor takes in data to be plotted about the flight.
 *
 * @param waypoints A vector of waypoints for the flight
 * @param winds A vector of Wind objects representing wind vectors in the area
 *
 */
FlightPlotter::FlightPlotter(const std::vector<Eigen::Vector2d>& waypoints,
                             const std::vector<Wind> winds) {
  // Store x and y coordinates of waypoints
  for (auto& waypoint : waypoints) {
    x_waypoints_.push_back(waypoint[0]);
    y_waypoints_.push_back(waypoint[1]);
  }

  // Store x and y components of origins anf velocities of wind data
  for (auto& wind : winds) {
    x_wind_data_origin_.push_back(wind.origin()[0]);
    y_wind_data_origin_.push_back(wind.origin()[1]);
    u_wind_data_velocity_.push_back(wind.velocity()[0]);
    v_wind_data_velocity_.push_back(wind.velocity()[1]);
  }
}

/**
 * @brief This function adds computation data to the plot including segment
 * vectors, along with intermittent windspeed and groundspeed vectors.
 *
 * @param start_segment Starting position of segment
 * @param end_segment Ending position of segment
 * @param segment_increment Airspeed distance travel vector of segment
 * @param wind_travel_ms Wind distance travel vector for segment
 * @param ground_travel_m Ground distance travel vector for segment
 * @param windspeed_ms Wind velocity vector for segment
 *
 */
void FlightPlotter::AddData(const Eigen::Vector2d& start_segment,
                            const Eigen::Vector2d& end_segment,
                            const Eigen::Vector2d& segment_increment,
                            const Eigen::Vector2d& wind_travel_m,
                            const Eigen::Vector2d& ground_travel_m,
                            const Eigen::Vector2d& windspeed_ms) {
  // Store x and y components of segment positions, segment velocities, wind
  // velocities, and ground velocities
  x_segment_start_.push_back(start_segment[0]);
  y_segment_start_.push_back(start_segment[1]);
  x_segment_end_.push_back(end_segment[0]);
  y_segment_end_.push_back(end_segment[1]);

  u_segment_travel_.push_back(segment_increment[0]);
  v_segment_travel_.push_back(segment_increment[1]);

  u_wind_travel_.push_back(wind_travel_m[0]);
  v_wind_travel_.push_back(wind_travel_m[1]);

  u_ground_travel_.push_back(ground_travel_m[0]);
  v_ground_travel_.push_back(ground_travel_m[1]);

  u_wind_velocity_.push_back(windspeed_ms[0]);
  v_wind_velocity_.push_back(windspeed_ms[1]);
}

/**
 * @brief This function plots all of the data provided to this class.
 *
 */
void FlightPlotter::Plot() const {
  plt::figure();
  plt::title(
      "Distance Travel vectors in the xy plane(m): Black: Air, Red: Ground, "
      "Green: Wind");
  plt::xlabel("x coordinate (m)");
  plt::ylabel("y coordinate (m)");
  double padding = 10;
  double min_x = *(std::min_element(x_waypoints_.begin(), x_waypoints_.end()));
  double max_x = *(std::max_element(x_waypoints_.begin(), x_waypoints_.end()));
  double min_y = *(std::min_element(y_waypoints_.begin(), y_waypoints_.end()));
  double max_y = *(std::max_element(y_waypoints_.begin(), y_waypoints_.end()));

  plt::xlim(min_x - padding, max_x + padding);
  plt::ylim(min_y - padding, max_y + padding);

  // Plot segments
  plt::quiver(x_segment_start_, y_segment_start_, u_segment_travel_,
              v_segment_travel_,
              {{"angles", "xy"}, {"scale_units", "xy"}, {"pivot", "tail"}});

  // Plot ground travel vectors
  plt::quiver(x_segment_start_, y_segment_start_, u_ground_travel_,
              v_ground_travel_,
              {{"angles", "xy"},
               {"scale_units", "xy"},
               {"color", "r"},
               {"pivot", "tail"}});

  // Plot wind travel vectors
  plt::quiver(x_segment_end_, y_segment_end_, u_wind_travel_, v_wind_travel_,
              {{"angles", "xy"},
               {"scale_units", "xy"},
               {"color", "g"},
               {"pivot", "tip"}});

  plt::figure();
  plt::title(
      "Wind velocity vectors (not to scale) in the xy plane(m): Blue: Wind "
      "data, Green: Wind velocity");
  plt::xlabel("x coordinate (m)");
  plt::ylabel("y coordinate (m)");

  // Plot wind data
  plt::quiver(x_wind_data_origin_, y_wind_data_origin_, u_wind_data_velocity_,
              v_wind_data_velocity_,
              {{"angles", "xy"},
               {"scale_units", "xy"},
               {"color", "b"},
               {"pivot", "tip"}});

  plt::quiver(x_segment_end_, y_segment_end_, u_wind_velocity_,
              v_wind_velocity_,
              {{"angles", "xy"},
               {"scale_units", "xy"},
               {"color", "g"},
               {"pivot", "tail"}});

  // Plot waypoints
  plt::show();
}
