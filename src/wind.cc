#include <flight_margin/wind.hh>

#include <util/conversion.hh>

#include <eigen3/Eigen/Geometry>

using namespace flight_margin;
/**
 * @brief Wind object constructor. This collects wind information and stores it
 * as an origin and velocity vector.
 *
 * @param origin Origin of wind vector (location of measurement)
 * @param speed_ms Magnitude of wind speed in m/s
 * @param direction_deg Direction of wind in degrees from North (cardinal)
 *
 */
Wind::Wind(const Eigen::Vector2d& origin, const double& speed_ms,
           const double& direction_deg)
    : origin_(origin) {
  // Rotate speed vector in the direction specified (cardinal direction)
  Eigen::Rotation2D<double> rot2d(DegToRad(direction_deg));
  vector_ = rot2d * Eigen::Vector2d(0.0, speed_ms);
}