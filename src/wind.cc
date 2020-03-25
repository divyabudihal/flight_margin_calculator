#include <flight_margin/wind.hh>

#include <eigen3/Eigen/Geometry>
#include <util/conversion.hh>

using namespace flight_margin;

Wind::Wind(const Eigen::Vector2d& origin, const double& speed_ms, const double& direction_deg) : origin_(origin) {

// Rotate speed vector in the direction specified (cardinal direction)
Eigen::Rotation2D<double> rot2d(DegToRad(direction_deg));
vector_ = rot2d * Eigen::Vector2d(0.0, speed_ms);

}