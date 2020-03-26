#include <util/semivariogram.hh>

#include <cmath>
#include <iostream>

using namespace util;

/**
 * @brief This is the semivariogram constructor that takes in config parameters.
 *
 * @param config YAML config node with the parameters nugget, sill, and range_km
 * for the semivariogram
 *
 */
Semivariogram::Semivariogram(const YAML::Node& config) {
  try {
    nugget_ = config["nugget"].as<double>();
    sill_ = config["sill"].as<double>();
    range_m_ = config["range_km"].as<double>() * 1000;
  } catch (const YAML::Exception& e) {
    std::cout << "Semivariogram constructor failed with an exception: "
              << e.what();
  }
}

/**
 * @brief This function calculates the semivariogram for a given distance value.
 * The spherical semivariogram model is used in this calculation.
 *
 * @param distance_m Distance value to calculate semivariogram value.
 *
 * @return Semivariogram value for the given distance.
 *
 */
double Semivariogram::Calculate(const double& distance_m) const {
  // Calculate semivariogram value using the spherical model
  double value =
      nugget_ + (sill_ - nugget_) * ((3 / 2) * distance_m / range_m_ -
                                     (1 / 2) * pow(distance_m / range_m_, 3));
  return value;
}