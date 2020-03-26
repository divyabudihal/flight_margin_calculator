#include <util/semivariogram.hh>

#include <cmath>
#include <iostream>

using namespace util;

Semivariogram::Semivariogram(const YAML::Node& config) {
  try {
    nugget_ = config["nugget"].as<double>();
    sill_ = config["sill"].as<double>();
    range_m_ = config["range_km_"].as<double>() * 1000;
  } catch (const YAML::Exception& e) {
    std::cout << "Semivariogram constructor failed with an exception: "
              << e.what();
  }
}

double Semivariogram::Calculate(const double& distance_m) const {
  // Calculate semivariogram value using a spherical model
  double value = nugget_ + (sill_ - nugget_) * ((3 / 2) * distance_m / range_m_ -
                                      (1 / 2) * pow(distance_m / range_m_, 3));
  return value;
}