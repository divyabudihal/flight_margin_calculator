cmake_minimum_required(VERSION 2.8.3)
project(flight_margin_calculator)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED NO_MODULE)
# find_package(GLOG REQUIRED NO_MODULE)

include_directories(
  include
)

add_library(flight_margin 
  src/wind.cc
  src/flight_margin.cc
  src/semivariogram.cc
)

# add_executable(execute_mission 
#   src/calculate_flight_margin.cc
# )

# target_link_libraries(calculate_flight_margin PUBLIC flight_margin)

install()
