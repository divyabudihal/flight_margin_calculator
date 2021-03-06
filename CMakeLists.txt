cmake_minimum_required(VERSION 3.10.2)
project(flight_margin_calculator)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

find_package(PythonLibs 2.7)
find_package(GTest)

add_compile_options(-g -Wall -Wextra -Wpedantic)
add_definitions (-DPROJECT_ROOT="${CMAKE_SOURCE_DIR}")
enable_testing()

include_directories(
  include
  ${GTEST_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(flight_margin 
  src/wind.cc
  src/flight_margin.cc
  src/semivariogram.cc
  src/flight_plotter.cc
)

target_link_libraries(flight_margin ${PYTHON_LIBRARIES})

add_executable(tests 
      test/flight_margin_test.cc
      test/main.cc)

add_test(all_tests tests COMMAND runtests)

target_link_libraries(tests PUBLIC flight_margin ${GTEST_BOTH_LIBRARIES} pthread yaml-cpp)

install()
