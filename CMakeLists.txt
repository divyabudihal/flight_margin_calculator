cmake_minimum_required(VERSION 3.10.2)
project(flight_margin_calculator)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

find_package(PythonLibs 2.7)
find_package(GTest)

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
)

target_link_libraries(flight_margin ${PYTHON_LIBRARIES})
# add_executable(execute_mission 
#   src/calculate_flight_margin.cc
# )

add_executable(tests 
      test/flight_margin_test.cc
      test/main.cc)

add_test(all_tests tests COMMAND runtests)

# target_include_directories(tests PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(tests PUBLIC flight_margin ${GTEST_BOTH_LIBRARIES} pthread yaml-cpp)

add_compile_options(-g -DCMAKE_BUILD_TYPE=Debug -Wall -Wextra -Wpedantic)

# target_link_libraries(calculate_flight_margin PUBLIC flight_margin)
install()
