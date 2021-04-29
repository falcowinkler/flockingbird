cmake_minimum_required(VERSION 3.2)
project(flockingbird_tests)

# TODO: Can we recursively add all tests?
set(SOURCE_FILES
  src/nearest_neighbors/nanoflann_exploratory_tests.cpp
  src/nearest_neighbors/visible_proximity_tests.cpp
  src/flock_simulation/flock_tests.cpp
  src/flock_simulation/rules_tests.cpp
  src/utility/vector_operations_tests.cpp
  src/flock_simulation/simulation_test.cpp)

add_executable(flockingbird_tests ${SOURCE_FILES})

add_test(flockingbird_tests, flockingbird_tests)

target_link_libraries(flockingbird_tests flockingbird gtest_main)

install(TARGETS flockingbird_tests DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
