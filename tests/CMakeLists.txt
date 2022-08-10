cmake_minimum_required(VERSION 3.2)
project(flockingbird_tests)

include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# TODO: Can we recursively add all tests? -
# Check for cmake function to discover all files?
set(SOURCE_FILES
  src/nearest_neighbors/nanoflann_exploratory_tests.cpp
  src/nearest_neighbors/visible_proximity_tests.cpp
  src/flock_simulation/flock_tests.cpp
  src/flock_simulation/rules_tests.cpp
  src/utility/vector_operations_tests.cpp
  src/flock_simulation/simulation_test.cpp)

add_executable(flockingbird_tests ${SOURCE_FILES})

add_test(flockingbird_tests, flockingbird_tests)

target_link_libraries(flockingbird_tests flockingbird gtest_main gmock_main)

install(TARGETS flockingbird_tests DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
