cmake_minimum_required(VERSION 3.2)
project(flockingbird_tests)

set(SOURCE_FILES main.cpp src/divider_tests.cpp)

add_executable(flockingbird_tests ${SOURCE_FILES})

target_link_libraries(flockingbird_tests gtest_main)

install(TARGETS flockingbird_tests DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
