cmake_minimum_required(VERSION 3.2)
project(flockingbird_tests)

set(SOURCE_FILES main.cpp src)

add_executable(flockingbird_tests ${SOURCE_FILES})

add_test(flockingbird_tests, flockingbird_tests)

target_link_libraries(flockingbird_tests flockingbird gtest_main)

install(TARGETS flockingbird_tests DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
