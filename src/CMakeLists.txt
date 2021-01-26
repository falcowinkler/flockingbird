cmake_minimum_required(VERSION 3.2)
project(flockingbird)

set(SOURCE_FILES main.cpp)

add_executable(flockingbird ${SOURCE_FILES})

install(TARGETS flockingbird DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
