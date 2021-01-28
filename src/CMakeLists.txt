cmake_minimum_required(VERSION 3.2)
project(flockingbird_demo)

set(SOURCE_FILES main.cpp)

add_executable(flockingbird_demo ${SOURCE_FILES})

install(TARGETS flockingbird_demo DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
