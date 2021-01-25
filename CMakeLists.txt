cmake_minimum_required(VERSION 3.2)

project(flockingbird
    VERSION 0.0.1
    LANGUAGES C CXX
    DESCRIPTION "Implementations of flocking simulation algorithms."
    )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

set(FLOCKINGBIRD_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(FLOCKINGBIRD_INSTALL_BUILD_DIR ${PROJECT_SOURCE_DIR}/build)

add_subdirectory(src)
