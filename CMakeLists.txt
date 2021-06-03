cmake_minimum_required(VERSION 3.2)

project(flockingbird
  VERSION 0.0.1
  LANGUAGES C CXX
  DESCRIPTION "Implementations of flocking simulation algorithms.")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

set(FLOCKINGBIRD_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(FLOCKINGBIRD_INSTALL_BUILD_DIR ${PROJECT_SOURCE_DIR}/build)


# TODO if this is really needed, can we recursively inclued all headers?
set(FLOCKINGBIRD_HEADERS_DIR ${PROJECT_SOURCE_DIR}/src)

# TODO clarify, need this for the tests to work.
# even though i created a interface library?
include_directories(${FLOCKINGBIRD_HEADERS_DIR})

add_subdirectory(src)

# Tests
enable_testing()
add_subdirectory(tests)

# Add googletest (refer to https://github.com/google/googletest/blob/master/googletest/README.md)
