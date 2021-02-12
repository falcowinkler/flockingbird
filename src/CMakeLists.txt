cmake_minimum_required(VERSION 3.2)

project(flockingbird
  VERSION 0.0.1
  LANGUAGES C CXX
  DESCRIPTION "Implementations of flocking simulation algorithms.")

set(SOURCE_FILES ./)

add_library(flockingbird INTERFACE ./)

install(TARGETS flockingbird DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
