cmake_minimum_required(VERSION 3.2)

set(HEADER_FILES flockingbird.h)

add_library(flockingbird INTERFACE ${HEADER_FILES})

project(flockingbird_demo
  VERSION 0.0.1
  LANGUAGES C CXX
  DESCRIPTION "Graphical test demo of flockingbird")

set(SOURCE_FILES main.cpp)

add_executable(flockingbird_demo ${SOURCE_FILES})

target_link_libraries(flockingbird_demo flockingbird)

install(TARGETS flockingbird_demo DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})



install(TARGETS flockingbird DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
