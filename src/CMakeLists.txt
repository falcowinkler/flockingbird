cmake_minimum_required(VERSION 3.2)

set(HEADER_FILES flockingbird.h)

add_library(flockingbird INTERFACE ${HEADER_FILES})

project(flockingbird_demo
  VERSION 0.0.1
  LANGUAGES C CXX
  DESCRIPTION "Graphical test demo of flockingbird")

set(SOURCE_FILES main.cpp)

INCLUDE(FindPkgConfig)

PKG_CHECK_MODULES(CAIRO REQUIRED cairo)
pkg_check_modules(GTKMM gtkmm-2.4)

add_executable(flockingbird_demo ${SOURCE_FILES})
target_compile_features(flockingbird_demo PRIVATE cxx_std_17)

TARGET_INCLUDE_DIRECTORIES(
  flockingbird_demo
  PRIVATE
	${CAIRO_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
  )

target_link_directories(
  flockingbird_demo
  PRIVATE
  ${CAIRO_LIBRARY_DIRS}
  ${GTKMM_LIBRARY_DIRS}
)

target_link_libraries(
  flockingbird_demo
  flockingbird
  ${CAIRO_LIBRARIES}
  ${GTKMM_LIBRARIES}
)

install(TARGETS flockingbird_demo DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})



install(TARGETS flockingbird DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
