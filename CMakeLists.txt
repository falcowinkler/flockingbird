cmake_minimum_required(VERSION 3.2)

project(flockingbird
    VERSION 0.0.1
    LANGUAGES C CXX
    DESCRIPTION "Implementations of flocking simulation algorithms.")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

set(FLOCKINGBIRD_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(FLOCKINGBIRD_INSTALL_BUILD_DIR ${PROJECT_SOURCE_DIR}/build)

add_subdirectory(src)
add_subdirectory(tests)

# Add googletest (refer to https://github.com/google/googletest/blob/master/googletest/README.md)

configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)
