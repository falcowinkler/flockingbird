project(flockingbird
    VERSION 0.0.1
    LANGUAGES C CXX
    DESCRIPTION "Implementations of flocking simulation algorithms.")

add_library (flockingbird flockingbird.cpp)

target_include_directories (flockingbird PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
