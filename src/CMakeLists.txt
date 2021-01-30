cmake_minimum_required(VERSION 3.2)

set(SOURCE_FILES ./)

add_library(flockingbird ${SOURCE_FILES})

install(TARGETS flockingbird DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
