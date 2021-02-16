cmake_minimum_required(VERSION 3.2)

set(SOURCE_FILES ./)

add_library(flockingbird INTERFACE ./)

install(TARGETS flockingbird DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
