cmake_minimum_required(VERSION 3.2)

set(HEADER_FILES flockingbird.h)

add_library(flockingbird INTERFACE ${HEADER_FILES})

install(TARGETS flockingbird DESTINATION ${FLOCKINGBIRD_INSTALL_BUILD_DIR})
