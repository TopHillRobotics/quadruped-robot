# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

# Find packages
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

# Include and link Gazebo and SDF libraries
include_directories(${GAZEBO_INCLUDE_DIRS} ${SDF_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Include custom messages
include_directories(${PROJECT_BINARY_DIR}/msgs)
link_directories(${PROJECT_BINARY_DIR}/msgs)

# Header-only library for debug utilities
add_library(gap_debug INTERFACE)
target_include_directories(gap_debug INTERFACE . )

# Shared library for domain randomization plugin interface
add_library(dr_interface SHARED
    DRInterface.cc)
target_link_libraries(dr_interface
    gap_debug
    gap_msgs)
add_dependencies(dr_interface
    gap_debug
    gap_msgs)

# Install library
install(TARGETS dr_interface
  DESTINATION "${utils_lib_dest}")
install(FILES DRInterface.hh
  DESTINATION "${utils_include_dest}")

