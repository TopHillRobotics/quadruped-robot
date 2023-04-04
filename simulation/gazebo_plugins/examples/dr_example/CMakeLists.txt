# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

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

# Include custom utilities
include_directories(${PROJECT_BINARY_DIR}/utils)
link_directories(${PROJECT_BINARY_DIR}/utils)

# Domain randomization plugin example client
add_executable (dr_example dr_example.cc)
target_link_libraries(dr_example
    gap_msgs
    dr_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(dr_example
    gap_msgs
    dr_interface)