cmake_minimum_required(VERSION 3.0)

# Example client that uses our custom message definitions
# as an external library, when project is installed

# Link our custom message library
find_package(gap REQUIRED VERSION 1.5)
include_directories(${GAP_INCLUDE_DIRS})
link_directories(${GAP_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAP_CXX_FLAGS}")

# Gazebo 
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

# External example client
add_executable (external_example external_example.cc)
target_link_libraries(external_example
    gap_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
