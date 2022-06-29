cmake_minimum_required(VERSION 3.10.2)
project(quadruped)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  unitree_legged_msgs
  extern
)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  include
)

## add all cpp files into QUADRUPED_SOURCE_FILES
file(GLOB_RECURSE QUADRUPED_SOURCE_FILES src/*.cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}
   ${QUADRUPED_SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIRECTORIES})

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES} Eigen3::Eigen)
