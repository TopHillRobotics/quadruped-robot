cmake_minimum_required(VERSION 3.0.2)
project(a1sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
catkin_package(
   CATKIN_DEPENDS roscpp roslib quadruped
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  quadruped
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(trot_velocity_motion trot_velocity_motion/main.cpp)

target_link_libraries(trot_velocity_motion ${catkin_LIBRARIES})
