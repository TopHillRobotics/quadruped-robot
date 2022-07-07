cmake_minimum_required(VERSION 3.0.2)
project(a1sim)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  quadruped
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(trot_velocity_motion trot_velocity_motion/main.cpp)

target_link_libraries(trot_velocity_motion ${YAML_CPP_LIBRARIES} Eigen3::Eigen ${catkin_LIBRARIES})
