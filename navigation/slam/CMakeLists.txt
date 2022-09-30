cmake_minimum_required(VERSION 3.0.2)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  visualization_msgs
)

catkin_package(
)

include_directories(
${catkin_INCLUDE_DIRS}
)

