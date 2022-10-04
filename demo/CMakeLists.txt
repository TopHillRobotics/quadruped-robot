cmake_minimum_required(VERSION 3.0.2)
project(demo)

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

add_executable(demo_helloworld demo_helloworld/demo_helloworld.cpp)
add_executable(demo_trot_velocity demo_trot_velocity/demo_trot_velocity.cpp)
add_executable(demo_trot_velocity_mpc demo_trot_velocity_mpc/demo_trot_velocity_mpc.cpp)
add_executable(demo_trot_keyboard demo_trot_keyboard/demo_trot_keyboard.cpp)
add_executable(demo_trot_position demo_trot_position/demo_trot_position.cpp)
add_executable(demo_trot_joystick demo_trot_joystick/demo_trot_joystick.cpp)
add_executable(external_force_motion external_force_motion/external_force_motion.cpp)
add_executable(demo_publish_odom demo_publish_odom/demo_publish_odom.cpp)
add_executable(demo_slam_gmapping demo_slam_gmapping/demo_slam_gmapping.cpp)
add_executable(demo_slam_cartographer demo_slam_cartographer/demo_slam_cartographer.cpp)

target_link_libraries(demo_helloworld ${catkin_LIBRARIES})
target_link_libraries(demo_trot_velocity ${catkin_LIBRARIES})
target_link_libraries(demo_trot_velocity_mpc ${catkin_LIBRARIES})
target_link_libraries(demo_trot_keyboard ${catkin_LIBRARIES})
target_link_libraries(demo_trot_position ${catkin_LIBRARIES})
target_link_libraries(demo_trot_joystick ${catkin_LIBRARIES})
target_link_libraries(external_force_motion ${catkin_LIBRARIES})
target_link_libraries(demo_publish_odom ${catkin_LIBRARIES})
target_link_libraries(demo_slam_gmapping ${catkin_LIBRARIES})
target_link_libraries(demo_slam_cartographer ${catkin_LIBRARIES})
