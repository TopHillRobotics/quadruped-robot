cmake_minimum_required(VERSION 3.0.2)
project(navigation_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	rospy
	std_msgs
	message_generation
	controller_manager
	gazebo_ros
	move_base
	map_server
	amcl
	gmapping
	dwa_local_planner
)

catkin_package(
)

include_directories(
${catkin_INCLUDE_DIRS}
)

