cmake_minimum_required(VERSION 2.8.3)
project(unitree_gazebo)

set (CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(gazebo REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

catkin_package(
    CATKIN_DEPENDS unitree_legged_msgs
)

include_directories(
    # include
    ${Qt5Core_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(GaitParameterGuiPlugin SHARED plugin/gait_parameter_gui_plugin.cc)
target_link_libraries(GaitParameterGuiPlugin
${catkin_LIBRARIES}
${GAZEBO_LIBRARIES}
${Qt5Core_LIBRARIES}
${Qt5Widgets_LIBRARIES})

add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

