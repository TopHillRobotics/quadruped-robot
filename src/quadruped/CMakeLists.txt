cmake_minimum_required(VERSION 3.10.2)
project(quadruped)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  unitree_legged_msgs
  tinyekf
  tinynurbs
  quadprogpp
)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp unitree_legged_msgs tinyekf tinynurbs quadprogpp
)

## add all cpp files into QUADRUPED_SOURCE_FILES
file(GLOB_RECURSE QUADRUPED_SOURCE_FILES src/*.cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}
   ${QUADRUPED_SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} include/${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES} Eigen3::Eigen ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h+")
