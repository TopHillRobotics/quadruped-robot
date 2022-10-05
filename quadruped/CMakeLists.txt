cmake_minimum_required(VERSION 3.0.2)
project(quadruped LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "RELEASE") #"RELEASE", "Debug"
#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ADD_COMPILE_OPTIONS(-D_cplusplus)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        unitree_legged_msgs
        unitree_legged_sdk
        amd
        osqp
        qpOASES
        tinyekf
        quadprogpp
        tinynurbs
        nav_msgs
        tf
)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib amd osqp unitree_legged_msgs tinyekf quadprogpp tinynurbs unitree_legged_sdk
)

file(GLOB_RECURSE sources "src/*.cpp")
add_library(quadruped ${sources})

target_include_directories(quadruped PUBLIC include/quadruped/ config/ ${catkin_INCLUDE_DIRS})

target_link_libraries(quadruped PUBLIC ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES} Eigen3::Eigen)

add_dependencies(quadruped ${catkin_EXPORTED_TARGETS})

install(TARGETS quadruped
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/quadruped/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h+")

