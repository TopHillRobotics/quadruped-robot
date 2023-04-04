# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_BINARY_DIR}/msgs
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PROJECT_BINARY_DIR}/msgs
)

# Scene example
add_executable (scene_example
  scene_example.cc
  utils.cc
  ObjectGrid.cc)
target_link_libraries(scene_example
  gap_msgs
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(scene_example
  gap_msgs)
