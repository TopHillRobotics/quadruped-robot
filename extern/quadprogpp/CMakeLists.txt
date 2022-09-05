cmake_minimum_required(VERSION 3.0.2)
project(quadprogpp)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES quadprog
)

include_directories(include)

add_subdirectory(include/${PROJECT_NAME})

install(TARGETS quadprog
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/quadprogpp
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/quadprogpp
        FILES_MATCHING PATTERN "*.h+")
