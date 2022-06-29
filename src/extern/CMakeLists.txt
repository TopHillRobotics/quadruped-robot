cmake_minimum_required(VERSION 3.0.2)
project(extern)

find_package(catkin REQUIRED)

set(TinyEKF_DIR ${PROJECT_SOURCE_DIR}/TinyEKF)
set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/QuadProgpp)
set(Tinynurbs_DIR ${PROJECT_SOURCE_DIR}/tinynurbs)

catkin_package(
   INCLUDE_DIRS "${TinyEKF_DIR}/src" "${QuadProgdpp_DIR}/src" "${Tinynurbs_DIR}/include"
   LIBRARIES TinyEKF quadprog tinynurbs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 "${TinyEKF_DIR}/src"
 "${QuadProgpp_DIR}/src"
 "${Tinynurbs_DIR}/include"
 #${catkin_INCLUDE_DIRS}
)

add_library(TinyEKF INTERFACE)

add_subdirectory(QuadProgpp)
add_subdirectory(tinynurbs)

### Mark cpp header files for installation
 install(TARGETS quadprog TinyEKF tinynurbs
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

 install(DIRECTORY include/${PROJECT_NAME}/
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
         FILES_MATCHING PATTERN "*.h+")
