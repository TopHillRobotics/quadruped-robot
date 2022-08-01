cmake_minimum_required(VERSION 3.0.2)
project(quadruped LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "RELEASE") #"RELEASE", "Debug"
#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ADD_COMPILE_OPTIONS(-D_cplusplus)
#ADD_COMPILE_OPTIONS(-D_useros)

option(USE_GO1                            "WHICH ROBOT"                     OFF)
option(USE_IGL                            "use igl library"                 OFF)
option(ENABLE_TEST                        "UNIT TEST"                   OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        unitree_legged_msgs
        )

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp unitree_legged_msgs
)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}") # x86_64, amd64, arm64
if(${ARCHITECTURE} STREQUAL "x86_64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB "unitree_legged_sdk_amd64.so")
elseif(${ARCHITECTURE} STREQUAL "amd64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB "unitree_legged_sdk_amd64.so")
elseif(${ARCHITECTURE} STREQUAL "arm64")
    set(UNITREE_SDK_LIB "unitree_legged_sdk_arm64.so")
else()
    message(FATAL_ERROR "unrecongnized arch.")
endif()


set(UNITREE_SDK_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_legged_sdk)
add_subdirectory(${UNITREE_SDK_DIR})
include_directories("${UNITREE_SDK_DIR}/include")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/unitree_legged_sdk)
link_directories(${UNITREE_SDK_DIR}/lib)


# TinyEKF
set(TinyEKF_DIR ${PROJECT_SOURCE_DIR}/third_party/TinyEKF)
include_directories("${TinyEKF_DIR}/src")
file(GLOB_RECURSE TinyEKF_SRC "${TinyEKF_DIR}/src/*.c")

# TinyNURBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/tinynurbs)

# Matplotlib-cpp
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp)

# QuadProgpp
set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/third_party/QuadProgpp)
add_subdirectory(${QuadProgpp_DIR})
include_directories("${QuadProgpp_DIR}/src")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/QuadProgpp)


###########
file(GLOB_RECURSE sources "src/*.cpp")       # common library cpp files
#include_directories("include/quadruped/")     # common includes
#include_directories("config/")                 # common includes
#include_directories(${catkin_INCLUDE_DIRS})
add_library(quadruped ${sources} ${TinyEKF_SRC})

target_include_directories(quadruped PUBLIC
    include/quadruped/
    config/
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/third_party/cnpy
    ${UNITREE_SDK_DIR}/include
    ${TinyEKF_DIR}/src
    ${QuadProgpp_DIR}/src)

target_link_directories(quadruped PUBLIC ${UNITREE_SDK_DIR}/lib)

target_link_libraries(quadruped PUBLIC ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
    ${UNITREE_SDK_LIB} robot_interface quadprog lcm
    tinynurbs::tinynurbs Eigen3::Eigen)

add_dependencies(quadruped ${catkin_EXPORTED_TARGETS})

install(TARGETS quadruped
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/quadruped/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h+")

