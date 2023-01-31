cmake_minimum_required(VERSION 3.10)
project(quadruped LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "RELEASE") #"RELEASE", "Debug" 
set(CMAKE_CXX_FLAGS "-O3 -DEIGEN_NO_DEBUG -fopenmp ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
# message("CMAKE_CXX_FLAGS = " ${CMAKE_CXX_FLAGS})
ADD_COMPILE_OPTIONS(-D_cplusplus)
ADD_COMPILE_OPTIONS(-D_useros)

option(USE_GO1                            "WHICH ROBOT"                     OFF)
option(USE_IGL                            "use igl library"                 OFF)
option(ENABLE_TEST                        "UNIT TEST"                   OFF)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}") # x86_64, amd64, arm64
if(${ARCHITECTURE} STREQUAL "x86_64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB "unitree_legged_sdk_amd64.so")
elseif(${ARCHITECTURE} STREQUAL "amd64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB "unitree_legged_sdk_amd64.so")
elseif((${ARCHITECTURE} STREQUAL "arm64") OR (${ARCHITECTURE} STREQUAL "aarch64"))
    set(UNITREE_SDK_LIB "unitree_legged_sdk_arm64.so")
else()
    message(FATAL_ERROR "unrecongnized arch.")
endif()



set(includePath)
set(linkPath)
list(APPEND includePath "/usr/include/eigen3")

# eigen3
# find_package(Eigen3 REQUIRED)
# include_directories(
#     ${EIGEN3_INCLUDE_DIRS}
# )
#include_directories("/usr/include/eigen3")
## if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel") # GNU
#    set(BLA_VENDOR Intel10_64lp)
#    add_definitions(-DEIGEN_USE_MKL_ALL)
#    # set(BLA_VENDOR OpenBLAS)
#    # add_definitions(-DEIGEN_USE_BLAS)
## endif()
#find_package(BLAS REQUIRED)
#message(${BLAS_LIBRARIES})

# yaml
find_package(yaml-cpp REQUIRED)
list(APPEND includePath ${YAML_INCLUDE_DIRS})

# libigl, A LIBRARY FOR DISCRTE GEMOTRY
if(${USE_IGL})
    ADD_COMPILE_OPTIONS(-DIGL_STATIC_LIBRARY)
    option(LIBIGL_USE_STATIC_LIBRARY     "Use libIGL as static librarie" ON)
    option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
    option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
    find_package(LIBIGL REQUIRED)
    list(APPEND includePath ${LIBIGL_INCLUDE_DIRS}
                        /home/ascend/Documents/libigl/include
    )
endif()

# Unitree SDK
if(${USE_GO1})
    set(UNITREE_SDK_GO1_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_legged_sdk_3.5)
    add_subdirectory(${UNITREE_SDK_GO1_DIR})
    list(APPEND includePath "${UNITREE_SDK_GO1_DIR}/include")

    list(APPEND linkPath ${CMAKE_CURRENT_BINARY_DIR}/third_party/unitree_legged_sdk_3.5)
    list(APPEND linkPath ${UNITREE_SDK_GO1_DIR}/lib)
else() #A1 & AligenGo
    set(UNITREE_SDK_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_legged_sdk)
    add_subdirectory(${UNITREE_SDK_DIR})
    list(APPEND includePath "${UNITREE_SDK_DIR}/include")
    list(APPEND linkPath ${CMAKE_CURRENT_BINARY_DIR}/third_party/unitree_legged_sdk)
    list(APPEND linkPath ${UNITREE_SDK_DIR}/lib)
endif()

# TinyEKF
set(TinyEKF_DIR ${PROJECT_SOURCE_DIR}/third_party/TinyEKF)
list(APPEND includePath "${TinyEKF_DIR}/src")
file(GLOB_RECURSE TinyEKF_SRC "${TinyEKF_DIR}/src/*.c")               # TinyEKF library cpp files

# TinyNURBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/tinynurbs)

# Matplotlib-cpp
find_package(PythonLibs REQUIRED)
list(APPEND includePath ${PYTHON_INCLUDE_DIRS})
list(APPEND includePath ${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp)

# cnpy
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/cnpy)
list(APPEND includePath ${PROJECT_SOURCE_DIR}/third_party/cnpy)

# QuadProgpp
set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/third_party/QuadProgpp)
add_subdirectory(${QuadProgpp_DIR})
list(APPEND includePath "${QuadProgpp_DIR}/src")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/QuadProgpp)

# qpOASES
set(qpOASES_DIR ${PROJECT_SOURCE_DIR}/third_party/qpOASES)
add_subdirectory(${qpOASES_DIR})
list(APPEND includePath "${qpOASES_DIR}")
list(APPEND includePath "${qpOASES_DIR}/include")
# link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/qpOASES)

# osQP
set(osQP_DIR ${PROJECT_SOURCE_DIR}/third_party/osqp)
add_subdirectory(${osQP_DIR})
list(APPEND includePath "${osQP_DIR}/include")

# find_package(OsqpEigen REQUIRED)
# add_executable(example example.cpp)
# target_link_libraries(example OsqpEigen::OsqpEigen)

add_subdirectory("include/quadruped/utils/amd")

# xpp
set(XPP_DIR ${PROJECT_SOURCE_DIR}/../xpp)
list(APPEND includePath ${XPP_DIR}/xpp_states/include
                    ${XPP_DIR}/xpp_msgs/include)

# ROS
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        nav_msgs
        tf
        pcl_ros
        pcl_conversions
        unitree_legged_msgs
        roslib
        )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
                   geometry_msgs
                   std_msgs
                   sensor_msgs
                   nav_msgs
                   tf
                   pcl_ros
                   pcl_conversions
                   unitree_legged_msgs
                   roslib
)

list(APPEND includePath include/quadruped src)

###########
file(GLOB_RECURSE sources "src/*.cpp")       # common library cpp files

list(APPEND includePath "config/")                 # common includes

list(FILTER sources EXCLUDE REGEX "src/exec/.*.cpp")
list(APPEND sources ${PROJECT_SOURCE_DIR}/src/exec/runtime.cpp)

# create library
add_library(quadruped SHARED ${sources} ${TinyEKF_SRC})

target_include_directories(quadruped PUBLIC ${includePath} ${catkin_INCLUDE_DIRS})

target_link_directories(quadruped PUBLIC ${linkPath})

target_link_libraries(quadruped PUBLIC ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
        ${UNITREE_SDK_LIB} quadprog lcm xpp_states xpp_vis
        tinynurbs::tinynurbs cnpy qpOASES osqp MITAMD)

add_dependencies(quadruped ${catkin_EXPORTED_TARGETS})


if(${USE_GO1})
    target_link_libraries(quadruped PUBLIC robot_interface_3.5)
else() #A1 & AligenGo
    target_link_libraries(quadruped PUBLIC robot_interface)
endif()

#################################TEST#################################
if(${ENABLE_TEST})
    find_package(GTest REQUIRED)
    find_package(Threads REQUIRED)

    target_include_directories(${GTEST_INCLUDE_DIRS} ${GMOCK_INCLUDE_DIRS})

    # add_executable(Test_Robot tests/ut/test_robot.cpp ${sources} ${TinyEKF_SRC})
    # add_executable(Test_Gait tests/ut/test_openloop_gait_generator.cpp ${sources} ${TinyEKF_SRC})
    # add_executable(Test_SwingLeg tests/ut/test_swing_leg_controller.cpp ${sources} ${TinyEKF_SRC})
    # add_executable(Test_stance_controller tests/ut/test_torque_stance_leg_controller.cpp ${sources} ${TinyEKF_SRC})
    # add_executable(Test_qp_solver tests/ut/test_qp_torque_optimizer.cpp ${sources} ${TinyEKF_SRC})
    # add_executable(Test_vel_estimator tests/ut/test_robot_velocity_estimator.cpp ${sources} ${TinyEKF_SRC})
    # add_executable(Test_locomotion_controller tests/ut/test_locomotion_controller.cpp ${sources} ${TinyEKF_SRC})
    # add_executable(Test_AnomalyDetection tests/ut/test_anomaly_detection.cpp)
    add_executable(Test_FootHolder tests/ut/test_foothold.cpp)
    # add_executable(MyTest tests/ut/test_pose_planner.cpp)
    target_link_libraries(Test_FootHolder
                            quadruped
                            ${GTEST_BOTH_LIBRARIES}
                            gmock
                            ${CMAKE_THREAD_LIBS_INIT}
                        )
        
    # target_link_libraries(Test_Robot
    #        robot_runtime
    #         ${GTEST_BOTH_LIBRARIES}
    #         gmock
    #        ${CMAKE_THREAD_LIBS_INIT}
    #        )

    # target_link_libraries(Test_Gait
    #                     robot_runtime
    #                     ${GTEST_BOTH_LIBRARIES}
    #                     gmock
    #                     ${CMAKE_THREAD_LIBS_INIT}
    # )
    # target_link_libraries(Test_SwingLeg
    #         robot_runtime
    #         ${GTEST_BOTH_LIBRARIES}
    #         gmock
    #        ${CMAKE_THREAD_LIBS_INIT}
    # )
    # target_link_libraries(Test_stance_controller
    #                     robot_runtime
    #                     ${GTEST_BOTH_LIBRARIES}
    #                     gmock
    #                     ${CMAKE_THREAD_LIBS_INIT}
    # )
    # target_link_libraries(Test_qp_solver
    #                     robot_runtime
    #                     ${GTEST_BOTH_LIBRARIES}
    #                     gmock
    #                     ${CMAKE_THREAD_LIBS_INIT}
    # )
    # target_link_libraries(Test_vel_estimator
    #                     robot_runtime
    #                     ${GTEST_BOTH_LIBRARIES}
    #                     gmock
    #                     ${CMAKE_THREAD_LIBS_INIT}
    # )
    # target_link_libraries(Test_locomotion_controller
    #                     robot_runtime
    #                     ${GTEST_BOTH_LIBRARIES}
    #                     gmock
    #                     ${CMAKE_THREAD_LIBS_INIT}
    # )

    # add_test(Test Test_stance_controller)
    # add_test(Test Test_qp_solver)
    # add_test(Test Test_Robot)
    # add_test(Test Test_Gait)
    # add_test(Test Test_SwingLeg)
    # add_test(Test Test_vel_estimator)
    # add_test(Test Test_locomotion_controller)
    enable_testing()
endif()
