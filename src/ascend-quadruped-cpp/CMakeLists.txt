cmake_minimum_required(VERSION 3.10)
project(ascend_quadruped_cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "RELEASE") #"RELEASE", "Debug"
#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ADD_COMPILE_OPTIONS(-D_cplusplus)
ADD_COMPILE_OPTIONS(-D_useros)

option(USE_GO1                            "WHICH ROBOT"                     OFF)
option(USE_IGL                            "use igl library"                 OFF)
option(ENABLE_TEST                            "UNIT TEST"                   OFF)

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

# eigen3
include_directories("/usr/include/eigen3")

# yaml
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

# libigl, A LIBRARY FOR DISCRTE GEMOTRY
if(${USE_IGL})
    ADD_COMPILE_OPTIONS(-DIGL_STATIC_LIBRARY)
    option(LIBIGL_USE_STATIC_LIBRARY     "Use libIGL as static librarie" ON)
    option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
    option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
    find_package(LIBIGL REQUIRED)
    include_directories(${LIBIGL_INCLUDE_DIRS} 
                        /home/ascend/Documents/libigl/include
    )
endif()

# Unitree SDK
if(${USE_GO1})
    set(UNITREE_SDK_GO1_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_legged_sdk_3.4)
    add_subdirectory(${UNITREE_SDK_GO1_DIR})
    include_directories("${UNITREE_SDK_GO1_DIR}/include")
    link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/unitree_legged_sdk_3.4)
    link_directories(${UNITREE_SDK_GO1_DIR}/lib)
else() #A1 & AligenGo
    set(UNITREE_SDK_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_legged_sdk)
    add_subdirectory(${UNITREE_SDK_DIR})
    include_directories("${UNITREE_SDK_DIR}/include")
    link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/unitree_legged_sdk)
    link_directories(${UNITREE_SDK_DIR}/lib)
endif()

# TinyEKF
set(TinyEKF_DIR ${PROJECT_SOURCE_DIR}/third_party/TinyEKF)
include_directories("${TinyEKF_DIR}/src")
file(GLOB_RECURSE TinyEKF_SRC "${TinyEKF_DIR}/src/*.c")               # TinyEKF library cpp files
# TinyNURBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/tinynurbs)

# Matplotlib-cpp
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp)

# cnpy
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/cnpy)
include_directories(${PROJECT_SOURCE_DIR}/third_party/cnpy)

# QuadProgpp
set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/third_party/QuadProgpp)
add_subdirectory(${QuadProgpp_DIR})
include_directories("${QuadProgpp_DIR}/src")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/QuadProgpp)

# xpp
set(XPP_DIR ${PROJECT_SOURCE_DIR}/../xpp)
include_directories(${XPP_DIR}/xpp_states/include
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
        )
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

###########
file(GLOB_RECURSE sources "src/*.cpp")       # common library cpp files
include_directories("include/")                 # common includes
include_directories("config/")                 # common includes
# list(APPEND sources ${PROJECT_SOURCE_DIR}/include/exec/runtime.h)
## METHOD 1:
#FILE(GLOB_RECURSE EXECPP "src/exec/*.cpp")
#list(REMOVE_ITEM sources ${EXECPP})
## METHOD 2:
list(FILTER sources EXCLUDE REGEX "src/exec/.*.cpp")
list(APPEND sources ${PROJECT_SOURCE_DIR}/src/exec/runtime.cpp)

if(${USE_GO1})
    ## Add Runtime lib
    add_library(robot_runtime SHARED ${sources} ${TinyEKF_SRC})
    target_link_libraries(robot_runtime PUBLIC ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
            ${UNITREE_SDK_LIB} robot_interface_3.4 quadprog lcm)
    add_dependencies(robot_runtime ${catkin_EXPORTED_TARGETS})

    ## Add Executable
    add_executable(trot_go1 src/exec/trot_go1.cpp)
    target_link_libraries(trot_go1 robot_runtime)

else() #A1 & AligenGo
    ## Add Runtime lib
    add_library(robot_runtime SHARED ${sources} ${TinyEKF_SRC})
    target_link_libraries(robot_runtime PUBLIC ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
            ${UNITREE_SDK_LIB} robot_interface quadprog lcm xpp_states xpp_vis 
            tinynurbs::tinynurbs cnpy)
    add_dependencies(robot_runtime ${catkin_EXPORTED_TARGETS})

    ## Add Executable
    #add_executable(a1_real src/exec/a1_real.cpp)
    #target_link_libraries(a1_real robot_runtime)
    
    # add_executable(aligengo_real src/exec/aligengo_real.cpp)
    # target_link_libraries(aligengo_real robot_runtime)
    
    #  add_executable(a1_sim src/exec/a1_sim.cpp)
    #  target_link_libraries(a1_sim robot_runtime)
    add_executable(sim src/exec/a1_sim_pose_optim.cpp)
    target_link_libraries(sim robot_runtime)
endif()

#################################TEST#################################
if(ENABLE_TEST)
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# include_directories(${GTEST_INCLUDE_DIRS} ${GMOCK_INCLUDE_DIRS})

# add_executable(Test_Robot tests/ut/test_robot.cpp ${sources} ${TinyEKF_SRC})
# add_executable(Test_Gait tests/ut/test_openloop_gait_generator.cpp ${sources} ${TinyEKF_SRC})
# add_executable(Test_SwingLeg tests/ut/test_swing_leg_controller.cpp ${sources} ${TinyEKF_SRC})
# add_executable(Test_stance_controller tests/ut/test_torque_stance_leg_controller.cpp ${sources} ${TinyEKF_SRC})
add_executable(Test_qp_solver tests/ut/test_qp_torque_optimizer.cpp ${sources} ${TinyEKF_SRC})
# add_executable(Test_vel_estimator tests/ut/test_robot_velocity_estimator.cpp ${sources} ${TinyEKF_SRC})
# add_executable(Test_locomotion_controller tests/ut/test_locomotion_controller.cpp ${sources} ${TinyEKF_SRC})

# add_executable(MyTest tests/ut/test_pose_planner.cpp)
# target_link_libraries(MyTest
#                         robot_runtime
#                         ${GTEST_BOTH_LIBRARIES}
#                         gmock
#                         ${CMAKE_THREAD_LIBS_INIT}
# )
       
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
 target_link_libraries(Test_qp_solver
                     robot_runtime
                     ${GTEST_BOTH_LIBRARIES}
                     gmock
                     ${CMAKE_THREAD_LIBS_INIT}
 )
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
add_test(Test Test_qp_solver)
# add_test(Test Test_Robot)
# add_test(Test Test_Gait)
# add_test(Test Test_SwingLeg)
# add_test(Test Test_vel_estimator)
# add_test(Test Test_locomotion_controller)
enable_testing()
endif()
