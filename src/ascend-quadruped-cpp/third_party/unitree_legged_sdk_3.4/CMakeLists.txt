cmake_minimum_required(VERSION 2.8.12)
project(unitree_legged_sdk)
message("unitree legged sdk 3.4!")

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

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread ${UNITREE_SDK_LIB} lcm)

set(CMAKE_CXX_FLAGS "-O3")

#add_subdirectory(pybind11)
#pybind11_add_module(robot_interface python_interface.cpp)
add_library(robot_interface_3.4 SHARED unitree_interface.cpp)
# add_executable(robot_interface unitree_interface.cpp)
target_link_libraries(robot_interface_3.4 ${EXTRA_LIBS})


# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_wirelessHandle examples/example_wirelessHandle.cpp)
target_link_libraries(example_wirelessHandle ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})

# lcm server
add_executable(lcm_server examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})