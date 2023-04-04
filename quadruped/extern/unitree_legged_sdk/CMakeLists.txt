cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

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

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread ${UNITREE_SDK_LIB} lcm)

set(CMAKE_CXX_FLAGS "-O3")

#add_subdirectory(pybind11)
#pybind11_add_module(robot_interface python_interface.cpp)
add_library(robot_interface SHARED unitree_interface.cpp)
target_link_libraries(robot_interface ${EXTRA_LIBS})
