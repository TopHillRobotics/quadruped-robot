cmake_minimum_required(VERSION 3.0.2)
project(unitree_legged_sdk)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O3")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}") # x86_64, amd64, arm64
if(${ARCHITECTURE} STREQUAL "x86_64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB_NAME "unitree_legged_sdk_amd64")
elseif(${ARCHITECTURE} STREQUAL "amd64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB_NAME "unitree_legged_sdk_amd64")
elseif(${ARCHITECTURE} STREQUAL "arm64")
    set(UNITREE_SDK_LIB_NAME "unitree_legged_sdk_arm64")
else()
    message(FATAL_ERROR "unrecongnized arch.")
endif()

include_directories(include)

#link_directories(lib)
find_library(UNITREE_SDK_LIB ${UNITREE_SDK_LIB_NAME} HINT ${CMAKE_CURRENT_SOURCE_DIR}/lib)


set(EXTRA_LIBS -pthread ${UNITREE_SDK_LIB} lcm)

#add_subdirectory(pybind11)
#pybind11_add_module(robot_interface python_interface.cpp)
add_library(${PROJECT_NAME} SHARED unitree_interface.cpp)
#target_link_directories(${PROJECT_NAME} PUBLIC lib)
target_link_libraries(${PROJECT_NAME} ${EXTRA_LIBS})

#install(TARGETS ${PROJECT_NAME}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h+")
