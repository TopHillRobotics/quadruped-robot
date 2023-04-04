cmake_minimum_required(VERSION 3.0.2)
project(deeprobotics_legged_sdk)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O3")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)
include_directories(include)

#set(EXTRA_LIBS -pthread ${DEEPROBOTICS_SDK_LIB} lcm)
file(GLOB_RECURSE sources "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${sources})

#target_link_libraries(${PROJECT_NAME} ${EXTRA_LIBS})
target_link_libraries(${PROJECT_NAME} -lpthread -lm -lrt -ldl -lstdc++ )
