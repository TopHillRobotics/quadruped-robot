cmake_minimum_required(VERSION 3.0.2)
project(quadprogpp)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES quadprogpp
)

include_directories(include)

file(GLOB sources include/${PROJECT_NAME}/*.cc)
file(GLOB headers include/${PROJECT_NAME}/*.hh)

set(library_sources ${sources})
add_library(quadprogpp STATIC ${library_sources} ${headers})
set_property(TARGET quadprogpp PROPERTY POSITION_INDEPENDENT_CODE 1)
set_property(TARGET quadprogpp PROPERTY PUBLIC_HEADER ${headers})
