cmake_minimum_required(VERSION 3.0.2)
project(examples)

find_package(catkin REQUIRED quadruped)

catkin_package(
    CATKIN_DEPENDS roscpp quadruped
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(example_a1_real example_a1_real/example_a1_real.cpp)
add_executable(example_a1_sim example_a1_sim/example_a1_sim.cpp)

target_link_libraries(example_a1_real ${catkin_LIBRARIES})
target_link_libraries(example_a1_sim ${catkin_LIBRARIES})

