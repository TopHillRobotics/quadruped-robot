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

add_executable(example_lite3_sim example_lite3_sim/example_lite3_sim.cpp)
add_executable(example_lite3_real example_lite3_real/example_lite3_real.cpp)

add_executable(example_keyboard example_keyboard/example_keyboard.cpp)
target_link_libraries(example_a1_real ${catkin_LIBRARIES})
target_link_libraries(example_a1_sim ${catkin_LIBRARIES})

target_link_libraries(example_lite3_sim ${catkin_LIBRARIES})
target_link_libraries(example_lite3_real ${catkin_LIBRARIES})

target_link_libraries(example_keyboard ${catkin_LIBRARIES})

