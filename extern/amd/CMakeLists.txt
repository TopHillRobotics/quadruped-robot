cmake_minimum_required(VERSION 3.0.2)
project(amd)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES MITAMD
)

include_directories(
    include
)

add_library(MITAMD SHARED
    src/amd_1.c
    src/amd_2.c
    src/amd_aat.c
    src/amd_control.c
    src/amd_defaults.c
    src/amd_info.c
    src/amd_order.c
    src/amd_post_tree.c
    src/amd_postorder.c
    src/amd_preprocess.c
    src/amd_valid.c
    src/SuiteSparse_config.c)

target_link_libraries(MITAMD pthread)
