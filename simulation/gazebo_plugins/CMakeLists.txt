cmake_minimum_required(VERSION 3.0)

# Project name
project(gap VERSION 1.5.0)
message (STATUS "${PROJECT_NAME} version: ${PROJECT_VERSION}")
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

# Additional cmake modules
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# CMake safeguards e.g. prevent in-source builds
include(safeguards)

# Check dependencies
find_package(Boost COMPONENTS system REQUIRED)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

# Setup directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

# [project]-[major].[minor] String
set (project_major_minor
    "${PROJECT_NAME_LOWER}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
# Setup directories for library installation
set (msgs_include_dest "include/${project_major_minor}")
set (msgs_lib_dest "lib/${project_major_minor}")
# Setup directories for plugin installation
set (plugins_lib_dest "lib/${project_major_minor}")
# Setup directories for utils installation
set (utils_include_dest "include/${project_major_minor}")
set (utils_lib_dest "lib/${project_major_minor}")

# Add uninstall command
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/uninstall.cmake"
)

# Build project

# Messages
add_subdirectory(msgs)
# Plugins
add_subdirectory(plugins)
# Utilities
add_subdirectory(utils)
# Examples
add_subdirectory(examples)
# Documentation
add_subdirectory(doc)

# Configuration file
set(PKG_NAME ${PROJECT_NAME_UPPER})
set(cmake_conf_file "cmake/gap-config.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}" @ONLY)

# Version control
set(cmake_version_file "cmake/gap-config-version.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_version_file}.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${cmake_version_file}" @ONLY)

# Create package
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_version_file}
    DESTINATION ${msgs_lib_dest}/
    COMPONENT cmake)
