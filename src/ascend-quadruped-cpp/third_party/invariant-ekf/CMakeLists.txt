cmake_minimum_required(VERSION 3.10.0)
project(inekf_interface)
message("================")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -W -fopenmp ")

option(USE_CPP11 "Use flag -std=c++11" ON)
option(USE_MUTEX "Use mutex in code" OFF)
option(PYTHON_INTERFACE "Compile INEKF python interface" OFF)
option(CPP_INTERFACE "Compile INEKF cpp interface" ON)

message("USE_CPP11: " ${USE_CPP11})
message("USE_MUTEX: " ${USE_MUTEX})

if (USE_MUTEX AND NOT USE_CPP11)
message("test")
    message(FATAL_ERROR "You cannot have USE_CPP11 = OFF AND USE_MUTEX = ON")
endif (USE_MUTEX AND NOT USE_CPP11)

# Set compiler flags
if (USE_CPP11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(USE_CPP11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") #-DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
if (USE_MUTEX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DINEKF_USE_MUTEX=true")
else (USE_MUTEX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DINEKF_USE_MUTEX=false")
endif(USE_MUTEX)
# SET(CMAKE_CXX_COMPILER /usr/bin/g++)

message("CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
unset(USE_CPP11)
unset(USE_MUTEX)

#Set the output folder where your program will be created  
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# ############################################
# #  The following folders will be included  #
# ############################################
include_directories("${PROJECT_SOURCE_DIR}/include/")

#Threading
find_package (Threads)

#Boost
find_package(Boost 1.57 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})

##Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

##Adding all classes
file(GLOB src_files
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

######################
#  Add Execuatables  #
######################
# link_directories(${PROJECT_SOURCE_DIR}/lib ${Boost_LIBRARY_DIRS})

# add_executable(landmarks ${PROJECT_SOURCE_DIR}/src/examples/landmarks.cpp ${src_files})
# add_executable(kinematics ${PROJECT_SOURCE_DIR}/src/examples/kinematics.cpp ${src_files})
# add_executable(propagation_speed ${PROJECT_SOURCE_DIR}/src/tests/propagation_speed.cpp ${src_files})
# add_executable(correction_speed ${PROJECT_SOURCE_DIR}/src/tests/correction_speed.cpp ${src_files})

# target_link_libraries(landmarks ${Boost_LIBRARIES})
# target_link_libraries(kinematics ${Boost_LIBRARIES})
# target_link_libraries(propagation_speed ${Boost_LIBRARIES})
# target_link_libraries(correction_speed ${Boost_LIBRARIES})

# # Create Library
add_library(inekf SHARED
    "${PROJECT_SOURCE_DIR}/src/InEKF.cpp"
    "${PROJECT_SOURCE_DIR}/src/RobotState.cpp"
    "${PROJECT_SOURCE_DIR}/src/LieGroup.cpp"
    "${PROJECT_SOURCE_DIR}/src/NoiseParams.cpp"
)
set_target_properties(inekf PROPERTIES LIBRARY_OUTPUT_NAME inekf)
#####################################################################
if(${USE_GO1})
    set(UNITREE_SDK "../unitree_legged_sdk_3.4")
else()
    set(UNITREE_SDK "../unitree_legged_sdk")
endif()
message("inekf use " ${UNITREE_SDK})
include_directories(${UNITREE_SDK}/include)
link_directories(${UNITREE_SDK}/lib)

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

include_directories("${PROJECT_SOURCE_DIR}/../../include") # for se3.h

if(${PYTHON_INTERFACE})
    message("begin to build python_interface of INEKF!")
    link_directories(${PROJECT_SOURCE_DIR}/lib)   
    add_subdirectory(${PROJECT_SOURCE_DIR}/pybind11/ ${PROJECT_SOURCE_DIR}/lib)
    pybind11_add_module(inekf_interface ${PROJECT_SOURCE_DIR}/src/tests/inekf_python_interface.cpp)
    # include_directories(${SDK}/pybind11/include)
    target_link_libraries(inekf_interface PRIVATE       
                        inekf
                        -pthread
                        libunitree_legged_sdk_amd64.so
                        lcm
                        ${Boost_LIBRARIES}
                        )
endif()

if(${CPP_INTERFACE})
    message("begin to build cpp_interface of INEKF!")
    add_library(inekf_cpp_interface SHARED
    # add_executable(inekf_cpp_interface
        "${PROJECT_SOURCE_DIR}/src/tests/inekf_cpp_interface.cpp")
    target_link_libraries(inekf_cpp_interface PUBLIC       
                        inekf
                        ${UNITREE_SDK_LIB}
                        # libunitree_legged_sdk_amd64.so
                        lcm
                        
                        ${Boost_LIBRARIES}
                        )
    
endif()
