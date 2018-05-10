# Copyright 2018, zhouyoujoin.
#
# The basic code were came from github.com/ros2/demos/blob/master/intra_process_demo. 

cmake_minimum_required(VERSION 3.5)
project(intra_process_performance)

#########################################################################################
# Definitions
#########################################################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Wno-unknown-pragmas -std=c++14")

#########################################################################################
# Package Dependencies
#########################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# OpenCV was orignal needed by image pipeline, which we do not need current.
# find_package(OpenCV REQUIRED)

include_directories(include)
ament_export_include_directories(include)
 
#########################################################################################
# Binaries 
#########################################################################################
set (INTRAPROCESSTEST_SRC two_node_pipeline.cpp
    cyclic_pipeline.cpp
    main_IntraProcessTest.cpp
    )
add_executable(IntraProcessTest ${INTRAPROCESSTEST_SRC})
ament_target_dependencies(IntraProcessTest
    "rclcpp"
    "std_msgs"
    )

install(TARGETS
    IntraProcessTest
    DESTINATION bin/${PROJECT_NAME}
    )
