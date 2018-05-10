# Copyright 2018, zhouyoujoin. 
#
# I wrote this CMakeList.txt for isolate the performance compiling from fast-rtps project,
# but we still need the fast-rtps be installed at the beginning.

cmake_minimum_required(VERSION 3.5)

#########################################################################################
# Definitions
#########################################################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Wno-unknown-pragmas -std=c++11")
add_definitions(
    -DBOOST_ASIO_STANDALONE
    -DASIO_STANDALONE
    )

#########################################################################################
# Includes
#########################################################################################
include_directories(${ASIO_INCLUDE_DIR})

#########################################################################################
# Binaries
#########################################################################################
set (LATENCYTEST_SOURCE LatencyTestPublisher.cpp
    LatencyTestSubscriber.cpp
    LatencyTestTypes.cpp
    main_LatencyTest.cpp
    )
add_executable(LatencyTest ${LATENCYTEST_SOURCE})
target_link_libraries(LatencyTest fastrtps ${CMAKE_THREAD_LIBS_INIT})

set (THROUGHPUTTEST_SOURCE ThroughputPublisher.cpp
    ThroughputSubscriber.cpp
    ThroughputTypes.cpp
    main_ThroughputTest.cpp
    )
add_executable(ThroughputTest ${THROUGHPUTTEST_SOURCE})
target_link_libraries(ThroughputTest fastrtps ${CMAKE_THREAD_LIBS_INIT})
