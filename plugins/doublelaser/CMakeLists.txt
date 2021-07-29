#Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia - iCub Facility
#Authors: see AUTHORS file.
#CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT


cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_DoubleLaser)

include(AddGazeboYarpPluginTarget)

set(doubleLaser_source      src/DoubleLaser.cc  src/DoubleLaserLog.cpp)

set(doubleLaser_headers     include/gazebo/DoubleLaser.hh include/gazebo/DoubleLaserLog.h)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)

if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

add_gazebo_yarp_plugin_target(LIBRARY_NAME doublelaser
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin ${Boost_LIBRARIES}
                              HEADERS ${doubleLaser_headers}
                              SOURCES ${doubleLaser_source})
