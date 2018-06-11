# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_ForceTorque)

include(AddGazeboYarpPluginTarget)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

add_gazebo_yarp_plugin_target(LIBRARY_NAME forcetorque
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ForceTorquePlugin  ${Boost_LIBRARIES}
                              HEADERS include/gazebo/ForceTorque.hh
                                      include/yarp/dev/ForceTorqueDriver.h
                              SOURCES src/ForceTorque.cc
                                      src/ForceTorqueDriver.cpp
                                 )
