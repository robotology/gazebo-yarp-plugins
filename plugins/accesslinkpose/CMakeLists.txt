# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Yeshasvi Tirupachuri
# CopyPolicy:

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_AccessLinkPose)

include(AddGazeboYarpPluginTarget)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

add_gazebo_yarp_plugin_target(LIBRARY_NAME accesslinkpose
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo
                              SYSTEM_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_accesslinkpose ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS include/gazebo/AccessLinkPose.h
                              SOURCES src/AccessLinkPose.cpp
                                 )


