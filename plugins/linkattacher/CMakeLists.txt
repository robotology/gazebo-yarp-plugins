# Copyright (C) 2016 Istituto Italiano di Tecnologia iCub Facility & RBCS Department
# Authors: Yeshasvi Tirupachuri
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 3.5)

PROJECT(Plugin_LinkAttacher)

include(AddGazeboYarpPluginTarget)

set(linkattacher_sources src/linkattacherserverimpl.cpp src/linkattacher.cc)
set(linkattacher_headers include/GazeboYarpPlugins/linkattacherserverimpl.h include/gazebo/linkattacher.hh)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)

add_gazebo_yarp_plugin_target(LIBRARY_NAME linkattacher
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo include/GazeboYarpPlugins
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_rpc_linkattacher ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS ${linkattacher_headers}
                              SOURCES ${linkattacher_sources}
                            )
