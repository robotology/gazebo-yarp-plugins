# Copyright (C) 2018 Istituto Italiano di Tecnologia iCub Facility
# Authors: Lorenzo Rapetti
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 3.5)

PROJECT(Plugin_ConfigurationOverride)

include(AddGazeboYarpPluginTarget)

set(configurationoverride_sources src/ConfigurationOverride.cc)
set(configurationoverride_headers include/gazebo/ConfigurationOverride.hh)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)

add_gazebo_yarp_plugin_target(LIBRARY_NAME configurationoverride
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo include/GazeboYarpPlugins
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS ${configurationoverride_headers}
                              SOURCES ${configurationoverride_sources}
                            )