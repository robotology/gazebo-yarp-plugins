# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.11)

PROJECT(Library_Singleton)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

include(AddGazeboYarpPluginTarget)

add_gazebo_yarp_plugin_target(LIBRARY_NAME singleton
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/GazeboYarpPlugins
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} ${YARP_LIBRARIES} ${SDFORMAT_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS include/GazeboYarpPlugins/Handler.hh
                                      include/GazeboYarpPlugins/ConfHelpers.hh
                              SOURCES src/Handler.cc
                                      src/ConfHelpers.cc)

