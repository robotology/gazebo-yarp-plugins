# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

project(Plugin_MultiCamera)

include(AddGazeboYarpPluginTarget)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

include_directories(/usr/local/src/robot/gazebo-yarp-plugins/libraries/singleton/include)

add_gazebo_yarp_plugin_target(LIBRARY_NAME MultiCameraPlugin
                              INCLUDE_DIRS include
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS}
                                                  ${Boost_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${GAZEBO_LIBRARIES}
                                               ${Boost_LIBRARIES}
                              HEADERS include/gazebo/plugins/MultiCameraPlugin.hh
                              SOURCES src/MultiCameraPlugin.cpp)

add_gazebo_yarp_plugin_target(LIBRARY_NAME multicamera
                              INCLUDE_DIRS include
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS}
                                                 
                                                  ${Boost_INCLUDE_DIRS}
                                                  ${GAZEBO_INCLUDE_DIRS}
                                                  ${SDFORMAT_INCLUDE_DIRS}
                                                  ${PROTOBUF_INCLUDE_DIRS}
                                                  ${OGRE_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME}
                                               gazebo_yarp_singleton
                                               gazebo_yarp_MultiCameraPlugin
                                               ${YARP_LIBRARIES}
                                               ${GAZEBO_LIBRARIES}
                                               ${Boost_LIBRARIES}
                              HEADERS include/gazebo/MultiCamera.hh
                                      include/yarp/dev/MultiCameraDriver.h
                              SOURCES src/MultiCamera.cc
                                      src/MultiCameraDriver.cpp)
