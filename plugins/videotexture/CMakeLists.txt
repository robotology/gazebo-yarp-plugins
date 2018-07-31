# Copyright (C) 2016 iCub Facility
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_Videotexture)

include(AddGazeboYarpPluginTarget)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

# find opencv dependency
find_package(OpenCV REQUIRED)

add_gazebo_yarp_plugin_target(LIBRARY_NAME videotexture
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS}
                                                 
                                                  ${Boost_INCLUDE_DIRS}
                                                  ${GAZEBO_INCLUDE_DIRS}
                                                  ${SDFORMAT_INCLUDE_DIRS}
                                                  ${PROTOBUF_INCLUDE_DIRS}
                                                  ${OGRE_INCLUDE_DIRS}
                                                  ${OpenCV_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME}
                                               gazebo_yarp_singleton
                                               ${YARP_LIBRARIES}
                                               ${GAZEBO_LIBRARIES}
                                               ${Boost_LIBRARIES}
                                               ${OpenCV_LIBRARIES}
                              HEADERS include/VideoTexture.hh
                              SOURCES src/VideoTexture.cc)
