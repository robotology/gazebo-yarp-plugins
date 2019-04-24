# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Alberto Cardellino, Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_DepthCamera)

include(AddGazeboYarpPluginTarget)



add_gazebo_yarp_plugin_target(LIBRARY_NAME depthCamera
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_lib_common gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} DepthCameraPlugin
                              HEADERS include/gazebo/DepthCamera.hh
                                      include/yarp/dev/DepthCameraDriver.h
                              SOURCES src/DepthCamera.cc
                                      src/DepthCameraDriver.cpp
                                 )
