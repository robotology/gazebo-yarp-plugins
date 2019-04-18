# Copyright (C) 2018 Istituto Italiano di Tecnologia 
# Authors: see AUTHORS file.
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


PROJECT(Plugin_ModelPosePublisher)

include(AddGazeboYarpPluginTarget)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)

add_gazebo_yarp_plugin_target(LIBRARY_NAME modelposepublisher
                              INCLUDE_DIRS include/gazebo
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS include/gazebo/ModelPosePublisher.hh
                              SOURCES src/ModelPosePublisher.cc
                              )
