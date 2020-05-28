# Copyright (C) 2016 iCub Facility
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


PROJECT(Plugin_Videotexture)

include(AddGazeboYarpPluginTarget)

add_gazebo_yarp_plugin_target(LIBRARY_NAME videotexture
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS}
                                                 
                                                  ${Boost_INCLUDE_DIRS}
                                                  ${GAZEBO_INCLUDE_DIRS}
                                                  ${SDFORMAT_INCLUDE_DIRS}
                                                  ${PROTOBUF_INCLUDE_DIRS}
                                                  ${OGRE_INCLUDE_DIRS}
                                                  ${OpenCV_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_lib_common
                                               gazebo_yarp_singleton
                                               ${YARP_LIBRARIES}
                                               ${GAZEBO_LIBRARIES}
                                               ${Boost_LIBRARIES}
                                               ${OpenCV_LIBRARIES}
                              HEADERS include/VideoTexture.hh
                              SOURCES src/VideoTexture.cc)
