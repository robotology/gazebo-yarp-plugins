# Copyright (C) 2015 iCub Facility
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_WorldInterface)

include(AddGazeboYarpPluginTarget)

set(worldinterface_source src/worldinterface.cpp src/worldinterfaceserverimpl.cpp src/worldproxy.cpp)
                                           
set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()
                            
add_gazebo_yarp_plugin_target(LIBRARY_NAME worldinterface
                              INCLUDE_DIRS include/gazebo 
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_rpc_worldinterface ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS ${worldinterface_headers}
                                      src/worldinterfaceserverimpl.h
                                      src/worldproxy.h
                              SOURCES ${worldinterface_source}
                            )
