# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.11)

PROJECT(Library_Singleton)

include(AddGazeboYarpPluginTarget)

set(Singleton_HEADERS include/GazeboYarpPlugins/Handler.hh
                      include/GazeboYarpPlugins/ConfHelpers.hh)

add_gazebo_yarp_plugin_target(LIBRARY_NAME singleton
                              INCLUDE_DIRS include/GazeboYarpPlugins
                              SYSTEM_INCLUDE_DIRS ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${YARP_LIBRARIES} ${SDFORMAT_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS ${Singleton_HEADERS}
                              SOURCES src/Handler.cc
                                      src/ConfHelpers.cc)

install(FILES ${Singleton_HEADERS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/GazeboYarpPlugins)



