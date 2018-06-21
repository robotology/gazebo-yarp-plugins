# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.11)

PROJECT(Library_Common)

if(CMAKE_VERSION VERSION_LESS 3.0.0)
    set_property(GLOBAL APPEND PROPERTY GAZEBO_YARP_COMMON_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include")
else()
    add_library(gazebo_yarp_lib_common INTERFACE)
    target_include_directories(gazebo_yarp_lib_common INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/include>
    )

    install(TARGETS gazebo_yarp_lib_common EXPORT GazeboYARPPlugins)
    install(FILES include/GazeboYarpPlugins/common.h
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/GazeboYarpPlugins)
endif()
