# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_ControlBoard)

include(AddGazeboYarpPluginTarget)

set(controlBoard_source     src/ControlBoard.cc
                            src/ControlBoardDriver.cpp
                            src/ControlBoardDriverControlMode.cpp
                            src/ControlBoardDriverInteractionMode.cpp
                            src/ControlBoardDriverDeviceDriver.cpp
                            src/ControlBoardDriverEncoders.cpp
                            src/ControlBoardDriverOthers.cpp
                            src/ControlBoardDriverRemoteVariables.cpp
                            src/ControlBoardDriverPositionControl.cpp
                            src/ControlBoardDriverTorqueControl.cpp
                            src/ControlBoardDriverVelocityControl.cpp
                            src/ControlBoardDriverImpedancePositionControl.cpp
                            src/ControlBoardDriverOpenLoop.cpp
                            src/ControlBoardDriverPidControl.cpp
                            src/ControlBoardDriverTrajectory.cpp
                            src/ControlBoardDriverCoupling.cpp)

set(controlBoard_headers    include/gazebo/ControlBoard.hh
                            include/yarp/dev/ControlBoardDriver.h
                            include/yarp/dev/ControlBoardDriverTrajectory.h
                            include/yarp/dev/ControlBoardDriverCoupling.h)
                            
                            
set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()
                            
add_gazebo_yarp_plugin_target(LIBRARY_NAME controlboard
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS ${controlBoard_headers}
                              SOURCES ${controlBoard_source}
                            )
