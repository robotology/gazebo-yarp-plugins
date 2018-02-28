# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

PROJECT(Plugin_ControlBoard)

# Copied from YARP
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)


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
                            src/ControlBoardDriverPWM.cpp
                            src/ControlBoardDriverCurrent.cpp
                            src/ControlBoardDriverPidControl.cpp
                            src/ControlBoardDriverTrajectory.cpp
                            src/ControlBoardDriverCoupling.cpp
                            src/ControlBoardDriverVirtualAnalogSensor.cpp)

set(controlBoard_headers    include/gazebo/ControlBoard.hh
                            include/yarp/dev/ControlBoardDriver.h
                            include/yarp/dev/ControlBoardDriverTrajectory.h
                            include/yarp/dev/ControlBoardDriverCoupling.h
                            include/yarp/dev/ControlBoardDriverRange.h)
                            
                            
                            
add_gazebo_yarp_plugin_target(LIBRARY_NAME controlboard
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_lib_common gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS ${controlBoard_headers}
                              SOURCES ${controlBoard_source})
