# Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

PROJECT(yarp_gazebo)
list ( APPEND CMAKE_CXX_FLAGS "-std=c++0x -Wall -pedantic -Wmissing-field-initializers -W -Wunused -Wuninitialized -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -fno-strict-aliasing  -Werror=address -Werror=parentheses  ${CMAKE_CXX_FLAGS}" )

find_package(YARP REQUIRED)
find_package(Boost REQUIRED serialization)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${YARP_INCLUDE_DIRS} include)


add_library(gazebo_yarp_controlboard SHARED src/GazeboYarpControlBoard.cc include/GazeboYarpControlBoard.hh)
add_library(gazebo_yarp_controlboard_driver SHARED
                            include/GazeboYarpControlBoardDriver.h
                            src/GazeboYarpControlBoardDriver.cpp
                            src/GazeboYarpControlBoardDriverControlMode.cpp
                            src/GazeboYarpControlBoardDriverDeviceDriver.cpp
                            src/GazeboYarpControlBoardDriverEncoders.cpp
                            src/GazeboYarpControlBoardDriverOthers.cpp
                            src/GazeboYarpControlBoardDriverPositionControl.cpp
                            src/GazeboYarpControlBoardTorqueDriverControl.cpp
                            src/GazeboYarpControlBoardDriverVelocityControl.cpp)

add_library(gazebo_yarp_forcetorque SHARED  include/analogServer.h
                                            include/GazeboYarpForceTorque.hh
                                            include/GazeboYarpForceTorqueDriver.h
                                            src/GazeboYarpForceTorqueDriver.cpp
                                            src/GazeboYarpForceTorque.cc
                                            src/analogServer.cpp)

add_library(gazebo_yarp_imu SHARED include/GazeboYarpIMU.hh src/GazeboYarpIMU.cc )


target_link_libraries(gazebo_yarp_controlboard gazebo_yarp_controlboard_driver ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_forcetorque ${YARP_LIBRARIES} ${GAZEBO_libraries} ForceTorquePlugin  ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_imu  ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})

add_executable(testmotor src/test/test.cpp)
target_link_libraries(testmotor ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})
