# Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

PROJECT(yarp_gazebo)
#list ( APPEND CMAKE_CXX_FLAGS "-std=c++0x -Wall -pedantic -Wmissing-field-initializers -W -Wunused -Wuninitialized -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -fno-strict-aliasing  -Werror=address -Werror=parentheses  ${CMAKE_CXX_FLAGS}" )
list ( APPEND CMAKE_CXX_FLAGS "-std=c++0x ")

find_package(YARP REQUIRED)
find_package(Boost REQUIRED serialization)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${YARP_INCLUDE_DIRS} include)

set(singleton_source        src/gazebo_plugins/Handler.cc)
set(singleton_headers        include/gazebo_yarp_plugins/Handler.hh)
# for visual studio
SOURCE_GROUP("Source Files" FILES ${singleton_source})
SOURCE_GROUP("Header Files" FILES ${singleton_headers})

set(controlBoard_source     src/gazebo_plugins/ControlBoard.cc
                            src/yarp_drivers/ControlBoardDriver.cpp
                            src/yarp_drivers/ControlBoardDriverControlMode.cpp
                            src/yarp_drivers/ControlBoardDriverDeviceDriver.cpp
                            src/yarp_drivers/ControlBoardDriverEncoders.cpp
                            src/yarp_drivers/ControlBoardDriverOthers.cpp
                            src/yarp_drivers/ControlBoardDriverPositionControl.cpp
                            src/yarp_drivers/ControlBoardTorqueDriverControl.cpp
                            src/yarp_drivers/ControlBoardDriverVelocityControl.cpp)

set(controlBoard_headers    include/gazebo_yarp_plugins/ControlBoard.hh
                            include/gazebo_yarp_plugins/ControlBoardDriver.h )
# for visual studio
SOURCE_GROUP("Source Files" FILES ${controlBoard_source})
SOURCE_GROUP("Header Files" FILES ${controlBoard_headers})


set(forceTorque_source      src/gazebo_plugins/ForceTorque.cc
                            src/yarp_drivers/ForceTorqueDriver.cpp
                            src/yarp_drivers/analogServer.cpp)

set(forceTorque_headers     include/gazebo_yarp_plugins/ForceTorque.hh
                            include/gazebo_yarp_plugins/ForceTorqueDriver.h)

# for visual studio
SOURCE_GROUP("Source Files" FILES ${forceTorque_source})
SOURCE_GROUP("Header Files" FILES ${forceTorque_headers})

set(imu_headers             include/gazebo_yarp_plugins/IMU.hh)
set(imu_source              src/gazebo_plugins/IMU.cc )

# for visual studio
SOURCE_GROUP("Source Files" FILES ${imu_source})
SOURCE_GROUP("Header Files" FILES ${imu_headers})



add_library(singleton                SHARED ${singleton_source}    ${singleton_headers})
add_library(gazebo_yarp_controlboard SHARED ${controlBoard_source} ${controlBoard_headers})
add_library(gazebo_yarp_forcetorque  SHARED ${forceTorque_source}  ${forceTorque_headers})
add_library(gazebo_yarp_imu          SHARED ${imu_source}          ${imu_headers})


target_link_libraries(gazebo_yarp_controlboard  singleton ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_forcetorque   singleton ${YARP_LIBRARIES} ${GAZEBO_libraries} ForceTorquePlugin  ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_imu           singleton ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})

add_executable(testmotor src/test/test.cpp)
target_link_libraries(testmotor ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})
