# Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

PROJECT(yarp_gazebo)

#list ( APPEND CMAKE_CXX_FLAGS "-Wall -pedantic -Wmissing-field-initializers -W -Wuninitialized -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -fno-strict-aliasing  -Werror=address -Werror=parentheses  ${CMAKE_CXX_FLAGS}" )


#### Find YARP, require that YARP version is at least GYP_REQYARP_VERSION
SET(GYP_REQYARP_VERSION_MAJOR "2")
SET(GYP_REQYARP_VERSION_MINOR "3")
SET(GYP_REQYARP_VERSION_PATCH "60")
set(GYP_REQYARP_VERSION_TWEAK "0")

if(GYP_REQYARP_VERSION_TWEAK)
    set(GYP_REQYARP_VERSION ${GYP_REQYARP_VERSION_MAJOR}.${GYP_REQYARP_VERSION_MINOR}.${GYP_REQYARP_VERSION_PATCH}.${GYP_REQYARP_VERSION_TWEAK})
else()
    set(GYP_REQYARP_VERSION ${GYP_REQYARP_VERSION_MAJOR}.${GYP_REQYARP_VERSION_MINOR}.${GYP_REQYARP_VERSION_PATCH})
endif()

find_package(YARP ${GYP_REQYARP_VERSION} REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")

if(YARP_VERSION VERSION_LESS GYP_REQYARP_VERSION)
    message(FATAL_ERROR "YARP version is not compatible, please update YARP (using VERSION_LESS)")
else()
    message(STATUS "found compatible YARP version")
endif()

find_package(Gazebo REQUIRED)

find_package(Boost REQUIRED serialization system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDFORMAT sdformat)
  pkg_check_modules(PROTOBUF protobuf)
endif()

include_directories(${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDFORMAT_LIBRARY_DIRS} ${PROTOBUF_LIBRARY_DIRS})

set(singleton_source        src/gazebo_plugins/Handler.cc)
set(singleton_headers       include/gazebo_yarp_plugins/Handler.hh)
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
                            src/yarp_drivers/ForceTorqueDriver.cpp)

set(forceTorque_headers     include/gazebo_yarp_plugins/ForceTorque.hh
                            include/gazebo_yarp_plugins/ForceTorqueDriver.h)

# for visual studio
SOURCE_GROUP("Source Files" FILES ${forceTorque_source})
SOURCE_GROUP("Header Files" FILES ${forceTorque_headers})

set(imu_headers             include/gazebo_yarp_plugins/IMU.hh
                            include/gazebo_yarp_plugins/IMUDriver.h)
set(imu_source              src/gazebo_plugins/IMU.cc 
                            src/yarp_drivers/IMUDriver.cpp)

# for visual studio
SOURCE_GROUP("Source Files" FILES ${imu_source})
SOURCE_GROUP("Header Files" FILES ${imu_headers})


add_library(singleton                SHARED ${singleton_source}    ${singleton_headers})
add_library(gazebo_yarp_controlboard MODULE ${controlBoard_source} ${controlBoard_headers})
add_library(gazebo_yarp_forcetorque  MODULE ${forceTorque_source}  ${forceTorque_headers})
add_library(gazebo_yarp_imu          MODULE ${imu_source}          ${imu_headers})


target_link_libraries(singleton ${YARP_LIBRARIES} ${SDFORMAT_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_controlboard  singleton ${YARP_LIBRARIES} ${Gazebo_libraries} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_forcetorque   singleton ${YARP_LIBRARIES} ${GAZEBO_libraries} ForceTorquePlugin  ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_imu           singleton ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})


add_executable(testmotor src/test/test.cpp)
target_link_libraries(testmotor ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(testmotor ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
