# Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(GazeboYarpPlugins)

find_package(YARP 2.3.62 REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")

# Finding dependencies
find_package(Gazebo REQUIRED)

find_package(Boost REQUIRED serialization system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDFORMAT sdformat)
  pkg_check_modules(PROTOBUF protobuf)
endif()

if(CMAKE_BUILD_TYPE MATCHES DEBUG)
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Weverything -pedantic -Wextra")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic ")
endif()
endif(CMAKE_BUILD_TYPE MATCHES DEBUG)

#define debug flag
set_property(
    DIRECTORY
    APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG=1
)

# add local cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)


include_directories(${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDFORMAT_LIBRARY_DIRS} ${PROTOBUF_LIBRARY_DIRS})

set(commons_headers         include/gazebo_yarp_plugins/common.h)
#if this is not a target it will not work :(
SOURCE_GROUP("Common" FILES ${commons_headers})
# SET_SOURCE_FILES_PROPERTIES(${commons_headers} PROPERTIES HEADER_FILE_ONLY TRUE)

set(singleton_source        src/gazebo_plugins/Handler.cc)
set(singleton_headers       include/gazebo_yarp_plugins/Handler.hh)

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
                            src/yarp_drivers/ControlBoardDriverVelocityControl.cpp
                            src/yarp_drivers/ControlBoardDriverImpedancePositionControl.cpp
                            src/yarp_drivers/ControlBoardDriverOpenLoop.cpp
                            src/yarp_drivers/ControlBoardDriverPidControl.cpp)

set(controlBoard_headers    include/gazebo_yarp_plugins/ControlBoard.hh
                            include/gazebo_yarp_plugins/ControlBoardDriver.h )

SOURCE_GROUP("Source Files" FILES ${controlBoard_source})
SOURCE_GROUP("Header Files" FILES ${controlBoard_headers})

set(forceTorque_source      src/gazebo_plugins/ForceTorque.cc
                            src/yarp_drivers/ForceTorqueDriver.cpp)

set(forceTorque_headers     include/gazebo_yarp_plugins/ForceTorque.hh
                            include/gazebo_yarp_plugins/ForceTorqueDriver.h)


SOURCE_GROUP("Source Files" FILES ${forceTorque_source})
SOURCE_GROUP("Header Files" FILES ${forceTorque_headers})


set(imu_headers             include/gazebo_yarp_plugins/IMU.hh
                            include/gazebo_yarp_plugins/IMUDriver.h)
set(imu_source              src/gazebo_plugins/IMU.cc
                            src/yarp_drivers/IMUDriver.cpp)

SOURCE_GROUP("Source Files" FILES ${imu_source})
SOURCE_GROUP("Header Files" FILES ${imu_headers})

set(jointSensors_headers    include/gazebo_yarp_plugins/JointSensors.hh
                            include/gazebo_yarp_plugins/JointSensorsDriver.h)
set(jointSensors_source     src/gazebo_plugins/JointSensors.cc
                            src/yarp_drivers/JointSensorsDriver.cpp)

SOURCE_GROUP("Source Files" FILES ${jointSensors_source})
SOURCE_GROUP("Header Files" FILES ${jointSensors_headers})

set(clock_headers           include/gazebo_yarp_plugins/Clock.hh)
set(clock_source            src/gazebo_plugins/Clock.cc)

SOURCE_GROUP("Source Files" FILES ${clock_source})
SOURCE_GROUP("Header Files" FILES ${clock_headers})

add_library(gazebo_yarp_singleton    SHARED ${singleton_source}    ${singleton_headers})
add_library(gazebo_yarp_controlboard SHARED ${controlBoard_source} ${controlBoard_headers})
add_library(gazebo_yarp_forcetorque  SHARED ${forceTorque_source}  ${forceTorque_headers})
add_library(gazebo_yarp_imu          SHARED ${imu_source}          ${imu_headers})
add_library(gazebo_yarp_jointsensors SHARED ${jointSensors_source} ${jointSensors_headers})
add_library(gazebo_yarp_clock        SHARED ${clock_source}        ${clock_headers})


target_link_libraries(gazebo_yarp_singleton ${YARP_LIBRARIES} ${SDFORMAT_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_controlboard  gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_forcetorque   gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ForceTorquePlugin  ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_imu           gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_jointsensors  gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(gazebo_yarp_clock         gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


# Add install target
install(TARGETS gazebo_yarp_singleton
                gazebo_yarp_controlboard
                gazebo_yarp_forcetorque
                gazebo_yarp_imu
                gazebo_yarp_jointsensors
                gazebo_yarp_clock
                DESTINATION lib)

# Add uninstall target                
include(AddUninstallTarget)
