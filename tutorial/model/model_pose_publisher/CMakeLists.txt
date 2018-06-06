# Copyright (C) 2018 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: see AUTHORS file.
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

project(gazebo_yarp_model_pose_publisher_example)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
LIST(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()

set(TARGET_NAME example)

add_executable(${TARGET_NAME} example.cpp)
target_include_directories(${TARGET_NAME} PRIVATE)
target_link_libraries(${TARGET_NAME} ${YARP_LIBRARIES})

install(TARGETS ${TARGET_NAME} DESTINATION bin)
