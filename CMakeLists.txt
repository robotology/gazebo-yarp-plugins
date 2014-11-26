# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

PROJECT(GazeboYarpPlugins)

find_package(YARP 2.3.63.2 REQUIRED)

set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Finding dependencies
find_package(Gazebo REQUIRED)

find_package(Boost REQUIRED serialization system)
find_package(OGRE)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDFORMAT sdformat)
  pkg_check_modules(PROTOBUF protobuf)
endif()

# add local cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#used for dir suffixes
include(GNUInstallDirs)

# Define (depending on CMAKE_BUILD_TYPE) the DEBUG macro
include(AddDebugMacro)

# Enable all possible warning if CMAKE_BUILD_TYPE is debug
include(AddDebugWarnings)

#this is required for all plugins
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDFORMAT_LIBRARY_DIRS} ${PROTOBUF_LIBRARY_DIRS})

#build common libraries first
add_subdirectory(libraries)
#build RPC libraries
add_subdirectory(thrift)
#now build plugins
add_subdirectory(plugins)


include(AddUninstallTarget)
