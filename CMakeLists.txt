# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)
PROJECT(GazeboYARPPlugins)

# Project version
set(${PROJECT_NAME}_MAJOR_VERSION 3)
set(${PROJECT_NAME}_MINOR_VERSION 1)
set(${PROJECT_NAME}_PATCH_VERSION 0)
set(${PROJECT_NAME}_VERSION
    ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

# Finding dependencies
find_package(YARP 3.0.101 REQUIRED)
find_package(Boost REQUIRED serialization system)
find_package(Protobuf REQUIRED)
find_package(Gazebo REQUIRED)
if (Gazebo_VERSION_MAJOR LESS 7.0)
    message(status "Gazebo version : " ${Gazebo_VERSION_MAJOR}.${Gazebo_VERSION_MINOR}.${Gazebo_VERSION_PATCH})
    message(FATAL_ERROR "Your Gazebo version is older than Gazebo 7.0. Gazebo Yarp plugins are supported with gazebo versions >= 7.0. Please update to a newer version")
endif()
find_package(SDFormat REQUIRED)
find_package(OGRE QUIET)

# Add Gazebo CXX flags, to support Gazebo 6 reckless dependency on C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# add local cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#used for dir suffixes
include(GNUInstallDirs)
if (CMAKE_VERSION LESS 3.0)
    # hack: by default this would be 'lib/x86_64-linux-gnu' on linux
    set(CMAKE_INSTALL_LIBDIR lib)
endif()

OPTION(GAZEBO_YARP_PLUGINS_ENABLE_RPATH "Enable installation with RPATH" TRUE)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          DEPENDS GAZEBO_YARP_PLUGINS_ENABLE_RPATH
                          USE_LINK_PATH)

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

# Install the files necessary to call find_package(GazeboYarpPlugins) in CMake projects
include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VARS_PREFIX ${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            TARGETS_PROPERTY ${PROJECT_NAME}_TARGETS
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)


# add a dox target to generate doxygen documentation
add_subdirectory(doc)
