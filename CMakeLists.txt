# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 3.5)
PROJECT(GazeboYARPPlugins)

# Project version
set(${PROJECT_NAME}_MAJOR_VERSION 3)
set(${PROJECT_NAME}_MINOR_VERSION 6)
set(${PROJECT_NAME}_PATCH_VERSION 1)

set(${PROJECT_NAME}_VERSION
    ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

# Build all the plugins in the same directory to simplify running tests
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")


#
option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
    enable_testing()
endif()

# Finding dependencies
find_package(OpenCV QUIET)
option(GAZEBO_YARP_PLUGINS_HAS_OPENCV "Compile plugins that depend on OpenCV" ${OpenCV_FOUND})
option(GAZEBO_YARP_PLUGINS_HAS_YARP_ROBOTINTERFACE "Compile plugins that depend on libYARP_robotinterface" ON)

if(GAZEBO_YARP_PLUGINS_HAS_OPENCV)
    find_package(OpenCV REQUIRED)
    set(YARP_ADDITIONAL_COMPONENTS_REQUIRED "cv")
else()
    set(YARP_ADDITIONAL_COMPONENTS_REQUIRED "")
endif()

if(GAZEBO_YARP_PLUGINS_HAS_YARP_ROBOTINTERFACE)
    list(APPEND YARP_ADDITIONAL_COMPONENTS_REQUIRED "robotinterface")
endif()

find_package(YARP 3.4.102 REQUIRED COMPONENTS os sig dev math idl_tools ${YARP_ADDITIONAL_COMPONENTS_REQUIRED})
find_package(Gazebo REQUIRED)
if (Gazebo_VERSION_MAJOR LESS 7.0)
    message(status "Gazebo version : " ${Gazebo_VERSION_MAJOR}.${Gazebo_VERSION_MINOR}.${Gazebo_VERSION_PATCH})
    message(FATAL_ERROR "Your Gazebo version is older than Gazebo 7.0. Gazebo Yarp plugins are supported with gazebo versions >= 7.0. Please update to a newer version")
endif()

# Custom workaround for chocolatey distributed Gazebo and OGRE
# See https://github.com/ms-iot/ROSOnWindows/issues/48 for the original issue
if(MSVC)
    find_package(OGRE REQUIRED CONFIG)
    # It is tipically better to use target_include_directories in place of
    # global include_directories, but in this case it is more compact to use include_directories
    include_directories(${OGRE_Paging_INCLUDE_DIRS})
    
    # Workaround for https://github.com/robotology/gazebo-yarp-plugins/issues/482
    add_definitions(-D__TBB_NO_IMPLICIT_LINKAGE=1)
endif()

# On Windows, export all symbols by default as *nix
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# add local cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#used for dir suffixes
include(GNUInstallDirs)

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

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# add a dox target to generate doxygen documentation
add_subdirectory(doc)


