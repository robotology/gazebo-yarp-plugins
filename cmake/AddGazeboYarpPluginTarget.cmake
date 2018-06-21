#.rst:
# AddGazeboYarpPluginTarget
# ----------------------
#
#
#
#=======================================================================
# Copyright 2014 RBCS, Istituto Italiano di Tecnologia
# @author Francesco Romano <francesco.romano@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=======================================================================
# (To distribute this file outside of CMake, substitute the full
# License text for the above reference.)


include(CMakeParseArguments)

macro(ADD_GAZEBO_YARP_PLUGIN_TARGET)

set(options BUILD_AS_MODULE)
set(oneValueArgs    LIBRARY_NAME)
set(multiValueArgs  INCLUDE_DIRS
                    SYSTEM_INCLUDE_DIRS
                    LINKED_LIBRARIES
                    HEADERS
                    SOURCES
                )

cmake_parse_arguments(GAZEBO_PLUGIN "${options}"
                                    "${oneValueArgs}"
                                    "${multiValueArgs}"
                                    "${ARGN}")

include_directories(${GAZEBO_PLUGIN_INCLUDE_DIRS})
include_directories(SYSTEM ${GAZEBO_PLUGIN_SYSTEM_INCLUDE_DIRS})

SOURCE_GROUP("Source Files" FILES ${GAZEBO_PLUGIN_SOURCES})
SOURCE_GROUP("Header Files" FILES ${GAZEBO_PLUGIN_HEADERS})

if(NOT DEFINED GAZEBO_PLUGIN_LIBRARY_NAME OR NOT GAZEBO_PLUGIN_LIBRARY_NAME)
    set(GAZEBO_PLUGIN_LIBRARY_NAME ${PROJECT_NAME})
else()
    set(GAZEBO_PLUGIN_LIBRARY_NAME "gazebo_yarp_${GAZEBO_PLUGIN_LIBRARY_NAME}")
endif()

set(LIBRARY_TYPE SHARED)
if (GAZEBO_PLUGIN_BUILD_AS_MODULE)
    set(LIBRARY_TYPE MODULE)
endif()

add_library(${GAZEBO_PLUGIN_LIBRARY_NAME} ${LIBRARY_TYPE} ${GAZEBO_PLUGIN_SOURCES} ${GAZEBO_PLUGIN_HEADERS})
target_link_libraries(${GAZEBO_PLUGIN_LIBRARY_NAME} ${GAZEBO_PLUGIN_LINKED_LIBRARIES})

target_include_directories(${GAZEBO_PLUGIN_LIBRARY_NAME} PUBLIC 
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# Add install target
install(TARGETS ${GAZEBO_PLUGIN_LIBRARY_NAME}
        EXPORT GazeboYARPPlugins
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")

endmacro()
