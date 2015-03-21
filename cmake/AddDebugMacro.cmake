#.rst:
# AddDebugMacro
# ------------------
#
# Add the DEBUG macro for your project
#
# ::
#
# include(AddDebugMacro)
#
#
#=============================================================================
# Copyright 2008-2013 Kitware, Inc.
# Copyright 2014 RBCS, Istituto Italiano di Tecnologia
# @author Francesco Romano <francesco.romano@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
# License text for the above reference.)
if(DEFINED __ADD_DEBUG_MACRO_INCLUDED)
return()
endif()
set(__ADD_DEBUG_MACRO_INCLUDED TRUE)
if(CMAKE_VERSION VERSION_GREATER 2.8.10)
    #define debug flag
    set_property(DIRECTORY APPEND PROPERTY
      COMPILE_DEFINITIONS $<$<CONFIG:Debug>:DEBUG=1>
    )
else()
    #define debug flag
    set_property(
        DIRECTORY
        APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG=1
    )
endif()
