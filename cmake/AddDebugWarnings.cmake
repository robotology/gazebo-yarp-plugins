#.rst:
# AddDebugWarnings
# ------------------
#
# Enable all the possible warnings if the CMAKE_BUILD_TYPE is DEBUG
#
# ::
#
# include(AddDebugWarnings)
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
if(DEFINED __ADD_DEBUG_WARNINGS_INCLUDED)
return()
endif()
set(__ADD_DEBUG_WARNINGS_INCLUDED TRUE)
if(MSVC)
    ###
else()
    ##Other systems
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Weverything -pedantic -Wnon-virtual-dtor -Woverloaded-virtual")
        #disable padding alignment warnings. Cast align is more subtle. On X86 it should not create any problem but for different architecture we should handle this warning better.
        SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wno-padded -Wno-cast-align")
        #MESSAGE(STATUS "Clang compiler - Debug configuration flags: -Weverything -pedantic -Wnon-virtual-dtor -Woverloaded-virtual")
    elseif(${CMAKE_COMPILER_IS_GNUCC})
        SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -Weffc++ -Woverloaded-virtual")
        #MESSAGE(STATUS "Gcc compiler - Debug configuration flags: -Wall -Wextra -pedantic -Weffc++ -Woverloaded-virtual")
    endif()
endif()

