# Copyright (C) 2014 CoDyCo
# Author: Daniele Pucci, Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME DoublePendulumExample)
PROJECT(${PROJECTNAME})

find_package(YARP)
#find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})


set(SOURCES 
    src/main.cpp 
    )

SOURCE_GROUP("Source Files" FILES ${SOURCES})

# include(iCubHelpers)
# include(iCubOptions)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")


INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
 ${YARP_LIBRARIES})

if(WIN32)
        INSTALL_TARGETS(/bin/Release ${PROJECTNAME})    
else(WIN32)
        INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)
