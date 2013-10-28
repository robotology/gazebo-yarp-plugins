# Copyright: (C) iCub Facility 2013 Istituto Italiano di Tenologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

PROJECT(yarp_gazebo)
list ( APPEND CMAKE_CXX_FLAGS "-std=c++0x -Wall -pedantic -Wmissing-field-initializers -W -Wunused -Wuninitialized -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -fno-strict-aliasing  -Werror=address -Werror=parentheses  ${CMAKE_CXX_FLAGS}" )

find_package(YARP REQUIRED)
find_package(Boost REQUIRED serialization)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${YARP_INCLUDE_DIRS} include)

add_library(yarp_gazebo SHARED src/coman_yarp_gazebo_plugin.cc)
add_library(fakebot SHARED src/coman.cpp )
#add_library(FakebotForceTorquePlugin SHARED src/FakebotForceTorquePlugin.cc)
#add_library(fakebotFTsensor SHARED src/fakebotFTsensor.cpp)

target_link_libraries(yarp_gazebo  fakebot ${YARP_LIBRARIES} ${GAZEBO_libraries} -lboost_system ${Boost_LIBRARIES})
#target_link_libraries(FakebotForceTorquePlugin  fakebotFTsensor ${YARP_LIBRARIES} ${GAZEBO_libraries} ${Boost_LIBRARIES})


add_executable(testmotor src/test/test.cpp)
target_link_libraries(testmotor ${YARP_LIBRARIES} ${GAZEBO_libraries} -lboost_system ${Boost_LIBRARIES})
