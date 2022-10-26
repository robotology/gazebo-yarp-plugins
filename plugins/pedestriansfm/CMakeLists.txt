# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Francesco Romano
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(PedestrianSFMPlugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
  SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries")
endif()

include(AddGazeboYarpPluginTarget)

include_directories(/usr/local/src/robot/gazebo-yarp-plugins/libraries/singleton/include)

add_gazebo_yarp_plugin_target(LIBRARY_NAME PedestrianSFMPlugin
                              INCLUDE_DIRS include
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS}
                                                  ${Boost_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${GAZEBO_LIBRARIES}
                                               ${Boost_LIBRARIES}
					                                     YARP::YARP_os
					                                     YARP::YARP_init
                              HEADERS include/PedestrianSFMPlugin/PedestrianSFMPlugin.h
			                                include/PedestrianSFMPlugin/angle.hpp
				                              include/PedestrianSFMPlugin/map.hpp
				                              include/PedestrianSFMPlugin/sfm.hpp
				                              include/PedestrianSFMPlugin/vector2d.hpp	
                              SOURCES src/PedestrianSFMPlugin.cpp)


