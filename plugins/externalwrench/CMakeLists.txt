# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Jorhabib Eljaik, Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


PROJECT(Plugin_ExternalWrench)

include(AddGazeboYarpPluginTarget)

add_gazebo_yarp_plugin_target(LIBRARY_NAME externalwrench
                              INCLUDE_DIRS include/gazebo
                              SYSTEM_INCLUDE_DIRS ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_lib_common ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS include/gazebo/ApplyExternalWrench.hh include/gazebo/ExternalWrench.hh
                              SOURCES src/ApplyExternalWrench.cc src/ExternalWrench.cc
                                 )


