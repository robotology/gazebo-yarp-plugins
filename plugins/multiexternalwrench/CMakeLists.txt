cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_MultiExternalWrench)

include(AddGazeboYarpPluginTarget)

add_gazebo_yarp_plugin_target(LIBRARY_NAME multiexternalwrench
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo
                              SYSTEM_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS include/externalwrenchserver.h include/multiexternalwrenchinterface.h
                              SOURCES src/externalwrenchserver.cpp src/multiexternalwrenchinterface.cpp
                                 )


