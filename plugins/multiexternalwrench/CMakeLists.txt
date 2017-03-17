cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_MultiExternalWrench)

include(AddGazeboYarpPluginTarget)

set(multiexternalwrench_headers include/externalwrench.h include/applymultiexternalwrench.h)
set(multiexternalwrench_sources src/externalwrench.cpp src/applymultiexternalwrench.cpp)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

add_gazebo_yarp_plugin_target(LIBRARY_NAME multiexternalwrench
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS ${multiexternalwrench_headers}
                              SOURCES ${multiexternalwrench_sources}
                                 )


