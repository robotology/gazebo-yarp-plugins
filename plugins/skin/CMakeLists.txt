PROJECT(Plugin_Skin)

include(AddGazeboYarpPluginTarget)


add_gazebo_yarp_plugin_target(LIBRARY_NAME skin
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFORMAT_INCLUDE_DIRS}
                              LINKED_LIBRARIES gazebo_yarp_lib_common gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin  ${Boost_LIBRARIES}
                              HEADERS include/gazebo/GazeboYarpSkin.hh
                              SOURCES src/GazeboYarpSkin.cc
                                 )

