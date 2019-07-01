add_subdirectory(camera)
add_subdirectory(clock)
add_subdirectory(controlboard)
add_subdirectory(externalwrench)
add_subdirectory(fakecontrolboard)
add_subdirectory(forcetorque)
add_subdirectory(imu)
add_subdirectory(inertialmtb)
add_subdirectory(inertialmtbPart)

# This plugin will be removed soon ( https://github.com/robotology/gazebo-yarp-plugins/issues/213 )
# for the time being do not update it for compliance with Gazebo 9
if (GAZEBO_VERSION VERSION_LESS 9.0)
    add_subdirectory(jointsensors)
endif()

add_subdirectory(showmodelcom)
add_subdirectory(multicamera)
add_subdirectory(maissensor)

find_package(OpenCV QUIET)
option(GAZEBO_YARP_PLUGINS_HAS_OPENCV "Compile plugins that depend on OpenCV" ${OpenCV_FOUND})

if(GAZEBO_YARP_PLUGINS_HAS_OPENCV)
    add_subdirectory(videotexture)
else()
    message(STATUS "GAZEBO_YARP_PLUGINS_HAS_OPENCV CMake option set to off, disabling videotexture plugin.")
endif()

add_subdirectory(worldinterface)
add_subdirectory(linkattacher)
add_subdirectory(lasersensor)
add_subdirectory(doublelaser)
add_subdirectory(depthCamera)


add_subdirectory(contactloadcellarray)
add_subdirectory(modelposepublisher)
add_subdirectory(basestate)
add_subdirectory(configurationoverride)