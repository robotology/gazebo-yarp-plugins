add_subdirectory(camera)
add_subdirectory(clock)
add_subdirectory(controlboard)

# Disable externalwrench plugin if Gazebo version is < 8.0
if (NOT GAZEBO_VERSION VERSION_LESS 8.0)
    add_subdirectory(externalwrench)
endif()

add_subdirectory(fakecontrolboard)
add_subdirectory(forcetorque)
add_subdirectory(imu)
add_subdirectory(inertialmtb)
add_subdirectory(inertialmtbPart)
add_subdirectory(showmodelcom)
add_subdirectory(multicamera)
add_subdirectory(maissensor)

if(GAZEBO_YARP_PLUGINS_HAS_OPENCV)
    add_subdirectory(videotexture)
else()
    message(STATUS "GAZEBO_YARP_PLUGINS_HAS_OPENCV CMake option set to off, disabling videotexture plugin.")
endif()

add_subdirectory(worldinterface)
add_subdirectory(linkattacher)
add_subdirectory(lasersensor)
add_subdirectory(depthCamera)


add_subdirectory(contactloadcellarray)
add_subdirectory(modelposepublisher)
add_subdirectory(basestate)
add_subdirectory(configurationoverride)

if(GAZEBO_YARP_PLUGINS_HAS_YARP_ROBOTINTERFACE)
  add_subdirectory(robotinterface)
endif()
