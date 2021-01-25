# Copyright (C) 2020 Istituto Italiano di Tecnologia
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


add_executable(RobotInterfaceTest RobotInterfaceTest.cc)
target_include_directories(RobotInterfaceTest PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(RobotInterfaceTest PUBLIC ${GAZEBO_LIBRARIES} ${GAZEBO_TEST_LIB} gyp-gazebo-classic-gtest YARP::YARP_dev YARP::YARP_os ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} gazebo_yarp_lib_common gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(RobotInterfaceTest PRIVATE -DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(RobotInterfaceTest PRIVATE -DCMAKE_CURRENT_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(RobotInterfaceTest PRIVATE -DGAZEBO_YARP_PLUGINS_DIR="$<TARGET_FILE_DIR:gazebo_yarp_robotinterface>")
add_test(NAME RobotInterfaceTest COMMAND RobotInterfaceTest)
# Ensure that YARP devices can be found
# Disable use of online model database
set_property(TEST RobotInterfaceTest PROPERTY ENVIRONMENT YARP_DATA_DIRS=${YARP_DATA_INSTALL_DIR_FULL};GAZEBO_MODEL_DATABASE_URI=)