# Copyright (C) 2020 Istituto Italiano di Tecnologia
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

add_executable(ClockTest ClockTest.cc)
target_include_directories(ClockTest PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(ClockTest PUBLIC ${GAZEBO_LIBRARIES} ${GAZEBO_TEST_LIB} gyp-gazebo-classic-gtest YARP::YARP_dev YARP::YARP_os ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} gazebo_yarp_lib_common gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(ClockTest PRIVATE -DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(ClockTest PRIVATE -DCMAKE_CURRENT_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(ClockTest PRIVATE -DGAZEBO_YARP_PLUGINS_DIR="$<TARGET_FILE_DIR:gazebo_yarp_clock>")
target_compile_definitions(ClockTest PRIVATE -DYARP_SERVER_LOCATION="${YARP_INSTALL_PREFIX}/bin/yarpserver")
add_test(NAME ClockTest COMMAND ClockTest)
set_tests_properties(ClockTest PROPERTIES TIMEOUT 15) 
# Ensure that YARP devices can be found
# Disable use of online model database
set_property(TEST ClockTest PROPERTY ENVIRONMENT YARP_DATA_DIRS=${YARP_DATA_INSTALL_DIR_FULL};GAZEBO_MODEL_DATABASE_URI=) 
# As this test does not use localmode, launch yarpserver explicitly via tiny-process-library
target_link_libraries(ClockTest PUBLIC tiny-process-library)