# Copyright (C) 2020 Istituto Italiano di Tecnologia
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

include(FetchContent)

FetchContent_Declare(
  gazebo-classic-gtest
  GIT_REPOSITORY https://github.com/robotology-dependencies/gazebo-classic-gtest.git
  GIT_TAG         gazebo11_11.3.0
)

FetchContent_GetProperties(gazebo-classic-gtest)
if(NOT gazebo-classic-gtest_POPULATED)
  FetchContent_Populate(gazebo-classic-gtest)
  add_library(gyp-gazebo-classic-gtest STATIC ${gazebo-classic-gtest_SOURCE_DIR}/src/gtest-all.cc)
  target_include_directories(gyp-gazebo-classic-gtest PUBLIC ${gazebo-classic-gtest_SOURCE_DIR} ${gazebo-classic-gtest_SOURCE_DIR}/include)
endif()

add_executable(ControlBoardControlTest ControlBoardControlTest.cc)
target_include_directories(ControlBoardControlTest PUBLIC ${GAZEBO_INCLUDE_DIRS})
find_library(GAZEBO_TEST_LIB NAMES gazebo_test_fixture HINTS ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(ControlBoardControlTest PUBLIC ${GAZEBO_LIBRARIES} ${GAZEBO_TEST_LIB} gyp-gazebo-classic-gtest YARP::YARP_dev YARP::YARP_os ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} gazebo_yarp_lib_common gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(ControlBoardControlTest PRIVATE -DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(ControlBoardControlTest PRIVATE -DCMAKE_CURRENT_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(ControlBoardControlTest PRIVATE -DGAZEBO_YARP_PLUGINS_DIR="$<TARGET_FILE_DIR:gazebo_yarp_controlboard>")
add_test(NAME ControlBoardControlTest COMMAND ControlBoardControlTest)
# Ensure that YARP devices can be found
set_property(TEST ControlBoardControlTest PROPERTY ENVIRONMENT YARP_DATA_DIRS=${YARP_DATA_INSTALL_DIR_FULL})

if(GAZEBO_YARP_PLUGINS_HAS_YARP_ROBOTINTERFACE)
  add_subdirectory(robotinterface)
endif()

# Workaround for https://github.com/robotology/gazebo-yarp-plugins/issues/530
if(NOT APPLE)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
  include(Fetchtiny-process-library)
  add_subdirectory(clock)
endif()
