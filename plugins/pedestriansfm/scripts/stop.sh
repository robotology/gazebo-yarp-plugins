#!/usr/bin/env bash

killall -9 gazebo
killall -9 gzserver
killall -9 gzclient
killall -9 baseControl2
killall -9 yarpmobilebasegui
killall -9 state_publisher
killall -9 amcl
killall -9 rviz2
killall -9 navigationGui
killall -9 yarpmanager
yarp clean --timeout 0.1
killall -9 rqt
killall -9 yarpserver
killall -9 yarp
killall -9 yarplogger
killall -9 yarview
killall -9 yarprobotinterface
killall -9 yarprun
killall -9 yarpdev
