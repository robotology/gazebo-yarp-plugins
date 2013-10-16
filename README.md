gazebo_yarp_plugins
===================

Gazebo plugin to interface GAZEBO with YARP.

# Setup OS

Ubuntu 13.04 system with gazebo 1.9 and Ros hydro

# Get coman urdf

clone the repository iit-coman-ros-pkg
cd coman_urdf/script
source /usr/share/ros-xxx/setup.sh
./create_bla_bla_bla coman_robot_plain.urdf.xacro
Note: we need to fix the use of ROS to get the sdf file

# Compile the libyarp_gazebo

clone the repository
mkdir build
cd build
cmake ..
make
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_yarp_plugins/build
Run

'''
yarpserver
gazebo yarp_gazebo.world
'''
You should have something like this:

'''
Robot Name: COMAN
Joints: 23
Links: 24
ControlBoard subdevice is fakebot
Joint Limits
COMAN::LHipSag max_pos: 45.0001 min_pos: -110.002
COMAN::LHipLat max_pos: 60.0001 min_pos: -24.9981
COMAN::LHipYaw max_pos: 50.002 min_pos: -50.002
COMAN::LKneeSag max_pos: 110.002 min_pos: -9.99811
COMAN::LAnkLat max_pos: 35.002 min_pos: -35.002
COMAN::LAnkSag max_pos: 69.9983 min_pos: -69.9983
COMAN::RHipSag max_pos: 45.0001 min_pos: -110.002
COMAN::RHipLat max_pos: 24.9981 min_pos: -60.0001
COMAN::RHipYaw max_pos: 50.002 min_pos: -50.002
COMAN::RKneeSag max_pos: 110.002 min_pos: -9.99811
COMAN::RAnkLat max_pos: 35.002 min_pos: -35.002
COMAN::RAnkSag max_pos: 69.9983 min_pos: -69.9983
COMAN::WaistSag max_pos: 29.9943 min_pos: -30.0001
COMAN::WaistLat max_pos: 50.002 min_pos: -20.002
COMAN::WaistYaw max_pos: 79.9964 min_pos: -80.0021
COMAN::LShSag max_pos: 95.0021 min_pos: -195
COMAN::LShLat max_pos: 119.977 min_pos: -17.9995
COMAN::LShYaw max_pos: 90.0002 min_pos: -90.0002
COMAN::LElbj max_pos: 0 min_pos: -135
COMAN::RShSag max_pos: 95.0021 min_pos: -195
COMAN::RShLat max_pos: 17.9995 min_pos: -119.977
COMAN::RShYaw max_pos: 90.0002 min_pos: -90.0002
COMAN::RElbj max_pos: 0 min_pos: -135
FakeBot thread started
yarp: created device <fakebot>. See C++ class fakebot for documentation.
'''
