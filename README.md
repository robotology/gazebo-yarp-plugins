gazebo_yarp_plugins
===================
[![Build Status](https://travis-ci.org/robotology/gazebo_yarp_plugins.png)](https://travis-ci.org/robotology/gazebo_yarp_plugins)



Gazebo plugin to interface a Coman in GAZEBO with a YARP interface.

# Setup OS

Ubuntu 13.04 system with gazebo 1.9 and Ros hydro

Get coman sdf from the URDF sources
------------------------

Clone the repository iit-coman-ros-pkg

Inside the cloned folder do the following:
```
cd coman_urdf/script
source /usr/share/ros-xxx/setup.sh
./create_bla_bla_bla coman_robot_plain.urdf.xacro
```
Note: we need to fix the use of ROS to get the sdf file

Another way to get coman sdf without having to use ROS
-----------------------

Clone the repository github.com/EnricoMingo/iit-coman-ros-pkg

Inside the cloned folder search for coman_gazebo/sdf folder.
Copy all the folder contents to ~/.gazebo/models/coman_urdf/

So you should have something like this:
~/.gazebo/models/coman_urdf/coman.sdf
~/.gazebo/models/coman_urdf/conf/...
~/.gazebo/models/coman_urdf/meshes/...
...

Done.


Get and compile the custom yarp branch
-------------------------

Clone the repository github.com/barbalberto/yarp/tree/wrappers
Inside the cloned folder do:
```
mkdir build
cd build
cmake ..
ccmake ..
```

Now enable the option "Shared library". Press c to configure. Press g to confirm.

```
make
sudo make install
```

# Compile the libyarp_gazebo

Finally clone this repository
```
mkdir build
cd build
cmake ..
make
```
Please change the following line to reflect your folder structure, do NOT simply copy and paste:
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_yarp_plugins/build
```

Run in a shell (don't forget this one!):
```
yarpserver
```

In the same shell where you exported the GAZEBO_PLUGIN_PATH start gazebo like this:
```
gazebo yarp_gazebo.world
```

You should have something like this:

```
>Robot Name: COMAN
>#Joints: 23
>#Links: 24
>ControlBoard subdevice is coman/test
>Joint Limits
>COMAN::LHipSag max_pos: 45.0001 min_pos: -110.002
>COMAN::LHipLat max_pos: 60.0001 min_pos: -24.9981
>COMAN::LHipYaw max_pos: 50.002 min_pos: -50.002
>COMAN::LKneeSag max_pos: 110.002 min_pos: -9.99811
>COMAN::LAnkLat max_pos: 35.002 min_pos: -35.002
>COMAN::LAnkSag max_pos: 69.9983 min_pos: -69.9983
>COMAN::RHipSag max_pos: 45.0001 min_pos: -110.002
>COMAN::RHipLat max_pos: 24.9981 min_pos: -60.0001
>COMAN::RHipYaw max_pos: 50.002 min_pos: -50.002
>COMAN::RKneeSag max_pos: 110.002 min_pos: -9.99811
>COMAN::RAnkLat max_pos: 35.002 min_pos: -35.002
>COMAN::RAnkSag max_pos: 69.9983 min_pos: -69.9983
>COMAN::WaistSag max_pos: 29.9943 min_pos: -30.0001
>COMAN::WaistLat max_pos: 50.002 min_pos: -20.002
>COMAN::WaistYaw max_pos: 79.9964 min_pos: -80.0021
>COMAN::LShSag max_pos: 95.0021 min_pos: -195
>COMAN::LShLat max_pos: 119.977 min_pos: -17.9995
>COMAN::LShYaw max_pos: 90.0002 min_pos: -90.0002
>COMAN::LElbj max_pos: 0 min_pos: -135
>COMAN::RShSag max_pos: 95.0021 min_pos: -195
>COMAN::RShLat max_pos: 17.9995 min_pos: -119.977
>COMAN::RShYaw max_pos: 90.0002 min_pos: -90.0002
>COMAN::RElbj max_pos: 0 min_pos: -135
>yarp: created device <controlboard>. See C++ class controlboard for documentation.
```

Testing and moving coman joints
==================

Position control
------------------

Run in another shell: 
```
./testmotor --robot coman --part test
```
This is a simple_control interface taken from 
http://wiki.icub.org/iCub_documentation/main_2src_2tools_2simpleClient_2main_8cpp_source.html

To get help type:
```
help
```
To enable the position control (default)
```
-> icmd set cmp joint
```
To set the jth joint position to angle type:
```
set pos j angle
```
To get the angles of the joints type:
```
get encs
```

You can also set the reference speed:

Set the reference speed (should be degrees/second, need to check)
```
-> set vel 20 10
```
Set the new position
```
-> set pos 20 -40
```

Velocity control
------------------
Start by changing  the robot control style
```
-> icmd set cmv joint
```
e.g.
```
-> icmd set cmv 20
```
set the desired velocity (degree/secods)

```
-> set vmo joint velocity
```
e.g.
```
-> set vmo 20 10
```

Torque control
------------------
Start by changing  the robot control style
```
-> icmd set cmt joint
```
e.g.
```
-> icmd set cmt 0
```
set the desired torque (Nm)

```
-> torq set ref joint torque
```
e.g.
```
-> torq set ref 0 -0.7
```


This is all you can do with coman inside gazebo at the moment.

Testing and moving coman joints with the robotMotorGui
------------------
You can also use the robotMotorGui for controlling the coman simulation.

You should start the coman simulation with 
```
gazebo coman_parts.world
```
To instantiate a coman simulating the 5 different control boards (torso,left_leg,right_leg,left_arm,right_arm).

Please make sure you have the right configuration files for the control boards (they should be in your coman_urdf package in the conf/ subdirectory), if necessary
download the latest coman_urdf.tar.gz.

You can then execute the robotMotorGui:
```
robotMotorGui --name coman --parts "(torso left_arm right_arm left_leg right_leg)"
```

Plot IMU & FT data:
--------------
Inside tools folder run
```
yarpscope --xml imu_yarp_scope.xml
```
or
```
yarpscope --xml FT_yarp_scope.xml
```


Troubleshooting
=============
- If the plugin does not compile complaning about an -fPIC option missing, check that you compiled yarp with shared library option enabled

- If gazebo complains about not finding the libyarp, check if you exported the GAZEBO_PLUGIN_PATH in the same shell where you are launching gazebo

- If gazebo complains about not finding coman model, maybe you forgot to compile the URDF and run the script provided in the URDF git repository (or to extract the files from the coman.tar.gz)

- If you have problems with the Coman inside gazebo starting in strange positions, try
```
gazebo --pause yarp_gazebo.world
```
and check what is the startup configuration. The starting position can be changed inside the *.world file.

- The coman starts with all the joints enabled and controlled by a PID to 0.
  At the moment it is not so easy to change PID gains, we will implement this and other control modes soon.

- Remember to start yarpserver in another shell, otherwise gazebo will run without a working coman interface (we will make gazebo crash in the next versions)

