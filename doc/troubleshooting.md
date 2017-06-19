Troubleshooting {#troubleshooting}
===============

Troubleshooting
===============

## General
- If gazebo-yarp-plugins does not compile complaning about an -fPIC option missing, check that you compiled yarp with shared library option enabled.

- If Gazebo complains about not finding the libgazebo_yarp_*.so files, check if you properly defined the GAZEBO_PLUGIN_PATH enviroment variable.

- If Gazebo complains that a yarp server is not available, remember to launch a yarp server on your machine (i.e. open a new terminal a launch the yarpserver command).

- If the simulation run slowly, check that you are using the latest graphics driver available for your platform. If you a running a recent Ubuntu version (13.10 or 14.04) you can try (at your own risk) to use the drivers provided by the [oibaf PPA](https://launchpad.net/~oibaf/+archive/graphics-drivers).

- If Gazebo crashes without any error message while loading a model using gazebo-yarp-plugins, you have issue with shared library linking. Probably gazebo-yarp-plugins is linked against a manually installed library (for example boost), while the gazebo binary is linking against the system version of the same library. For more information read [issue  71 discussion](https://github.com/robotology/gazebo-yarp-plugins/issues/71) or file [a new issue](https://github.com/robotology/gazebo-yarp-plugins/issues/new) to get help from the developers.

## Old Gazebo Versions
- All the plugins compile fine on the latest version of Gazebo, but for some plugins compilation is disabled for old versions of Gazebo, see [plugins/CMakeLists.txt](plugins/CMakeLists.txt).
- In versions of Gazebo prior to 7.2 there is a bug related to the use of Reset World with `gazebo-yarp-plugins` powered models. For more informations check [issue 92](https://github.com/robotology/gazebo-yarp-plugins/issues/92) .
- In versions of Gazebo prior to 4.1 there is a bug affecting the use of `gazebo-yarp-plugins` powered model with `roslaunch` . For more informations check [issue 123](https://github.com/robotology/gazebo-yarp-plugins/issues/123) .
- In versions of Gazebo prior to 3.1 there is a bug related to the coordinates frame of the six axis force torque sensor measure, so you
  have to handle with care the force torque measurement returned by the `gazebo_yarp_forcetorque` plugin. For more informations check [issue 73]( https://github.com/robotology/gazebo-yarp-plugins/issues/73).
- In versions of Gazebo prior to 3.0 there is a bug related to the integral part of the low-level position controller. If you are using
  Gazebo 1.9 or 2.2 then the low level position control will excert no integral action on the model. If you want to get more informations on
  this bug, please check [issue 119](https://github.com/robotology/gazebo-yarp-plugins/issues/119) and [Gazebo issue 1082] (https://bitbucket.org/osrf/gazebo/issue/1082/jointcontroller-does-not-handle-correctly).