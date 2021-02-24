Troubleshooting {#troubleshooting}
===============

Troubleshooting
===============

## General
- If gazebo-yarp-plugins does not compile complaning about an `-fPIC` option missing, check that you compiled yarp with shared library option enabled.

- If Gazebo complains about not finding the `libgazebo_yarp_*.so` files, check if you properly defined the `GAZEBO_PLUGIN_PATH` enviroment variable.

- If Gazebo complains that a yarp server is not available, remember to launch a `yarpserver` on your machine (i.e. open a new terminal a launch the `yarpserver` command).

- If Gazebo crashes without any error message while loading a model using gazebo-yarp-plugins, you have issue with shared library linking. Probably gazebo-yarp-plugins is linked against a manually installed library (for example boost), while the gazebo binary is linking against the system version of the same library. For more information read [issue  71 discussion](https://github.com/robotology/gazebo-yarp-plugins/issues/71) or file [a new issue](https://github.com/robotology/gazebo-yarp-plugins/issues/new) to get help from the developers.

## Old Gazebo Versions
- All the plugins compile fine on the latest version of Gazebo, but for some plugins compilation is disabled for old versions of Gazebo, see [plugins/CMakeLists.txt](plugins/CMakeLists.txt).
