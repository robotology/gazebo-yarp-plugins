Downloading and Installing gazebo-yarp-plugins {#install}
==================================================

Downloading and Installing gazebo-yarp-plugins 
==================================================

Please note that only Linux and macOS are currently supported by Gazebo, 
and hence by the `gazebo-yarp-plugins`.


### Dependencies
For using YARP with Gazebo, you shall install:
 * Gazebo simulator and its header files (at least version 7), following the [instructions on the official Gazebo website](http://gazebosim.org/tutorials?cat=install). If you are on Debian or Ubuntu, remember to install the header files for Gazebo, that are contained in the `libgazebo*-dev` package.
 * YARP (at least version 2.3.70, i.e. the version available in the master branch of the yarp repository) following the [instructions on the YARP documentation](hhttp://www.yarp.it/install.html).

**Gazebo is under active development, so it is recommended to use the latest released version of Gazebo.**

**If you are a user of the YARP `devel` branch, you should use the `devel` branch of `gazebo-yarp-plugins`.**

*You could prefer to run an older version of Gazebo if you want to use it with ROS integration. Depending on your ROS version you have to stick to a given Gazebo version.*
*YARP integration provided by gazebo-yarp-plugins is not affected by this kind of limitations.*


### Compilation
You get the gazebo-yarp-plugins source code from this git repository repository (if you do not have git on your computer, [follow this guide to install it](http://git-scm.com/downloads))
```
git clone https://github.com/robotology/gazebo-yarp-plugins.git
```
This will create a gazebo-yarp-plugins directory with all the source code.
You can enter this directory:
```
cd gazebo-yarp-plugins
```
You can then create a build directory inside it to hold all the compiled files:
```
mkdir build
```
You can use CMake to generate the necessary file for the compilation, and compile gazebo-yarp-plugins using make. The plugins should be install in the directory specified by `CMAKE_INSTALL_PREFIX` (by default `/usr/local`):
```
cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=/path/to/the/install/folder
make install
```

To notify Gazebo of the new plugins compiled, it is necessary to modify the GAZEBO_PLUGIN_PATH environment variable:
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/path/to/the/install/folder/lib
```
where `/path/to/the/install/folder/lib` is the directory containing the `libgazebo_yarp_controlboard.so`, `libgazebo_yarp_forcetorque.so`... files.

To avoid having to modify this environment variable each time, you can place this command in the `.bashrc` file in your home directory.

### Usage
To use the gazebo-yarp-plugins you can try to use a YARP-enabled Gazebo model of a robot. 
You can use a robot model already equipped for simulating its YARP-interfaces, such as:
* The iCub model, available at https://github.com/robotology/icub-gazebo .
* The Coman model, available at https://github.com/ADVRHumanoids/iit-coman-ros-pkg .
* The Vizzy model, available at https://github.com/vislab-tecnico-lisboa/vizzy .

To embed the plugins contained in gazebo-yarp-plugins in a new Gazebo model, check \ref embed_plugins . 