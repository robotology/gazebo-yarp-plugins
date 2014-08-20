gazebo-yarp-plugins
===================
[![Build Status](https://travis-ci.org/robotology/gazebo-yarp-plugins.png)](https://travis-ci.org/robotology/gazebo-yarp-plugins)

Plugins for exposing [Yarp](http://yarp.it/) interfaces on [Gazebo simulator](http://gazebosim.org/) models.

Installation
------------
### Dependencies
For using Yarp with Gazebo, you shall install:
 * Gazebo simulator and its header files (at least version 1.9), following the [instructions on the official Gazebo website](http://gazebosim.org/wiki/Install).
 * Yarp (at least version 2.3.62, i.e. the version available in the master branch of the yarp repository) following the [instructions on the official Yarp wiki](http://wiki.icub.org/wiki/Linux:Installation_from_sources#Getting_the_YARP_and_iCub_sources) and enabling the CREATE_SHARED_LIBRARY CMake option to compile Yarp as a shared library.

**Gazebo is under active development, so it is recommended to use the latest released version of Gazebo.**

*You could prefer to run an older version of Gazebo if you want to use it with ROS integration. Depending on your ROS version you have to stick to a given Gazebo version.*
*Yarp integration provided by gazebo-yarp-plugins is not affected by this kind of limitations.*

###Compile time dependencies
For compiling gazebo-yarp-plugins you need the headers for the following libraries:
  * [tinyxml](http://www.grinninglizard.com/tinyxml/)
  * [Boost System](http://www.boost.org/doc/libs/release/libs/system/)

For example on Ubuntu you can install them with the following command:
```
sudo apt-get install libtinyxml-dev libboost-system-dev
```
If you are using Gazebo 3 (or any later version) installed using .deb packages, you need 
also to install the Gazebo development files. This files are provided by the `libgazebo-dev` package. 

On OS X you can instead use brew:
```
brew install tinyxml boost
```



### Operating systems support
Linux and OS X are currently supported by Gazebo.
OS X support in Gazebo is still experimental, and there could be problems.

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
You can use CMake to generate the necessary file for the compilation, and compile gazebo-yarp-plugins using make:
```
cd build
cmake ../
make
```

To notify Gazebo of the new plugins compiled, it is necessary to modify the GAZEBO_PLUGIN_PATH enviroment variable:
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/path/to/gazebo-yarp-plugins/build
```
Where "/path/to/gazebo-yarp-plugins/build" is the path on your computer where you located the build directory.
To avoid having to modify this enviroment variable each time, you can place this command in the .bashrc file in your home directory.

You can decide to install the plugins by issuing a
~~~
sudo make install
~~~

at which point you need to update the GAZEBO_PLUGIN_PATH accordingly, if the installation path is not in the LD_LIBRARY_PATH of your system

Usage
-----
To use the gazebo-yarp-plugins you can try to use a Yarp-enabled Gazebo model of a robot. Currently two robot support gazebo-yarp-plugins: Coman and iCub.

### Coman
To use Coman in Gazebo, please follow [the instructions on gazebo-yarp-plugins wiki](https://github.com/robotology/gazebo-yarp-plugins/wiki/Using-Coman-model-with-gazebo-yarp-plugins)

### iCub
To use iCub in Gazebo, please follow [the instruction in the icub_gazebo repository](https://github.com/robotology-playground/icub_gazebo)

### Arbitrary robot
To add gazebo-yarp-plugins to another robot, please follow the instructions on [gazebo-yarp-plugins wiki](https://github.com/robotology/gazebo-yarp-plugins/wiki/Embed-gazebo-yarp-plugins-in-an-SDF-model)

Troubleshooting
---------------

#### General 
- If gazebo-yarp-plugins does not compile complaning about an -fPIC option missing, check that you compiled yarp with shared library option enabled.

- If Gazebo complains about not finding the libgazebo_yarp_*.so files, check if you properly defined the GAZEBO_PLUGIN_PATH enviroment variable.

- If Gazebo complains that a yarp server is not available, remember to launch a yarp server on your machine (i.e. open a new terminal a launch the yarpserver command).

- If the simulation run slowly, check that you are using the latest graphics driver available for your platform. If you a running a recent Ubuntu version (13.10 or 14.04) you can try (at your own risk) to use the drivers provided by the [oibaf PPA](https://launchpad.net/~oibaf/+archive/graphics-drivers).

- If Gazebo crashes without any error message while loading a model using gazebo-yarp-plugins, you have issue with shared library linking. Probably gazebo-yarp-plugins is linked against a manually installed library (for example boost), while the gazebo binary is linking against the system version of the same library. For more information read [issue  71 discussion](https://github.com/robotology/gazebo-yarp-plugins/issues/71) or file [a new issue](https://github.com/robotology/gazebo-yarp-plugins/issues/new) to get help from the developers.

#### Old Gazebo Versions
- In versions of Gazebo prior to 3.0 there is a bug related to the integral part of the low-level position controller. If you are using 
  Gazebo 1.9 or 2.2 then the low level position control will excert no integral action on the model. If you want to get more information on
  this bug, please check [issue 119](https://github.com/robotology/gazebo-yarp-plugins/issues/119) and [Gazebo issue 1082] (https://bitbucket.org/osrf/gazebo/issue/1082/jointcontroller-does-not-handle-correctly). 

- In versions of Gazebo prior to 3.1 there is a bug related to the coordinates frame of the six axis force torque sensor measure, so you 
  have to handle with care the force torque measurement returned by the `gazebo_yarp_forcetorque` plugin. For more information check [issue 73]( https://github.com/robotology/gazebo-yarp-plugins/issues/73). 

Design
------
More information on the internal structure of gazebo-yarp-plugins is [available in this wiki page](https://github.com/robotology/gazebo-yarp-plugins/wiki/Design).


Contributing
------------
See [Contributing File](CONTRIBUTING.md)
