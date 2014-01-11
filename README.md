gazebo_yarp_plugins
===================
[![Build Status](https://travis-ci.org/robotology/gazebo_yarp_plugins.png)](https://travis-ci.org/robotology/gazebo_yarp_plugins)

Plugins for exposing [Yarp](http://yarp.it/) interfaces on [Gazebo simulator](http://gazebosim.org/) models. 

Installation
------------
### Dependencies 
For using Yarp with the Gazebo simulator, you should install:
 * Gazebo simulator (at least version 2.0), following the [instructions on the official Gazebo website](http://gazebosim.org/wiki/Install).
 * Yarp (in the version available in the master branch of the yarp repository) following the [instructions on the official Yarp wiki](http://wiki.icub.org/wiki/Linux:Installation_from_sources#Getting_the_YARP_and_iCub_sources) and enabling the CREATE_SHARED_LIBRARY CMake option to compile Yarp as a shared library.
 
At the moment (26/11/13) you can have issues in running yarp compiled from source. If you are using Ubuntu 13.04 you will have to face some problems due to the new multiarch support.
[Have a look here why ld will not find yarp in /usr/local/lib/x86_64-linux-gnu](https://help.ubuntu.com/community/MultiArch)
The solution is to manually add /usr/local/lib/x86_64-linux-gnu in the config file /etc/ld.so.conf.d/x86_64-linux-gnu.conf and do ```sudo ldconfig```.

###Compile time dependencies
For compiling gazebo_yarp_plugins you need the headers for the following libraries:
  * [tinyxml](http://www.grinninglizard.com/tinyxml/)
  * [Boost System](www.boost.org/doc/libs/release/libs/system/)

For example on Ubuntu you can install them with the following command:
```
sudo apt-get install libtinyxml-dev libboost-system-dev
```
On OS X you can instead use brew:
```
brew install tinyxml boost
```



### Operating systems support 
Linux and OS X are currently supported by Gazebo. 
OS X support in Gazebo is still experimental, and there could be problems.

### Compilation 
You get the gazebo_yarp_plugins source code from this git repository repository (if you do not have git on your computer, [follow this guide to install it](http://git-scm.com/downloads))
```
git https://github.com/robotology/gazebo_yarp_plugins.git
```
This will create a gazebo_yarp_plugins directory with all the source code.
You can enter this directory:
```
cd gazebo_yarp_plugins
```
You can then create a build directory inside it to hold all the compiled files:
```
mkdir build
```
You can use CMake to generate the necessary file for the compilation, and compile gazebo_yarp_plugins using make:
```
cd build
cmake ../
make
```

To notify Gazebo of the new plugins compiled, it is necessary to modify the GAZEBO_PLUGIN_PATH enviroment variable: 
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/path/to/gazebo_yarp_plugins/build
```
Where "/path/to/gazebo_yarp_plugins/build" is the path on your computer where you located the build directory.
To avoid having to modify this enviroment variable each time, you can place this command in the .bashrc file in your home directory.

Setting Initial Configuration for a Kinematic Chain:
----------------------------------------------------
One of the tags of the plugin is:
```
<initialConfiguration></initialConfiguration>
```
that can be used to set an initial configuration for a particular kinematic chain. 
Suppose that you want to set the initial configuration for the left_arm of COMAN, if it has 5 DOFs so you will write something like:
```
<plugin name="coman_yarp_gazebo_plugin_left_arm" filename="libgazebo_yarp_controlboard.so">
	 <yarpConfigurationFile>model://coman_urdf/conf/coman/coman_gazebo_left_arm.ini</yarpConfigurationFile>
	 <initialConfiguration>0.0 0.17 0.0 0.0 0.0</initialConfiguration>
</plugin>
```
inside your world file. Notice that the configuration is specified in RADIANTS.

Usage
-----
To use the gazebo_yarp_plugins you can try to use a Yarp-enabled Gazebo model of a robot. Currently two robot support gazebo_yarp_plugins: Coman and iCub.

### Coman
To use Coman in Gazebo, please follow [the instructions on gazebo_yarp_plugins wiki](https://github.com/robotology/gazebo_yarp_plugins/wiki/Using-Coman-model-with-gazebo_yarp_plugins)

### iCub 
To use iCub in Gazebo, please follow [the instruction in the icub_gazebo repository](https://github.com/traversaro/icub_gazebo)

Design
------
More information on the internal structure of gazebo_yarp_plugins is [available in this wiki page](https://github.com/robotology/gazebo_yarp_plugins/wiki/Design).

Contributing
------------
If you would like to contribute to the development of gazebo_yarp_plugins, please get in contact with the development team using GitHub issues. 



