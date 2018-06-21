

# Use the gazebo-yarp-plugins as a C++ library using CMake {#use_as_library}

# Use the gazebo-yarp-plugins as a C++ library using CMake

You can search the `gazebo-yarp-plugins` as a CMake package, using the `GazeboYARPPlugins` CMake package name.

~~~cmake
find_package(GazeboYARPPlugins REQUIRED)
~~~

If the `GazeboYARPPlugins` CMake package, the following imported targets will be defined:

| Target name        | Description          |
|:------------------:|:--------------------:|
| `GazeboYARPPlugins::gazebo_yarp_lib_common` | `INTERFACE` library containing just the `GazeboYarpPlugins/common.h` header file. |
| `GazeboYARPPlugins::gazebo_yarp_singleton` | Library containing the `GazeboYarpPlugins::Handler` singleton library. |
| `GazeboYARPPlugins::gazebo_yarp_rpc_worldinterface` | The [thrift-generated YARP RPC interface](http://www.yarp.it/thrift_tutorial_simple.html) to the `gazebo_yarp_worldinterface` plugin. |
| `GazeboYARPPlugins::gazebo_yarp_rpc_clock` | The [thrift-generated YARP RPC interface](http://www.yarp.it/thrift_tutorial_simple.html) to the `gazebo_yarp_clock` plugin. |

Furthermore every plugin location is encoded in the `GazeboYARPPlugins::gazebo_yarp_pluginname` CMake imported target.
