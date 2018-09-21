

# Use the gazebo-yarp-plugins in Gazebo Models {#embed_plugins}

# Use the gazebo-yarp-plugins in Gazebo Models

To understand the structure of gazebo-yarp-plugins, it is useful to understand what Gazebo plugins and Yarp device drivers are.

In a nutshell, Gazebo plugins are C++ classes that extend the functionalities of the Gazebo simulator, while Yarp device drivers are classes used in Yarp for abstracting the functionality of devices used in robots.

For additional information it is possible to access the official documentation for both [Gazebo plugins](http://gazebosim.org/tutorials?cat=write_plugin) and [YARP Device Drivers](http://wiki.icub.org/yarpdoc/note_devices.html).

The `gazebo-yarp-plugins` consists of:
* gazebo plugins that instantiate yarp device drivers and
* yarp device drivers that wrap gazebo functionalities inside the yarp device interfaces .

Furthermore, in `gazebo-yarp-plugins` there are several plugins that do not mimic any interface or capability present in real robots,
but are nevertheless useful for simulating complex scenarios.

## Plugins exposing functionalities available on real robots

|  Functionality     | Plugin Name  | Plugin Type |  Gazebo Plugin class  | YARP Device class (if any)  |
| :----------------: |:-------------:| :-----:|:---------------------------------:|:-------------------:|
| Control Board (encoder readings, motor control, ...) | `gazebo_yarp_controlboard` | Model |  gazebo::GazeboYarpControlBoard | yarp::dev::GazeboYarpControlBoardDriver |
| 6-Axis Force Torque sensor | `gazebo_yarp_forcetorque` | Sensor |  gazebo::GazeboYarpForceTorque |  yarp::dev::GazeboYarpForceTorqueDriver |
| Inertial Measurement Unit | `gazebo_yarp_imu` | Sensor | gazebo::GazeboYarpIMU | yarp::dev::GazeboYarpIMUDriver |
| Camera  | `gazebo_yarp_camera` | Sensor | gazebo::GazeboYarpCamera | yarp::dev::GazeboYarpCameraDriver |
| Depth Camera | `gazebo_yarp_depthCamera` | Sensor | gazebo::GazeboYarpDepthCamera | yarp::dev::GazeboYarpDepthCameraDriver |
| Fake Control Board (the same interface of the Control Board, but over non-existing joints) |  `gazebo_yarp_fakecontrolboard` | Model | gazebo::GazeboYarpFakeControlBoard |yarp::dev::GazeboYarpFakeControlBoardDriver |
| Lidar Sensor        | `gazebo_yarp_lasersensor` | Sensor | gazebo::GazeboYarpLaserSensor | yarp::dev::GazeboYarpLaserSensorDriver |
| Coupled Joint Encoders | `gazebo_yarp_maissensor` | Model |  gazebo::GazeboYarpMaisSensor | yarp::dev::GazeboYarpMaisSensorDriver |
| Stereo Cameras         | `gazebo_yarp_multicamera` | Sensor | gazebo::GazeboYarpMultiCamera | yarp::dev::GazeboYarpMultiCameraDriver |
| Contact Load Cell Array| `gazebo_contactloadcellarray` | Model | gazebo::GazeboYarpContactLoadCellArray | yarp::dev::GazeboYarpContactLoadCellArrayDriver |

## Plugins exposing simulation-specific functionalities
|  Functionality     | Plugin Name  | Plugin Type |  Gazebo Plugin class  |
| :----------------: |:-------------:| :-----:|:---------------------------------:|
| Clock synchronization  | `gazebo_yarp_clock` | System | gazebo::GazeboYarpClock |
| Apply external force/torque via YARP facilities | `gazebo_yarp_externalwrench` | Model |  gazebo::ApplyExternalWrench |
| Display the Center of Mass of a model | `gazebo_yarp_showmodelcom` | Model | gazebo::ShowModelCoM |
| Project an image stream on a simulated surface | `gazebo_yarp_videotexture` | Visual  | gazebo::VideoTexture |
| Expose a YARP RPC interface to create/manipulate objects programatically. | `gazebo_yarp_worldinterface` | Model |  gazebo::WorldInterface |
| Publish the absolute pose of the root link of a model. | `gazebo_yarp_modelposepublisher` | Model | gazebo::GazeboYarpModelPosePublisher |
| Expose a YARP RPC interface to attach/detach links of the models spawned to the links of the robot using a fixed joint. | `gazebo_yarp_rpc_linkattacher` | Model | gazebo::GazeboYarpLinkAttacher |
| Exposes the absolute pose, velocity and acceleration of a desired link through YARP analog server device| `gazebo_basestate` | Model | gazebo::GazeboYarpBaseState | yarp::dev::GazeboYarpBaseStateDriver |

## Using the plugins in Gazebo Models
In Gazebo, the simulated models are described using the [SDF (simulation description format)]((http://gazebosim.org/sdf.html), an XML-based file format.
Depending on the Gazebo Plugin Type, the way of loading or adding a plugin to a model changes, and the complete documentation is available at http://gazebosim.org/tutorials?cat=write_plugin .

For model plugins such as the `gazebo_yarp_controlboard`, a plugin can be loaded by adding a `<plugin>` SDF tag as a child of the `<model>` SDF tag that describes the model.
~~~
<plugin name="example_controlboard" filename="libgazebo_yarp_controlboard.so">
    <yarpConfigurationFile>model://example/example_controlboard.ini</yarpConfigurationFile>
</plugin>
~~~
Most Model and Sensor plugins present in `gazebo-yarp-plugins` read their configuration from the file referenced in the `yarpConfigurationFile` tag.
The location of the file is defined by using a [Gazebo URI](https://bitbucket.org/osrf/gazebo/wiki/uri), while the file is a [.ini YARP configuration file](http://www.yarp.it/yarp_config_files.html).
For specific information consumed by each plugin, please refer to the doxygen documentation of the specific plugin class.

System plugins instead need to be loaded at the beginning of the simulation, and hence they are passed as command line arguments when launching `gazebo` (or `gzserver`).
For using the YARP clock plugin, for example:
~~~
gazebo -s libgazebo_yarp_clock.so
~~~

## Example models
A collection of example model showcasing the capabilities of the `gazebo-yarp-plugins` are available in the `tutorial/model` subfolder of this repository.
To spawn this models in Gazebo, add the `tutorial/model` directory to the `GAZEBO_MODEL_PATH` enviromental variable, or load it from the Gazebo GUI.
