gazebo_yarp_robotinterface
==========================

The `gazebo_yarp_robotinterface` plugin permits to load several [YARP devices](http://www.yarp.it/git-master/note_devices.html) that can be attached to YARP devices 
already opened by other Gazebo-YARP plugins using the same XML format used by the [`yarprobotinterface`](http://www.yarp.it/git-master/yarprobotinterface.html) tool and the [`libYARP_robotinterface` C++ library](https://github.com/robotology/yarp/tree/master/src/libYARP_robotinterface).

## Usage 

### Add the plugin in the SDF model
The `gazebo_yarp_robotinterface` plugin can be used by including in any SDF model:
~~~xml
<model name="model_name">
  <!-- Insert models, links, joints -->
  <!-- ... -->

  <!-- Insert other plugins -->
  <!-- ... -->
  
  <!-- Insert gazebo_yarp_robotinterface -->
  <plugin name="robotinterface" filename="libgazebo_yarp_robotinterface.so">
    <!-- This file can be specified with the model:// URI already used in Gazebo, see http://gazebosim.org/tutorials?tut=components&cat=get_started -->
    <yarpRobotInterfaceConfigurationFile>model://RobotInterfaceConfigurationFile.xml</yarpRobotInterfaceConfigurationFile>
  </plugin>
</model>
~~~

**Warning: the `gazebo_yarp_robotinterface` plugin is only able to be attached to YARP devices that have been already created once the `gazebo_yarp_robotinterface` plugin has been spawned, so it is a good practice to always include it as last tag in a model.**

### Example of the robotinterface XML file

The file specified in `yarpRobotInterfaceConfigurationFile` is a XML file specified according to the `yarprobotinterface` format, for example:
~~~xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE robot PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">
<robot name="robot_name" prefix="robot_prefix">
  <!-- YARP devices to spawn in the robotinterface be specified with the devices tag -->
  <devices>
    <!-- each YARP device is specified with the device tag -->
    <device name="yarp_device_name" type="yarp_device_type">
      <!-- specify any parameter to the device with param tags -->
      <param name="integer_param_example"> 10 </param>
      <param name="string_param_example">this_is_a_string/param>
      <!-- To specify the existing YARP devices to which the newly spawned device needs to attach -->
      <action phase="startup" level="5" type="attach">
          <!-- This should match the yarpDeviceName in the .ini file loaded in the Gazebo plugin. -->
          <!-- The yarpDeviceName plays the same role of the name attribute of the device XML element
               for devices that are created via the robotinterface XML format. -->
        <param name="device">yarp_device_name_of_device_to_which_to_attach</param>
        <!-- Multiple devices can be attached by passing the "networks" list parameter instead, i.e.
        <paramlist name="networks">
          <elem name="list_key1_example"> yarp_device_name_of_first_device_to_which_to_attach </elem>
          <elem name="list_key2_example"> yarp_device_name_of_second_device_to_which_to_attach </elem>
        </paramlist>-->
      </action>
      <!-- If you added an attach action, always specify the detach action during the shutdown phase -->
      <action phase="shutdown" level="5" type="detach" />
    </device>
  </devices>
</robot>
~~~


### How to specify existing YARP devices to which to attach

The main use of the `gazebo_yarp_robotinterface` plugin is to spawn YARP devices that are **attached** to other devices that have been already spawned by other Gazebo plugins. For this reason, we need a way to specify to which target devices the devices created by robotinterface needs to be attached.
This is achieved by setting the elements in the `<param name="device">` or in the `<paramlist name="networks">` list under the `<action phase="startup" level="5" type="attach">` XML element.
In this context, we call this "device instance identified" as **YARP device instance name**, as for devices created by the robotinterface, this is specified by the `name` attribute of the `device` XML tag. It is important not to confuse this with the **YARP device type name**, i.e. the name that identifies the type of plugin that is spawned, i.e. the `type` attribute of the `device` tag.

The `gazebo_yarp_robotinterface` can be attached to any YARP device created by any plugin inside the model, or in any plugin  contained in any nested sensor or model. 

For historic reason, not all the `gazebo-yarp-plugins` support specifying the **YARP device instance name** for the device that they spawn to permit to use them with the `gazebo_yarp_robotinterface` plugin. If you need to have this functionality in a specific plugin, feel free to open an issue. 

**Warning: as the YARP device instance name is specified without any specific Gazebo model or sensor namespace, it is important to observe, while using the `gazebo_yarp_robotinterface` plugin, that all the YARP devices contained in the model have a unique YARP device instance name. If this is not the case, the plugin will print a clear error and exit without starting.**

The plugins that spawn YARP devices in a way that they can be then attached to the yarprobotinterface as specified in the following table.
For all the following plugins, the **YARP device instance name** can be specified by the `yarpDeviceName` parameter in the plugin configuration.
For the `gazebo_yarp_controlboard`, if the `yarpDeviceName` parameter is not specified, for legacy reason the **YARP device instance name** for each created device can also be specified with the `networks` parameter list in the plugin configuration.

A cmake option (`GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS`) has been added, if this option is enabled then implicit wrappers present in`gazebo_yarp_multicamera`, `gazebo_yarp_lasersensor`, `gazebo_yarp_controlboard` and `gazebo_yarp_depthCamera` are removed, the new way to have them is to attach the new nws to gazebo devices via yarprobotinterface as above.

| Plugin                     | Details                                              |
|:--------------------------:|:----------------------------------------------------:|
| `gazebo_yarp_controlboard` | This plugin can create multiple YARP devices that expose joint-level motor and control interfaces such as [`yarp::dev::IPositionControl`](https://www.yarp.it/git-master/classyarp_1_1dev_1_1IPositionControl.html), [`yarp::dev::ITorqueControl`](https://www.yarp.it/git-master/classyarp_1_1dev_1_1ITorqueControl.html) and [`yarp::dev::ITorqueControl`](https://www.yarp.it/git-master/classyarp_1_1dev_1_1IEncoders.html). |
| `gazebo_yarp_depthcamera`  | This plugin can create a YARP device that expose a depth-camera interface. |
| `gazebo_yarp_lasersensor`  | This plugin can create a YARP device that expose a laser-seensor interface. |
| `gazebo_yarp_doublelaser`  | This plugin can create a YARP device network wrapper server that expose two existing laser sensors. |
| `gazebo_yarp_multicamera`  | This plugin can create a YARP device that expose a multicamera interface. |
