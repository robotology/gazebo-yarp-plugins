### ExternalWrench Plugin

The ExternalWrench is a Gazebo _model_plugin_. It is used to apply external wrenches on any of the links of the model (to which this plugin is added).

### Usage
The plugin takes `gazebo_icub_robotname.ini` as the configuration file. Add the following lines to the sdf model.


```
<plugin name='externalwrench' filename='libgazebo_yarp_externalwrench.so'>
      <yarpConfigurationFile>model://icub/conf/gazebo_icub_robotname.ini</yarpConfigurationFile>
</plugin>
```

On launching the model in Gazebo, the _rpc_ port will be opened e.g. `/<model>/applyExternalWrench/rpc:i`. Connect to this port and type `help` to know the available commands.

```
Responses:
  The default operation mode is with single wrench
  Insert [single] or [multiple] to change the operation mode
  Insert a command with the following format:
  [link] [force] [torque] [duration]
  e.g. chest 10 0 0 0 0 0 1
  [link]:     (string) Link ID of the robot as specified in robot's SDF
  [force]:    (double x, y, z) Force components in N w.r.t. world reference frame
  [torque]:   (double x, y, z) Torque components in N.m w.r.t world reference frame
  [duration]: (double) Duration of the applied force in seconds
  Note: The reference frame is the base/root robot frame with x pointing backwards and z upwards.
```

The plugin will operate in _single_ wrench operation mode by default. Users can switch to _multiple_ wrenches operation mode. To change the operation mode of the plugin enter `single` or `multiple` in the rpc terminal. Every time the operation mode is changed the wrenches present at the moment will be cleared. To apply external wrenches enter the command format `[link] [force] [torque] [duration]` e.g. `l_hand 10 0 0 0 0 0 10` which applies an external wrench of pure force with 10 N magnitude for a duration of 10 seconds. Resetting Gazebo world will clear all the existing wrenches and change the operation mode of the plugin to default option i.e. `single`.
