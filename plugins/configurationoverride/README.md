### ConfigurationOverride Plugin
The ConfigurationOverride is a Gazebo _model plugin_. It can be attached to a model in order to override the [`sdf`](http://sdformat.org/spec) configuration of any other plugin that is attached to the same model.
Each ConfigurationOverride plugin points to a plugin and can override the values of multiple configuration parameters. The parameters that are not pointed in the ConfigurationOverride will remain untouched.
If the desired parameter is not found in the plugin, it is created accordingly to `configurationOverride`.

N.B.
If a parameter is defined multiple times inside a plugin, only the first occurence will be overritten accordingly to the element retreived with [`GetElement()`](http://osrf-distributions.s3.amazonaws.com/sdformat/api/3.0/classsdf_1_1Element.html#a29fa74bb6bf29701e82b6353e732d31d).
Currently, this doesn't represent a problem since also the plugins are reading the configurations with `GetElement()`.

### Usage
Add the ConfigurationOverride plugin to the model sdf before the overriden plugin declaration (i.e. before the `<include>`) and define the `plugin_name` as the `name` of the plugin that will be modified. 
The properties of the ConfigurationOverride plugin will be then used in order to override the properties of the desired plugin.

e.g. If you are using the model from [`icub-gazebo`](https://github.com/robotology/icub-gazebo) and you wanto to change the `<initialConfiguration>` of the `ControlBoard` (`controlboard_right_arm`), the model can be defined as follow:
```xml
    <!-- iCub -->
    <model name="myiCub">
    <plugin name='configuration_override' filename='libgazebo_yarp_configurationoverride.so'>
      <yarpPluginConfigurationOverride plugin_name='controlboard_right_arm'> </yarpPluginConfigurationOverride>
      <initialConfiguration>-2.0 0.52 0.0 0.785 0 0 0.698</initialConfiguration>
    </plugin>
    <include>
      <uri>model://icub</uri>
    </include>
    </model>
```
then instead of loading the model `icub` with the standard right arm `<initialConfiguration>`, the configuration defined in the ConfigurationOverride will be used.
