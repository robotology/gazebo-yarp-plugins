### LinkAttacher Plugin
The LinkAttacher is a Gazebo _model plugin_. It can be used to attach/detach any _link_ of the _model_ spawned in Gazebo environment. The attachment is made by adding a _fixed joint_ between the links specified. The link of the model in Gazebo environment is the parent link and the link of the robot(to which this plugin) is used becomes the child link for the newly created fixed joint. So, the fixed joint is named by the `model_link_name` suffixed with the phrase *_magnet_joint* in the name e.g, l_hand_magnet_joint.

Additionally, this plugin also provides control over _gravity_ of the models spawned in Gazebo environment. Currently, this plugin is used for coupling the hands of the robot model and human model in pHRI experiments. Also, this plugin can be used to attach objects to the robot links.

### Usage
Create a `linkattacher.ini` configuration file and add the following lines to it to open the rpc port for communicating with the linkattacher plugin

```
name /<model-name>/linkattacher/rpc:i
```
Now, add the following lines to the sdf model

```
<plugin name='link_attacher' filename='libgazebo_yarp_linkattacher.so'>
  <yarpConfigurationFile>model://<path-to-model-configuration-files>/linkattacher.ini</yarpConfigurationFile>
</plugin>
```  

On launching the model in Gazebo, the _rpc_ port will be opened. Connect to this port and type `help` to know the available commands.

The available commands are:
- attachUnscoped
- detachUnscoped
- enableGravity
