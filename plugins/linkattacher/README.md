### LinkAttacher Plugin
The LinkAttacher is a Gazebo _model plugin_. It can be used to attach/detach two _link_ of _model_ spawned in Gazebo environment. The attachment is made by adding a _joint_ between the links specified, with a given _jointType_ (available joint types are those in [SDF documentation](http://sdformat.org/spec?ver=1.6&elem=joint#joint_type) with the exception of `gearbox` that is not available). The link of the model we want to attach is the parent link and the link of the robot model (to which this plugin is added) becomes the child link for the newly created joint. So, the new joint is named by the `model_link_name` suffixed with the phrase *_jointType_joint* in the name e.g, l_hand_fixed_joint.

Additionally, this plugin also provides control over _gravity_ of the models spawned in Gazebo environment.
Currently, this plugin is used for coupling the hands of two robots, or those robot model and human model in pHRI experiments. Also, this plugin can be used to attach objects to the robot links.

### Usage
Create a `linkattacher.ini` configuration file defining the rpc port name for communicating with the linkattacher plugin, and defining the desired type of joint (optional: default type is `fixed`).
e.g.

```
name /<model-name>/linkattacher/rpc:i
jointType ball
```
Now, add the following lines to the sdf model

```
<plugin name='link_attacher' filename='libgazebo_yarp_linkattacher.so'>
  <yarpConfigurationFile>model://<path-to-model-configuration-files>/linkattacher.ini</yarpConfigurationFile>
</plugin>
```  

On launching the model in Gazebo, the _rpc_ port will be opened. Connect to this port and type `help` to know the available commands.

The available commands are:
- `attachUnscoped <parent_model_name> <parent_model_link_name> <child_model_name> <child_model_link_name>`
- `detachUnscoped <model_name> <link_name>`
- `enableGravity <model_name> <enable>`
The `link_name` can be both the scoped or unscoped name of the link.