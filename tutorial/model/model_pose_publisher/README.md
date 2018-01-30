# Tutorial for plugin GazeboYarpModelPosePublisher

This tutorial shows how to retrieve the global pose of a model loaded in the Gazebo
environment using the plugin GazeboYarpModelPosePublisher.

## Setup the environment
Hereafter it is supposed that the repository `gazebo-yarp-plugins` is located at
`$GAZEBO_YARP_PLUGINS`.

In order to use the `SDF model` provided for this tutorial it is required to
add the path `$GAZEBO_YARP_PLUGINS/tutorial/model/model_pose_publisher` to the
environment variable `GAZEBO_MODEL_PATH`:
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_YARP_PLUGINS/tutorial/model/model_pose_publisher
```

After this when launching `Gazebo` a new model `Gazebo Yarp Model Pose Publisher` should be available.

## SDF model
The [model](scenario/model.sdf) provided consists in a box equipped with the plugin:
```
<?xml version="1.0" ?>
<sdf version="1.5">

  <!-- Box -->
  <model name="box">
    <pose>1 1 0.5 0 0 0.7</pose>	
    <link name="box_root">
      <pose>0 0 0 0 0 0</pose>
      <collision name="box_collison">
	<geometry>
          <box>
            <size>1.0 1.0 1.0</size>
          </box>
	</geometry>
      </collision>
      <visual name="box_visual">
	<geometry>
          <box>
            <size>1.0 1.0 1.0</size>
          </box>
	</geometry>
      </visual>
    </link>

    <!-- Gazebo Yarp Model Pose Publisher plugin -->
    <plugin name='pose publisher' filename='libgazebo_yarp_modelposepublisher.so'></plugin>
    
  </model>      
</sdf>
```

## How to run the tutorial
- Compile the example code provided
```
cd $GAZEBO_YARP_PLUGINS/tutorial/model/model_pose_publisher
mkdir build
cd build
cmake ../
make install
```
The executable is available in `$GAZEBO_YARP_PLUGINS/tutorial/model/model_pose_publisher/build/bin/example`.

- Run `yarpserver`
```
yarpserver 
```

- Run the Frame Transfom Server via `yarpdev`
```
yarpdev --device transformServer --ROS::enable_ros_publisher false --ROS::enable_ros_subscriber false
```

- Run gazebo from a terminal in which the variable `$GAZEBO_MODEL_PATH` was updated as shown above
- Insert the model `Gazebo Yarp Model Pose Publisher` in the environment
- Run the executable provided
```
cd `$GAZEBO_YARP_PLUGINS/tutorial/model/model_pose_publisher/build/bin/
./example
```

Then, a part from some information printed by the Frame Transform Client, you should see
the current pose of the model
```
[INFO]Transform from /inertial to /box/frame : 
[INFO]Position:   0.425452	-1.087350	 0.500001 
[INFO]Rotation (Euler ZYX):  -1.570796	-0.000001	 2.270795
```
updated every second.
