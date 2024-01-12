# Example of a model using a gazebo_yarp_forcetorque plugin

This directory contains the example of a model using the `gazebo_yarp_forcetorque` plugin.

For more information on simulation of Force/Torque sensors in Gazebo Classic, see [https://classic.gazebosim.org/tutorials?tut=force_torque_sensor&cat=sensors](https://classic.gazebosim.org/tutorials?tut=force_torque_sensor&cat=sensors).


To run this example, first of all run `yarpserver` in a terminal, then make sure to have added `gazebo-yarp-plugins/tutorial/model` directory to `GAZEBO_MODEL_PATH` as documented in the "Example models" section of the `doc/embed_plugins.md` documentation and then run the world.

Then:
~~~
cd <gazebo-yarp-plugins-path>/tutorial/model/forcetorque
gazebo --verbose forcetorque_world.world
~~~

If the simulation starts correctly, you should see a model composed by a cylinder and a box. The FT sensor measures the force exchanged between the cylinder and the box. To check if the YARP plugin works as expected, open a new terminal and type:
~~~
yarp name list
~~~

To list all the YARP ports open. You should see the port `/forcetorque/measures:o`, that is the one opened by the plugin.

You can then read the measures from the the `/forcetorque/measures:o` with the command:
~~~
yarp read ... /forcetorque/measures:o
~~~

As output you should see:
~~~
[INFO] |yarp.os.Port|/tmp/port/1| Port /tmp/port/1 active at tcp://172.22.196.221:10004/
[INFO] |yarp.os.impl.PortCoreInputUnit|/tmp/port/1| Receiving input from /forcetorque/measures:o to /tmp/port/1 using tcp
() () () () () (((0.0 0.0 -98.0000000000000568434 0.0 0.0 0.0) 59.0670000000000001705)) () () () ()
() () () () () (((0.0 0.0 -97.9999999999999573674 0.0 0.0 0.0) 59.542999999999999261)) () () () ()
() () () () () (((0.0 0.0 -98.0000000000000568434 0.0 0.0 0.0) 60.0240000000000009095)) () () () ()
() () () () () (((0.0 0.0 -98.0000000000000568434 0.0 0.0 0.0) 60.5)) () () () ()
() () () () () (((0.0 0.0 -97.9999999999999573674 0.0 0.0 0.0) 60.9759999999999990905)) () () () ()
() () () () () (((0.0 0.0 -97.9999999999999573674 0.0 0.0 0.0) 61.4530000000000029559)) () () () ()
() () () () () (((0.0 0.0 -98.0000000000000568434 0.0 0.0 0.0) 61.93200000000000216)) () () () ()
~~~

There is a parenthesis with six numbers, the first three are force and the last three are torques. In this case, it is possible
to see that the force measure make sense as the weight of the link is 10 Kg, and the acceleration of gravity is 9.8, so the measure
norm on the Z axis is correctly approximately 9.8*10 = 98.0 .
