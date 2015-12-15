Clock Plugin
===================

The clock plugin allows to synchronize the simulation with an external controller.
The plugin itself exposes two features:

- It publishes the simulation time on a Yarp port `/clock`
- It opens a Yarp RPC Port (`/clock/rpc`) to allows control of the simulation time.

### How to launch it
The plugin should be launched as a system plugin:

`gazebo -slibgazebo_yarp_clock.so`

### Plugin details

The RPC communication API is written in [thrift](https://github.com/robotology/gazebo-yarp-plugins/blob/master/thrift/clock/clock_rpc.thrift) and a stub is automatically generated and can be used for remote procedure calls. The APIs allow to stop, resume and step the simulation.


####Synchronize with the simulation
The features exposed by this plugin should be enough to cover most of the synchronization issues.

If you need to implement synchronization in your module, you should use the RPC calls to properly step the simulation.

##### Already-implemented synchronization methods
At the current state, you can use the synchronization out-of-the-box in these two cases:

###### 1) Yarp module
Yarp already supports the use of an external clock. In this case, the only thing that is needed is to set the environmental variable `YARP_CLOCK` to be equal to the clock port, e.g., `YARP_CLOCK=/clock`. 
We strongly advice you to **NOT** put the variable in your `.bashrc` but to do it explicitly for each module.

###### 2) Simulink with WB-Toolbox
If you are using the [WB-Toolbox](https://github.com/robotology/WB-Toolbox) a block called `Simulator Synchronizer` already implements the correct RPC calls. You only have to put the block in your Simulink model.
