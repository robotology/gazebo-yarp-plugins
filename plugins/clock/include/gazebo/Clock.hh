/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CLOCKPLUGIN_HH
#define GAZEBOYARP_CLOCKPLUGIN_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

namespace gazebo
{
    class GazeboYarpClock;
}

namespace yarp {
    namespace os {
        class Port;
        
        template <class T>
        class BufferedPort;
        
        class Network;
        class Bottle;
    }
}

namespace GazeboYarpPlugins {
    class ClockServer;
}

/**
 * Plugin exposing the Gazebo Clock on the YARP network, allowing to synchronize the simulation with an external controller.
 *
 * The plugin itself exposes two features:
 * - It publishes the simulation time on a Yarp port `/clock`
 * - It opens a Yarp RPC Port (`/clock/rpc`) to allows control of the simulation time.
 *
 *  ### How to launch it
 * The plugin should be launched as a system plugin:
 *
 * `gazebo -slibgazebo_yarp_clock.so`
 *
 * ### Plugin details
 *
 * The RPC communication API is written in [thrift](https://github.com/robotology/gazebo-yarp-plugins/blob/master/thrift/clock/clock_rpc.thrift) and a stub is automatically generated and can be used for remote procedure calls. The APIs allow to stop, resume and step the simulation.
 *
 *
 * ####Synchronize with the simulation
 * The features exposed by this plugin should be enough to cover most of the synchronization issues.
 *
 * If you need to implement synchronization in your module, you should use the RPC calls to properly step the simulation.
 *
 * ##### Already-implemented synchronization methods
 * At the current state, you can use the synchronization out-of-the-box in these two cases:
 *
 * ###### 1) Yarp module
 * Yarp already supports the use of an external clock. In this case, the only thing that is needed is to set the environmental variable `YARP_CLOCK` to be equal to the clock port, e.g., `YARP_CLOCK=/clock`.
 * We strongly advice you to **NOT** put the variable in your `.bashrc` but to do it explicitly for each module.
 *
 * ###### 2) Simulink with WB-Toolbox
 * If you are using the [WB-Toolbox](https://github.com/robotology/WB-Toolbox) a block called `Simulator Synchronizer` already implements the correct RPC calls. You only have to put the block in your Simulink model.
 *
 *
 */
class gazebo::GazeboYarpClock : public gazebo::SystemPlugin
{
public:
    GazeboYarpClock();
    virtual ~GazeboYarpClock();

    virtual void Load(int _argc = 0, char **_argv = NULL);

    void gazeboYarpClockLoad(std::string world_name);

    void clockUpdate();
    
    //Simulation time manipulation methods
    /** @brief pause the simulation
     */
    void clockPause();
    
    /** @brief resume the simulation
     */
    void clockContinue();
    
    /** @brief Step the simulation for the input number of steps. Defaults to 1
     * @param[in] steps number of steps
     */
    void clockStep(unsigned steps=1);
    
    /** @brief Returns the simulation time
     * @return the simulation time
     */
    common::Time getSimulationTime();
    
    /**
     * Reset the simulation time back to zero
     */
    void resetSimulationTime();

    /**
     * Reset the simulation time and state back to zero
     */
    void resetSimulation();    
    
    /**
     * Get the current step size in seconds.
     * @return the step size in seconds
     */
    virtual double getStepSize();

private:
    void cleanup();
    
    yarp::os::Network *m_network;
    std::string m_portName;
    yarp::os::BufferedPort<yarp::os::Bottle> *m_clockPort;

    gazebo::event::ConnectionPtr m_timeUpdateEvent;
    gazebo::event::ConnectionPtr m_worldCreatedEvent;
    gazebo::physics::WorldPtr m_world;
    
    //RPC variables
    yarp::os::Port *m_rpcPort;
    GazeboYarpPlugins::ClockServer *m_clockServer;

};


#endif
