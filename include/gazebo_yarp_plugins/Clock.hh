/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_CLOCKPLUGIN_HH
#define GAZEBOYARP_CLOCKPLUGIN_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

namespace yarp {
    namespace os {
        class Port;
        
        template <class T>
        class BufferedPort;
        
        class Network;
        class Bottle;
    }
}

namespace gazebo
{
    class ClockServer;
    
    class GazeboYarpClock : public SystemPlugin
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
        ClockServer *m_clockServer;

    };
}

#endif
