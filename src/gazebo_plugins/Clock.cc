/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "common.h"
#include "Clock.hh"

#include <gazebo/physics/physics.hh>
#include <yarp/os/Property.h>
#include <iostream>
#include <cmath>

namespace gazebo
{
    
    GazeboYarpClock::GazeboYarpClock()
    {
        
    }
    
    GazeboYarpClock::~GazeboYarpClock()
    {
        if (m_worldCreatedEvent.get())
            gazebo::event::Events::DisconnectWorldCreated(m_worldCreatedEvent);
        if (m_timeUpdateEvent.get())
            gazebo::event::Events::DisconnectWorldUpdateBegin(m_timeUpdateEvent);
        m_port.close();
        yarp::os::Network::fini();
    }
    
    
    void GazeboYarpClock::Load(int _argc, char **_argv)
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
            std::cerr << "GazeboYarpClock::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
            return;
        }
        
        std::cout << "GazeboYarpClock loaded." << std::endl;
        
        m_portName = "/clock";
        
        //The proper loading is done when the world is created
        m_worldCreatedEvent = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboYarpClock::gazeboYarpClockLoad,this,_1));
    }
    
    void GazeboYarpClock::gazeboYarpClockLoad(std::string world_name)
    {
        if (m_worldCreatedEvent.get()) {
            gazebo::event::Events::DisconnectWorldCreated(m_worldCreatedEvent);
            m_worldCreatedEvent = gazebo::event::ConnectionPtr();
        }
        
        //Opening port
        m_port.open(m_portName);
        
        //Getting world pointer
        m_world = gazebo::physics::get_world(world_name);
        
        m_timeUpdateEvent = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpClock::clockUpdate,this));
    }
    
    void GazeboYarpClock::clockUpdate()
    {
        gazebo::common::Time currentTime = m_world->GetSimTime();
        yarp::os::Bottle& b = m_port.prepare();
        b.clear();
        b.addInt(currentTime.sec);
        b.addInt(currentTime.nsec);
        m_port.write();
    }
    
    void GazeboYarpClock::clockPause()
    {
        m_world->SetPaused(true);
    }
    
    void GazeboYarpClock::clockContinue()
    {
        m_world->SetPaused(false);
    }
    
    void GazeboYarpClock::clockStep(unsigned int step)
    {
        m_world->Step(step);
    }
    
    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(GazeboYarpClock)
}
