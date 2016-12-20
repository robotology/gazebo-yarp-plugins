/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <GazeboYarpPlugins/common.h>
#include "Clock.hh"
#include "ClockServerImpl.h"

#include <gazebo/gazebo_config.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <iostream>

namespace gazebo
{
    
    GazeboYarpClock::GazeboYarpClock()
    : m_network(0)
    , m_clockPort(0)
    , m_rpcPort(0)
    , m_clockServer(0)
    {
        
    }
    
    GazeboYarpClock::~GazeboYarpClock()
    {
        cleanup();
    }
    
    void GazeboYarpClock::cleanup()
    {
        if (m_worldCreatedEvent.get()) {
            gazebo::event::Events::DisconnectWorldCreated(m_worldCreatedEvent);
            m_worldCreatedEvent = gazebo::event::ConnectionPtr(); //resetting the pointer to NULL. I don't know if boost does it automatically with the above call
        }
        if (m_timeUpdateEvent.get()) {
            gazebo::event::Events::DisconnectWorldUpdateBegin(m_timeUpdateEvent);
            m_timeUpdateEvent = gazebo::event::ConnectionPtr();
        }
        
        if (m_clockPort) {
            m_clockPort->close();
            delete m_clockPort; m_clockPort = 0;
        }
        
        if (m_rpcPort) {
            m_rpcPort->close();
            delete m_rpcPort; m_rpcPort = 0;
        }
        
        if (m_clockServer) {
            delete m_clockServer; m_clockServer = 0;
        }
        
        if (m_network) {
            delete m_network; m_network = 0;
        }
    }
    
    
    void GazeboYarpClock::Load(int _argc, char **_argv)
    {
        m_network = new yarp::os::Network();
        if (!m_network
            || !yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
            yError() << "GazeboYarpClock::Load error: yarp network does not seem to be available, is the yarpserver running?";
            cleanup();
            return;
        }
        
        //This does not work yet. See: https://bitbucket.org/osrf/gazebo/issue/280/allow-for-command-line-arguments-to-system
        yarp::os::Property commandLine;
        commandLine.fromCommand(_argc, _argv, true, true);
        
        m_portName = "/clock";
        //read port name from the command line
        //--port port_name
        yarp::os::Value portName = commandLine.find("port");
        if (!portName.isNull()) {
            m_portName = portName.asString();
        }

        yDebug() << "GazeboYarpClock loaded. Clock port will be " << m_portName;
        
        //The proper loading is done when the world is created
        m_worldCreatedEvent = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboYarpClock::gazeboYarpClockLoad,this,_1));
    }
    
    void GazeboYarpClock::gazeboYarpClockLoad(std::string world_name)
    {
        if (m_worldCreatedEvent.get()) {
            gazebo::event::Events::DisconnectWorldCreated(m_worldCreatedEvent);
            m_worldCreatedEvent = gazebo::event::ConnectionPtr();
        }
        
        //Create ports
        m_clockPort = new yarp::os::BufferedPort<yarp::os::Bottle>();
        if (!m_clockPort) {
            yError() << "GazeboYarpClock: Failed to create clock port.";
            cleanup();
            return;
        }
        
        if (!m_clockPort->open(m_portName)) {
            yError() << "GazeboYarpClock: Failed to open clock port.";
            cleanup();
            return;
        }
        
        m_rpcPort = new yarp::os::Port();
        if (!m_rpcPort) {
            yError() << "GazeboYarpClock: Failed to create rpc port.";
            cleanup();
            return;
        }
        
        m_clockServer = new GazeboYarpPlugins::ClockServerImpl(*this);
        if (!m_clockServer) {
            yError() << "GazeboYarpClock: Could not create Clock Server.";
            cleanup();
            return;
        }
        
        if (!m_clockServer->yarp().attachAsServer(*m_rpcPort)) {
            yError() << "GazeboYarpClock: Failed to attach Clock Server to RPC port.";
            cleanup();
            return;
        }
        
        if (!m_rpcPort->open(m_portName + "/rpc")) {
            yError() << "GazeboYarpClock: Failed to open rpc port " << (m_portName + "/rpc");
            cleanup();
            return;
        }
        
        //Getting world pointer
        m_world = gazebo::physics::get_world(world_name);
        
        m_timeUpdateEvent = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpClock::clockUpdate,this));
    }
    
    void GazeboYarpClock::clockUpdate()
    {
        if (m_clockPort) {
            gazebo::common::Time currentTime = m_world->GetSimTime();
            yarp::os::Bottle& b = m_clockPort->prepare();
            b.clear();
            b.addInt(currentTime.sec);
            b.addInt(currentTime.nsec);
            m_clockPort->write();
        }
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
#if GAZEBO_MAJOR_VERSION >= 3
        m_world->Step(step);
#else
        m_world->StepWorld(step);
#endif
    }
    
    void GazeboYarpClock::resetSimulationTime()
    {
        m_world->ResetTime();
    }
    
    common::Time GazeboYarpClock::getSimulationTime()
    {
        return m_world->GetSimTime();
    }
    
    double GazeboYarpClock::getStepSize()
    {
        if (m_world->GetPhysicsEngine()) {
            return m_world->GetPhysicsEngine()->GetMaxStepSize();
        }
        return -1;
    }
    
    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(GazeboYarpClock)
}
