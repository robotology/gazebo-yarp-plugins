/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <GazeboYarpPlugins/common.h>
#include "Clock.hh"
#include "ClockServerImpl.h"

#include <boost/bind/bind.hpp>
#include <gazebo/gazebo_config.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <iostream>

using namespace boost::placeholders;

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
        m_worldCreatedEvent.reset();
        m_timeUpdateEvent.reset();

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
        // To avoid deadlock during initialization if YARP_CLOCK is set,
        // if the YARP network is not initialized we always initialize
        // it with system clock, and then we switch back to the default clock later
        // This avoid the problems discussed in https://github.com/robotology/gazebo-yarp-plugins/issues/526
        bool networkIsNotInitialized = !yarp::os::NetworkBase::isNetworkInitialized();

        if (networkIsNotInitialized) {
            m_network = new yarp::os::Network(yarp::os::YARP_CLOCK_SYSTEM);
            m_resetYARPClockAfterPortCreation = true;
        } else {
            m_network = new yarp::os::Network();
            m_resetYARPClockAfterPortCreation = false;
        }

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

        yInfo() << "GazeboYarpClock loaded. Clock port will be " << m_portName;

        //The proper loading is done when the world is created
        m_worldCreatedEvent = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboYarpClock::gazeboYarpClockLoad,this,_1));
    }

    void GazeboYarpClock::gazeboYarpClockLoad(std::string world_name)
    {
        m_worldCreatedEvent.reset();

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
#if GAZEBO_MAJOR_VERSION >= 8
            gazebo::common::Time currentTime = m_world->SimTime();
#else
            gazebo::common::Time currentTime = m_world->GetSimTime();
#endif
            yarp::os::Bottle& b = m_clockPort->prepare();
            b.clear();
            b.addInt32(currentTime.sec);
            b.addInt32(currentTime.nsec);
            m_clockPort->write();
        }

        // As the port is now created and contains streams data,
        // if necessary reset the YARP clock to YARP_CLOCK_DEFAULT
        // Unfortunatly, the yarpClockInit blocks on the port until it
        // receives data, so we need to launch it in a different thread
        if (m_resetYARPClockAfterPortCreation) {
            std::cerr << "Resetting YARP clock to default" << std::endl;
            auto resetYARPNetworkClockLambda =
                []() { yarp::os::NetworkBase::yarpClockInit(yarp::os::YARP_CLOCK_DEFAULT); };
            std::thread resetYARPNetworkClockThread(resetYARPNetworkClockLambda);
            resetYARPNetworkClockThread.detach();
            m_resetYARPClockAfterPortCreation = false;
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
        m_world->Step(step);
    }

    void GazeboYarpClock::resetSimulationTime()
    {
        m_world->ResetTime();
    }

    void GazeboYarpClock::resetSimulation()
    {
        m_world->Reset();
    }

    void GazeboYarpClock::resetSimulationState()
    {
        m_world->ResetEntities(gazebo::physics::Base::BASE);
        event::Events::worldReset();
    }    

    common::Time GazeboYarpClock::getSimulationTime()
    {
#if GAZEBO_MAJOR_VERSION >= 8
        return m_world->SimTime();
#else
        return m_world->GetSimTime();
#endif
    }

    double GazeboYarpClock::getStepSize()
    {
#if GAZEBO_MAJOR_VERSION >= 8
        physics::PhysicsEnginePtr physics = m_world->Physics();
#else
        physics::PhysicsEnginePtr physics = m_world->GetPhysicsEngine();
#endif
        if (physics) {
            return physics->GetMaxStepSize();
        }
        return -1.0;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(GazeboYarpClock)
}
