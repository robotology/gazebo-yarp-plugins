/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "TimeSynchronizer.hh"

#include <gazebo/common/common.hh>
#include <yarp/os/Property.h>

namespace gazebo {
    
    
    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(GazeboYarpTimeSynchronizer)
    
    GazeboYarpTimeSynchronizer::GazeboYarpTimeSynchronizer() {}
    
    GazeboYarpTimeSynchronizer::~GazeboYarpTimeSynchronizer()
    {
        if (m_worldCreatedConnection.get())
            gazebo::event::Events::DisconnectWorldCreated(m_worldCreatedConnection);
    }
    
    void GazeboYarpTimeSynchronizer::Load(int _argc, char** _argv)
    {
        yarp::os::Property commandLine;
        commandLine.fromCommand(_argc, _argv, true, true);
        
        //read port name from the command line
        //--port port_name
        yarp::os::Value portName = commandLine.find("port");
        if (portName.isNull()) {
            std::cout << "TimeSynchronizer: launched with no port option. Defaults to ?" << std::endl;
        } else {
            std::cout << "TimeSynchronizer: launched with port " << portName.asString() << std::endl;
        }
        
//        for (int i = 0; i < _argc; i++) {
//            std::cout << _argv[i] << "\n";
//        }
        
        
        m_worldCreatedConnection = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboYarpTimeSynchronizer::worldHasBeenCreated,this, _1));
        
    }
    
    void GazeboYarpTimeSynchronizer::worldHasBeenCreated(std::string worldName)
    {
        if (m_worldCreatedConnection.get()) {
            gazebo::event::Events::DisconnectWorldCreated(m_worldCreatedConnection);
            m_worldCreatedConnection = gazebo::event::ConnectionPtr();
        }
    }
    
}