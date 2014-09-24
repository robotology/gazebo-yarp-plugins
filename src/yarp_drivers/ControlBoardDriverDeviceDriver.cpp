/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ControlBoardDriver.h"
#include "gazebo_yarp_plugins/Handler.hh"

#include <gazebo/common/Events.hh>

using namespace yarp::dev;
using namespace gazebo;


bool GazeboYarpControlBoardDriver::open(yarp::os::Searchable& config) 
{
    m_pluginParameters.fromString(config.toString().c_str());

    std::string robotName(m_pluginParameters.find("robotScopedName").asString().c_str());
    std::cout << "DeviceDriver is looking for robot " << robotName << "..." << std::endl;

    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if(!m_robot) {
        std::cout << "GazeboYarpControlBoardDriver error: robot was not found" << std::endl;
        return false;
    }
    
    return gazebo_init();
}

bool GazeboYarpControlBoardDriver::close()
{
    //unbinding events
    if (this->m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin (this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
    }
    
    delete [] m_controlMode;
    delete [] m_interactionMode;
    delete [] m_isMotionDone;
    return true;
}
