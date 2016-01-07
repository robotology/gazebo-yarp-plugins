/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/log.h>

#include <gazebo/common/Events.hh>

using namespace yarp::dev;
using namespace gazebo;


bool GazeboYarpControlBoardDriver::open(yarp::os::Searchable& config)
{
    m_pluginParameters.fromString(config.toString().c_str());

    deviceName = m_pluginParameters.find("name").asString().c_str();

    std::string robotName(m_pluginParameters.find("robotScopedName").asString().c_str());

    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if(!m_robot) {
        GYPERR << "GazeboYarpControlBoardDriver error: robot was not found" << std::endl;
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
