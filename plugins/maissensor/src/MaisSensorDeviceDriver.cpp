/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "MaisSensorDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <yarp/os/LogStream.h>
#include <gazebo/common/Events.hh>

using namespace yarp::dev;
using namespace gazebo;


bool GazeboYarpMaisSensorDriver::open(yarp::os::Searchable& config)
{
    //yTrace() << "GazeboYarpMaisSensorDriver::open()";
    m_pluginParameters.fromString(config.toString().c_str());

    deviceName = m_pluginParameters.find("name").asString().c_str();

    std::string robotName;
    robotName = m_pluginParameters.find("robotScopedName").asString().c_str();
	
    if (robotName == "")
    {
        yError() << "GazeboYarpMaisSensorDriver error: 'robotName' parameter not found'";
        return false;
    }

    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if(!m_robot) {
        yError() << "GazeboYarpMaisSensorDriver error: robot was not found";
        return false;
    }

    return gazebo_init();
}

bool GazeboYarpMaisSensorDriver::close()
{
    //unbinding events
    if (this->m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin (this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
    }

    return true;
}
