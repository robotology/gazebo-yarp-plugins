/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <yarp/os/LogStream.h>
#include <gazebo/common/Events.hh>

using namespace yarp::dev;
using namespace gazebo;


bool GazeboYarpControlBoardDriver::open(yarp::os::Searchable& config)
{
    m_pluginParameters.fromString(config.toString().c_str());

    m_deviceName = m_pluginParameters.find("name").asString().c_str();

    std::string robotName(m_pluginParameters.find("robotScopedName").asString().c_str());

    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if(!m_robot) {
        yError() << "GazeboYarpControlBoardDriver error: robot was not found";
        return false;
    }

    return gazebo_init();
}

bool GazeboYarpControlBoardDriver::close()
{
    this->m_updateConnection.reset();

    delete [] m_controlMode;
    delete [] m_interactionMode;
    delete [] m_isMotionDone;

    for (size_t i = 0; i < m_trajectory_generator.size(); i++)
    {
        if (m_trajectory_generator[i])
        {
            delete m_trajectory_generator[i];
            m_trajectory_generator[i] = 0;
        }
    }

    for (size_t i = 0; i < m_coupling_handler.size(); i++)
    {
        if (m_coupling_handler[i])
        {
            delete m_coupling_handler[i];
            m_coupling_handler[i] = 0;
        }
    }

    for (size_t i = 0; i < m_speed_ramp_handler.size(); i++)
    {
        if (m_speed_ramp_handler[i])
        {
            delete m_speed_ramp_handler[i];
            m_speed_ramp_handler[i] = 0;
        }
    }
    
    return true;
}
