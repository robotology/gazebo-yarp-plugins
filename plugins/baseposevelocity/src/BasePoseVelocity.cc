/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "BasePoseVelocity.hh"
#include "BasePoseVelocityDriver.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpBasePoseVelocity)

namespace gazebo
{

    GazeboYarpBasePoseVelocity::GazeboYarpBasePoseVelocity() : ModelPlugin(),
                                                               m_networkWrapper(0)
    {   

    }
    
    GazeboYarpBasePoseVelocity::~GazeboYarpBasePoseVelocity()
    {
        if (m_networkWrapper)
        {
            m_networkWrapper->detachAll();
            m_networkWrapper = nullptr;
        }
        
        if (m_networkDevice.isValid())
        {
            m_networkDevice.close();
        }
        
        if (m_deviceDriver.isValid())
        {
            m_deviceDriver.close();
        }
        
        GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_robot);
        yarp::os::Network::fini();
    }
    
    void GazeboYarpBasePoseVelocity::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yError() << "GazeboYarpBasePoseVelocity::Load erros: yarp network does not seem to be available, is the yarp server running?";
            return;
        }
        
        if (!_parent)
        {
            yError() << "GazeboYarpBasePoseVelocity plugin requires a parent \n";
            return;
        }

    }


}