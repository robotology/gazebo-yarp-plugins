/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "FakeControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <cstdio>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>

#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>

namespace yarp { 
namespace dev {
 
GazeboYarpFakeControlBoardDriver::GazeboYarpFakeControlBoardDriver() {}

GazeboYarpFakeControlBoardDriver::~GazeboYarpFakeControlBoardDriver() {}

bool GazeboYarpFakeControlBoardDriver::open(yarp::os::Searchable& config)
{
    m_pluginParameters.fromString(config.toString().c_str());

    yarp::os::Bottle joint_names_bottle = m_pluginParameters.findGroup("jointNames");

    if (joint_names_bottle.isNull()) {
        yError() << "GazeboYarpFakeControlBoardDriver::open(): Error cannot find jointNames." ;
        return false;
    }

    m_numberOfJoints = joint_names_bottle.size()-1;

    m_jointNames.resize(m_numberOfJoints);
    m_jointTypes.resize(m_numberOfJoints);
    
    for (unsigned int i = 0; i < m_jointNames.size(); i++) {
         m_jointNames[i] = joint_names_bottle.get(i+1).asString().c_str();
         m_jointTypes[i] = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
    }
   
    
    m_positions.resize(m_numberOfJoints);
    m_velocities.resize(m_numberOfJoints);
    m_torques.resize(m_numberOfJoints);
    m_zeroPosition.resize(m_numberOfJoints);
    
    m_positions.zero();
    m_velocities.zero();
    m_torques.zero();
    m_zeroPosition.zero();
    
    m_controlMode.resize(m_numberOfJoints,VOCAB_CM_NOT_CONFIGURED);
    m_interactionMode.resize(m_numberOfJoints,VOCAB_IM_STIFF);
    
    m_updateConnection =
    gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpFakeControlBoardDriver::onUpdate,
                                                                     this, _1));
    
    return true;
}

bool GazeboYarpFakeControlBoardDriver::close()
{    
    m_updateConnection.reset();
    
    return true;
}

void GazeboYarpFakeControlBoardDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    yarp::os::LockGuard lock(m_lastTimestampMutex);
    m_lastTimestamp.update(_info.simTime.Double());
}


}
}
