/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "BasePoseVelocityDriver.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>


using namespace yarp::dev;

GazeboYarpBasePoseVelocityDriver::GazeboYarpBasePoseVelocityDriver() : m_robot(0), m_baseLink(0)
{

}

GazeboYarpBasePoseVelocityDriver::~GazeboYarpBasePoseVelocityDriver()
{
    if (m_robot)
    {
        m_robot = nullptr;
    }
    
    if (m_baseLink)
    {
        m_baseLink = nullptr;
    }
}

bool GazeboYarpBasePoseVelocityDriver::open(yarp::os::Searchable& config)
{
    yarp::os::Property deviceProp;
    deviceProp.fromString(config.toString().c_str());
       
    std::string robot(deviceProp.find("robot").asString().c_str());
    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robot);
    if (m_robot == NULL)
    {
        yError() << "GazeboYarpBasePoseVelocityDriver: robot model not found";
        return false;
    }

    m_baseLinkName = deviceProp.find("baseLink").asString().c_str();
    std::string linkNameScopedEnding = "::" + m_baseLinkName;
    const gazebo::physics::Link_V &links = m_robot->GetLinks();
    for (size_t idx = 0; idx < links.size(); idx++)
    {
      std::string linkName = links[idx]->GetScopedName();
      if (GazeboYarpPlugins::hasEnding(linkName, linkNameScopedEnding))
      {
        m_baseLink = m_robot->GetLink(linkName);
        break;
      }
    }
    
    if (m_baseLink == nullptr)
    {
        yError() << "GazeboYarpBasePoseVelocityDriver: link not found in the model";
        return false;
    }
    
    m_baseState.resize(m_stateDimensions);
    
    m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpBasePoseVelocityDriver::onUpdate, this, _1));
    
    return true;
}

bool GazeboYarpBasePoseVelocityDriver::close()
{
    yarp::os::LockGuard guard(m_dataMutex);
    this->m_updateConnection.reset();
    return true;
}

void GazeboYarpBasePoseVelocityDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    ignition::math::Vector3d m_worldBasePosition = m_baseLink->WorldPose().Pos();
    ignition::math::Quaterniond m_worldBaseOrientation = m_baseLink->WorldPose().Rot();
    ignition::math::Vector3d m_worldBaseLinVel = m_baseLink->WorldLinearVel();
    ignition::math::Vector3d m_worldBaseAngVel = m_baseLink->WorldAngularVel();
    ignition::math::Vector3d m_worldBaseLinAcc = m_baseLink->WorldLinearAccel();
    ignition::math::Vector3d m_worldBaseAngAcc = m_baseLink->WorldAngularAccel();

    // Serializing the state vector
    m_baseState[0] = m_worldBasePosition.X();
    m_baseState[1] = m_worldBasePosition.Y();
    m_baseState[2] = m_worldBasePosition.Z();
    m_baseState[3] = m_worldBaseOrientation.Roll();
    m_baseState[4] = m_worldBaseOrientation.Pitch();
    m_baseState[5] = m_worldBaseOrientation.Yaw();
    
    m_baseState[6] = m_worldBaseLinVel.X();
    m_baseState[7] = m_worldBaseLinVel.Y();
    m_baseState[8] = m_worldBaseLinVel.Z();
    m_baseState[9] = m_worldBaseAngVel.X();
    m_baseState[10] = m_worldBaseAngVel.Y();
    m_baseState[11] = m_worldBaseAngVel.Z();
    
    m_baseState[12] = m_worldBaseLinAcc.X();
    m_baseState[13] = m_worldBaseLinAcc.Y();
    m_baseState[14] = m_worldBaseLinAcc.Z();
    m_baseState[15] = m_worldBaseAngAcc.X();
    m_baseState[16] = m_worldBaseAngAcc.Y();
    m_baseState[17] = m_worldBaseAngAcc.Z();
    
    m_stamp.update(_info.simTime.Double());
    m_dataAvailable = true;
}

int GazeboYarpBasePoseVelocityDriver::read(yarp::sig::Vector& out)
{
    yarp::os::LockGuard guard(m_dataMutex);
     
    out.resize(m_baseState.size());
    out = m_baseState;
  
    return AS_OK;
}

yarp::os::Stamp GazeboYarpBasePoseVelocityDriver::getLastInputStamp()
{
    return m_stamp;
}


int GazeboYarpBasePoseVelocityDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

int GazeboYarpBasePoseVelocityDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int GazeboYarpBasePoseVelocityDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int GazeboYarpBasePoseVelocityDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpBasePoseVelocityDriver::getChannels()
{
    return AS_OK;
}

int GazeboYarpBasePoseVelocityDriver::getState(int ch)
{
    return AS_OK;
}
