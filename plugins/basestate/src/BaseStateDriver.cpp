/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "BaseStateDriver.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>


using namespace yarp::dev;

GazeboYarpBaseStateDriver::GazeboYarpBaseStateDriver() : m_robot(0), m_baseLink(0)
{

}

GazeboYarpBaseStateDriver::~GazeboYarpBaseStateDriver()
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

bool GazeboYarpBaseStateDriver::open(yarp::os::Searchable& config)
{
    yarp::os::Property deviceProp;
    deviceProp.fromString(config.toString().c_str());
       
    std::string robot(deviceProp.find("robot").asString().c_str());
    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robot);
    if (m_robot == NULL)
    {
        yError() << "GazeboYarpBaseStateDriver: robot model not found";
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
        yError() << "GazeboYarpBaseStateDriver: link not found in the model";
        return false;
    }
    
    m_baseState.resize(m_stateDimensions);
    
    m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpBaseStateDriver::onUpdate, this, _1));
    
    return true;
}

bool GazeboYarpBaseStateDriver::close()
{
    yarp::os::LockGuard guard(m_dataMutex);
    this->m_updateConnection.reset();
    return true;
}

void GazeboYarpBaseStateDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d m_worldBasePosition = m_baseLink->WorldPose().Pos();
    ignition::math::Quaterniond m_worldBaseOrientation = m_baseLink->WorldPose().Rot();
    
    // Get the velocity of the origin of the link frame in the world reference frame
    ignition::math::Vector3d m_worldBaseLinVel = m_baseLink->WorldLinearVel();
    ignition::math::Vector3d m_worldBaseAngVel = m_baseLink->WorldAngularVel();
    
    // Get the acceleration of the center of mass of the link in the world reference frame
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
#else  
    gazebo::math::Vector3d m_worldBasePosition = m_baseLink->GetWorldPose().pos();
    gazebo::math::Quaterniond m_worldBaseOrientation = m_baseLink->GetWorldPose().rot();
    gazebo::math::Vector3d m_worldBaseLinVel = m_baseLink->GetWorldLinearVel();
    gazebo::math::Vector3d m_worldBaseAngVel = m_baseLink->GetWorldAngularVel();
    gazebo::math::Vector3d m_worldBaseLinAcc = m_baseLink->GetWorldLinearAccel();
    gazebo::math::Vector3d m_worldBaseAngAcc = m_baseLink->GetWorldAngularAccel();

    // Serializing the state vector
    m_baseState[0] = m_worldBasePosition.x;
    m_baseState[1] = m_worldBasePosition.y;
    m_baseState[2] = m_worldBasePosition.z;
    m_baseState[3] = m_worldBaseOrientation.GetRoll();
    m_baseState[4] = m_worldBaseOrientation.GetPitch();
    m_baseState[5] = m_worldBaseOrientation.GetYaw();
    
    m_baseState[6] = m_worldBaseLinVel.x;
    m_baseState[7] = m_worldBaseLinVel.y;
    m_baseState[8] = m_worldBaseLinVel.z;
    m_baseState[9] = m_worldBaseAngVel.x;
    m_baseState[10] = m_worldBaseAngVel.y;
    m_baseState[11] = m_worldBaseAngVel.z;
    
    m_baseState[12] = m_worldBaseLinAcc.x;
    m_baseState[13] = m_worldBaseLinAcc.y;
    m_baseState[14] = m_worldBaseLinAcc.z;
    m_baseState[15] = m_worldBaseAngAcc.x;
    m_baseState[16] = m_worldBaseAngAcc.y;
    m_baseState[17] = m_worldBaseAngAcc.z;   
#endif
    
    m_stamp.update(_info.simTime.Double());
    m_dataAvailable = true;
}

int GazeboYarpBaseStateDriver::read(yarp::sig::Vector& out)
{
    yarp::os::LockGuard guard(m_dataMutex);
    
    if (!m_dataAvailable)
    {
      return AS_TIMEOUT;
    }
    
    out.resize(m_baseState.size());
    out = m_baseState;
  
    return AS_OK;
}

yarp::os::Stamp GazeboYarpBaseStateDriver::getLastInputStamp()
{
    return m_stamp;
}


int GazeboYarpBaseStateDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

int GazeboYarpBaseStateDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int GazeboYarpBaseStateDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int GazeboYarpBaseStateDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpBaseStateDriver::getChannels()
{
    return AS_OK;
}

int GazeboYarpBaseStateDriver::getState(int ch)
{
    return AS_OK;
}

