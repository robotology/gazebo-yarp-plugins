/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ForceTorqueDriver.h"
#include "gazebo_yarp_plugins/Handler.hh"

#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/ForceTorqueSensor.hh>

using namespace yarp::dev;

const int YarpForceTorqueChannelsNumber = 6; //The IMU has 6 fixed channels
const std::string YarpScopedName = "sensorScopedName";

GazeboYarpForceTorqueDriver::GazeboYarpForceTorqueDriver() {}
GazeboYarpForceTorqueDriver::~GazeboYarpForceTorqueDriver() {}

/**
 *
 * Export a force/torque sensor.
 * 
 * \todo check forcetorque data
 */
void GazeboYarpForceTorqueDriver::onUpdate(const gazebo::common::UpdateInfo& /*_info*/)
{
    gazebo::math::Vector3 force;
    gazebo::math::Vector3 torque;
    
    force = this->m_parentSensor->GetForce();
    torque = this->m_parentSensor->GetTorque();
    
    /** \todo ensure that the timestamp is the right one */
    /** \todo TODO use GetLastMeasureTime, not GetLastUpdateTime */
    m_lastTimestamp.update(this->m_parentSensor->GetLastUpdateTime().Double());
    
    int i=0;
    m_dataMutex.wait();

    for (i = 0; i < 3; i++) {
        m_forceTorqueData[0 + i] = force[i];
    }
    
    for (i = 0; i < 3; i++) {
        m_forceTorqueData[3 + i] = torque[i];
    }
    
    m_dataMutex.post();
    return;
}
    
//DEVICE DRIVER
bool GazeboYarpForceTorqueDriver::open(yarp::os::Searchable& config)
{
    std::cout << "GazeboYarpForceTorqueDriver::open() called" << std::endl;
  
    m_dataMutex.wait();
    m_forceTorqueData.resize(YarpForceTorqueChannelsNumber, 0.0);
    m_dataMutex.post();
    
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());
    std::cout << "GazeboYarpForceTorqueDriver::open is looking for sensor " << sensorScopedName << "..." << std::endl;
    
    m_parentSensor = (gazebo::sensors::ForceTorqueSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    
    if (!m_parentSensor)
    {
        std::cout << "Error, ForceTorque sensor was not found" << std::endl;
        return AS_ERROR;
    }
    
    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpForceTorqueDriver::onUpdate, this, _1));
  
    std::cout << "GazeboYarpForceTorqueDriver::open() returning true" << std::endl;
    return true;
}

bool GazeboYarpForceTorqueDriver::close()
{
    if (this->m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
    }
    return true;
}
    
//ANALOG SENSOR
int GazeboYarpForceTorqueDriver::read(yarp::sig::Vector& out)
{
    ///< \todo TODO in my opinion the reader should care of passing a vector of the proper dimension to the driver, but apparently this is not the case
    /*
    if( (int)m_forceTorqueData.size() != YarpForceTorqueChannelsNumber ||
        (int)out.size() != YarpForceTorqueChannelsNumber ) {
        return AS_ERROR;
    }
    */
    
   if ((int)m_forceTorqueData.size() != YarpForceTorqueChannelsNumber) {
        return AS_ERROR;
   }
   
   if ((int)out.size() != YarpForceTorqueChannelsNumber) {
       out.resize(YarpForceTorqueChannelsNumber);
   }
    
    m_dataMutex.wait();
    out = m_forceTorqueData;
    m_dataMutex.post();
    
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::getChannels()
{
    return YarpForceTorqueChannelsNumber;
}

int GazeboYarpForceTorqueDriver::getState(int ch)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpForceTorqueDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}
