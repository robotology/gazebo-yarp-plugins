/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/IMUDriver.h"

#include "gazebo_yarp_plugins/Handler.hh"
#include "gazebo_yarp_plugins/common.h"

#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/ImuSensor.hh>

using namespace yarp::dev;

const int YarpIMUChannelsNumber = 12; //The IMU has 12 fixed channels
const std::string YarpScopedName = "sensorScopedName";

GazeboYarpIMUDriver::GazeboYarpIMUDriver() {}
GazeboYarpIMUDriver::~GazeboYarpIMUDriver() {}

/**
 *
 * Export an inertial sensor.
 * The network interface is a single Port.
 * We will stream bottles with 12 floats:
 * 0  1   2  = Euler orientation data (Kalman filter processed)
 * 3  4   5  = Calibrated 3-axis (X, Y, Z) acceleration data
 * 6  7   8  = Calibrated 3-axis (X, Y, Z) gyroscope data
 * 9 10 11   = Calibrated 3-axis (X, Y, Z) magnetometer data
 *
 * \todo TODO check orientation data
 */
void GazeboYarpIMUDriver::onUpdate(const gazebo::common::UpdateInfo &/*_info*/)
{
    gazebo::math::Vector3 euler_orientation;
    gazebo::math::Vector3 linear_acceleration;
    gazebo::math::Vector3 angular_velocity;
        
    euler_orientation = this->m_parentSensor->GetOrientation().GetAsEuler();
    linear_acceleration = this->m_parentSensor->GetLinearAcceleration();
    angular_velocity = this->m_parentSensor->GetAngularVelocity();
    
    /** \todo TODO ensure that the timestamp is the right one */
    m_lastTimestamp.update(this->m_parentSensor->GetLastUpdateTime().Double());
    
    int i=0;
    
    m_dataMutex.wait();
    
    for (i = 0; i < 3; i++) {
        m_imuData[0 + i] = GazeboYarpPlugins::convertRadiansToDegrees(euler_orientation[i]);
    }
    
    for (i = 0; i < 3; i++) {
        m_imuData[3 + i] = linear_acceleration[i];
    }
    
    for (i = 0; i < 3; i++) {
        m_imuData[6 + i] = GazeboYarpPlugins::convertRadiansToDegrees(angular_velocity[i]);
    }
    
    m_dataMutex.post();
    return;
}
    
//DEVICE DRIVER
bool GazeboYarpIMUDriver::open(yarp::os::Searchable& config)
{
    m_dataMutex.wait();
    m_imuData.resize(YarpIMUChannelsNumber, 0.0);
    m_dataMutex.post();
    
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());
    std::cout << "GazeboYarpIMUDriver is looking for sensor " << sensorScopedName << "..." << std::endl;
    
    m_parentSensor = (gazebo::sensors::ImuSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    
    if (!m_parentSensor) {
        std::cout << "GazeboYarpIMUDriver Error: IMU sensor was not found" << std::endl;
        return false;
    }
    
    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpIMUDriver::onUpdate, this, _1));
  
    return true;
}

bool GazeboYarpIMUDriver::close()
{
    if (this->m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
    }
    return true;
}
    
//GENERIC SENSOR
bool GazeboYarpIMUDriver::read(yarp::sig::Vector &out)
{    
    if ((int)m_imuData.size() != YarpIMUChannelsNumber ) {
        return false;
    }
    
    //< \todo TODO this should be avoided by properly modifyng the wrapper
    if (out.size() != m_imuData.size()) {
        out.resize(m_imuData.size());
    }
    
    m_dataMutex.wait();
    out = m_imuData;
    m_dataMutex.post();
    
    return true;
}

bool GazeboYarpIMUDriver::getChannels(int *nc)
{
    if (!nc) return false;
    *nc = YarpIMUChannelsNumber;
    return true;
}

bool GazeboYarpIMUDriver::calibrate(int ch, double v)
{
    return true; //Calibration is not needed in simulation
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpIMUDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}
