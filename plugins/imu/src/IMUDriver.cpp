/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "IMUDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/sensors/ImuSensor.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;

const unsigned YarpIMUChannelsNumber = 12; //The IMU has 12 fixed channels
const std::string YarpIMUScopedName = "sensorScopedName";

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
    ignition::math::Vector3d euler_orientation = this->m_parentSensor->Orientation().Euler();
    ignition::math::Vector3d linear_acceleration = this->m_parentSensor->LinearAcceleration();
    ignition::math::Vector3d angular_velocity = this->m_parentSensor->AngularVelocity();

    /** \todo TODO ensure that the timestamp is the right one */
    m_lastTimestamp.update(this->m_parentSensor->LastUpdateTime().Double());

    m_dataMutex.wait();

    for (unsigned i = 0; i < 3; i++) {
        m_imuData[0 + i] = GazeboYarpPlugins::convertRadiansToDegrees(euler_orientation[i]);
    }

    for (unsigned i = 0; i < 3; i++) {
        m_imuData[3 + i] = linear_acceleration[i];
    }

    for (unsigned i = 0; i < 3; i++) {
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
    std::string sensorScopedName(config.find(YarpIMUScopedName.c_str()).asString().c_str());

    m_parentSensor = dynamic_cast<gazebo::sensors::ImuSensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));

    if (!m_parentSensor) {
        yError() << "GazeboYarpIMUDriver Error: IMU sensor was not found";
        return false;
    }

    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpIMUDriver::onUpdate, this, _1));

    return true;
}

bool GazeboYarpIMUDriver::close()
{
    this->m_updateConnection.reset();
    return true;
}

//GENERIC SENSOR
bool GazeboYarpIMUDriver::read(yarp::sig::Vector &out)
{
    if (m_imuData.size() != YarpIMUChannelsNumber ) {
        return false;
    }

    ///< \todo TODO this should be avoided by properly modifyng the wrapper
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

bool GazeboYarpIMUDriver::calibrate(int /*ch*/, double /*v*/)
{
    return true; //Calibration is not needed in simulation
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpIMUDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}
