/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "IMUDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <boost/bind/bind.hpp>
#include <gazebo/sensors/ImuSensor.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace boost::placeholders;

using namespace yarp::dev;

const unsigned YarpIMUChannelsNumber = 12; //The IMU has 12 fixed channels
constexpr size_t rpyStartIdx   = 0;
constexpr size_t accelStartIdx = 3;
constexpr size_t gyroStartIdx  = 6;
constexpr size_t magnStartIdx  = 9;
const std::string YarpIMUScopedName {"sensorScopedName"};

GazeboYarpIMUDriver::GazeboYarpIMUDriver()  = default;
GazeboYarpIMUDriver::~GazeboYarpIMUDriver() = default;

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


    std::lock_guard<std::mutex> lock(m_dataMutex);

    for (unsigned i = 0; i < 3; i++) {
        m_imuData[0 + i] = GazeboYarpPlugins::convertRadiansToDegrees(euler_orientation[i]);
    }

    for (unsigned i = 0; i < 3; i++) {
        m_imuData[3 + i] = linear_acceleration[i];
    }

    for (unsigned i = 0; i < 3; i++) {
        m_imuData[6 + i] = GazeboYarpPlugins::convertRadiansToDegrees(angular_velocity[i]);
    }

}

//DEVICE DRIVER
bool GazeboYarpIMUDriver::open(yarp::os::Searchable& config)
{
    {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        m_imuData.resize(YarpIMUChannelsNumber, 0.0);
    }

    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpIMUScopedName).asString());
    std::string sensorName(config.find("sensor_name").asString());

    m_sensorName=sensorName;
    m_frameName=sensorName;

    m_parentSensor = dynamic_cast<gazebo::sensors::ImuSensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));

    if (!m_parentSensor) {
        yError() << "GazeboYarpIMUDriver Error: IMU sensor was not found";
        return false;
    }

    //Connect the driver to the gazebo simulation
    using namespace boost::placeholders;
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

    std::lock_guard<std::mutex> lock(m_dataMutex);
    out = m_imuData;

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

size_t GazeboYarpIMUDriver::getNrOfThreeAxisGyroscopes() const
{
    return 1;
}


yarp::dev::MAS_status GazeboYarpIMUDriver::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool GazeboYarpIMUDriver::getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool GazeboYarpIMUDriver::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool GazeboYarpIMUDriver::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(sens_index, out, timestamp, gyroStartIdx);
}

size_t GazeboYarpIMUDriver::getNrOfThreeAxisLinearAccelerometers() const
{
    return 1;
}


yarp::dev::MAS_status GazeboYarpIMUDriver::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool GazeboYarpIMUDriver::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool GazeboYarpIMUDriver::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool GazeboYarpIMUDriver::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(sens_index, out, timestamp, accelStartIdx);
}


size_t GazeboYarpIMUDriver::getNrOfOrientationSensors() const
{
    return 1;
}

yarp::dev::MAS_status GazeboYarpIMUDriver::getOrientationSensorStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool GazeboYarpIMUDriver::getOrientationSensorName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool GazeboYarpIMUDriver::getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool GazeboYarpIMUDriver::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const
{
    return genericGetMeasure(sens_index, rpy, timestamp, rpyStartIdx);
}

size_t GazeboYarpIMUDriver::getNrOfThreeAxisMagnetometers() const
{
    return 1;
}

yarp::dev::MAS_status GazeboYarpIMUDriver::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool GazeboYarpIMUDriver::getThreeAxisMagnetometerName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool GazeboYarpIMUDriver::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool GazeboYarpIMUDriver::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(sens_index, out, timestamp, magnStartIdx);
}
yarp::dev::MAS_status GazeboYarpIMUDriver::genericGetStatus(size_t sens_index) const
{
    if (sens_index != 0)
    {
        yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return yarp::dev::MAS_status::MAS_ERROR;
    }

    return yarp::dev::MAS_status::MAS_OK;
}

bool GazeboYarpIMUDriver::genericGetSensorName(size_t sens_index, std::string& name) const
{
    if (sens_index != 0)
    {
        yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    name = m_sensorName;
    return true;
}

bool GazeboYarpIMUDriver::genericGetFrameName(size_t sens_index, std::string& frameName) const
{
    if (sens_index != 0)
    {
        yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    frameName = m_frameName;
    return true;

}

bool GazeboYarpIMUDriver::genericGetMeasure(size_t sens_index, yarp::sig::Vector &out, double &timestamp, size_t startIdx) const {

    if (sens_index != 0)
    {
        yError() << "GazeboYarpIMUDriver: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    out.resize(3);

    std::lock_guard<std::mutex> lock(m_dataMutex);
    out[0] = m_imuData[startIdx];
    out[1] = m_imuData[startIdx + 1];
    out[2] = m_imuData[startIdx + 2];

    timestamp = m_lastTimestamp.getTime();
    return true;
}
