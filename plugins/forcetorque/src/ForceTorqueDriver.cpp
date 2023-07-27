/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ForceTorqueDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <boost/bind/bind.hpp>
#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace boost::placeholders;

using namespace yarp::dev;

const unsigned YarpForceTorqueChannelsNumber = 6; //The ForceTorque sensor has 6 fixed channels
const std::string YarpForceTorqueScopedName = "sensorScopedName";

const unsigned YarpTemperatureChannelsNumber = 1; //The Temperature sensor has 1 fixed channel
const double fakeTemperatureValue = 25.0;

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
    ignition::math::Vector3d force = this->m_parentSensor->Force();
    ignition::math::Vector3d torque = this->m_parentSensor->Torque();

    /** \todo ensure that the timestamp is the right one */
    /** \todo TODO use GetLastMeasureTime, not GetLastUpdateTime */
    m_lastTimestamp.update(this->m_parentSensor->LastUpdateTime().Double());


    std::lock_guard<std::mutex> lock(m_dataMutex);

    for (unsigned i = 0; i < 3; i++) {
        m_forceTorqueData[0 + i] = force[i];
    }

    for (unsigned i = 0; i < 3; i++) {
        m_forceTorqueData[3 + i] = torque[i];
    }

    return;
}

//DEVICE DRIVER
bool GazeboYarpForceTorqueDriver::open(yarp::os::Searchable& config)
{
    {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        m_forceTorqueData.resize(YarpForceTorqueChannelsNumber, 0.0);
    }

    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpForceTorqueScopedName.c_str()).asString().c_str());

    m_sensorName = GazeboYarpPlugins::lastPartOfStringAfterSeparator(config.find("sensor_name").asString(), "::");
    m_frameName = m_sensorName;

    m_parentSensor = dynamic_cast<gazebo::sensors::ForceTorqueSensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));

    if (!m_parentSensor)
    {
        yError() << "Error, ForceTorque sensor was not found";
        return AS_ERROR;
    }

    //Connect the driver to the gazebo simulation
    using namespace boost::placeholders;
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpForceTorqueDriver::onUpdate, this, _1));

    return true;
}

bool GazeboYarpForceTorqueDriver::close()
{
    this->m_updateConnection.reset();
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

   if (m_forceTorqueData.size() != YarpForceTorqueChannelsNumber) {
        return AS_ERROR;
   }

   if (out.size() != YarpForceTorqueChannelsNumber) {
       out.resize(YarpForceTorqueChannelsNumber);
   }

    std::lock_guard<std::mutex> lock(m_dataMutex);
    out = m_forceTorqueData;

    return AS_OK;
}

int GazeboYarpForceTorqueDriver::getChannels()
{
    return YarpForceTorqueChannelsNumber;
}

int GazeboYarpForceTorqueDriver::getState(int /*ch*/)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateSensor(const yarp::sig::Vector& /*value*/)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateChannel(int /*ch*/)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateChannel(int /*ch*/, double /*v*/)
{
    return AS_OK;
}

// SIX AXIS FORCE TORQUE SENSORS
size_t GazeboYarpForceTorqueDriver::getNrOfSixAxisForceTorqueSensors() const
{
    return 1;
}

yarp::dev::MAS_status GazeboYarpForceTorqueDriver::getSixAxisForceTorqueSensorStatus(size_t sens_index) const
{
    if (sens_index >= 1)
    {
        return MAS_UNKNOWN;
    }

    return MAS_OK;
}

bool GazeboYarpForceTorqueDriver::getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const
{
    if (sens_index >= 1)
    {
        return false;
    }

    name = m_sensorName;
    return true;
}

bool GazeboYarpForceTorqueDriver::getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const
{
    if (sens_index >= 1)
    {
        return false;
    }

    frameName = m_frameName;
    return true;
}

bool GazeboYarpForceTorqueDriver::getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index >= 1)
    {
        return false;
    }

   if (m_forceTorqueData.size() != YarpForceTorqueChannelsNumber) {
        return false;
   }

   if (out.size() != YarpForceTorqueChannelsNumber) {
       out.resize(YarpForceTorqueChannelsNumber);
   }

    std::lock_guard<std::mutex> lock(m_dataMutex);
    out = m_forceTorqueData;
    timestamp = m_lastTimestamp.getTime();

    return true;
}

// TEMPERATURE SENSORS
size_t GazeboYarpForceTorqueDriver::getNrOfTemperatureSensors() const
{
    return 1;
}

yarp::dev::MAS_status GazeboYarpForceTorqueDriver::getTemperatureSensorStatus(size_t sens_index) const
{
    if (sens_index >= 1)
    {
        return MAS_UNKNOWN;
    }

    return MAS_OK;
}

bool GazeboYarpForceTorqueDriver::getTemperatureSensorName(size_t sens_index, std::string &name) const
{
    if (sens_index >= 1)
    {
        return false;
    }

    name = m_sensorName;
    return true;
}

bool GazeboYarpForceTorqueDriver::getTemperatureSensorFrameName(size_t sens_index, std::string &frameName) const
{
    if (sens_index >= 1)
    {
        return false;
    }

    frameName = m_frameName;
    return true;
}

bool GazeboYarpForceTorqueDriver::getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index >= 1)
    {
        return false;
    }

   if (out.size() != YarpTemperatureChannelsNumber) {
       out.resize(YarpTemperatureChannelsNumber);
   }

    std::lock_guard<std::mutex> lock(m_dataMutex);
    // No need for a Mutex here since we are just setting a value
    out = fakeTemperatureValue;
    timestamp = m_lastTimestamp.getTime();

    return true;
}


//PRECISELY TIMED
yarp::os::Stamp GazeboYarpForceTorqueDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}
